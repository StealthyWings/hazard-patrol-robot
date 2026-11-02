// =====================================================================
//  Multi-Hazard Environmental Patrol Robot - main firmware (ESP32)
//  Patrols a taped line, stops at RFID checkpoints, measures mold risk,
//  air quality, gas/smoke and temperature anomalies against per-zone
//  adaptive baselines, then alerts (LED/buzzer/OLED/Telegram) and logs to SD.
// =====================================================================
#include "config.h"
#include "detection.h"
#include "sensors.h"
#include "alerts.h"
#include "navigation.h"
#include "logger.h"

using namespace detect;

enum State { FOLLOW_LINE, AT_CHECKPOINT, LEAVING_CHECKPOINT, WAIT_OBSTACLE, GAS_ALERT };
State state = FOLLOW_LINE;

Thresholds TH;
Baseline zoneMq135[MAX_ZONES], zoneTAir[MAX_ZONES], zoneMq2[MAX_ZONES];
bool zoneAlertActive[MAX_ZONES] = {false};
float mq2CleanV = 0.0f;       // clean-air MQ2 level learned at start-up
float lastTAir = 25.0f;
int currentZone = -1;
uint32_t stateSince = 0;

void enter(State s) { state = s; stateSince = millis(); }

// ---------------- start-up ----------------
void warmUpAndCalibrate() {
  for (int s = WARMUP_SECONDS; s > 0; s--) {
    oledLines("Patrol robot", "Gas sensors warming", "Keep air clean", String(s) + " s left", "");
    setColor(0, 0, 1); delay(500); setColor(0, 0, 0); delay(500);
  }
  float sum = 0;
  for (int i = 0; i < 20; i++) { sum += readMQVolts(PIN_MQ2); delay(100); }
  mq2CleanV = sum / 20.0f;
  Serial.printf("MQ2 clean-air baseline: %.3f V\n", mq2CleanV);
}

void setup() {
  Serial.begin(115200);
  alertsBegin();
  sensorsBegin();
  navigationBegin();
  loggerBegin();
  bool wifi = wifiBegin();
  oledLines("Patrol robot", String("SHT31 ") + (shtOK ? "OK" : "--") + "  MLX " + (mlxOK ? "OK" : "--"),
            String("SD ") + (sdOK ? "OK" : "--") + "  WiFi " + (wifi ? "OK" : "--"), "", "");
  delay(1500);
  warmUpAndCalibrate();
  if (wifi) sendTelegram("Patrol robot online. MQ2 baseline " + String(mq2CleanV, 2) + " V");
  enter(FOLLOW_LINE);
}

// ---------------- checkpoint sweep ----------------
void doCheckpointSweep(int zone) {
  Reading r = readEnvironment();
  if (!isValid(r)) {
    logRow(timestamp() + "," + String(zone) + ",invalid reading,,,,,,,,SKIPPED");
    return;
  }
  lastTAir = r.tAir;
  float td = dewPoint(r.tAir, r.rh);
  float rhs = surfaceRH(r.tAir, r.rh, r.tSurf);

  Scores z;
  bool freeze = (zone >= 0) && zoneAlertActive[zone];
  if (zone >= 0) {
    z.zMq135 = scoreAndUpdate(zoneMq135[zone], r.mq135V, freeze);
    z.zTAir = scoreAndUpdate(zoneTAir[zone], r.tAir, freeze);
    z.zMq2 = scoreAndUpdate(zoneMq2[zone], r.mq2V, freeze);
  }
  Alert a = classify(r, mq2CleanV, z, TH);
  bool wasActive = (zone >= 0) && zoneAlertActive[zone];
  if (zone >= 0) zoneAlertActive[zone] = (a != ALERT_OK);

  String zname = zone >= 0 ? ZONES[zone].name : "Unknown zone";
  oledLines(zname,
            "Air " + String(r.tAir, 1) + "C RH " + String(r.rh, 0) + "%",
            "Wall " + String(r.tSurf, 1) + "C RHs " + String(rhs, 0) + "%",
            "MQ2 " + String(r.mq2V, 2) + " MQ135 " + String(r.mq135V, 2),
            ">> " + String(alertName(a)));
  signalAlert(a);

  logRow(timestamp() + "," + zname + "," + String(r.tAir, 2) + "," + String(r.rh, 1) + "," +
         String(r.tSurf, 2) + "," + String(td, 2) + "," + String(rhs, 1) + "," +
         String(r.mq2V, 3) + "," + String(r.mq135V, 3) + "," + String(z.zMq135, 2) + "," +
         alertName(a));

  // Telegram only when an alert STARTS (avoids spamming / rate limits)
  if (a != ALERT_OK && !wasActive)
    sendTelegram(String(alertName(a)) + " at " + zname + " | wall RH " + String(rhs, 0) +
                 "%, air " + String(r.tAir, 1) + "C, MQ2 " + String(r.mq2V, 2) + "V");
}

// ---------------- main loop (state machine) ----------------
void loop() {
  // Gas is checked continuously, whatever the robot is doing
  float mq2 = readMQVolts(PIN_MQ2);
  if (state != GAS_ALERT && (mq2 > TH.gasAbsLimitV || mq2 > mq2CleanV * TH.gasRatio)) {
    stopMotors();
    String where = currentZone >= 0 ? ZONES[currentZone].name : "between checkpoints";
    sendTelegram("GAS/SMOKE detected near " + where + " (MQ2 " + String(mq2, 2) + " V)");
    logRow(timestamp() + "," + where + ",,,,,," + String(mq2, 3) + ",,,GAS/SMOKE");
    enter(GAS_ALERT);
  }

  switch (state) {
    case FOLLOW_LINE: {
      if (readDistanceCm(lastTAir) < OBSTACLE_STOP_CM) { stopMotors(); enter(WAIT_OBSTACLE); break; }
      String uid = readTagUID();
      if (uid.length()) {
        stopMotors();
        currentZone = zoneForUID(uid);
        if (currentZone < 0) Serial.println("Unknown tag " + uid);
        enter(AT_CHECKPOINT);
        break;
      }
      followLineStep();
      setColor(0, 1, 0);
      break;
    }
    case AT_CHECKPOINT:
      if (millis() - stateSince >= CHECKPOINT_SETTLE_MS) {   // let readings settle
        doCheckpointSweep(currentZone);
        enter(LEAVING_CHECKPOINT);
      }
      break;

    case LEAVING_CHECKPOINT:                                 // drive off the tag
      followLineStep();
      if (millis() - stateSince >= CHECKPOINT_LEAVE_MS) enter(FOLLOW_LINE);
      break;

    case WAIT_OBSTACLE:
      setColor(1, 1, 1);
      if (readDistanceCm(lastTAir) >= OBSTACLE_STOP_CM) enter(FOLLOW_LINE);
      break;

    case GAS_ALERT:
      stopMotors();
      signalAlert(ALERT_GAS);
      oledLines("!! GAS / SMOKE !!", "MQ2 " + String(mq2, 2) + " V",
                "Baseline " + String(mq2CleanV, 2) + " V", "Robot stopped", "");
      if (mq2 < mq2CleanV * 1.2f && mq2 < TH.gasAbsLimitV) {  // cleared
        buzzerQuiet();
        sendTelegram("Gas level back to normal. Resuming patrol.");
        enter(FOLLOW_LINE);
      }
      break;
  }
  delay(20);
}
