// =====================================================================
//  Bench demo (no motors): all environmental sensors + OLED + LED + buzzer.
//  Uses the same detection logic as the robot (detection.h).
//  Prints one CSV row per reading on Serial (115200 baud).
// =====================================================================
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_MLX90614.h>
#include "detection.h"
using namespace detect;

const int PIN_MQ2 = 34, PIN_MQ135 = 35;
const int PIN_R = 25, PIN_G = 26, PIN_B = 27, PIN_BUZ = 4;
const int BUZ_IDLE = HIGH;   // change to LOW if the buzzer hums when idle

Adafruit_SSD1306 oled(128, 64, &Wire, -1);
Adafruit_SHT31 sht;
Adafruit_MLX90614 mlx;
Thresholds TH;
Baseline bMq135;
float mq2Clean = 0;

float readMQ(int pin) {
  long s = 0;
  for (int i = 0; i < 32; i++) { s += analogReadMilliVolts(pin); delay(2); }
  return (s / 32.0f) * 2.0f / 1000.0f;   // undo 10k/10k divider
}
void color(bool r, bool g, bool b) { digitalWrite(PIN_R, r); digitalWrite(PIN_G, g); digitalWrite(PIN_B, b); }
void quiet() { noTone(PIN_BUZ); pinMode(PIN_BUZ, OUTPUT); digitalWrite(PIN_BUZ, BUZ_IDLE); }
void beep(int f, int ms) { tone(PIN_BUZ, f); delay(ms); quiet(); delay(80); }

void show(String a, String b, String c, String d, String e) {
  oled.clearDisplay(); oled.setCursor(0, 0);
  oled.println(a); oled.println(b); oled.println(c); oled.println(d); oled.println(e);
  oled.display();
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_R, OUTPUT); pinMode(PIN_G, OUTPUT); pinMode(PIN_B, OUTPUT); quiet();
  Wire.begin(21, 22);
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) Serial.println("OLED not found");
  oled.setTextColor(SSD1306_WHITE); oled.setTextSize(1);
  if (!sht.begin(0x44)) Serial.println("SHT31 not found (try 0x45)");
  if (!mlx.begin()) Serial.println("MLX90614 not found");

  for (int s = 60; s > 0; s--) {
    show("Gas sensors warming", "Keep air clean", "", String(s) + " s left", "");
    color(0, 0, 1); delay(500); color(0, 0, 0); delay(500);
  }
  for (int i = 0; i < 20; i++) { mq2Clean += readMQ(PIN_MQ2); delay(100); }
  mq2Clean /= 20;
  Serial.println("t_air,rh,t_surface,t_dew,rh_surface,mq2_v,mq135_v,z_mq135,alert");
}

void loop() {
  Reading r { sht.readTemperature(), sht.readHumidity(), (float)mlx.readObjectTempC(),
              readMQ(PIN_MQ2), readMQ(PIN_MQ135) };
  if (!isValid(r)) { Serial.println("invalid reading - skipped"); delay(1000); return; }

  static bool alertActive = false;
  Scores z;
  z.zMq135 = scoreAndUpdate(bMq135, r.mq135V, alertActive);
  Alert a = classify(r, mq2Clean, z, TH);
  alertActive = (a != ALERT_OK);

  float td = dewPoint(r.tAir, r.rh), rhs = surfaceRH(r.tAir, r.rh, r.tSurf);
  show("Air " + String(r.tAir, 1) + "C RH " + String(r.rh, 0) + "%",
       "Wall " + String(r.tSurf, 1) + "C Dew " + String(td, 1),
       "Wall RH " + String(rhs, 0) + "%",
       "MQ2 " + String(r.mq2V, 2) + " MQ135 " + String(r.mq135V, 2),
       ">> " + String(alertName(a)));
  Serial.println(String(r.tAir, 2) + "," + String(r.rh, 1) + "," + String(r.tSurf, 2) + "," +
                 String(td, 2) + "," + String(rhs, 1) + "," + String(r.mq2V, 3) + "," +
                 String(r.mq135V, 3) + "," + String(z.zMq135, 2) + "," + alertName(a));

  switch (a) {
    case ALERT_GAS:      for (int i = 0; i < 3; i++) { color(1,0,0); beep(1000,200); color(0,0,0); beep(1600,200); } break;
    case ALERT_HOT_SPOT: color(1,0,1); beep(2000,100); beep(2000,100); beep(2000,100); break;
    case ALERT_MOLD:     color(0,0,1); beep(1500,80); beep(1500,80); break;
    case ALERT_BAD_AIR:  color(1,1,0); beep(800,150); break;
    default:             color(0,1,0);
  }
  delay(1500);
}
