// logger.h - SD-card CSV logging, NTP time and Telegram alerts
#pragma once
#include <SD.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <time.h>
#include "config.h"
#include "secrets.h"   // copy secrets_example.h -> secrets.h

static bool sdOK = false;
static WiFiClientSecure tgClient;
static UniversalTelegramBot bot(BOT_TOKEN, tgClient);

inline void loggerBegin() {
  sdOK = SD.begin(PIN_SD_CS);
  if (sdOK && !SD.exists("/log.csv")) {
    File f = SD.open("/log.csv", FILE_WRITE);
    if (f) {
      f.println("timestamp,zone,t_air,rh,t_surface,t_dew,rh_surface,mq2_v,mq135_v,z_mq135,alert");
      f.close();
    }
  }
}

// Connect to WiFi (phone hotspot) with a timeout so the robot still runs offline
inline bool wifiBegin(uint32_t timeoutMs = 15000) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < timeoutMs) delay(250);
  if (WiFi.status() != WL_CONNECTED) return false;
  tgClient.setCACert(TELEGRAM_CERTIFICATE_ROOT);
  configTime(19800, 0, "pool.ntp.org");   // IST = UTC+5:30
  return true;
}

inline String timestamp() {
  struct tm t;
  if (!getLocalTime(&t, 50)) return String(millis() / 1000) + "s";
  char buf[24];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &t);
  return String(buf);
}

inline void logRow(const String &row) {
  Serial.println(row);   // always mirror to Serial
  if (!sdOK) return;
  File f = SD.open("/log.csv", FILE_APPEND);
  if (f) { f.println(row); f.close(); }
}

inline void sendTelegram(const String &msg) {
  if (WiFi.status() != WL_CONNECTED) return;
  bot.sendMessage(CHAT_ID, msg, "");
}
