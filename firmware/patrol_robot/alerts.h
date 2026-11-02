// alerts.h - OLED, RGB LED and buzzer feedback
#pragma once
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "config.h"
#include "detection.h"

static Adafruit_SSD1306 oled(128, 64, &Wire, -1);
static bool oledOK = false;

inline void setColor(bool r, bool g, bool b) {
  digitalWrite(PIN_LED_R, r); digitalWrite(PIN_LED_G, g); digitalWrite(PIN_LED_B, b);
}
inline void buzzerQuiet() {
  noTone(PIN_BUZZER);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, BUZZER_IDLE_LEVEL);
}
inline void beep(int freq, int ms) { tone(PIN_BUZZER, freq); delay(ms); buzzerQuiet(); delay(80); }

inline void alertsBegin() {
  pinMode(PIN_LED_R, OUTPUT); pinMode(PIN_LED_G, OUTPUT); pinMode(PIN_LED_B, OUTPUT);
  buzzerQuiet();
  oledOK = oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (oledOK) { oled.setTextColor(SSD1306_WHITE); oled.setTextSize(1); }
}

inline void oledLines(const String &l1, const String &l2, const String &l3,
                      const String &l4, const String &l5) {
  if (!oledOK) return;
  oled.clearDisplay(); oled.setCursor(0, 0);
  oled.println(l1); oled.println(l2); oled.println(l3); oled.println(l4); oled.println(l5);
  oled.display();
}

// Colour + sound pattern per alert category
inline void signalAlert(detect::Alert a) {
  switch (a) {
    case detect::ALERT_GAS:        // red flashing + two-tone siren
      for (int i = 0; i < 3; i++) { setColor(1,0,0); beep(1000, 200); setColor(0,0,0); beep(1600, 200); }
      setColor(1,0,0);
      break;
    case detect::ALERT_HOT_SPOT:   // magenta + three high beeps
      setColor(1,0,1); beep(2000, 100); beep(2000, 100); beep(2000, 100);
      break;
    case detect::ALERT_MOLD:       // blue + two chirps
      setColor(0,0,1); beep(1500, 80); beep(1500, 80);
      break;
    case detect::ALERT_BAD_AIR:    // yellow + one low beep
      setColor(1,1,0); beep(800, 150);
      break;
    default:
      setColor(0,1,0);             // green = OK
  }
}
