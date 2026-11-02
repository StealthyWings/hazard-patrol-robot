// navigation.h - motors (TB6612FNG), line following, RFID checkpoints
#pragma once
#include <SPI.h>
#include <MFRC522.h>
#include "config.h"

static MFRC522 rfid(PIN_RFID_SS, UINT8_MAX);   // RST tied to 3.3 V -> no reset pin

inline void setMotor(int pwmPin, int in1, int in2, int speed) {   // speed -255..255
  speed = constrain(speed, -MOTOR_MAX_PWM, MOTOR_MAX_PWM);
  digitalWrite(in1, speed > 0);
  digitalWrite(in2, speed < 0);
  analogWrite(pwmPin, abs(speed));
}
inline void drive(int left, int right) {
  setMotor(PIN_PWMA, PIN_AIN1, PIN_AIN2, left);
  setMotor(PIN_PWMB, PIN_BIN1, PIN_BIN2, right);
}
inline void stopMotors() { drive(0, 0); }

inline void navigationBegin() {
  const int pins[] = {PIN_PWMA, PIN_AIN1, PIN_AIN2, PIN_PWMB, PIN_BIN1, PIN_BIN2};
  for (int p : pins) pinMode(p, OUTPUT);
  stopMotors();
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);
  rfid.PCD_Init();
}

// Two-sensor line follower: tape runs between the sensors
inline void followLineStep() {
  bool L = digitalRead(PIN_IR_LEFT) == IR_ON_TAPE;
  bool R = digitalRead(PIN_IR_RIGHT) == IR_ON_TAPE;
  if (!L && !R)     drive(CRUISE_SPEED, CRUISE_SPEED);   // centred
  else if (L && !R) drive(TURN_SPEED, CRUISE_SPEED);     // drifted right -> steer left
  else if (!L && R) drive(CRUISE_SPEED, TURN_SPEED);     // drifted left -> steer right
  else              stopMotors();                        // cross line / lost
}

// Returns card UID as upper-case hex, or "" if no new card
inline String readTagUID() {
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) return "";
  String uid;
  for (byte i = 0; i < rfid.uid.size; i++) {
    if (rfid.uid.uidByte[i] < 0x10) uid += "0";
    uid += String(rfid.uid.uidByte[i], HEX);
  }
  rfid.PICC_HaltA();
  uid.toUpperCase();
  return uid;
}

// Map a UID to a zone index (or -1 if unknown)
inline int zoneForUID(const String &uid) {
  for (int i = 0; i < MAX_ZONES; i++)
    if (uid.equalsIgnoreCase(ZONES[i].uid)) return i;
  return -1;
}
