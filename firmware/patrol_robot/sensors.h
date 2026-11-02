// sensors.h - environmental + navigation sensor reading
#pragma once
#include <Wire.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_MLX90614.h>
#include "config.h"
#include "detection.h"

static Adafruit_SHT31 sht31;
static Adafruit_MLX90614 mlx;
static bool shtOK = false, mlxOK = false;

inline void sensorsBegin() {
  Wire.begin(PIN_SDA, PIN_SCL);
  shtOK = sht31.begin(SHT31_ADDR);
  mlxOK = mlx.begin();
  analogReadResolution(12);
  pinMode(PIN_IR_LEFT, INPUT);
  pinMode(PIN_IR_RIGHT, INPUT);
  pinMode(PIN_US_TRIG, OUTPUT);
  pinMode(PIN_US_ECHO, INPUT);
}

// Average ADC_SAMPLES readings, return sensor output voltage (divider undone)
inline float readMQVolts(int pin) {
  long sum = 0;
  for (int i = 0; i < ADC_SAMPLES; i++) { sum += analogReadMilliVolts(pin); delay(2); }
  return (sum / (float)ADC_SAMPLES) * MQ_DIVIDER_GAIN / 1000.0f;
}

// Median of 5 reads - removes single-sample spikes
inline float medianOf5(float (*fn)()) {
  float v[5];
  for (int i = 0; i < 5; i++) v[i] = fn();
  for (int i = 1; i < 5; i++)
    for (int j = i; j > 0 && v[j - 1] > v[j]; j--) { float t = v[j]; v[j] = v[j - 1]; v[j - 1] = t; }
  return v[2];
}
inline float rawSurface() { return mlx.readObjectTempC(); }

// Full environmental sweep (used at checkpoints)
inline detect::Reading readEnvironment() {
  detect::Reading r;
  r.tAir = shtOK ? sht31.readTemperature() : NAN;
  r.rh = shtOK ? sht31.readHumidity() : NAN;
  r.tSurf = mlxOK ? medianOf5(rawSurface) : NAN;
  r.mq2V = readMQVolts(PIN_MQ2);
  r.mq135V = readMQVolts(PIN_MQ135);
  return r;
}

// Ultrasonic distance with temperature-compensated speed of sound
inline float readDistanceCm(float tempC) {
  digitalWrite(PIN_US_TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(PIN_US_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_US_TRIG, LOW);
  unsigned long us = pulseIn(PIN_US_ECHO, HIGH, 30000UL);
  if (us == 0) return 999.0f;
  if (isnan(tempC)) tempC = 25.0f;
  float v = 331.3f + 0.606f * tempC;          // m/s
  return (us / 1.0e6f) * v / 2.0f * 100.0f;   // cm
}
