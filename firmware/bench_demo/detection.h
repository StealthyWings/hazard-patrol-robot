// detection.h - hazard detection logic (pure C++, no Arduino dependencies)
// Kept hardware-independent so it can be unit-tested on a laptop (see firmware/tests).
#pragma once
#include <math.h>
#include <stdint.h>

namespace detect {

// ---------- Magnus formula (Alduchov & Eskridge, 1996) ----------
const float MAG_A = 17.62f;
const float MAG_B = 243.12f;   // deg C

// Saturation vapour pressure in hPa at temperature tC (deg C)
inline float satVapourPressure(float tC) {
  return 6.112f * expf(MAG_A * tC / (MAG_B + tC));
}

// Actual vapour pressure of room air (hPa)
inline float vapourPressure(float tAir, float rh) {
  return (rh / 100.0f) * satVapourPressure(tAir);
}

// Dew point (deg C)
inline float dewPoint(float tAir, float rh) {
  float g = logf(vapourPressure(tAir, rh) / 6.112f);
  return MAG_B * g / (MAG_A - g);
}

// Relative humidity at a surface of temperature tSurf (%). Can exceed 100 (= condensation).
inline float surfaceRH(float tAir, float rh, float tSurf) {
  return 100.0f * vapourPressure(tAir, rh) / satVapourPressure(tSurf);
}

// ---------- MQ gas sensor ----------
// vOut: sensor output voltage (after undoing the divider), vc: supply, rl: load resistor (ohm)
inline float mqResistance(float vOut, float vc, float rl) {
  if (vOut < 0.01f) vOut = 0.01f;
  return rl * (vc - vOut) / vOut;
}

// ---------- Per-zone adaptive baseline (EWMA mean / variance) ----------
struct Baseline {
  float mean = 0.0f;
  float var = 0.0f;
  uint16_t n = 0;
};

const uint16_t WARMUP_SAMPLES = 10;   // plain running mean/var until this many samples
const float Z_MIN_STD = 0.05f;        // floor on std-dev so z never explodes on flat data

// Returns z-score of x against the baseline BEFORE updating.
// While n < WARMUP_SAMPLES it only learns and returns 0.
// If freeze is true (an alert is active) the baseline is not updated.
inline float scoreAndUpdate(Baseline &b, float x, bool freeze, float alpha = 0.05f) {
  if (b.n < WARMUP_SAMPLES) {
    b.n++;
    float d = x - b.mean;
    b.mean += d / b.n;
    b.var += (d * (x - b.mean) - b.var) / b.n;
    return 0.0f;
  }
  float sd = sqrtf(b.var);
  if (sd < Z_MIN_STD) sd = Z_MIN_STD;
  float z = (x - b.mean) / sd;
  if (!freeze) {
    float d = x - b.mean;
    b.mean += alpha * d;
    b.var = (1.0f - alpha) * (b.var + alpha * d * d);
  }
  return z;
}

// ---------- Alert classification ----------
enum Alert : uint8_t { ALERT_OK = 0, ALERT_BAD_AIR, ALERT_MOLD, ALERT_HOT_SPOT, ALERT_GAS };

inline const char *alertName(Alert a) {
  switch (a) {
    case ALERT_GAS: return "GAS/SMOKE";
    case ALERT_HOT_SPOT: return "HOT SPOT";
    case ALERT_MOLD: return "MOLD RISK";
    case ALERT_BAD_AIR: return "BAD AIR";
    default: return "OK";
  }
}

struct Thresholds {
  float moldSurfaceRH = 80.0f;   // % (ISO 13788 criterion)
  float hotSpotDelta = 10.0f;    // deg C surface above air
  float gasAbsLimitV = 2.5f;     // fixed MQ2 safety limit (V) - never adapts
  float gasRatio = 1.5f;         // MQ2 voltage vs clean-air baseline
  float zLimit = 3.0f;           // per-zone anomaly z-score
};

struct Reading {
  float tAir, rh, tSurf;   // deg C, %, deg C
  float mq2V, mq135V;      // sensor output volts
};

struct Scores {           // per-zone z-scores (0 during warm-up)
  float zMq135 = 0, zTAir = 0, zMq2 = 0;
};

// Highest-priority hazard wins: gas > hot spot > mold > bad air.
inline Alert classify(const Reading &r, float mq2CleanV, const Scores &z, const Thresholds &t) {
  if (r.mq2V > t.gasAbsLimitV || r.mq2V > mq2CleanV * t.gasRatio || z.zMq2 > t.zLimit)
    return ALERT_GAS;
  if (r.tSurf - r.tAir > t.hotSpotDelta) return ALERT_HOT_SPOT;
  if (surfaceRH(r.tAir, r.rh, r.tSurf) >= t.moldSurfaceRH) return ALERT_MOLD;
  if (z.zMq135 > t.zLimit) return ALERT_BAD_AIR;
  return ALERT_OK;
}

// Basic sanity check on a reading (rejects I2C glitches / impossible values)
inline bool isValid(const Reading &r) {
  if (isnan(r.tAir) || isnan(r.rh) || isnan(r.tSurf)) return false;
  if (r.rh <= 0.0f || r.rh > 100.0f) return false;
  if (r.tAir < -10.0f || r.tAir > 60.0f) return false;
  if (r.tSurf < -20.0f || r.tSurf > 120.0f) return false;
  if (r.mq2V < 0.0f || r.mq2V > 5.2f || r.mq135V < 0.0f || r.mq135V > 5.2f) return false;
  return true;
}

}  // namespace detect
