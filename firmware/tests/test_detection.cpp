// Host-side unit tests for detection.h
// Build & run:  g++ -std=c++17 -I../patrol_robot test_detection.cpp -o test && ./test
#include <cstdio>
#include <cmath>
#include "detection.h"

using namespace detect;
static int fails = 0, passes = 0;

static void check(bool ok, const char *name) {
  if (ok) { passes++; printf("  PASS  %s\n", name); }
  else    { fails++;  printf("  FAIL  %s\n", name); }
}
static bool near(float a, float b, float tol) { return fabsf(a - b) <= tol; }

int main() {
  printf("Magnus / mold-risk chain\n");
  check(near(satVapourPressure(0), 6.112f, 0.001f), "es(0 C) = 6.112 hPa");
  check(near(satVapourPressure(20), 23.37f, 0.1f), "es(20 C) ~ 23.4 hPa");
  check(near(dewPoint(28, 75), 23.15f, 0.05f), "dew point 28 C / 75% ~ 23.2 C");
  check(near(surfaceRH(28, 75, 24), 95.0f, 0.2f), "surface RH, 24 C wall ~ 95%");
  check(near(surfaceRH(25, 60, 25), 60.0f, 0.01f), "surface RH = room RH when wall = air");
  check(surfaceRH(28, 75, 22) > 100.0f, "wall below dew point -> >100% (condensation)");

  printf("Adaptive baseline\n");
  Baseline b;
  for (int i = 0; i < 10; i++) scoreAndUpdate(b, 1.0f + 0.02f * (i % 3), false);
  check(b.n == 10 && near(b.mean, 1.02f, 0.02f), "warm-up learns mean");
  float zNormal = scoreAndUpdate(b, 1.03f, false);
  check(fabsf(zNormal) < 3.0f, "normal reading -> |z| < 3");
  float meanBefore = b.mean;
  float zSpike = scoreAndUpdate(b, 2.0f, true);
  check(zSpike > 3.0f, "spike -> z > 3");
  check(b.mean == meanBefore, "frozen baseline does not move during alert");
  scoreAndUpdate(b, 2.0f, false);
  check(b.mean > meanBefore, "unfrozen baseline adapts");

  printf("Classification priority\n");
  Thresholds t; Scores z;
  Reading ok   {26, 55, 25.5f, 0.6f, 0.8f};
  Reading mold {28, 75, 24.0f, 0.6f, 0.8f};
  Reading hot  {26, 55, 45.0f, 0.6f, 0.8f};
  Reading gas  {28, 75, 24.0f, 2.8f, 0.8f};   // also mold-risky, gas must win
  check(classify(ok, 0.6f, z, t) == ALERT_OK, "clean room -> OK");
  check(classify(mold, 0.6f, z, t) == ALERT_MOLD, "cold humid wall -> MOLD");
  check(classify(hot, 0.6f, z, t) == ALERT_HOT_SPOT, "hot surface -> HOT SPOT");
  check(classify(gas, 0.6f, z, t) == ALERT_GAS, "gas overrides mold");
  Reading rise {26, 55, 25.5f, 1.0f, 0.8f};
  check(classify(rise, 0.6f, z, t) == ALERT_GAS, "MQ2 1.67x baseline -> GAS");
  Scores zAir; zAir.zMq135 = 4.0f;
  check(classify(ok, 0.6f, zAir, t) == ALERT_BAD_AIR, "MQ135 z=4 -> BAD AIR");

  printf("Validation\n");
  Reading bad {26, 120, 25, 0.5f, 0.5f};
  check(!isValid(bad), "RH 120% rejected");
  check(isValid(ok), "normal reading accepted");
  check(!isValid(Reading{NAN, 50, 25, 0.5f, 0.5f}), "NaN rejected");

  printf("\n%d passed, %d failed\n", passes, fails);
  return fails ? 1 : 0;
}
