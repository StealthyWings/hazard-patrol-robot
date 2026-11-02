# Multi-Hazard Environmental Patrol Robot

An ESP32-based service robot that patrols hostel corridors along a taped line, stops at
RFID-tagged checkpoints and checks each zone for four hazards:

| Hazard | Sensors | Rule |
|---|---|---|
| Mold / condensation risk | SHT31 + MLX90614 | Wall-surface RH (Magnus formula) >= 80 % (ISO 13788) |
| Bad air / stale odour | MQ135 | Per-zone adaptive z-score > 3 |
| Gas / smoke (safety) | MQ2 | > 1.5x clean-air level, or > 2.5 V fixed limit, checked continuously |
| Temperature anomaly | MLX90614 vs SHT31 | Surface > 10 C hotter than room air |

Alerts: RGB LED colour + buzzer tone per hazard, OLED, Telegram message; every reading is
logged to microSD as CSV.

**Novelty:** per-zone adaptive baselines (EWMA mean/variance, frozen during alerts so a slow
leak is never learned as normal), a physics-based mold metric, and categorised multi-hazard
alerts on one low-cost mobile platform (~Rs 5,000).

## Repository layout

```
firmware/
  patrol_robot/        full robot firmware (state machine)
    patrol_robot.ino   main loop: FOLLOW_LINE / AT_CHECKPOINT / WAIT_OBSTACLE / GAS_ALERT
    config.h           pin map, thresholds, zone list
    detection.h        hardware-independent detection logic (Magnus, EWMA, classifier)
    sensors.h          SHT31, MLX90614, MQ2/MQ135, HC-SR04
    alerts.h           OLED, RGB LED, buzzer
    navigation.h       TB6612FNG motors, line following, RC522 RFID
    logger.h           SD-card CSV, NTP time, Telegram
    secrets_example.h  copy to secrets.h (git-ignored)
  bench_demo/          sensors + alerts only, no motors (Review 1 / bench testing)
  tests/               host-side unit tests for detection.h (g++)
simulation/
  detection_sim.py     offline evaluation on a synthetic dataset
  results/             metrics, plots, confusion matrix, unit-test output
docs/
  architecture.png, pin_diagram.png, make_diagrams.py
```

## Status (Review 2)

| Part | Status |
|---|---|
| Design, pin map, architecture | Done |
| Components | Bought (battery/BMS/charger ordered) |
| Firmware | Written; detection logic unit-tested on PC (20/20 pass); not yet flashed to hardware |
| Detection algorithm | Evaluated offline on synthetic data |
| Hardware wiring / integration | Not started |

## Build

1. Arduino IDE 2.x + board package "esp32 by Espressif" -> board **ESP32 Dev Module**.
2. Libraries: Adafruit SSD1306, Adafruit GFX, Adafruit SHT31, Adafruit MLX90614,
   MFRC522 (GithubCommunity), UniversalTelegramBot, ArduinoJson.
3. `firmware/patrol_robot`: copy `secrets_example.h` to `secrets.h` and fill it in.
4. Put the RFID card UIDs into `ZONES[]` in `config.h`.

Unit tests (no hardware):

```
cd firmware/tests
g++ -std=c++17 -I../patrol_robot test_detection.cpp -o test && ./test
```

Simulation:

```
pip install numpy pandas matplotlib
python3 simulation/detection_sim.py
```

## Wiring summary

See `docs/pin_diagram.png`. ESP32 pins are 3.3 V only: MQ analog outputs go through 10k/10k
dividers, HC-SR04 ECHO through 4.7k/6.8k. RC522 is 3.3 V only. LM2596 must be set to 5.0 V
before anything is connected. TB6612 STBY tied to 3.3 V.
