# Multi-Hazard Environmental Patrol Robot (SERBOT)

An ESP32-based service robot that patrols hostel corridors and checks the environment for
hazards: gas or smoke, poor air quality, high temperature and humidity. It avoids obstacles
on its own, reads RFID cards at checkpoints and shows every reading on an OLED display,
an RGB LED, a buzzer and the Serial Monitor.

Course project for **21CSE305P – SERBOT: Project Based Learning in Robotics**,
Department of Networking and Communications, SRM Institute of Science and Technology,
Kattankulathur. Guide: Dr. Asha Abraham.

Team: Nevin Joshua Jose (RA2411026011201), Devansh Srivastava (RA2411033010177),
Sreerag Krishna (RA2411026010903), Vedaang Pratap Singh (RA2411003011530).

## Hardware on the robot

| Part | Job |
|---|---|
| ESP32 DevKit V1 (30-pin) | Controller |
| SHT31 | Air temperature and humidity (I2C 0x44) |
| MQ-2 | Gas / smoke (analog, raw ADC) |
| MQ-135 | Air quality (analog, raw ADC) |
| HC-SR04 | Obstacle distance |
| 2 × IR sensor modules | Left / right edge and obstacle detection |
| RC522 + RFID cards | Checkpoint identification (card UID) |
| TB6612FNG + 2 DC motors | Drive (2WD chassis) |
| SSD1306 0.96" OLED | Live readings (I2C 0x3C) |
| RGB LED + active buzzer | Status and alarm |
| 2 × 18650, 2S BMS, LM2596 (5 V) | Power |

The MLX90614 IR thermometer is supported by the code but is **optional and not fitted**
on the current build. If it is missing, the sketch prints a warning and shows "--" for
surface temperature; everything else keeps working.

## How it works (`firmware/SERBOT_ESP32_Integrated`)

Every second the robot:

1. **Reads all sensors:** distance (HC-SR04), both IR sensors, MQ-2 and MQ-135 (average
   of 10 ADC samples, 12-bit), SHT31 temperature and humidity, and the MLX90614 if present.
2. **Checks for hazards:**
   - **Gas alarm:** MQ-2 or MQ-135 raw value at or above its warning threshold
     (`MQ2_WARNING_RAW`, `MQ135_WARNING_RAW`, default 1800).
   - **Temperature alarm:** air temperature at or above 50 °C, or surface temperature
     at or above 60 °C (if the MLX90614 is fitted).
3. **Sets the alert:**
   - Red LED with a repeating beep for a gas or temperature alarm
   - Yellow for an obstacle or an IR sensor trigger
   - Green for normal
4. **Moves:**
   - Stops completely while any hazard alarm is active.
   - Both IR sensors or only the left one triggered: reverses and turns right.
   - Only the right IR sensor triggered: reverses and turns left.
   - Obstacle closer than 25 cm on the ultrasonic sensor: reverses and turns right.
   - Otherwise it drives forward at patrol speed.

Separately, the RC522 is polled every 250 ms. When a card is read, its UID is printed to
the Serial Monitor (for example `RFID UID: A3:4F:12:9C`) and the buzzer gives a short beep.
These UIDs are what we use to map cards to checkpoint zones.

The OLED refreshes every 500 ms and the Serial Monitor (115200 baud) prints a full reading
every second.

## Pin map (as used in the integrated sketch)

| Function | ESP32 GPIO | Notes |
|---|---|---|
| I2C SDA / SCL (SHT31, OLED, MLX90614) | 21 / 22 | 3.3 V bus |
| HC-SR04 TRIG / ECHO | 16 / 17 | ECHO through a voltage divider |
| IR left / right | 34 / 35 | Input-only pins |
| MQ-2 AO / MQ-135 AO | 36 (VP) / 39 (VN) | ADC1; each through its own divider |
| TB6612 AIN1 / AIN2 / PWMA | 25 / 26 / 27 | Motor A |
| TB6612 BIN1 / BIN2 / PWMB | 14 / 13 / 33 | Motor B |
| TB6612 STBY | 32 | Driven by the sketch |
| RC522 SS / RST | 5 / 4 | 3.3 V only |
| RC522 SCK / MISO / MOSI | 18 / 19 / 23 | VSPI |
| RGB R / G / B | 2 / 15 / 0 | Common cathode by default |
| Buzzer | 12 | Active buzzer |

**Check before wiring:**

- **GPIO 0, 2, 12 and 15 are ESP32 boot-strapping pins.** If the board will not boot or
  upload, disconnect the RGB LED and buzzer from these pins while uploading.
- **GPIO 0 is not broken out on most 30-pin DevKit V1 boards.** If yours has no GPIO 0
  header, move RGB blue to a free GPIO and change `RGB_B` to match.
- **This map differs from `firmware/patrol_robot/config.h`**, which uses an older layout.
  The integrated sketch is the one wired on the robot.

## Wiring safety

- ESP32 inputs are **3.3 V only**. MQ analog outputs and the HC-SR04 ECHO pin are 5 V signals
  and must each go through a voltage divider (we use 10 kΩ on top, 20 kΩ to GND).
- RC522 runs on **3.3 V only**.
- Set the LM2596 to **5.0 V with a multimeter** before connecting anything to it.
- Join all grounds: battery, buck converter, ESP32 and every module.

## Build and upload

1. Arduino IDE 2.x, board package **esp32 by Espressif** (core 3.x), board **ESP32 Dev Module**.
2. Install libraries from the Library Manager:
   - Adafruit SSD1306 (with Adafruit GFX)
   - Adafruit SHT31
   - Adafruit MLX90614
   - MFRC522 (GithubCommunity)
3. Open `firmware/SERBOT_ESP32_Integrated/SERBOT_ESP32_Integrated.ino`, select the COM port
   and upload. Open the Serial Monitor at **115200 baud**.

## Calibration

1. Burn in new MQ sensors for 24–48 hours, then let them warm up for a few minutes after
   every power-on.
2. With `ENABLE_AUTONOMOUS_MOTION = false`, leave the robot in clean air and note the
   MQ-2 and MQ-135 raw values in the Serial Monitor.
3. Set `MQ2_WARNING_RAW` and `MQ135_WARNING_RAW` comfortably above those clean-air values,
   then test safely (for example with a lighter's gas, unlit, held near the MQ-2).
4. If the IR sensors read the wrong way round, change `IR_ACTIVE_LEVEL` to `HIGH`.
   For a common-anode RGB LED, set `RGB_COMMON_ANODE = true`.
5. Tap each RFID card on the reader and record its UID for the checkpoint list.

MQ readings are raw ADC counts, not ppm. The robot is a monitoring aid and does not
replace certified smoke or gas alarms.

## Repository layout

```
firmware/
  SERBOT_ESP32_Integrated/   current sketch running on the robot (single file)
  patrol_robot/              planned full version: line following, RFID zone lookup,
                             per-zone adaptive (EWMA) baselines, SD logging, Telegram
                             (older pin map, not yet flashed)
  bench_demo/                sensors + alerts only, no motors (bench testing)
  tests/                     host-side unit tests for patrol_robot/detection.h (g++)
simulation/                  offline evaluation of the per-zone detection idea
docs/                        architecture and pin diagrams
```

## Status

| Part | Status |
|---|---|
| Chassis, motors, power | Assembled |
| Integrated sketch (sensors, OLED, RGB, buzzer, RFID UID read, obstacle avoidance) | Written and in this repo |
| MQ threshold calibration | Pending; defaults are placeholders |
| RFID card → zone mapping | Next step |
| Per-zone adaptive baselines, SD logging, Telegram (`patrol_robot`) | Planned |

## Next steps

- Map RFID UIDs to zone names and show the zone on the OLED.
- Bring in the per-zone EWMA baseline from `patrol_robot/detection.h` for MQ-135 and humidity.
- Add SD-card logging and Telegram alerts.
