// config.h - pin map, thresholds and timing for the patrol robot
#pragma once

// ---------------- I2C bus (OLED 0x3C, SHT31 0x44, MLX90614 0x5A) ----------------
#define PIN_SDA 21
#define PIN_SCL 22
#define OLED_ADDR 0x3C
#define SHT31_ADDR 0x44

// ---------------- Gas sensors (ADC1 only - ADC2 stops working with WiFi) ----------------
#define PIN_MQ2   34   // via 10k/10k divider
#define PIN_MQ135 35   // via 10k/10k divider
#define MQ_DIVIDER_GAIN 2.0f   // undo the 10k/10k divider

// ---------------- Outputs ----------------
#define PIN_LED_R 25
#define PIN_LED_G 26
#define PIN_LED_B 27
#define PIN_BUZZER 4
#define BUZZER_IDLE_LEVEL HIGH   // low-level-trigger module; set LOW if it hums when idle

// ---------------- Motor driver TB6612FNG (STBY tied to 3.3 V) ----------------
#define PIN_PWMA 32
#define PIN_AIN1 33
#define PIN_AIN2 12
#define PIN_PWMB 16
#define PIN_BIN1 17
#define PIN_BIN2 2
#define MOTOR_MAX_PWM 190   // ~75% of 255: TT motors are rated 3-6 V, battery is 7.4 V
#define CRUISE_SPEED 150
#define TURN_SPEED 60

// ---------------- Navigation sensors ----------------
#define PIN_IR_LEFT  36   // TCRT5000 DO (VP)
#define PIN_IR_RIGHT 39   // TCRT5000 DO (VN)
#define IR_ON_TAPE HIGH   // flip to LOW if your modules read LOW on black tape
#define PIN_US_TRIG 13
#define PIN_US_ECHO 14    // via 4.7k / 6.8k divider (ECHO is 5 V)
#define OBSTACLE_STOP_CM 20.0f

// ---------------- SPI bus: RC522 + SD card ----------------
#define PIN_SCK  18
#define PIN_MISO 19
#define PIN_MOSI 23
#define PIN_RFID_SS 5     // RC522 RST tied to 3.3 V
#define PIN_SD_CS 15

// ---------------- Timing ----------------
#define WARMUP_SECONDS 60          // MQ heater warm-up after power-on
#define CHECKPOINT_SETTLE_MS 3000  // wait after stopping before measuring
#define CHECKPOINT_LEAVE_MS 1500   // drive past a tag before re-arming the RFID reader
#define ADC_SAMPLES 32

// ---------------- Zones (one RFID card per checkpoint) ----------------
// Fill in the UIDs printed by the RFID test sketch.
#define MAX_ZONES 6
struct ZoneInfo { const char *uid; const char *name; };
static const ZoneInfo ZONES[MAX_ZONES] = {
  {"00000001", "Z1 Corridor entrance"},
  {"00000002", "Z2 Near bathroom"},
  {"00000003", "Z3 Room 4 corner"},
  {"00000004", "Z4 Pantry / LPG"},
  {"00000005", "Z5 Electrical panel"},
  {"00000006", "Z6 Spare"},
};
