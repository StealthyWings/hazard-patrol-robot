#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_MLX90614.h>
#include <MFRC522.h>

// I2C: SHT31 + MLX90614 + OLED share this bus
#define SDA_PIN 21
#define SCL_PIN 22
// HC-SR04 (ECHO through divider)
#define TRIG_PIN 16
#define ECHO_PIN 17
// IR digital outputs (GPIO34/35 are input-only)
#define IR_LEFT_PIN 34
#define IR_RIGHT_PIN 35
// MQ analog outputs: each through its own voltage divider
#define MQ2_PIN 36
#define MQ135_PIN 39
// TB6612FNG
#define AIN1 25
#define AIN2 26
#define PWMA 27
#define BIN1 14
#define BIN2 13
#define PWMB 33
#define STBY 32
// RC522 SPI (VSPI)
#define RFID_SS 5
#define RFID_RST 4
#define RFID_SCK 18
#define RFID_MISO 19
#define RFID_MOSI 23
// RGB LED and active buzzer
#define RGB_R 2
#define RGB_G 15
#define RGB_B 0
#define BUZZER_PIN 12

#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_ADDR 0x3C
#define OLED_RESET -1

// CONFIGURATION: calibrate these raw MQ warning thresholds experimentally.
int MQ2_WARNING_RAW = 1800;
int MQ135_WARNING_RAW = 1800;
const float OBSTACLE_CM = 25.0;
const float SURFACE_TEMP_WARNING_C = 60.0;
const float AIR_TEMP_WARNING_C = 50.0;
const int PATROL_SPEED = 150;
const int TURN_SPEED = 145;
const int REVERSE_SPEED = 135;
const int IR_ACTIVE_LEVEL = LOW;       // change to HIGH if your IR modules invert
const bool RGB_COMMON_ANODE = false;   // true for common-anode RGB
const bool ENABLE_AUTONOMOUS_MOTION = true; // set false for stationary sensor testing

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);
Adafruit_SHT31 sht31;
Adafruit_MLX90614 mlx;
MFRC522 rfid(RFID_SS, RFID_RST);

bool oledOK=false, shtOK=false, mlxOK=false;
float airC=NAN, humidity=NAN, objectC=NAN, distanceCM=-1;
int mq2=0, mq135=0;
bool irLeft=false, irRight=false, gasAlarm=false, tempAlarm=false, obstacle=false;
unsigned long sensorTimer=0, displayTimer=0, serialTimer=0, rfidTimer=0, beepTimer=0;

void rgb(bool r, bool g, bool b) {
  int rr=r?HIGH:LOW, gg=g?HIGH:LOW, bb=b?HIGH:LOW;
  if (RGB_COMMON_ANODE) { rr=!rr; gg=!gg; bb=!bb; }
  digitalWrite(RGB_R,rr); digitalWrite(RGB_G,gg); digitalWrite(RGB_B,bb);
}
void beep(unsigned int ms=100) {
  digitalWrite(BUZZER_PIN,HIGH); delay(ms); digitalWrite(BUZZER_PIN,LOW);
}

void oneMotor(int in1,int in2,int pwm,int speed) {
  speed=constrain(speed,-255,255);
  if(speed>0) { digitalWrite(in1,HIGH); digitalWrite(in2,LOW); analogWrite(pwm,speed); }
  else if(speed<0) { digitalWrite(in1,LOW); digitalWrite(in2,HIGH); analogWrite(pwm,-speed); }
  else { digitalWrite(in1,LOW); digitalWrite(in2,LOW); analogWrite(pwm,0); }
}
void motors(int left,int right) {
  digitalWrite(STBY,HIGH);
  oneMotor(AIN1,AIN2,PWMA,left);
  oneMotor(BIN1,BIN2,PWMB,right);
}
void stopMotors() {
  oneMotor(AIN1,AIN2,PWMA,0); oneMotor(BIN1,BIN2,PWMB,0);
  digitalWrite(STBY,LOW);
}
void reverseShort() { motors(-REVERSE_SPEED,-REVERSE_SPEED); delay(350); stopMotors(); }
void turnLeftShort() { motors(-TURN_SPEED,TURN_SPEED); delay(420); stopMotors(); }
void turnRightShort() { motors(TURN_SPEED,-TURN_SPEED); delay(420); stopMotors(); }

float readDistance() {
  digitalWrite(TRIG_PIN,LOW); delayMicroseconds(3);
  digitalWrite(TRIG_PIN,HIGH); delayMicroseconds(10); digitalWrite(TRIG_PIN,LOW);
  unsigned long us=pulseIn(ECHO_PIN,HIGH,30000UL);
  if(us==0) return -1;
  return us*0.0343f/2.0f;
}
int readADC(int pin) {
  long sum=0;
  for(int i=0;i<10;i++) { sum+=analogRead(pin); delay(3); }
  return sum/10;
}
void readSensors() {
  distanceCM=readDistance();
  irLeft=(digitalRead(IR_LEFT_PIN)==IR_ACTIVE_LEVEL);
  irRight=(digitalRead(IR_RIGHT_PIN)==IR_ACTIVE_LEVEL);
  mq2=readADC(MQ2_PIN); mq135=readADC(MQ135_PIN);
  if(shtOK) { airC=sht31.readTemperature(); humidity=sht31.readHumidity(); }
  if(mlxOK) objectC=mlx.readObjectTempC();
  gasAlarm=(mq2>=MQ2_WARNING_RAW || mq135>=MQ135_WARNING_RAW);
  tempAlarm=(!isnan(objectC) && objectC>=SURFACE_TEMP_WARNING_C) ||
            (!isnan(airC) && airC>=AIR_TEMP_WARNING_C);
  obstacle=(distanceCM>0 && distanceCM<OBSTACLE_CM);
}

String cardUID() {
  String s="";
  for(byte i=0;i<rfid.uid.size;i++) {
    if(rfid.uid.uidByte[i]<0x10) s+="0";
    s+=String(rfid.uid.uidByte[i],HEX);
    if(i<rfid.uid.size-1) s+=":";
  }
  s.toUpperCase(); return s;
}
void checkRFID() {
  if(millis()-rfidTimer<250) return;
  rfidTimer=millis();
  if(!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) return;
  Serial.print("RFID UID: "); Serial.println(cardUID());
  beep(90); rfid.PICC_HaltA(); rfid.PCD_StopCrypto1();
}

void updateAlerts() {
  if(gasAlarm || tempAlarm) {
    rgb(true,false,false);
    if(millis()-beepTimer>1200) { beep(150); beepTimer=millis(); }
  } else if(obstacle || irLeft || irRight) {
    rgb(true,true,false);
  } else {
    rgb(false,true,false);
  }
}

void navigate() {
  if(!ENABLE_AUTONOMOUS_MOTION || gasAlarm || tempAlarm) { stopMotors(); return; }
  if(irLeft && irRight) { stopMotors(); reverseShort(); turnRightShort(); }
  else if(irLeft) { stopMotors(); reverseShort(); turnRightShort(); }
  else if(irRight) { stopMotors(); reverseShort(); turnLeftShort(); }
  else if(obstacle) { stopMotors(); reverseShort(); turnRightShort(); }
  else motors(PATROL_SPEED,PATROL_SPEED);
}

void showOLED() {
  if(!oledOK) return;
  display.clearDisplay(); display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1); display.setCursor(0,0);
  display.println("SERBOT - PATROL");
  display.println("---------------------");
  display.print("Air: "); if(isnan(airC)) display.print("--"); else display.print(airC,1);
  display.print("C RH: "); if(isnan(humidity)) display.println("--"); else { display.print(humidity,0); display.println("%"); }
  display.print("Surface: "); if(isnan(objectC)) display.println("--"); else { display.print(objectC,1); display.println(" C"); }
  display.print("Distance: "); if(distanceCM<0) display.println("No echo"); else { display.print(distanceCM,1); display.println(" cm"); }
  display.print("MQ2:"); display.print(mq2); display.print(" MQ135:"); display.println(mq135);
  if(gasAlarm || tempAlarm) display.println("STATUS: HAZARD!");
  else if(obstacle || irLeft || irRight) display.println("STATUS: OBSTACLE");
  else display.println("STATUS: NORMAL");
  display.display();
}
void printStatus() {
  Serial.println("\n------ SERBOT READINGS ------");
  Serial.printf("Air temperature: %.1f C\n",airC);
  Serial.printf("Humidity: %.1f %%\n",humidity);
  Serial.printf("Object temperature: %.1f C\n",objectC);
  Serial.printf("Distance: %.1f cm\n",distanceCM);
  Serial.printf("IR left/right: %d / %d\n",irLeft,irRight);
  Serial.printf("MQ-2 raw ADC: %d\n",mq2);
  Serial.printf("MQ-135 raw ADC: %d\n",mq135);
  Serial.printf("Gas alarm: %s | Temperature alarm: %s\n",gasAlarm?"YES":"NO",tempAlarm?"YES":"NO");
  Serial.println("MQ values are raw ADC counts, not ppm.");
}

void setup() {
  Serial.begin(115200); delay(500);
  pinMode(AIN1,OUTPUT); pinMode(AIN2,OUTPUT); pinMode(PWMA,OUTPUT);
  pinMode(BIN1,OUTPUT); pinMode(BIN2,OUTPUT); pinMode(PWMB,OUTPUT);
  pinMode(STBY,OUTPUT); stopMotors();

  pinMode(TRIG_PIN,OUTPUT); pinMode(ECHO_PIN,INPUT); digitalWrite(TRIG_PIN,LOW);
  pinMode(IR_LEFT_PIN,INPUT); pinMode(IR_RIGHT_PIN,INPUT);
  pinMode(RGB_R,OUTPUT); pinMode(RGB_G,OUTPUT); pinMode(RGB_B,OUTPUT);
  pinMode(BUZZER_PIN,OUTPUT); rgb(false,false,false); digitalWrite(BUZZER_PIN,LOW);

  analogReadResolution(12);
  analogSetPinAttenuation(MQ2_PIN,ADC_11db);
  analogSetPinAttenuation(MQ135_PIN,ADC_11db);

  Wire.begin(SDA_PIN,SCL_PIN);
  oledOK=display.begin(SSD1306_SWITCHCAPVCC,OLED_ADDR);
  if(oledOK) { display.clearDisplay(); display.setTextSize(1); display.setTextColor(SSD1306_WHITE); display.setCursor(0,0); display.println("SERBOT starting..."); display.display(); }
  else Serial.println("OLED not found at 0x3C. Check address/wiring.");

  shtOK=sht31.begin(0x44);
  if(!shtOK) Serial.println("SHT31 not found at 0x44. Check wiring/address (possibly 0x45).");
  mlxOK=mlx.begin();
  if(!mlxOK) Serial.println("MLX90614 not found. Check shared I2C wiring.");

  SPI.begin(RFID_SCK,RFID_MISO,RFID_MOSI,RFID_SS);
  rfid.PCD_Init(); delay(50);
  byte ver=rfid.PCD_ReadRegister(rfid.VersionReg);
  if(ver==0x00 || ver==0xFF) Serial.println("RC522 not detected. Check 3.3V and SPI wiring.");
  else Serial.println("RC522 initialized.");

  Serial.println("\nSERBOT ready. Serial Monitor: 115200 baud.");
  Serial.println("Allow MQ sensors to warm up; calibrate thresholds before relying on alarms.");
  rgb(false,true,false); beep(100);
}

void loop() {
  checkRFID();
  if(millis()-sensorTimer>=1000) {
    sensorTimer=millis(); readSensors(); updateAlerts(); navigate();
  }
  if(millis()-displayTimer>=500) { displayTimer=millis(); showOLED(); }
  if(millis()-serialTimer>=1000) { serialTimer=millis(); printStatus(); }
}
