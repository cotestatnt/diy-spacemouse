#line 1 "D:\\diy-spacemouse\\useless\\test_vl53l0x\\test_vl53l0x.ino"
#include <Arduino.h>
#include <Adafruit_TinyUSB.h> 
#include <Wire.h>

// #define TIMER_INTERRUPT_DEBUG       1
// #define ISR_SERVO_DEBUG             1
#include "src/NRF52_ISR_Servo/src/NRF52_ISR_Servo.h"
#include "src/VL53L0X/VL53L0X.h"
#include "src/ServoConsole.h"

// Published values for SG90 servos; adjust if needed
#define MIN_MICROS  800
#define MAX_MICROS  2450

#define SERVO1_PIN  7
#define SERVO2_PIN  8

#define SDA_PIN     5	
#define SCL_PIN     4

VL53L0X sensor;

ServoConsole* servoConsole;

#line 40 "D:\\diy-spacemouse\\useless\\test_vl53l0x\\test_vl53l0x.ino"
void setup();
#line 70 "D:\\diy-spacemouse\\useless\\test_vl53l0x\\test_vl53l0x.ino"
void loop();
#line 25 "D:\\diy-spacemouse\\useless\\test_vl53l0x\\test_vl53l0x.ino"
uint16_t getDistance(uint32_t _time = 100) {
  static uint32_t lastTime = 0;
  if (millis() - lastTime < _time) {
    return 0;
  }
  lastTime = millis(); 
  return sensor.readRangeContinuousMillimeters();
}

ISR_servo_t ISR_servo[] = {
  { -1, SERVO1_PIN }, 
  { -1, SERVO2_PIN }
};


void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) delay(10);  // for nrf52840 with native usb

  Serial.println("Goodnight moon!");
  Wire.setPins(SDA_PIN, SCL_PIN);
  Wire.begin();

  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.println("Failed to detect and initialize sensor!");
  }
  sensor.startContinuous();

  for (int index = 0; index < sizeof(ISR_servo) / sizeof(ISR_servo[0]); index++) {
    ISR_servo[index].servoIndex = NRF52_ISR_Servos.setupServo(ISR_servo[index].servoPin, MIN_MICROS, MAX_MICROS);
    if (ISR_servo[index].servoIndex < 0) {
      Serial.print("Failed to attach servo on pin ");
    } else {
      Serial.print("Servo attached on pin ");
    }
    Serial.println(ISR_servo[index].servoPin);
  }

  // Inizializza la console per i servo
  servoConsole = new ServoConsole(ISR_servo, sizeof(ISR_servo) / sizeof(ISR_servo[0]));
  servoConsole->begin();
}

void loop() {
  // Processa comandi da console seriale
  servoConsole->handleSerialInput();

  uint16_t distance = getDistance();
  if (distance && distance <= 300) {
    uint16_t position = map(distance, 0, 300, 0, 180);
    Serial.println(distance);
    NRF52_ISR_Servos.setPosition(1, position);
  } 
}

