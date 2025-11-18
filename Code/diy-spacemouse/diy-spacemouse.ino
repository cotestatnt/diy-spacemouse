/*
 This sketch initializes the IMU filter and outputs Euler angles. Extended to optionally act as a USB HID Mouse
 (X/Y only) controlled via Serial commands to avoid losing pointer control.

 Serial commands:
   - "hid on"  : enable HID mouse output (X/Y only)
   - "hid off" : disable HID mouse output
   - "status"  : print current status and settings
   - "help"    : print this help
   - "reset"   : reboot the MCU via software
*/

/* NOTE: The accelerometer MUST be calibrated for the fusion to work. Adjust the 
   AX, AY, AND AZ offsets until the sensor reads (0,0,0) at rest. 
*/

#include "src/filter/basicMPU6050.h"       // Library for IMU sensor. See this link: https://github.com/RCmags/basicMPU6050
#include "src/filter/imuFilter.h"          // Library for IMU sensor fusion. 
#include <math.h>
// TinyUSB HID (composite with CDC) for mouse control
#include <TinyUSB_Mouse_and_Keyboard.h>
#define USE_TINYUSB_MOUSE 1


// ===== Forward declarations (prototypes) =====

void sendHIDreport();
void printStatus();
void printHelp();
void handleSerial();
void rebootMCU();

// Gyro settings:
#define         LP_FILTER   3           // Low pass filter.                    Value from 0 to 6
#define         GYRO_SENS   2           // Gyro sensitivity.                   Value from 0 to 3
#define         ACCEL_SENS  2           // Accelerometer sensitivity.          Value from 0 to 3
#define         ADDRESS_A0  LOW         // I2C address from state of A0 pin.   A0 -> GND : ADDRESS_A0 = LOW
                                        //                                     A0 -> 5v  : ADDRESS_A0 = HIGH
// Accelerometer offset:
constexpr int   AX_OFFSET = 0;          // Use these values to calibrate the accelerometer. The sensor should output 1.0g if held level. 
constexpr int   AY_OFFSET = 0;          // These values are unlikely to be zero.
constexpr int   AZ_OFFSET = 0;

//-- Set the template parameters:
basicMPU6050<LP_FILTER,  GYRO_SENS,  ACCEL_SENS, ADDRESS_A0,
             AX_OFFSET,  AY_OFFSET,  AZ_OFFSET
            >imu;
   
// =========== Settings ===========
imuFilter fusion;

// Serial print change-detection (must be declared before loop)
static const float kPrintEpsDeg = 0.02f;  // print only if any axis changes by >= this (deg)
static bool firstPrint = true;
static float lastPitch = 0.0f, lastYaw = 0.0f, lastRoll = 0.0f;

/* Fusion gain, value between 0 and 1 - Determines orientation correction with respect to gravity vector. 
If set to 1 the gyroscope is dissabled. If set to 0 the accelerometer is dissabled (equivant to gyro-only) */
#define GAIN          0.5     

/* Standard deviation of acceleration. Accelerations relative to (0,0,1)g outside of this band are suppresed.
Accelerations within this band are used to update the orientation. [Measured in g-force] */                          
#define SD_ACCEL      0.2     

/* Enable sensor fusion. Setting to "true" enables gravity correction */
#define FUSION        true    

void setup() {
  Mouse.begin();
  Keyboard.begin();

  delay(2000);  // Wait for serial monitor
  Serial.begin(115200);
  while (!Serial);
  
  // Calibrate imu
  imu.setup();
  imu.setBias();

  #if FUSION
    // Set quaternion with gravity vector
    fusion.setup( imu.ax(), imu.ay(), imu.az() );     
  #else 
    // Just use gyro
    fusion.setup();                                   
  #endif
  
  printHelp();
}

void loop() {
  // Update filter:
  #if FUSION
    /* NOTE: GAIN and SD_ACCEL are optional parameters */
    fusion.update( imu.gx(), imu.gy(), imu.gz(), imu.ax(), imu.ay(), imu.az(), GAIN, SD_ACCEL );
  #else
    // Only use gyroscope
    fusion.update( imu.gx(), imu.gy(), imu.gz() );
  #endif

  // Read current angles
  float pitch = fusion.pitch();
  float yaw   = fusion.yaw();
  float roll  = fusion.roll();

  // Display only when they change beyond threshold
  bool changed = firstPrint
              || fabs(pitch - lastPitch) >= kPrintEpsDeg
              || fabs(yaw   - lastYaw)   >= kPrintEpsDeg
              || fabs(roll  - lastRoll)  >= kPrintEpsDeg;
  if (changed) {
    Serial.print(pitch, 3);
    Serial.print(" ");
    Serial.print(yaw, 3);
    Serial.print(" ");
    Serial.print(roll, 3);
    Serial.println();

    lastPitch = pitch;
    lastYaw   = yaw;
    lastRoll  = roll;
    firstPrint = false;
  }

  // Handle serial commands (non-blocking)
  handleSerial();

  // Optionally send HID mouse movement based on IMU angles
  sendHIDreport();
}
static bool hidEnabled = false;           // Toggled via Serial
static uint32_t lastHidMs = 0;            // Rate limit

// Tuning parameters
static const float kSensitivity = 3.0f;   // pixels per degree (adjust as needed)
static const float kDeadzoneDeg = 1.0f;   // ignore small tilt
static const uint32_t kHidIntervalMs = 10; // 100 Hz max

// Response curve & anti-drift
static const float kGamma = 1.4f;         // >1 accelerates with angle magnitude
static const float kCenterLockDeg = 0.6f; // if inside, freeze residuals to prevent drift
static float fracX = 0.0f, fracY = 0.0f;  // fractional accumulators for sub-pixel motion


static inline int8_t clipInt8(int val) {
  if (val > 127) return 127;
  if (val < -127) return -127;
  return (int8_t)val;
}

static inline float applyDeadzone(float a_deg, float dz_deg) {
  float sa = fabs(a_deg);
  if (sa <= dz_deg) return 0.0f;
  float out = sa - dz_deg; // subtractive deadzone
  return (a_deg < 0 ? -out : out);
}

void sendHIDreport() {
  if (!hidEnabled) return;
  
  uint32_t now = millis();
  if (now - lastHidMs < kHidIntervalMs) return; // rate limit
  lastHidMs = now;

  // Map roll->X, pitch->Y (invert Y so forward tilt moves cursor up)
  float pitch = fusion.pitch();
  float roll  = fusion.roll();

  // Center lock to kill drift when essentially centered
  if (fabs(pitch) < kCenterLockDeg && fabs(roll) < kCenterLockDeg) {
    fracX = 0.0f;
    fracY = 0.0f;
    return;
  }

  // Apply subtractive deadzone and response curve (power gamma)
  float ex = applyDeadzone(roll,  kDeadzoneDeg);
  float ey = applyDeadzone(pitch, kDeadzoneDeg);

  float dx_f = (ex == 0.0f) ? 0.0f : (kSensitivity * (ex < 0 ? -powf(-ex, kGamma) : powf(ex, kGamma)));
  float dy_f = (ey == 0.0f) ? 0.0f : (-kSensitivity * (ey < 0 ? -powf(-ey, kGamma) : powf(ey, kGamma)));

  // Accumulate fractional motion to avoid quantization flat spots
  float accX = dx_f + fracX;
  float accY = dy_f + fracY;
  int dx = (accX >= 0 ? (int)floorf(accX) : (int)ceilf(accX));
  int dy = (accY >= 0 ? (int)floorf(accY) : (int)ceilf(accY));
  fracX = accX - (float)dx;
  fracY = accY - (float)dy;

  // Convert to int8 and clip
  int8_t dx8 = clipInt8(dx);
  int8_t dy8 = clipInt8(dy);
  if (dx8 == 0 && dy8 == 0) return;

  // High-level TinyUSB Mouse API
  Mouse.move(dx8, dy8, 0);  
  // Serial.print(F("HID move: "));
  // Serial.print(dx);   
  // Serial.print(" ");
  // Serial.println(dy);   
}

// ================= Serial Command Interface =================

String cmdBuf;

void printStatus() {
  Serial.print(F("HID: "));
  Serial.println(hidEnabled ? F("ON") : F("OFF"));
  Serial.print(F("Sensitivity: "));
  Serial.println(kSensitivity, 2);
  Serial.print(F("Deadzone (deg): "));
  Serial.println(kDeadzoneDeg, 2);
  Serial.print(F("Print eps (deg): "));
  Serial.println(kPrintEpsDeg, 2);
}

void printHelp() {
  Serial.println();
  Serial.println(F("Commands:"));
  Serial.println(F("  hid on     - enable HID mouse output (X/Y only)"));
  Serial.println(F("  hid off    - disable HID mouse output"));
  Serial.println(F("  status     - show current status"));
  Serial.println(F("  help       - show this help"));
  Serial.println(F("  reset      - reboot MCU"));
  Serial.println();
}

void handleSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      // Process command
      cmdBuf.trim();
      if (cmdBuf.length()) {
        if (cmdBuf.equalsIgnoreCase("hid on")) {
          hidEnabled = true;
          Serial.println(F("HID enabled"));
          // send a neutral report to wake host HID parser
          Mouse.move(0, 0);          
        } else if (cmdBuf.equalsIgnoreCase("hid off")) {
          hidEnabled = false;
          Serial.println(F("HID disabled"));
        } else if (cmdBuf.equalsIgnoreCase("status")) {
          printStatus();
        } else if (cmdBuf.equalsIgnoreCase("help")) {
          printHelp();
        } else if (cmdBuf.equalsIgnoreCase("reset")) {
          Serial.println(F("Rebooting..."));
          Serial.flush();
          delay(50);
          rebootMCU();
        } else {
          Serial.print(F("Unknown command: "));
          Serial.println(cmdBuf);
          printHelp();
        }
      }
      cmdBuf = "";
    } else {
      // accumulate (limit size)
      if (cmdBuf.length() < 64) cmdBuf += c;
    }
  }
}

// ================= Software Reboot =================

void rebootMCU() {
  // Attempt platform-specific reboot methods. Many TinyUSB boards are ARM Cortex-M (NVIC_SystemReset).
  NVIC_SystemReset();
}