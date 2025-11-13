#include <TinyUSB_Mouse_and_Keyboard.h>
#include <OneButton.h>
#include <Tlv493d.h>
#include <SimpleKalmanFilter.h>


#include <Adafruit_QMC5883P.h>

Adafruit_QMC5883P mag;

SimpleKalmanFilter xFilter(1, 1, 0.5), yFilter(1, 1, 0.5), zFilter(1, 1, 0.01);

// Setup buttons
OneButton button1(27, true);
OneButton button2(24, true);

float xOffset = 0, yOffset = 0, zOffset = 0;
float xCurrent = 0, yCurrent = 0, zCurrent = 0;

#define CAL_SAMPLES     300
#define FAST_CAL_TIME   100
#define SLOW_CAL_TIME   1000

float sensivity = 0.1;
int outRange = 127;        // Max allowed in HID report
float xyThreshold = 0.05;  // Center threshold
float zThreshold = xyThreshold * 1.5;

bool isOrbit = false;

void setup() {

  button1.attachClick(goHome);
  button1.attachLongPressStop(goHome);

  button2.attachClick(fitToScreen);
  button2.attachLongPressStop(fitToScreen);

  // mouse and keyboard init
  Mouse.begin();
  Keyboard.begin();

  Serial.begin(115200);
  Wire.begin();

  while (!Serial) delay(100);

  if (!mag.begin()) {
    Serial.println("Failed to find QMC5883P chip");
    while (1) delay(100);
  }

  Serial.println("QMC5883P Found!");

  // Set to normal mode
  mag.setMode(QMC5883P_MODE_CONTINUOUS);
  mag.setODR(QMC5883P_ODR_100HZ);
  mag.setOSR(QMC5883P_OSR_4);
  mag.setDSR(QMC5883P_DSR_2);
  mag.setRange(QMC5883P_RANGE_8G);
  mag.setSetResetMode(QMC5883P_SETRESET_ON);

  // crude offset calibration on first boot
  for (int i = 0; i < CAL_SAMPLES; ++i) {

    float gx, gy, gz;
    if (mag.getGaussField(&gx, &gy, &gz)) {
      xOffset += gx;
      yOffset += gy;
      zOffset += gz;
      Serial.print(".");
    }
  }

  xOffset = xOffset / CAL_SAMPLES;
  yOffset = yOffset / CAL_SAMPLES;
  zOffset = zOffset / CAL_SAMPLES;

  Serial.println();
  Serial.println(xOffset);
  Serial.println(yOffset);
  Serial.println(zOffset);
}

void loop() {

  // keep watching the push buttons
  button1.tick();
  button2.tick();

  float gx, gy, gz;
  if (mag.getGaussField(&gx, &gy, &gz)) {
    // update the filters
    xCurrent = xFilter.updateEstimate(gx - xOffset);
    yCurrent = yFilter.updateEstimate(gy - yOffset);
    zCurrent = zFilter.updateEstimate(gz - zOffset);
  }

  static uint32_t calTime = millis();
  static uint32_t lastMovementTime = millis();
  static float lastXCurrent = 0, lastYCurrent = 0;

  // Detect if there's actual movement (not just drift)
  bool hasMovement = (abs(xCurrent - lastXCurrent) > 0.01 || abs(yCurrent - lastYCurrent) > 0.01);
  if (hasMovement) {
    lastMovementTime = millis();
    lastXCurrent = xCurrent;
    lastYCurrent = yCurrent;
  }

  // check the center threshold
  if ((abs(xCurrent) > xyThreshold || abs(yCurrent) > xyThreshold) && (millis() - lastMovementTime < SLOW_CAL_TIME)) {  // Force calibration if no real movement for 2 seconds

    int xMove = outRange * xCurrent * sensivity;
    int yMove = outRange * yCurrent * sensivity;
    xMove = constrain(xMove, -outRange, outRange);
    yMove = constrain(yMove, -outRange, outRange);


    // press shift to orbit in Fusion 360 if the pan threshold is not corssed (zAxis)
    // if (abs(zCurrent) < zThreshold && !isOrbit)
    // {
    //   Keyboard.press(KEY_LEFT_SHIFT);
    //   isOrbit = true;
    // }

    // pan or orbit by holding the middle mouse button and moving propotionaly to the xy axis
    Mouse.press(MOUSE_MIDDLE);
    Mouse.move(yMove, xMove, 0);
    delay(2);
    

    Serial.print(xCurrent);
    Serial.print(",");
    Serial.print(yCurrent);
    Serial.print(",");
    Serial.print(zCurrent);
    Serial.print("  ->  ");
    Serial.print(xMove);
    Serial.print(",");
    Serial.print(yMove);
    Serial.println();

  } else if (millis() - calTime >= FAST_CAL_TIME || (millis() - lastMovementTime >= SLOW_CAL_TIME && (abs(xCurrent) > xyThreshold || abs(yCurrent) > xyThreshold))) {
    calTime = millis();

    // Dynamic calibration with proper exponential moving average
    const float alpha = 0.01;  // Small factor for slow adaptation (1% update rate)
    xOffset = xOffset * (1 - alpha) + gx * alpha;
    yOffset = yOffset * (1 - alpha) + gy * alpha;
    zOffset = zOffset * (1 - alpha) + gz * alpha;

    // Serial.print("Cal: ");
    // Serial.print(xOffset, 4);
    // Serial.print(", ");
    // Serial.print(yOffset, 4);
    // Serial.print(", ");
    // Serial.println(zOffset, 4);
    delay(20);
  } else {
    // release the mouse and keyboard if within the center threshold
    // Mouse.release(MOUSE_MIDDLE);
    // Keyboard.releaseAll();
    // isOrbit = false;
    // delay(10);
    Mouse.release(MOUSE_MIDDLE);
  }
}

// go to home view in Fusion 360 by pressing  (CMD + SHIFT + H) shortcut assigned to the custom Add-in command
void goHome() {
  Keyboard.press(KEY_LEFT_GUI);
  Keyboard.press(KEY_LEFT_SHIFT);
  Keyboard.write('h');

  delay(10);
  Keyboard.releaseAll();
  Serial.println("pressed home");
}

// fit to view by pressing the middle mouse button twice
void fitToScreen() {
  Mouse.press(MOUSE_MIDDLE);
  Mouse.release(MOUSE_MIDDLE);
  Mouse.press(MOUSE_MIDDLE);
  Mouse.release(MOUSE_MIDDLE);

  Serial.println("pressed fit");
}
