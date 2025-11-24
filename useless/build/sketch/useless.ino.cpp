#include <Arduino.h>
#line 1 "C:\\Cloud\\diy-spacemouse\\useless\\useless.ino"
// Include the ESP32 Arduino Servo Library instead of the original Arduino Servo Library
#include <ESP32Servo.h> 

Servo myservo;  // create servo object to control a servo
// Possible PWM GPIO pins on the ESP32-S3: 0(used by on-board button),1-21,35-45,47,48(used by on-board LED)
int servoPin = 5;       // GPIO pin used to connect the servo control (digital out)
int potPin = 4;         // GPIO pin used to connect the potentiometer (analog in)
int ADC_Max = 4096;     // This is the default ADC max value on the ESP32 (12 bit ADC width);
                        // this width can be set (in low-level oode) from 9-12 bits, for a
                        // a range of max values of 512-4096
  
int val, oldVal;        // variable to read the value from the analog pin


#line 15 "C:\\Cloud\\diy-spacemouse\\useless\\useless.ino"
void setup();
#line 26 "C:\\Cloud\\diy-spacemouse\\useless\\useless.ino"
void loop();
#line 15 "C:\\Cloud\\diy-spacemouse\\useless\\useless.ino"
void setup()
{
    // Allow allocation of all timers
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    myservo.setPeriodHertz(50);// Standard 50hz servo
    myservo.attach(servoPin, 400u, 2800u);   
}

void loop() {
  val = analogRead(potPin);               // read the value of the potentiometer (value between 0 and 1023)
  val = map(val, 0, ADC_Max, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
  if (abs(val - oldVal) > 1) {
    myservo.write(val);                   // set the servo position according to the scaled value
    oldVal = val;
  }
}


