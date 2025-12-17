#line 1 "D:\\diy-spacemouse\\useless\\test_vl53l0x\\test_vl53l0x.ino"
#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <Wire.h>
#include <FreeRTOS.h>
#include "src/NRF52_ISR_Servo/src/NRF52_ISR_Servo.h"
#include "src/VL53L0X/VL53L0X.h"

#include "src/ServoConsole.h"



#define MIN_MICROS 600
#define MAX_MICROS 2600

// --- Servo Positions (adjust if needed) ---
#define LID_CLOSED 0
#define LID_PEEK 45
#define LID_OPEN 90

#define ARM_IN 0
#define ARM_OUT 90
#define ARM_WAVE_LEFT 70
#define ARM_WAVE_RIGHT 110
#define ARM_PUSH_SWITCH 150
#define ARM_NEAR_SWITCH (ARM_PUSH_SWITCH - 20)

// --- Servo Speeds (delay between steps in ms) ---
#define SPEED_FASTEST 0
#define SPEED_VERY_FAST 4
#define SPEED_FAST 8
#define SPEED_MEDIUM 15
#define SPEED_SLOW 25

// --- Hardware Definitions ---
#define SERVO1_PIN 7 // Servo for the box lid
#define SERVO2_PIN 8 // Servo for the arm
#define SDA_PIN 5
#define SCL_PIN 4

// --- Global Objects and State ---
VL53L0X sensor;
ServoConsole* servoConsole;
bool scenario_executed = false;

// Struct to manage servo state and goals for tasks
ServoGoal servo_goals[2];

// ISR_servo_t array to hold servo indices and pins
ISR_servo_t ISR_servo[] = {
    { -1, SERVO1_PIN }, // index 0 for lid servo
    { -1, SERVO2_PIN }  // index 1 for arm servo
};


// Non-blocking function to set a servo's goal
#line 56 "D:\\diy-spacemouse\\useless\\test_vl53l0x\\test_vl53l0x.ino"
void setServoGoal(int servo_idx, int target_pos, int speed);
#line 63 "D:\\diy-spacemouse\\useless\\test_vl53l0x\\test_vl53l0x.ino"
void setServoInstant(int servo_idx, int pos);
#line 70 "D:\\diy-spacemouse\\useless\\test_vl53l0x\\test_vl53l0x.ino"
bool isServoMoving(int servo_idx);
#line 74 "D:\\diy-spacemouse\\useless\\test_vl53l0x\\test_vl53l0x.ino"
void waitForServo(int servo_idx);
#line 80 "D:\\diy-spacemouse\\useless\\test_vl53l0x\\test_vl53l0x.ino"
void waitForAllServos();
#line 87 "D:\\diy-spacemouse\\useless\\test_vl53l0x\\test_vl53l0x.ino"
void scenario_peek_and_hide();
#line 102 "D:\\diy-spacemouse\\useless\\test_vl53l0x\\test_vl53l0x.ino"
void scenario_hesitant();
#line 124 "D:\\diy-spacemouse\\useless\\test_vl53l0x\\test_vl53l0x.ino"
void scenario_distracted_by_user();
#line 155 "D:\\diy-spacemouse\\useless\\test_vl53l0x\\test_vl53l0x.ino"
void scenario_annoyed_poke();
#line 171 "D:\\diy-spacemouse\\useless\\test_vl53l0x\\test_vl53l0x.ino"
void scenario_shy();
#line 188 "D:\\diy-spacemouse\\useless\\test_vl53l0x\\test_vl53l0x.ino"
void scenario_drummer();
#line 206 "D:\\diy-spacemouse\\useless\\test_vl53l0x\\test_vl53l0x.ino"
void scenario_fake_out();
#line 218 "D:\\diy-spacemouse\\useless\\test_vl53l0x\\test_vl53l0x.ino"
void scenario_slow_wave();
#line 233 "D:\\diy-spacemouse\\useless\\test_vl53l0x\\test_vl53l0x.ino"
void scenario_do_nothing();
#line 238 "D:\\diy-spacemouse\\useless\\test_vl53l0x\\test_vl53l0x.ino"
void finalSequence();
#line 254 "D:\\diy-spacemouse\\useless\\test_vl53l0x\\test_vl53l0x.ino"
void scenario_nine_special();
#line 276 "D:\\diy-spacemouse\\useless\\test_vl53l0x\\test_vl53l0x.ino"
void execute_scenario(char command);
#line 297 "D:\\diy-spacemouse\\useless\\test_vl53l0x\\test_vl53l0x.ino"
void printMenu();
#line 316 "D:\\diy-spacemouse\\useless\\test_vl53l0x\\test_vl53l0x.ino"
void parseScenarios(const String& sequence);
#line 345 "D:\\diy-spacemouse\\useless\\test_vl53l0x\\test_vl53l0x.ino"
void servo_task(void *pvParameters);
#line 363 "D:\\diy-spacemouse\\useless\\test_vl53l0x\\test_vl53l0x.ino"
void setup();
#line 405 "D:\\diy-spacemouse\\useless\\test_vl53l0x\\test_vl53l0x.ino"
void loop();
#line 56 "D:\\diy-spacemouse\\useless\\test_vl53l0x\\test_vl53l0x.ino"
void setServoGoal(int servo_idx, int target_pos, int speed) {
    // Ensure target is within SG90 servo limits (0-180)
    servo_goals[servo_idx].target_pos = constrain(target_pos, 0, 180);
    servo_goals[servo_idx].speed_delay = speed;
}

// Instantly move servo (for initialization)
void setServoInstant(int servo_idx, int pos) {
    servo_goals[servo_idx].target_pos = pos;
    servo_goals[servo_idx].current_pos = pos;
    NRF52_ISR_Servos.setPosition(ISR_servo[servo_idx].servoIndex, pos);
}

// --- Synchronization Functions ---
bool isServoMoving(int servo_idx) {
    return servo_goals[servo_idx].current_pos != servo_goals[servo_idx].target_pos;
}

void waitForServo(int servo_idx) {
    while (isServoMoving(servo_idx)) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void waitForAllServos() {
    while (isServoMoving(0) || isServoMoving(1)) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// --- Scenarios ---
void scenario_peek_and_hide() {
    Serial.println("Scenario: Peek and Hide");
    setServoGoal(0, LID_PEEK, SPEED_MEDIUM);
    waitForServo(0);
    vTaskDelay(pdMS_TO_TICKS(700));
    setServoGoal(0, LID_PEEK + 20, SPEED_SLOW);
    waitForServo(0);
    vTaskDelay(pdMS_TO_TICKS(1000));
    setServoGoal(0, LID_CLOSED, SPEED_FAST);
    waitForServo(0);
    vTaskDelay(pdMS_TO_TICKS(300));
    Serial.println("Hiding...");
    vTaskDelay(pdMS_TO_TICKS(1000));
}

void scenario_hesitant() {
    Serial.println("Scenario: Hesitant");
    setServoGoal(0, LID_OPEN, SPEED_MEDIUM);
    vTaskDelay(pdMS_TO_TICKS(150));
    setServoGoal(1, ARM_OUT - 20, SPEED_MEDIUM);
    waitForAllServos();
    vTaskDelay(pdMS_TO_TICKS(400));
    setServoGoal(1, ARM_IN + 10, SPEED_FAST);
    waitForServo(1);
    vTaskDelay(pdMS_TO_TICKS(500));
    setServoGoal(1, ARM_OUT, SPEED_MEDIUM);
    waitForServo(1);
    vTaskDelay(pdMS_TO_TICKS(300));
    Serial.println("Looking around...");
    setServoGoal(1, ARM_WAVE_LEFT, SPEED_FAST);
    waitForServo(1);
    setServoGoal(1, ARM_WAVE_RIGHT, SPEED_FAST);
    waitForServo(1);
    setServoGoal(1, ARM_OUT, SPEED_FAST);
    waitForServo(1);
}

void scenario_distracted_by_user() {
    Serial.println("Scenario: Distracted by User. Come play with me!");
    setServoGoal(0, LID_OPEN, SPEED_MEDIUM);
    setServoGoal(1, ARM_OUT, SPEED_MEDIUM);
    waitForAllServos();
    TickType_t startTime = xTaskGetTickCount();
    TickType_t interactionDuration = pdMS_TO_TICKS(8000);
    while ((xTaskGetTickCount() - startTime) < interactionDuration) {
        uint16_t distance = sensor.readRangeContinuousMillimeters();
        if (sensor.timeoutOccurred()) {
            Serial.println("Sensor timeout. Holding position.");
            setServoGoal(1, ARM_OUT, SPEED_MEDIUM);
        } else {
            Serial.print("Dist: "); Serial.println(distance);
            if (distance < 60) {
                Serial.println("Hey, too close!");
                setServoGoal(1, ARM_IN + 10, SPEED_VERY_FAST);
                vTaskDelay(pdMS_TO_TICKS(400));
            } else if (distance < 300) {
                int follow_pos = map(distance, 60, 300, ARM_WAVE_LEFT, ARM_WAVE_RIGHT + 20);
                setServoGoal(1, follow_pos, SPEED_FAST);
            } else {
                setServoGoal(1, ARM_OUT, SPEED_MEDIUM);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(60));
    }
    Serial.println("That was fun! Ok, back to work.");
    vTaskDelay(pdMS_TO_TICKS(500));
}

void scenario_annoyed_poke() {
    Serial.println("Scenario: Annoyed Poke");
    setServoGoal(0, LID_OPEN, SPEED_FAST);
    setServoGoal(1, ARM_NEAR_SWITCH, SPEED_SLOW);
    waitForAllServos();
    vTaskDelay(pdMS_TO_TICKS(1000));
    Serial.println("*poke*");
    setServoGoal(1, ARM_NEAR_SWITCH + 5, SPEED_FAST); waitForServo(1);
    setServoGoal(1, ARM_NEAR_SWITCH, SPEED_FAST); waitForServo(1);
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.println("Grrr...");
    setServoGoal(1, ARM_IN + 20, SPEED_FASTEST);
    waitForServo(1);
    vTaskDelay(pdMS_TO_TICKS(800));
}

void scenario_shy() {
    Serial.println("Scenario: Shy");
    setServoGoal(0, LID_PEEK, SPEED_SLOW);
    waitForServo(0);
    vTaskDelay(pdMS_TO_TICKS(1500));
    uint16_t distance = sensor.readRangeContinuousMillimeters();
    if (!sensor.timeoutOccurred() && distance < 150) {
        Serial.println("Eek! Too close!");
        setServoGoal(0, LID_CLOSED, SPEED_VERY_FAST);
    } else {
        Serial.println("...closing slowly.");
        setServoGoal(0, LID_CLOSED, SPEED_SLOW);
    }
    waitForServo(0);
    vTaskDelay(pdMS_TO_TICKS(500));
}

void scenario_drummer() {
    Serial.println("Scenario: Drummer");
    setServoGoal(0, LID_OPEN, SPEED_MEDIUM);
    setServoGoal(1, ARM_OUT - 15, SPEED_MEDIUM);
    waitForAllServos();
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.println("Ba-dum-tss!");
    for (int i=0; i < 6; i++) {
        setServoGoal(1, ARM_OUT, SPEED_VERY_FAST);
        waitForServo(1);
        vTaskDelay(pdMS_TO_TICKS(50));
        setServoGoal(1, ARM_OUT - 25, SPEED_VERY_FAST);
        waitForServo(1);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    vTaskDelay(pdMS_TO_TICKS(500));
}

void scenario_fake_out() {
    Serial.println("Scenario: The Fake Out");
    setServoGoal(0, LID_OPEN, SPEED_FAST);
    setServoGoal(1, ARM_NEAR_SWITCH, SPEED_VERY_FAST);
    waitForAllServos();
    vTaskDelay(pdMS_TO_TICKS(1500));
    Serial.println("Changed my mind.");
    setServoGoal(1, ARM_IN, SPEED_SLOW);
    setServoGoal(0, LID_CLOSED, SPEED_SLOW);
    waitForAllServos();
}

void scenario_slow_wave() {
    Serial.println("Scenario: The Slow Wave");
    setServoGoal(0, LID_OPEN, SPEED_SLOW);
    setServoGoal(1, ARM_OUT, SPEED_SLOW);
    waitForAllServos();
    vTaskDelay(pdMS_TO_TICKS(500));
    setServoGoal(1, ARM_WAVE_LEFT - 10, SPEED_SLOW);
    waitForServo(1);
    setServoGoal(1, ARM_WAVE_RIGHT + 10, SPEED_SLOW);
    waitForServo(1);
    setServoGoal(1, ARM_OUT, SPEED_SLOW);
    waitForServo(1);
    vTaskDelay(pdMS_TO_TICKS(500));
}

void scenario_do_nothing() {
    Serial.println("Scenario: Do Nothing. (Taking a break)");
    vTaskDelay(pdMS_TO_TICKS(1500));
}

void finalSequence() {
    Serial.println("Final sequence: pressing switch.");
    setServoGoal(0, LID_OPEN, SPEED_MEDIUM);
    vTaskDelay(pdMS_TO_TICKS(150));
    setServoGoal(1, ARM_OUT, SPEED_FAST);
    waitForAllServos();
    vTaskDelay(pdMS_TO_TICKS(400));
    setServoGoal(1, ARM_PUSH_SWITCH, SPEED_FAST);
    waitForServo(1);
    vTaskDelay(pdMS_TO_TICKS(500));
    setServoGoal(1, ARM_IN, SPEED_MEDIUM);
    setServoGoal(0, LID_CLOSED, SPEED_MEDIUM);
    waitForAllServos();
    Serial.println("Switch should be pressed. If not, power off failed.");
}

void scenario_nine_special() {
    Serial.println("Scenario 9: Saluto speciale!");
    setServoGoal(0, LID_OPEN, SPEED_FAST);
    setServoGoal(1, ARM_OUT, SPEED_FAST);
    waitForAllServos();
    vTaskDelay(pdMS_TO_TICKS(400));
    for (int i = 0; i < 3; i++) {
        setServoGoal(1, ARM_WAVE_LEFT, SPEED_FAST);
        waitForServo(1);
        setServoGoal(1, ARM_WAVE_RIGHT, SPEED_FAST);
        waitForServo(1);
    }
    setServoGoal(1, ARM_OUT, SPEED_FAST);
    waitForServo(1);
    vTaskDelay(pdMS_TO_TICKS(300));
    setServoGoal(0, LID_CLOSED, SPEED_MEDIUM);
    setServoGoal(1, ARM_IN, SPEED_MEDIUM);
    waitForAllServos();
    Serial.println("Fine scenario 9!");
}


void execute_scenario(char command) {
    switch (command) {
        case '1': scenario_peek_and_hide(); break;
        case '2': scenario_hesitant(); break;
        case '3': scenario_distracted_by_user(); break;
        case '4': scenario_annoyed_poke(); break;
        case '5': scenario_shy(); break;
        case '6': scenario_drummer(); break;
        case '7': scenario_fake_out(); break;
        case '8': scenario_slow_wave(); break;
        case '9': scenario_nine_special(); break;
        case '0': scenario_do_nothing(); break;
        case '#': finalSequence(); break;
        default:
            Serial.print("Unknown scenario digit: ");
            Serial.println(command);
            break;
    }
}

// Print the scenario selection menu
void printMenu() {
    Serial.println("\n--- Useless Box Scenario Sequencer ---");
    Serial.println("Enter a sequence of digits (e.g., '1529#') and press Enter.");
    Serial.println("0: Do Nothing");
    Serial.println("1: Peek and Hide");
    Serial.println("2: Hesitant");
    Serial.println("3: Distracted by User");
    Serial.println("4: Annoyed Poke");
    Serial.println("5: Shy");
    Serial.println("6: Drummer");
    Serial.println("7: The Fake Out");
    Serial.println("8: The Slow Wave");
    Serial.println("9: Special Hello!");
    Serial.println("#: FINAL SEQUENCE (Presses Switch)");
    Serial.println("--------------------------------------");
    Serial.print("Sequence> ");
}

// Parse and execute a sequence of scenarios
void parseScenarios(const String& sequence) {
    Serial.print("Executing sequence: ");
    Serial.println(sequence);

    for (int i = 0; i < sequence.length(); i++) {
        char cmd = sequence.charAt(i);
        if (!isDigit(cmd) && cmd != '#') {
            Serial.print("Skipping invalid character: ");
            Serial.println(cmd);
            continue;
        }
        execute_scenario(cmd);
        // Se la sequenza finale è stata eseguita, interrompi la sequenza
        if (cmd == '#') {
            Serial.println("Final sequence executed. Halting.");
            break;
        }
        // Reset servos to initial state before the next scenario in the sequence
        waitForAllServos();
        vTaskDelay(pdMS_TO_TICKS(500));
        setServoInstant(0, LID_CLOSED);
        setServoInstant(1, ARM_IN);
        Serial.println("\nReady for next scenario in sequence...");
    }
    Serial.println("\nSequence finished!");
}


// --- Servo Task and Control Functions ---
void servo_task(void *pvParameters) {
    int servo_idx = *((int*)pvParameters);
    delete (int*)pvParameters; // Free the memory allocated for the parameter

    while (1) {
        if (servo_goals[servo_idx].current_pos != servo_goals[servo_idx].target_pos) {
            int step = (servo_goals[servo_idx].target_pos > servo_goals[servo_idx].current_pos) ? 1 : -1;
            servo_goals[servo_idx].current_pos += step;

            NRF52_ISR_Servos.setPosition(ISR_servo[servo_idx].servoIndex, servo_goals[servo_idx].current_pos);
            vTaskDelay(pdMS_TO_TICKS(servo_goals[servo_idx].speed_delay));
        } else {
            // Servo is idle, check less frequently
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

void setup() {
    Serial.begin(115200);
    // It can take a moment for the serial port to open, especially on native USB
    for(int i=0; i<5 && !Serial; i++) { vTaskDelay(pdMS_TO_TICKS(1000)); }

    Serial.println("Useless Box Initializing with FreeRTOS...");
    
    Wire.setPins(SDA_PIN, SCL_PIN);
    Wire.begin();

    sensor.setTimeout(500);
    if (!sensor.init()) Serial.println("Failed to detect and initialize sensor!");
    sensor.startContinuous();

    for (int i = 0; i < 2; i++) {
        ISR_servo[i].servoIndex = NRF52_ISR_Servos.setupServo(ISR_servo[i].servoPin, MIN_MICROS, MAX_MICROS);
        if (ISR_servo[i].servoIndex < 0) {
             Serial.print("Failed to attach servo on pin "); Serial.println(ISR_servo[i].servoPin);
        }
    }

    setServoInstant(0, LID_CLOSED);
    setServoInstant(1, ARM_IN);

    // Create servo tasks
    int* task_param_0 = new int(0);
    xTaskCreate(servo_task, "Servo0", 256, (void*)task_param_0, 2, NULL);

    int* task_param_1 = new int(1);
    xTaskCreate(servo_task, "Servo1", 256, (void*)task_param_1, 2, NULL);

    Serial.println("Initialization complete. Ready for manual control.");
    vTaskDelay(pdMS_TO_TICKS(500));

    String sequence = String(random(0, 9999));
    sequence += "#"; // Append final sequence command
    Serial.print("Auto-executing random sequence: ");
    Serial.println(sequence);
    parseScenarios(sequence);
    printMenu();
}

void loop() { // loop() runs as a FreeRTOS task
    if (Serial.available() > 0) {
        String sequence = Serial.readStringUntil('\n');
        sequence.trim();
        parseScenarios(sequence);
        printMenu();
    }

    vTaskDelay(pdMS_TO_TICKS(50)); // Poll for serial input
}

