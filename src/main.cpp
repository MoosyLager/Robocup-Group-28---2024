#include "collection.h"
#include "encoder.h"
#include "motor.h"
#include "sensor.h"
#include <Arduino.h>
#include "searchAlgorithm.h"
#include <elapsedMillis.h>
#include <SparkFunSX1509.h>
#include <VL53L0X.h>
#include <Wire.h>



RobotFSM fsm;
elapsedMicros timer;
elapsedMillis updateSensorsTimer = 0;
elapsedMillis updateFSM = 0;
elapsedMicros updatePWM = 0;

void setup()
{
    Serial.begin(9600);
    delay(1000);
    InitSensors();
    InitCircularBuffers();
    InitMotors();
    InitEncoders();
    // CalibrateCollector();
    initializeRobotFSM(&fsm);
    delay(10000);
    Serial.println("Setup complete");
}

void loop()
{
    // UpdateIMU();
    // if (updateSensorsTimer > 50) {
    //     float acc = GetOrientationPitch();
    //     Serial.print("Pitch: ");
    //     Serial.print(acc);
    //     float acc2 = GetOrientationRoll();
    //     Serial.print(" Roll: ");
    //     Serial.print(acc2);
    //     float acc3 = GetOrientationYaw();
    //     Serial.print(" Yaw: ");
    //     Serial.print(acc3);
        
    //     if (checkTargetHeading(fsm.targetHeading)) {
    //         Serial.print("At target heading");
    //     }
    //     Serial.println();
    //     updateSensorsTimer = 0;
    // }
    if (updateFSM > 47) {
        processFSM(&fsm);
        updateFSM = 0;
    }
    // if (updatePWM > 51) {
    //     // UpdateMotors();
    //     updatePWM = 0;
    //     PIDMotorControl(&leftMotor);
    //     PIDMotorControl(&rightMotor);
    // }
    if (updateSensorsTimer > 31) {
        UpdateTOFL0();
        UpdateTOFL1();
        UpdateIMU();
        updateSensorsTimer = 0;
    }

}
