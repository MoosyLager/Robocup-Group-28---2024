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
elapsedMicros updateSensorsTimer = 0;
elapsedMicros updateFSM = 0;
elapsedMicros updatePWM = 0;
elapsedMillis changePWM = 0;

void setup()
{
    Serial.begin(9600);
    InitSensors();
    InitMotors();
    initializeRobotFSM(&fsm);
    Serial.println("Initialised.");
    leftMotor.targetMotorSpeed = 1500;
    rightMotor.targetMotorSpeed = 1500;
}

void loop()
{
    // if (updateSensorsTimer > 300)
    // {
    //     updatePWM = 0;
    //     UpdateTOFL1();
    //     unsigned long time1 = updatePWM;
    //     UpdateTOFL0();
    //     unsigned long time = updatePWM;
    //     Serial.print(time1);
    //     Serial.print(" ");
    //     Serial.println(time - time1);
    //     updateSensorsTimer = 0;
    // }
    // if (updateFSM > 100000){
    //     processFSM(&fsm);
    //     updateFSM = 0;   
    // }
    if (updatePWM > 100000) {
        PIDMotorControl(&leftMotor);
        updatePWM = 0;
        Serial.print(leftMotorPos);
        Serial.print(" ");
        Serial.println(rightMotorPos);
    }
    //if (changePWM > 2000) {
    // moveForward(1000);
    //}

}
