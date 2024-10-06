#include "collection.h"
#include "encoder.h"
#include "motor.h"
#include "sensor.h"
#include <Arduino.h>
#include "searchAlgorithm.h"
#include <elapsedMillis.h>

#include <elapsedMillis.h>
elapsedMillis timer;

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
    InitEncoders();
    CalibrateCollector();
    delay(500);
    initializeRobotFSM(&fsm);
    Serial.println("Initialised.");
    leftMotor.targetMotorSpeed = 1500;
    rightMotor.targetMotorSpeed = 1500;
}

void loop()
{
    SetMotorSpeed(&collectionMotor, MAX_MOTOR_VAL);
    
    if (updatePWM > 100000) {
        PIDMotorControl(&leftMotor);
        updatePWM = 0;
        Serial.print(leftMotorPos);
        Serial.print(" ");
        Serial.println(rightMotorPos);
    }
}
