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
elapsedMicros updateFSM = 0;
elapsedMicros updatePWM = 0;
elapsedMillis changePWM = 0;

void setup()
{
    Serial.begin(9600);
    delay(1000);
    InitSensors();
    InitCircularBuffers();
    // InitMotors();
    // InitEncoders();
    // CalibrateCollector();

    initializeRobotFSM(&fsm);
    Serial.println("Initialised.");
}

void loop()
{
    UpdateIMU();
    float acc4 = GetAccelerationForward();
    float forwardPos = findPos(acc4, timer);
    if (updateSensorsTimer > 50) {
        float acc5 = GetAccelerationSideways();
        float acc6 = GetAccelerationUpwards();
        Serial.print("Forward Pos: ");
        Serial.print(forwardPos);
        Serial.print(" Forward: ");
        Serial.print(acc4);
        Serial.print(" Sideways: ");
        Serial.print(acc5);
        Serial.print(" Upwards: ");
        Serial.println(acc6);

        updateSensorsTimer = 0;
    }
}
