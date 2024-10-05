#include "collection.h"
#include "encoder.h"
#include "motor.h"
#include "sensor.h"
#include <Arduino.h>
#include "searchAlgorithm.h"

#include <SparkFunSX1509.h>
#include <VL53L0X.h>
#include <Wire.h>

RobotFSM fsm;

void setup()
{
    Serial.begin(9600);
    InitSensors();
    // InitMotors();
    initializeRobotFSM(&fsm);
    Serial.println("Initialised.");
}

void loop()
{
    UpdateTOFL1();
    uint16_t L1TL = GetL1TL();
    uint16_t L1TR = GetL1TR();
    uint16_t L1BL = GetL1BL();
    uint16_t L1BR = GetL1BR();
    Serial.print("L1TL: ");
    Serial.println(L1TL);
    Serial.print("L1TR: ");
    Serial.println(L1TR);
    Serial.print("L1BL: ");
    Serial.println(L1BL);
    Serial.print("L1BR: ");
    Serial.println(L1BR);
    Serial.println("-----");
    // Serial.println(L0TL);
    // processFSM(&fsm);
    delay(100);
}
