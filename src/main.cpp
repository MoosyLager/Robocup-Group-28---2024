#include "collection.h"
#include "encoder.h"
#include "motor.h"
#include "sensor.h"
#include <Arduino.h>

void setup()
{
    Serial.begin(9600);
    InitCollectionEncoder();
    InitCollectionMotor();
    InitIOExpander();
    InitLimitSwitch();
    CalibrateCollector();
}

void loop()
{
    if ( prevCollectionMotorPos != collectionMotorPos ) {
        Serial.println(collectionMotorPos);
        prevCollectionMotorPos = collectionMotorPos;
    }
}