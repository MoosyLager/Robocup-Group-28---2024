#include "collection.h"
#include "encoder.h"
#include "motor.h"
#include "sensor.h"
#include <Arduino.h>

#include <SparkFunSX1509.h>
#include <VL53L0X.h>
#include <Wire.h>

// #include <elapsedMillis.h>


// elapsedMillis motorDelay;
// uint16_t motorSpeed = 1100;
// elapsedMillis motorMicros;

void setup()
{
    Serial.begin(9600);
    // InitSensors();
    // InitIOExpander();
    // InitTOFL0();
    // InitTOFL1();
    // InitTOF();
    InitMotors();

    Serial.println("Initialised.");
}

void loop()
{
    
}
