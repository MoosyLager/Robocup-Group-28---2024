#include "collection.h"
#include "encoder.h"
#include "motor.h"
#include "searchAlgorithm.h"
#include "sensor.h"
#include <Arduino.h>
#include <elapsedMillis.h>

elapsedMillis timer;

void setup()
{
    Serial.begin(9600);
    InitSensors();
    InitMotors();
    InitEncoders();

    CalibrateCollector();
}

void loop()
{
    if ( BlueButtonState() && timer > 3000 ) {
        timer = 0;
        CollectWeight();
    }
    UpdateCollector();
}
