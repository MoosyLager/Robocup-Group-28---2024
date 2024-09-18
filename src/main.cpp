#include "encoder.h"
#include "motor.h"
#include <Arduino.h>

void setup()
{
    // InitDriveMotors();

    InitDriveEncoders();

    Serial.begin(9600);
}

void loop()
{
    if ( (lastReportedPos1 != encoderPos1) || (lastReportedPos2 != encoderPos2) ) {
        Serial.print("Index:");
        Serial.print(encoderPos1, DEC);
        Serial.print(":");
        Serial.print(encoderPos2, DEC);
        Serial.println();
        lastReportedPos1 = encoderPos1;
        lastReportedPos2 = encoderPos2;
    }
}