#include "encoder.h"
#include "motor.h"
#include "sensor.h"
#include <Arduino.h>

elapsedMillis motorDelay;
uint16_t motorSpeed = 1100;

void setup()
{
    Serial.begin(9600);

    InitDriveEncoders();
    InitDriveMotors();
    InitSensors();
}

void loop()
{
    // SetMotorSpeed(leftMotor, motorSpeed);

    // if ( motorDelay >= 1500 ) {
    //     motorSpeed = (motorSpeed == 1100) ? 1900 : 1100;
    //     motorDelay = 0;
    // }

    // if ( (lastReportedPos1 != encoderPos1) || (lastReportedPos2 != encoderPos2) ) {
    //     Serial.print("Index:");
    //     Serial.print(encoderPos1, DEC);
    //     Serial.print(":");
    //     Serial.print(encoderPos2, DEC);
    //     Serial.println();
    //     lastReportedPos1 = encoderPos1;
    //     lastReportedPos2 = encoderPos2;
    // }

    for ( uint8_t i = 0; i < NUM_TOF_L0; i++ ) {
        Serial.print(returnL0()[i].readRangeContinuousMillimeters());
        if ( returnL0()[i].timeoutOccurred() ) {
            Serial.print(" TIMEOUT");
        }
        Serial.print('\t');
    }
    Serial.println();
}
