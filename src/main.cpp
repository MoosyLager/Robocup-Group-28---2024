#include "encoder.h"
#include "motor.h"
#include "sensor.h"
#include <Arduino.h>
#include <elapsedMillis.h>

elapsedMillis motorDelay;
uint16_t motorSpeed = 1100;
elapsedMillis motorMicros;

void setup()
{
    Serial.begin(9600);

    InitDriveEncoders();
    InitDriveMotors();
    InitSensors();
}

void loop()
{
    SetMotorSpeed(leftMotor, motorSpeed);

    if ( motorDelay >= 1500 ) {
        motorSpeed = (motorSpeed == 1100) ? 1900 : 1100;
        motorDelay = 0;
    }

    // if ( (leftMotorPos != prevLeftMotorPos) || (rightMotorPos != prevRightMotorPos) ) {
    //     Serial.print("Index:");
    //     Serial.print(leftMotorPos, DEC);
    //     Serial.print(":");
    //     Serial.print(rightMotorPos, DEC);
    //     Serial.println();
    //     prevLeftMotorPos = leftMotorPos;
    //     prevRightMotorPos = rightMotorPos;
    // }

    // for ( uint8_t i = 0; i < NUM_TOF_L0; i++ ) {
    //     Serial.print(returnL0()[i].readRangeContinuousMillimeters());
    //     if ( returnL0()[i].timeoutOccurred() ) {
    //         Serial.print(" TIMEOUT");
    //     }
    //     Serial.print('\t');
    // }
    // Serial.println();

    Serial.print("test");
}
