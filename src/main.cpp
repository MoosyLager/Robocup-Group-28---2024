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
    delay(1000);
    InitDriveEncoders();
    InitCollectionMotor();
    InitDriveMotors();
    Serial.println("Setup");
    // SetMotorSpeed(leftMotor, 1800);
}

void loop()
{
    if ( (leftMotorPos != prevLeftMotorPos) || (rightMotorPos != prevRightMotorPos) || (collectionMotorPos != prevCollectionMotorPos)) {
        Serial.print("Index:");
        Serial.print(leftMotorPos, DEC);
        Serial.print(":");
        Serial.print(rightMotorPos, DEC);
        Serial.print(":");
        Serial.print(collectionMotorPos, DEC);
        Serial.println();
        prevLeftMotorPos = leftMotorPos;
        prevRightMotorPos = rightMotorPos;
        prevCollectionMotorPos = collectionMotorPos;
    }
    motorSpeed = 1900;  
    SetMotorSpeed(collectionMotor, motorSpeed);
}
