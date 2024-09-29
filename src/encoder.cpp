#include "encoder.h"

volatile unsigned int leftMotorPos = 0;
unsigned int prevLeftMotorPos = 1;
volatile unsigned int rightMotorPos = 0;
unsigned int prevRightMotorPos = 1;

boolean leftASet = false;
boolean leftBSet = false;
boolean rightASet = false;
boolean rightBSet = false;

// Interrupt on A changing state
void LeftEncoderIntHandler()
{
    // Test transition
    leftASet = digitalRead(leftMotorPinA) == HIGH;
    // and adjust counter - if A leads B
    leftMotorPos += (leftASet != leftBSet) ? -1 : +1;

    leftBSet = digitalRead(leftMotorPinB) == HIGH;
    // and adjust counter - if B follows A
    leftMotorPos += (leftASet == leftBSet) ? -1 : +1;
}

// Interrupt on A changing state
void RightEncoderIntHandler()
{
    // Test transition
    rightASet = digitalRead(rightMotorPinA) == HIGH;
    // and adjust counter + if A leads B
    rightMotorPos += (rightASet != rightBSet) ? +1 : -1;

    rightBSet = digitalRead(rightMotorPinB) == HIGH;
    // and adjust counter + if B follows A
    rightMotorPos += (rightASet == rightBSet) ? +1 : -1;
}

void InitDriveEncoders()
{
    pinMode(leftMotorPinA, INPUT);  // Set encoder pins as inputs
    pinMode(leftMotorPinB, INPUT);
    pinMode(rightMotorPinA, INPUT);
    pinMode(rightMotorPinB, INPUT);

    attachInterrupt(digitalPinToInterrupt(2), LeftEncoderIntHandler, CHANGE);  // Set up an interrupt for each encoder
    attachInterrupt(digitalPinToInterrupt(4), RightEncoderIntHandler, CHANGE);
}