#include "encoder.h"

volatile unsigned int leftMotorPos = 0;
unsigned int prevLeftMotorPos = 1;
volatile unsigned int rightMotorPos = 0;
unsigned int prevRightMotorPos = 1;
volatile unsigned int collectionMotorPos = 0;
unsigned int prevCollectionMotorPos = 1;

boolean leftASet = false;
boolean leftBSet = false;
boolean rightASet = false;
boolean rightBSet = false;
boolean collectionASet = false;
boolean collectionBSet = false;

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

void collectionEncoderIntHandler()
{
    // Test transition
    collectionASet = digitalRead(collectionMotorPinA) == HIGH;
    // and adjust counter + if A leads B
    collectionMotorPos += (collectionASet != collectionBSet) ? +1 : -1;

    collectionBSet = digitalRead(collectionMotorPinB) == HIGH;
    // and adjust counter + if B follows A
    collectionMotorPos += (collectionASet == collectionBSet) ? +1 : -1;
}

void InitDriveEncoders()
{
    pinMode(leftMotorPinA, INPUT);  // Set encoder pins as inputs
    pinMode(leftMotorPinB, INPUT);
    pinMode(rightMotorPinA, INPUT);
    pinMode(rightMotorPinB, INPUT);
    pinMode(collectionMotorPinA, INPUT);
    pinMode(collectionMotorPinB, INPUT);

    attachInterrupt(digitalPinToInterrupt(2), LeftEncoderIntHandler, CHANGE);  // Set up an interrupt for each encoder
    attachInterrupt(digitalPinToInterrupt(4), RightEncoderIntHandler, CHANGE);
    attachInterrupt(digitalPinToInterrupt(32), collectionEncoderIntHandler, CHANGE);
}