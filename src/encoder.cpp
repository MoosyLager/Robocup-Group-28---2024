#include "encoder.h"

extern volatile uint16_t PID_leftMotorPos = 0;
extern volatile uint16_t PID_rightMotorPos = 0;

extern const uint16_t Kp = 1; 
extern const uint16_t Ki = 0;
extern const uint16_t Kd = 0;

volatile unsigned int leftMotorPos = 0;
unsigned int prevLeftMotorPos = 1;
volatile unsigned int rightMotorPos = 0;
unsigned int prevRightMotorPos = 1;

volatile unsigned int collectionMotorPos = 0;
unsigned int prevCollectionMotorPos = 1;

// Set states for the drive encoders
boolean leftASet = false;
boolean leftBSet = false;
boolean rightASet = false;
boolean rightBSet = false;

// Set states for the collection encoder
boolean collectionASet;
boolean collectionBSet;

// Left drive motor encoder interrupt handler
void LeftEncoderIntHandler()
{
    // Test transition
    leftASet = digitalRead(leftEncoderPinA) == HIGH;
    // and adjust counter - if A leads B
    leftMotorPos += (leftASet != leftBSet) ? -1 : +1;

    leftBSet = digitalRead(leftEncoderPinB) == HIGH;
    // and adjust counter - if B follows A
    leftMotorPos += (leftASet == leftBSet) ? -1 : +1;
}

// Right drive motor encoder interrupt handler
void RightEncoderIntHandler()
{
    // Test transition
    rightASet = digitalRead(rightEncoderPinA) == HIGH;
    // and adjust counter + if A leads B
    rightMotorPos += (rightASet != rightBSet) ? +1 : -1;

    rightBSet = digitalRead(rightEncoderPinB) == HIGH;
    // and adjust counter + if B follows A
    rightMotorPos += (rightASet == rightBSet) ? +1 : -1;
}

// Collection motor encoder interrupt handler
void CollectionEncoderIntHandler()
{
    // Test transition
    collectionASet = digitalRead(collectionEncoderPinA) == HIGH;
    // and adjust counter + if A leads B
    collectionMotorPos += (collectionASet != collectionBSet) ? +1 : -1;

    collectionBSet = digitalRead(collectionEncoderPinB) == HIGH;
    // and adjust counter + if B follows A
    collectionMotorPos += (collectionASet == collectionBSet) ? +1 : -1;
}

void InitDriveEncoders()
{
    pinMode(leftEncoderPinA, INPUT);  // Set encoder pins as inputs
    pinMode(leftEncoderPinB, INPUT);
    pinMode(rightEncoderPinA, INPUT);
    pinMode(rightEncoderPinB, INPUT);

    attachInterrupt(digitalPinToInterrupt(leftEncoderPinA), LeftEncoderIntHandler, CHANGE);  // Set up an interrupt for each encoder
    attachInterrupt(digitalPinToInterrupt(rightEncoderPinA), RightEncoderIntHandler, CHANGE);
}

uint16_t PID_Controller(uint16_t desiredSpeed, uint16_t currentPos, uint16_t prevPos)
{
    /*uint16_t currentSpeed = (currentPos - prevPos) /  deltaTime;
    uint16_t error = desiredSpeed - currentSpeed;

    how to do the correct integral controller and derivative controller?

    uint16_t integral = 0;
    uint16_t derivative = 0;

    integral += error;
    derivative = currentPos - prevPos;

    return (Kp * error) + (Ki * integral) + (Kd * derivative);*/
}