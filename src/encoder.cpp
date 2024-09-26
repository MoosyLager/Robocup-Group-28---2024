#include "encoder.h"

extern volatile uint16_t PID_leftMotorPos = 0;
extern volatile uint16_t PID_rightMotorPos = 0;

extern const uint16_t Kp = 1; 
extern const uint16_t Ki = 0;
extern const uint16_t Kd = 0;

volatile unsigned int leftMotorPos = 0;
unsigned int prevLeftMotorPos = 1;
volatile unsigned int RightMotorPos = 0;
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
    RightMotorPos += (rightASet != rightBSet) ? +1 : -1;

    rightBSet = digitalRead(rightMotorPinB) == HIGH;
    // and adjust counter + if B follows A
    RightMotorPos += (rightASet == rightBSet) ? +1 : -1;
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