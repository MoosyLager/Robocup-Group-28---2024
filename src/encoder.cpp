#include "encoder.h"

volatile unsigned int encoderPos1 = 0;
unsigned int lastReportedPos1 = 1;
volatile unsigned int encoderPos2 = 0;
unsigned int lastReportedPos2 = 1;

bool aSet1 = false;
bool bSet1 = false;
bool aSet2 = false;
bool bSet2 = false;

/**
 * Initialise the encoders on the two drive motors
 */
void InitDriveEncoders()
{
    // Set encoder pins as inputs
    pinMode(encoder1PinA, INPUT);
    pinMode(encoder1PinB, INPUT);
    pinMode(encoder2PinA, INPUT);
    pinMode(encoder2PinB, INPUT);

    attachInterrupt(digitalPinToInterrupt(2), DoEncoder1A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(4), DoEncoder2A, CHANGE);
}

/**
 * Interrupt on A changing state
 */
void DoEncoder1A()
{
    // Test transition
    aSet1 = digitalRead(encoder1PinA) == HIGH;
    // and adjust counter - if A leads B
    encoderPos1 += (aSet1 != bSet1) ? -1 : +1;

    bSet1 = digitalRead(encoder1PinB) == HIGH;
    // and adjust counter - if B follows A
    encoderPos1 += (aSet1 == bSet1) ? -1 : +1;
}

/**
 * Interrupt on A changing state
 */
void DoEncoder2A()
{
    // Test transition
    aSet2 = digitalRead(encoder2PinA) == HIGH;
    // and adjust counter - if A leads B
    encoderPos2 += (aSet2 != bSet2) ? -1 : +1;

    bSet1 = digitalRead(encoder2PinB) == HIGH;
    // and adjust counter - if B follows A
    encoderPos2 += (aSet2 == bSet2) ? -1 : +1;
}