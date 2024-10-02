/**
 * Module for controlling all connected motors including 2 DC drive motors, 1 worm gear DC motor, and 1 stepper motor
 */

#include "motor.h"

Servo leftMotor;
Servo rightMotor;
Servo collectionMotor;
AccelStepper stepper;

/**
 * Initialise the left and right drive motors
 */
void InitDriveMotors()
{
    leftMotor.attach(LEFT_MOTOR_ADDRESS);
    rightMotor.attach(RIGHT_MOTOR_ADDRESS);
}

/**
 * Initialise the collection motor
 */
void InitCollectionMotor()
{
    collectionMotor.attach(COLLECTION_MOTOR_ADDRESS);
}

/**
 * Initialise the ramp stepper motor
 */
void InitStepper()
{
    stepper = AccelStepper(1, STEPPER_DIR_PIN, STEPPER_STEP_PIN);
    stepper.setMaxSpeed(1000);
    stepper.setAcceleration(500);
    stepper.setCurrentPosition(0);
}

/**
 * Initialise all connected motors
 */
void InitMotors()
{
    InitDriveMotors();
    InitCollectionMotor();
    InitStepper();
}

/**
 * Ensures a valid input then sets motor speed
 */
void SetMotorSpeed(Servo motor, uint16_t speed)
{
    uint16_t clampedSpeed = CheckSpeedLimits(speed);
    motor.writeMicroseconds(clampedSpeed);
}

/**
 * Ensures speed is within limits
 */
uint16_t CheckSpeedLimits(uint16_t speed)
{
    if ( speed > MAX_MOTOR_VAL ) {
        Serial.printf("Speed was out of bounds. Clamped to %u", MAX_MOTOR_VAL);
        return MAX_MOTOR_VAL;
    }
    if ( speed < MIN_MOTOR_VAL ) {
        Serial.printf("Speed was out of bounds. Clamped to %u", MIN_MOTOR_VAL);
        return MIN_MOTOR_VAL;
    }
    return speed;
}

/*

Needs a motion state machine to control the motors

*/