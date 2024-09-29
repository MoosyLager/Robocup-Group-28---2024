#include "motor.h"
#include "encoder.h"
#include <elapsedMillis.h>

uint16_t Ki = 0;
uint16_t Kp = 100;
uint16_t Kd = 0;

uint16_t prevSampledLeftMotorPos;
uint16_t prevSampledRightMotorPos;

elapsedMillis currentTime = 0;

Servo leftMotor;
Servo rightMotor;
Servo collectionMotor;

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
 * Initialise all connected motors
 */
void InitMotors()
{
    InitDriveMotors();
    InitCollectionMotor();
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




uint16_t pidMotorControl(uint16_t targetMotorSpeed, uint16_t currentMotorSpeed, uint16_t currentMotorPos, uint16_t prevMotorPos, unsigned long deltaT)
{
    // Proportional control
    uint16_t error = targetMotorSpeed - currentMotorSpeed;
    uint16_t p = Kp * error;

    // Integral control
    static uint16_t i = 0;
    i = i + Ki * error;

    // Deals with non-linear time intervals
    uint16_t d = (currentMotorPos - prevMotorPos) / deltaT;
    
    return p + i + d;
}


uint16_t leftJoystickRead(void) 
{
    return analogRead(LEFT_JOYSTICK_ADDRESS);
}



uint16_t rightJoystickRead(void) 
{
    return analogRead(RIGHT_JOYSTICK_ADDRESS);
}



void findTargetMotorSpeed(uint16_t* leftMotorTarget, uint16_t* rightMotorTarget )
{
    // Say 60Hz refresh rate
    if (USE_JOYSTICK) {
        *leftMotorTarget = 3000;
        *rightMotorTarget = 3000;
    } else {
        // Need to implement
    }
}

uint16_t findMotorSpeed(uint16_t motorPos, uint16_t prevMotorPos, unsigned long deltaT)
{
    uint16_t speed = (motorPos - prevMotorPos) * 1000 / deltaT; // ticks/s
    return speed;
}


void PIDMotorSpeedControl(void)
{
    static unsigned long prevTime = 0;
    unsigned long deltaT = currentTime - prevTime;

    uint16_t leftMotorTarget;
    uint16_t rightMotorTarget;
    findTargetMotorSpeed(&leftMotorTarget, &rightMotorTarget);

    uint16_t leftMotorSpeed = findMotorSpeed(leftMotorPos, prevSampledLeftMotorPos, deltaT);
    uint16_t rightMotorSpeed = findMotorSpeed(rightMotorPos, prevSampledRightMotorPos, deltaT);

    uint16_t leftMotorControl = pidMotorControl(leftMotorTarget, leftMotorSpeed, leftMotorPos, prevSampledLeftMotorPos, deltaT);
    uint16_t rightMotorControl = pidMotorControl(rightMotorTarget, rightMotorSpeed, rightMotorPos, prevSampledRightMotorPos, deltaT);

    SetMotorSpeed(leftMotor, leftMotorControl);
    Serial.println(leftMotorControl);
    SetMotorSpeed(rightMotor, rightMotorControl);
    Serial.println(rightMotorControl);

    prevSampledLeftMotorPos = leftMotorPos;
    prevSampledRightMotorPos = rightMotorPos;
    prevTime = currentTime;
}


