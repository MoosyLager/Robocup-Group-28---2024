#include "motor.h"
#include "encoder.h"
#include <elapsedMillis.h>

uint16_t Ki = 8;
uint16_t Kp = 50;
uint16_t Kd = 0;

signed long prevSampledLeftMotorPos = 0;
signed long prevSampledRightMotorPos = 0;

elapsedMillis currentTime = 0;

Servo leftMotor;
Servo rightMotor;
Servo collectionMotor;

Motor_t leftServo = {leftMotor, LEFT_MOTOR, 0, 0, 0, 0, 0};

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
void SetMotorSpeed(Servo motor, signed int speed)
{
    signed int clampedSpeed = CheckSpeedLimits(speed);
    motor.writeMicroseconds(clampedSpeed);
}



/**
 * Ensures speed is within limits
 */
signed int CheckSpeedLimits(signed int speed)
{
    if ( speed > MAX_MOTOR_VAL ) {
        // Serial.printf("Speed was out of bounds. Clamped to %u", MAX_MOTOR_VAL);
        return MAX_MOTOR_VAL;
    }
    if ( speed < MIN_MOTOR_VAL ) {
        // Serial.printf("Speed was out of bounds. Clamped to %u", MIN_MOTOR_VAL);
        return MIN_MOTOR_VAL;
    }
    return speed;
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
    *leftMotorTarget = 3000;
    *rightMotorTarget = 3000;
}

signed long findMotorSpeed(signed long deltaPos, signed int deltaT)
{
    signed long speed = deltaPos * 1000 / deltaT; // ticks/s
    return speed;
}

signed int pidMotorControl(signed int targetMotorSpeed, signed int currentMotorSpeed, unsigned long deltaT)
{
    // Proportional control
    signed int error = targetMotorSpeed - currentMotorSpeed;
    Serial.print("Error: ");
    Serial.print(error);
    Serial.print(" ");
    signed int p = - Kp * error;

    // Integral control
    static signed int i = 0;
    i = i - Ki * error;

    // Deals with non-linear time intervals
    static signed int prevMotorSpeed = 0;
    signed int d = - Kd * (currentMotorSpeed - prevMotorSpeed) / (deltaT);
    prevMotorSpeed = currentMotorSpeed;
    
    signed int control =  p + i + d;
    control = control / 1000;
    Serial.print("Control: ");
    Serial.print(control);
    Serial.print(" ");
    return -control;
}


void PIDMotorSpeedControl(void)
{
    static signed long prevTime = 0;

    signed int deltaT = currentTime - prevTime;
    prevTime = currentTime;

    signed long leftDeltaPos = leftMotorPos - prevSampledLeftMotorPos;
    signed long rightDeltaPos = rightMotorPos - prevSampledRightMotorPos;
    signed long collectionDeltaPos = collectionMotorPos - prevSampledCollectionMotorPos;
    prevSampledLeftMotorPos = leftMotorPos;
    prevSampledRightMotorPos = rightMotorPos;

    // findTargetMotorSpeed(&leftMotorTarget, &rightMotorTarget);

    // signed long leftMotorSpeed = findMotorSpeed(leftDeltaPos, deltaT);
    signed long rightMotorSpeed = findMotorSpeed(rightDeltaPos, deltaT);
    Serial.print("Right Motor: ");
    Serial.println(rightMotorSpeed);
    
    // Bruh sound effect
    signed int rightMotorControl = pidMotorControl(3000, rightMotorSpeed, deltaT);

    SetMotorSpeed(rightMotor, rightMotorControl);
    return;
}


