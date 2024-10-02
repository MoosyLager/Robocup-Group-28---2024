#include "motor.h"
#include "encoder.h"
#include <elapsedMillis.h>

uint16_t Ki = 8;
uint16_t Kp = 50;
uint16_t Kd = 0;



signed long prevSampledLeftMotorPos = 0;
signed long prevSampledRightMotorPos = 0;

elapsedMillis currentTime = 0;

Motor_t leftMotor;
Motor_t rightMotor;
Motor_t collectionMotor;


void InitMotors(Motor_t *leftMotor, Motor_t *rightMotor, Motor_t *collectionMotor) {
    // Initialize the left motor
    leftMotor->servoDriver.attach(LEFT_MOTOR_ADDRESS);
    leftMotor->motorType = LEFT_MOTOR;
    leftMotor->Kp = 50;  // Example gains
    leftMotor->Ki = 8;
    leftMotor->Kd = 0;
    leftMotor->isInverted = true; // Normal direction control
    leftMotor->motorSpeed = 0;
    leftMotor->currentMotorSpeed = 0;
    leftMotor->prevSampledMotorPos = 0;
    leftMotor->currentMotorPos = 0;
    leftMotor->targetPosition = 0; // Initialize target position

    // Initialize the right motor
    rightMotor->servoDriver.attach(RIGHT_MOTOR_ADDRESS);
    rightMotor->motorType = RIGHT_MOTOR;
    rightMotor->Kp = 50;
    rightMotor->Ki = 8;
    rightMotor->Kd = 0;
    rightMotor->isInverted = false; // Inverted control for right motor
    rightMotor->motorSpeed = 0;
    rightMotor->currentMotorSpeed = 0;
    rightMotor->prevSampledMotorPos = 0;
    rightMotor->currentMotorPos = 0;
    rightMotor->targetPosition = 0; // Initialize target position

    // Initialize the collection motor
    collectionMotor->servoDriver.attach(COLLECTION_MOTOR_ADDRESS);
    collectionMotor->motorType = COLLECTION_MOTOR;
    collectionMotor->Kp = 40;
    collectionMotor->Ki = 6;
    collectionMotor->Kd = 0;
    collectionMotor->isInverted = false; // Normal control
    collectionMotor->motorSpeed = 0;
    collectionMotor->currentMotorSpeed = 0;
    collectionMotor->prevSampledMotorPos = 0;
    collectionMotor->currentMotorPos = 0;
    collectionMotor->targetPosition = 0; // Initialize target position
}


void SetMotorSpeed(Motor_t *motor, signed int speed) {
    signed int clampedSpeed = CheckSpeedLimits(speed);
    motor->servoDriver.writeMicroseconds(clampedSpeed);
}

/*
Ensures speed is within limits
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


void findTargetMotorSpeed(uint16_t* leftMotorTarget, uint16_t* rightMotorTarget )
{
    *leftMotorTarget = 3000;
    *rightMotorTarget = 3000;
}

// Find motor speed (delta position / delta time)
signed long findMotorSpeed(signed long deltaPos, signed int deltaT)
{
    signed long speed = deltaPos * 1000 / deltaT; // ticks/s
    return speed;
}

signed int pidMotorDistanceControl(Motor_t *motor, signed long targetPosition, unsigned long deltaT) {
    // Calculate the error between current position and target position
    signed long positionError = targetPosition - motor->currentMotorPos;
    signed long p = motor->Kp * positionError;

    // Integral control
    static signed long i = 0;
    i += motor->Ki * positionError;

    // Derivative control
    signed long d = motor->Kd * (motor->currentMotorPos - motor->prevSampledMotorPos) / deltaT;

    // Store current position as previous for the next cycle
    motor->prevSampledMotorPos = motor->currentMotorPos;

    // Compute the total control signal
    signed long control = p + i - d;
    control /= 1000;  // Scaling the control value

    // Invert control if needed
    if (motor->isInverted) {
        control = -control;
    }

    return control;
}

signed int pidMotorSpeedControl(Motor_t *motor, signed int targetMotorSpeed, signed int currentMotorSpeed, unsigned long deltaT) {
    // Proportional control
    signed int error = targetMotorSpeed - currentMotorSpeed;
    signed int p = motor->Kp * error;

    // Integral control (updated to use per-motor integral)
    motor->integral += motor->Ki * error;

    // Derivative control (updated to use per-motor previous speed)
    signed int d = motor->Kd * (currentMotorSpeed - motor->prevMotorSpeed) / deltaT;
    motor->prevMotorSpeed = currentMotorSpeed;

    // Compute the control signal
    signed int control = p + motor->integral - d;
    control /= 1000;

    // Invert control if needed
    if (motor->isInverted) {
        control = -control;
    }

    // Return the control signal to set motor speed
    return control;
}

void PIDMotorPositionControl(Motor_t *leftMotor, Motor_t *rightMotor) {
    static signed long prevTime = 0;
    signed int deltaT = currentTime - prevTime;
    prevTime = currentTime;

    // Get the current positions (from encoders or similar)
    signed long leftDeltaPos = leftMotor->currentMotorPos - leftMotor->prevSampledMotorPos;
    signed long rightDeltaPos = rightMotor->currentMotorPos - rightMotor->prevSampledMotorPos;
    leftMotor->prevSampledMotorPos = leftMotor->currentMotorPos;
    rightMotor->prevSampledMotorPos = rightMotor->currentMotorPos;

    // Calculate the PID control output for the position
    signed int leftMotorControl = pidMotorDistanceControl(leftMotor, leftMotor->targetPosition, deltaT);
    signed int rightMotorControl = pidMotorDistanceControl(rightMotor, rightMotor->targetPosition, deltaT);

    // Set motor speeds based on the position control output
    SetMotorSpeed(leftMotor, leftMotorControl);
    SetMotorSpeed(rightMotor, rightMotorControl);
}


void PIDMotorSpeedControl(Motor_t *leftMotor, Motor_t *rightMotor) {
    static signed long prevTime = 0;
    signed int deltaT = currentTime - prevTime;
    prevTime = currentTime;

    // Get position deltas (from encoders or similar)
    signed long leftDeltaPos = leftMotor->currentMotorPos - leftMotor->prevSampledMotorPos;
    signed long rightDeltaPos = rightMotor->currentMotorPos - rightMotor->prevSampledMotorPos;
    leftMotor->prevSampledMotorPos = leftMotor->currentMotorPos;
    rightMotor->prevSampledMotorPos = rightMotor->currentMotorPos;

    // Calculate motor speeds
    signed int leftMotorSpeed = findMotorSpeed(leftDeltaPos, deltaT);
    signed int rightMotorSpeed = findMotorSpeed(rightDeltaPos, deltaT);

    // Calculate PID control output
    signed int leftMotorControl = pidMotorSpeedControl(leftMotor, leftMotor->targetMotorSpeed, leftMotorSpeed, deltaT);
    signed int rightMotorControl = pidMotorSpeedControl(rightMotor, rightMotor->targetMotorSpeed, rightMotorSpeed, deltaT);

    // Set motor speeds
    SetMotorSpeed(leftMotor, leftMotorControl);
    SetMotorSpeed(rightMotor, rightMotorControl);
}


