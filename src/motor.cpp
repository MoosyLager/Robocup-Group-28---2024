/**
 * Module for controlling all connected motors including 2 DC drive motors, 1 worm gear DC motor, and 1 stepper motor
 */

#include "motor.h"
#include "encoder.h"
#include <elapsedMillis.h>

Motor_t leftMotor;
Motor_t rightMotor;
Motor_t collectionMotor;
AccelStepper rampStepper;

elapsedMillis currentTime;

/**
 * Initialise the drive motors, collection motor and ramp stepper motor
 */
void InitMotors()
{
    // Initialize the left motor
    leftMotor.servoDriver.attach(LEFT_MOTOR_ADDRESS);
    leftMotor.motorType = LEFT_MOTOR;
    leftMotor.Kp = 100;  // Example gains
    leftMotor.Ki = 100;
    leftMotor.Kd = 0;
    leftMotor.speedInverted = true;  // Normal direction control
    leftMotor.currentMotorSpeed = 0;
    leftMotor.prevSampledMotorPos = 0;
    leftMotor.currentMotorPos = 0;
    leftMotor.targetMotorPos = 0;  // Initialize target value
    leftMotor.isPositionControl = false;  // Start in position control mode
    leftMotor.prevTime = 0;

    // Initialize the right motor
    rightMotor.servoDriver.attach(RIGHT_MOTOR_ADDRESS);
    rightMotor.motorType = RIGHT_MOTOR;
    rightMotor.Kp = 100;
    rightMotor.Ki = 100;
    rightMotor.Kd = 0;
    rightMotor.speedInverted = false;  // Inverted control for right motor
    rightMotor.currentMotorSpeed = 0;
    rightMotor.prevSampledMotorPos = 0;
    rightMotor.currentMotorPos = 0;
    rightMotor.targetMotorPos = 0;  // Initialize target value
    rightMotor.isPositionControl = false;  // Start in position control mode
    rightMotor.prevTime = 0;

    // Initialize the collection motor
    collectionMotor.servoDriver.attach(COLLECTION_MOTOR_ADDRESS);
    collectionMotor.motorType = COLLECTION_MOTOR;
    collectionMotor.Kp = 40;
    collectionMotor.Ki = 6;
    collectionMotor.Kd = 0;
    collectionMotor.speedInverted = false;  // Normal control
    collectionMotor.currentMotorSpeed = 0;
    collectionMotor.prevSampledMotorPos = 0;
    collectionMotor.currentMotorPos = 0;
    collectionMotor.targetMotorPos = 0;    // Initialize target value
    collectionMotor.targetMotorSpeed = 0;  // Initialize target value
    collectionMotor.isPositionControl = true;  // Start in position control mode
    collectionMotor.prevTime = 0;

    // Initialise the ramp stepper motor
    rampStepper = AccelStepper(1, STEPPER_DIR_PIN, STEPPER_STEP_PIN);
    rampStepper.setMaxSpeed(STEPPER_MAX_SPEED);
    rampStepper.setAcceleration(STEPPER_MAX_ACCELERATION);
    rampStepper.setCurrentPosition(0);
}

/**
 * Check that value is between limits and sets motor speed
 */
void SetMotorSpeed(Motor_t *motor, signed int speed)
{
    signed int clampedSpeed = CheckSpeedLimits(speed);
    motor->servoDriver.writeMicroseconds(clampedSpeed);
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

// Find motor speed (delta position / delta time)
void findMotorSpeed(Motor_t* motor, signed long deltaPos, signed int deltaT)
{
    motor->currentMotorSpeed = deltaPos * 1000 / deltaT; // ticks/s
}

signed int pidMotorControl(Motor_t *motor, bool isPositionControl, signed long target, unsigned long deltaT) {
    signed long error, p, d;
    
    // Proportional and Derivative control calculations
    if (isPositionControl) {
        // Position control
        error = target - motor->currentMotorPos;
        p = motor->Kp * error;
        d = motor->Kd * (motor->currentMotorPos - motor->prevSampledMotorPos) / deltaT;
        motor->prevSampledMotorPos = motor->currentMotorPos;
    } else {
        // Speed control
        error = target - motor->currentMotorSpeed;
        p = motor->Kp * error;
        d = motor->Kd * (motor->currentMotorSpeed - motor->prevMotorSpeed) / deltaT;
        motor->prevMotorSpeed = motor->currentMotorSpeed;
    }

    // Integral control (updated to use per-motor integral)
    signed int integral = motor->integral +  motor->Ki * error;
    if (integral > INTEGRAL_LIMIT) {
        integral = INTEGRAL_LIMIT;
    } else if (integral < -INTEGRAL_LIMIT) {
        integral = -INTEGRAL_LIMIT;
    }
    motor->integral = integral;

    // Compute the control signal
    signed long control = p + motor->integral - d;
    control /= 1000;  // Scaling the control value

    //Invert control if needed
    if ((motor->speedInverted)) {
        control = -control;
    }
    return control;
}

void PIDMotorControl(Motor_t *motor) {
    signed int deltaT = currentTime - motor->prevTime;
    motor->prevTime = currentTime;

    // Get the delta (for speed calculations)
    if (motor->motorType == LEFT_MOTOR) {
        Serial.print("Left motor: ");
        motor->currentMotorPos = leftMotorPos;
    } else if (motor->motorType == RIGHT_MOTOR) {
        Serial.print("Right motor: ");
        motor->currentMotorPos = rightMotorPos;
    } else if (motor->motorType == COLLECTION_MOTOR) {
        motor->currentMotorPos = collectionMotorPos;
    } 
    signed long deltaPos = motor->currentMotorPos - motor->prevSampledMotorPos;

    // Calculate motor speed if in speed control mode
    findMotorSpeed(motor, deltaPos, deltaT);
    Serial.print(" Current motor speed: ");
    Serial.print(motor->currentMotorSpeed);

    // For position control, target is position, for speed control, target is speed
    signed int target = motor->isPositionControl ? motor->targetMotorPos : motor->targetMotorSpeed;
    Serial.print(" Target: ");
    Serial.print(target);
    if (motor->motorType == RIGHT_MOTOR) {
        Serial.println(" ");
    }
    // Calculate PID control output
    signed int motorControl = pidMotorControl(motor, motor->isPositionControl, target, deltaT);

    motor->prevSampledMotorPos = motor->currentMotorPos;

    // Set motor speed
    SetMotorSpeed(motor, motorControl);
}


void moveForward(int speed) {
    leftMotor.targetMotorSpeed = speed;
    rightMotor.targetMotorSpeed = speed;
    leftMotor.isPositionControl = false;
    rightMotor.isPositionControl = false;
}

void rotateCW(int speed) {
    leftMotor.targetMotorSpeed = speed / 3;
    rightMotor.targetMotorSpeed = -speed;
    leftMotor.isPositionControl = false;
    rightMotor.isPositionControl = false;
}

void rotateCCW(int speed) {
    leftMotor.targetMotorSpeed = -speed;
    rightMotor.targetMotorSpeed = speed / 2;
    leftMotor.isPositionControl = false;
    rightMotor.isPositionControl = false;
}

void moveBackward(int speed) {
    leftMotor.targetMotorSpeed = -speed;
    rightMotor.targetMotorSpeed = -speed;
    leftMotor.isPositionControl = false;
    rightMotor.isPositionControl = false;
}

void moveDistance(int distance, Motor_t* motor) {
    motor->targetMotorPos = distance;
    motor->isPositionControl = true;
}







