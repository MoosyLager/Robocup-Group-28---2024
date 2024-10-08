#ifndef MOTOR_H
#define MOTOR_H

#include <AccelStepper.h>
#include <Arduino.h>
#include <Servo.h>
#include <elapsedMillis.h>
#include <stdbool.h>
#include <stdint.h>

#define LEFT_MOTOR_ADDRESS  0
#define RIGHT_MOTOR_ADDRESS 1
#define COLLECTION_MOTOR_ADDRESS 7

#define STEPPER_DIR_PIN          28
#define STEPPER_STEP_PIN         29
#define STEPPER_MAX_SPEED        1000
#define STEPPER_MAX_ACCELERATION 500

#define MIN_MOTOR_VAL       1010
#define MAX_MOTOR_VAL       1990
#define MOTOR_STOP_VAL      1500


#define LEFT_MOTOR_ADDRESS  0
#define RIGHT_MOTOR_ADDRESS 1
#define LEFT_JOYSTICK_ADDRESS A0
#define RIGHT_JOYSTICK_ADDRESS A1
#define COLLECTION_MOTOR_ADDRESS 7
#define MIN_MOTOR_VAL       1010
#define MAX_MOTOR_VAL       1990
#define MOTOR_STOP_VAL      1500
#define INTEGRAL_LIMIT      50000000
#define POSITIONAL_CONVERSION 1273/10 //Careful about using this
#define POSITIONAL_OFFSET 0

extern elapsedMillis currentTime;

typedef enum {
    LEFT_MOTOR,
    RIGHT_MOTOR,
    COLLECTION_MOTOR
} MotorType_t;

typedef struct {
    Servo servoDriver;
    MotorType_t motorType;
    signed int currentMotorSpeed;
    signed long currentMotorPos;
    signed int prevMotorSpeed;   // To store the previous motor speed for the derivative term
    signed long prevSampledMotorPos;
    signed int integral;         // To store the integral term for each motor
    bool speedInverted;             // To handle inverted control for some motors
    uint16_t Kp;                 // Proportional gain
    uint16_t Ki;                 // Integral gain
    uint16_t Kd;                 // Derivative gain
    signed long targetMotorPos;   // Target position for position control
    signed int targetMotorSpeed;  // Target speed for speed control
    bool isPositionControl;       // To switch between position and speed control
    signed long prevTime;
} Motor_t;

extern Motor_t leftMotor;
extern Motor_t rightMotor;
extern Motor_t collectionMotor;
extern AccelStepper rampStepper;


void InitMotors();
void SetMotorSpeed(Motor_t *motor, signed int speed);
signed int CheckSpeedLimits(signed int speed);
void findTargetMotorSpeed(uint16_t* leftMotorTarget, uint16_t* rightMotorTarget );
void findMotorSpeed(signed long deltaPos, signed int deltaT);
signed int pidMotorControl(Motor_t *motor, bool isPositionControl, signed long target, unsigned long deltaT);
void PIDMotorControl(Motor_t *motor);
void moveForward(int speed);
void moveBackward(int speed);
void rotateCW(int speed);
void rotateCCW(int speed);
void moveDistance(int distance, Motor_t *motor);
void move(int speedL, int speedR);

#endif
