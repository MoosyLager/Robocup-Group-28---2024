#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <Servo.h>
#include <stdint.h>
#include <stdbool.h>
#include <elapsedMillis.h>

#define LEFT_MOTOR_ADDRESS  0
#define RIGHT_MOTOR_ADDRESS 1
#define LEFT_JOYSTICK_ADDRESS A0
#define RIGHT_JOYSTICK_ADDRESS A1
#define COLLECTION_MOTOR_ADDRESS 7
#define MIN_MOTOR_VAL       1010
#define MAX_MOTOR_VAL       1990
#define MOTOR_STOP_VAL      1500

extern elapsedMillis currentTime;

typedef enum {
    LEFT_MOTOR,
    RIGHT_MOTOR,
    COLLECTION_MOTOR
} MotorType_t;

typedef struct {
    Servo servoDriver;
    MotorType_t motorType;
    signed int motorSpeed;
    signed int targetMotorSpeed;
    signed int currentMotorSpeed;
    signed long prevSampledMotorPos;
    signed long currentMotorPos;
    signed int prevMotorSpeed;   // To store the previous motor speed for the derivative term
    signed int integral;         // To store the integral term for each motor
    bool isInverted;             // To handle inverted control for some motors
    uint16_t Kp;                 // Proportional gain
    uint16_t Ki;                 // Integral gain
    uint16_t Kd;                 // Derivative gain
    signed long targetPosition;   // Target position for position control
} Motor_t;


void InitMotors(Motor_t *leftMotor, Motor_t *rightMotor, Motor_t *collectionMotor);
void SetMotorSpeed(Motor_t *motor, signed int speed);
signed int CheckSpeedLimits(signed int speed);
void findTargetMotorSpeed(uint16_t* leftMotorTarget, uint16_t* rightMotorTarget );
signed long findMotorSpeed(signed long deltaPos, signed int deltaT);
signed int pidMotorDistanceControl(Motor_t *motor, signed long targetPosition, unsigned long deltaT);
signed int pidMotorSpeedControl(Motor_t *motor, signed int targetMotorSpeed, signed int currentMotorSpeed, unsigned long deltaT);
void PIDMotorPositionControl(Motor_t *leftMotor, Motor_t *rightMotor);
void PIDMotorSpeedControl(Motor_t *leftMotor, Motor_t *rightMotor);

#endif
