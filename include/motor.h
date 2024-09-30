#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <Servo.h>
#include <stdint.h>
#include <stdbool.h>

#define LEFT_MOTOR_ADDRESS  0
#define RIGHT_MOTOR_ADDRESS 1
#define LEFT_JOYSTICK_ADDRESS A0
#define RIGHT_JOYSTICK_ADDRESS A1
#define COLLECTION_MOTOR_ADDRESS 7
#define MIN_MOTOR_VAL       1010
#define MAX_MOTOR_VAL       1990
#define MOTOR_STOP_VAL      1500
#define USE_JOYSTICK true
#define JOYSTICK_CONVERSION_FACTOR 60

extern uint16_t Ki;
extern uint16_t Kp;
extern uint16_t Kd;

extern elapsedMillis currentTime;

extern signed long prevSampledLeftMotorPos;
extern signed long prevSampledRightMotorPos;

extern Servo leftMotor;
extern Servo rightMotor;
extern Servo collectionMotor;

void InitDriveMotors();
void InitCollectionMotor();
void InitMotors();
void SetMotorSpeed(Servo motor, signed int speed);
signed int CheckSpeedLimits(signed int speed);
uint16_t pidMotorControl(uint16_t targetMotorSpeed, uint16_t currentMotorSpeed, uint16_t currentMotorPos, uint16_t prevMotorPos);
uint16_t leftJoystickRead(void);
uint16_t rightJoystickRead(void);
void findTargetMotorSpeed(uint16_t* leftMotorTarget, uint16_t* rightMotorTarget );
uint16_t findMotorSpeed(uint16_t motorPos, uint16_t prevMotorPos, unsigned long deltaT);
void PIDMotorSpeedControl(void);

#endif