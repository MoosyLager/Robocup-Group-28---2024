#ifndef MOTOR_H
#define MOTOR_H

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

#include <AccelStepper.h>
#include <Arduino.h>
#include <Servo.h>
#include <stdint.h>


extern Servo leftMotor;
extern Servo rightMotor;
extern Servo collectionMotor;
extern AccelStepper rampStepper;

void SetMotorSpeed(Servo motor, uint16_t speed);
uint16_t CheckSpeedLimits(uint16_t speed);
void InitMotors();
void InitDriveMotors();
void InitStepper();
void InitCollectionMotor();

#endif