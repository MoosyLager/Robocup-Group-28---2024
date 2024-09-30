#ifndef MOTOR_H
#define MOTOR_H

#define LEFT_MOTOR_ADDRESS  0
#define RIGHT_MOTOR_ADDRESS 1
#define COLLECTION_MOTOR_ADDRESS 7
#define MIN_MOTOR_VAL       1010
#define MAX_MOTOR_VAL       1990
#define MOTOR_STOP_VAL      1500

#include <Arduino.h>
#include <Servo.h>
#include <stdint.h>


extern Servo leftMotor;
extern Servo rightMotor;
extern Servo collectionMotor;

void SetMotorSpeed(Servo motor, uint16_t speed);
uint16_t CheckSpeedLimits(uint16_t speed);
void InitMotors();
void InitDriveMotors();
void InitCollectionMotor();

#endif