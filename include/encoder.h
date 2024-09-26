#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include <stdbool.h>
#include <stdint.h>

enum DriveEncoderPinAssignments {
    leftMotorPinA = 2,
    leftMotorPinB = 3,
    rightMotorPinA = 4,
    rightMotorPinB = 5,
};

extern volatile uint16_t PID_leftMotorPos;
extern volatile uint16_t PID_rightMotorPos;

extern const uint16_t Kp; 
extern const uint16_t Ki;
extern const uint16_t Kd;


extern volatile unsigned int leftMotorPos;
extern unsigned int prevLeftMotorPos;
extern volatile unsigned int RightMotorPos;
extern unsigned int prevRightMotorPos;

extern boolean leftASet;
extern boolean leftBSet;
extern boolean rightASet;
extern boolean rightBSet;

void LeftEncoderIntHandler();
void RightEncoderIntHandler();
void InitDriveEncoders();

#endif