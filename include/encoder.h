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
    collectionMotorPinA = 32,
    collectionMotorPinB = 33
};

extern volatile unsigned int leftMotorPos;
extern unsigned int prevLeftMotorPos;
extern volatile unsigned int rightMotorPos;
extern unsigned int prevRightMotorPos;
extern volatile unsigned int collectionMotorPos;
extern unsigned int prevCollectionMotorPos;

extern boolean leftASet;
extern boolean leftBSet;
extern boolean rightASet;
extern boolean rightBSet;
extern boolean collectionASet;
extern boolean collectionBSet;

void LeftEncoderIntHandler();
void RightEncoderIntHandler();
void collectionEncoderIntHandler();
void InitDriveEncoders();

#endif