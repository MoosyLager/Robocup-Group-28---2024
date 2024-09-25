#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include <stdbool.h>
#include <stdint.h>

enum DriveEncoderPinAssignments {
    leftEncoderPinA = 2,
    leftEncoderPinB = 3,
    rightEncoderPinA = 4,
    rightEncoderPinB = 5,
};

enum CollectionMotorPinAssignments {
    collectionEncoderPinA = 32,
    collectionEncoderPinB = 33,
};

extern volatile unsigned int leftMotorPos;
extern unsigned int prevLeftMotorPos;
extern volatile unsigned int RightMotorPos;
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
void CollectionEncoderIntHandler();
void InitDriveEncoders();
void InitCollectionEncoder();

#endif