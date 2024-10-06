#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include <stdbool.h>
#include <stdint.h>

/**
 * Collector encoder data
 */
// #define COLLECTION_MOTOR_PULSE_PER_REV 11
// #define MOTOR_REV_PER_COLLECTION 2.75
#define COLLECTOR_TICKS_PER_REV 6675

/**
 * Pins connected to drive encoder phases
 */
enum DriveEncoderPinAssignments {
    leftEncoderPinA = 2,
    leftEncoderPinB = 3,
    rightEncoderPinA = 4,
    rightEncoderPinB = 5,
};

/**
 * Pins connected to collector encoder phases
 */
enum CollectionMotorPinAssignments {
    collectionEncoderPinA = 32,
    collectionEncoderPinB = 33,
};

extern volatile uint16_t PID_leftMotorPos;
extern volatile uint16_t PID_rightMotorPos;

extern const uint16_t Kp; 
extern const uint16_t Ki;
extern const uint16_t Kd;


extern volatile unsigned int leftMotorPos;
extern unsigned int prevLeftMotorPos;
extern volatile unsigned int rightMotorPos;
extern unsigned int prevRightMotorPos;

extern volatile int collectionMotorPos;
extern int prevCollectionMotorPos;

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
void InitEncoders();

#endif