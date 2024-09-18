#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include <stdbool.h>
#include <stdint.h>

enum DriveMotorPinAssignments {
    encoder1PinA = 2,
    encoder1PinB = 3,

    encoder2PinA = 4,
    encoder2PinB = 5,
};

extern volatile unsigned int encoderPos1;
extern unsigned int lastReportedPos1;
extern volatile unsigned int encoderPos2;
extern unsigned int lastReportedPos2;

void InitDriveEncoders();

#endif