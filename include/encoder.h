#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include <stdbool.h>
#include <stdint.h>

enum DriveEncoderPinAssignments {
    encoder1PinA = 2,
    encoder1PinB = 3,
    encoder2PinA = 4,
    encoder2PinB = 5,
};

extern volatile unsigned int encoderPos1;
extern unsigned int lastReportedPos1;
extern volatile unsigned int encoderPos2;
extern unsigned int lastReportedPos2;

extern boolean A_set1;
extern boolean B_set1;
extern boolean A_set2;
extern boolean B_set2;

void doEncoder1A();
void doEncoder2A();
void InitDriveEncoders();

#endif