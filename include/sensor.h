#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>
#include <stdint.h>

#include <SparkFunSX1509.h>
#include <VL53L0X.h>
#include <VL53L1X.h>
#include <Wire.h>

// Peripheral Addresses
#define SX1509_ADDRESS            0x3F
#define VL53L0X_ADDRESS_START     0x30
#define VL53L1X_ADDRESS_START     0x35
#define COLLECTION_SWITCH_ADDRESS 0x3E
#define IR_ADDRESS_A              A9
#define IR_ADDRESS_B              A8

// 2 pin digital
#define AIO_0 0
#define AIO_1 1
#define AIO_2 2
#define AIO_3 3
#define AIO_4 4
#define AIO_5 5
#define AIO_6 6

// 3 pin digital
#define AIO_8  8
#define AIO_9  9
#define AIO_10 10
#define AIO_11 11
#define AIO_12 12
#define AIO_13 13
#define AIO_14 14
#define AIO_15 15

// Number of each type of TOF sensor
#define NUM_TOF_L0 2
#define NUM_TOF_L1 0

void InitSensors();
void InitTOFL0();
void InitTOFL1();
void InitLimit();

int IRValueA();
int IRValueB();

int CollectorPosition();

#endif