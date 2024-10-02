#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>
#include <stdint.h>

#include <SparkFunSX1509.h>
#include <VL53L0X.h>
#include <VL53L1X.h>
#include <Wire.h>

#define TOF_CONTROL_ADDRESS   0x3F  // Port expander address for TOF control pins
#define AIO_ADDRESS           0x3E  // Port expander address for AIO digital pins

#define VL53L0X_ADDRESS_START 0x30  // Start of VL53L0X address space
#define VL53L1X_ADDRESS_START 0x35  // Start of VL53L1X address space

// IR addresses
#define IR_ADDRESS_A A9
#define IR_ADDRESS_B A8

// AIO - 2 pin
#define AIO_0 0
#define AIO_1 1
#define AIO_2 2
#define AIO_3 3
#define AIO_4 4
#define AIO_5 5
#define AIO_6 6

// AIO - 3 pin
#define AIO_8  8
#define AIO_9  9
#define AIO_10 10
#define AIO_11 11
#define AIO_12 12
#define AIO_13 13
#define AIO_14 14
#define AIO_15 15

#define NUM_TOF_L0 2  // Number of VL53L0X TOF sensors
#define NUM_TOF_L1 0  // Number of VL53L1X TOF sensors

void InitSensors();
void InitIOExpander();
void InitTOFL0();
void InitTOFL1();
void InitLimit();

int IRValueA();
int IRValueB();

int CollectorPosition();

#endif