#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>
#include <stdint.h>

#include <SparkFunSX1509.h>
#include <VL53L0X.h>
#include <VL53L1X.h>
#include <Wire.h>

#define SX1509_ADDRESS        0x3F
#define VL53L0X_ADDRESS_START 0x30
#define VL53L1X_ADDRESS_START 0x35

#define NUM_TOF_L0            2
#define NUM_TOF_L1            0

#endif