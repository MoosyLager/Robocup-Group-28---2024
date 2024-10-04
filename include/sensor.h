#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>
#include <stdint.h>

#include "circularBuf.h"
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TCS34725.h>
#include <SparkFunSX1509.h>
#include <VL53L0X.h>
#include <VL53L1X.h>
#include <Wire.h>
#include <utility/imumaths.h>

/**
 * Colour Sensor
 */
#define COLOUR_SENSOR_ADDRESS 0x29
#define COLOUR_SENSOR_WIRE    Wire1

/**
 * Wrapper to hold RGBA colour information
 */
typedef struct {
    uint16_t R;
    uint16_t G;
    uint16_t B;
    uint16_t C;
} Colour_t;

extern Adafruit_TCS34725 colourSensor;


/**
 * IR Sensor
 */
#define IR_ADDRESS_A A9
#define IR_ADDRESS_B A8

/**
 * Port Expander AIO
 */
#define AIO_ADDRESS 0x3E  // Port expander address for AIO digital pins
#define AIO_0       0     // Collector Position Limit Switch
#define AIO_1       1     // Ramp Position Limit Switch
#define AIO_2       2
#define AIO_3       3
#define AIO_4       4
#define AIO_5       5
#define AIO_6       6
#define AIO_8       8  // Blue button
#define AIO_9       9
#define AIO_10      10
#define AIO_11      11
#define AIO_12      12
#define AIO_13      13
#define AIO_14      14
#define AIO_15      15

/**
 * TOF Sensor
 */
#define TOF_CONTROL_ADDRESS   0x3F  // Port expander address for TOF control pins
#define VL53L0X_ADDRESS_START 0x30  // Start of VL53L0X address space
#define VL53L1X_ADDRESS_START 0x35  // Start of VL53L1X address space
#define NUM_TOF_L0            4     // Number of VL53L0X TOF sensors
#define NUM_TOF_L1            4     // Number of VL53L1X TOF sensors

#define TOF_XSHUT_L0_1        0
#define TOF_XSHUT_L0_2        1
#define TOF_XSHUT_L0_3        2
#define TOF_XSHUT_L0_4        3

#define TOF_XSHUT_L1_1        4
#define TOF_XSHUT_L1_2        5
#define TOF_XSHUT_L1_3        6
#define TOF_XSHUT_L1_4        7

/**
 * IMU
 */
#define IMU_SAMPLE_RATE_MS 100
#define IMU_ID             55
#define IMU_ADDRESS        0x28
#define IMU_WIRE           Wire1

extern sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;
extern Adafruit_BNO055 bno;
extern int8_t boardTempIMU;

void InitSensors();
void InitTOF();
void InitIOExpander();
void InitLimitSwitch();
void InitColourSensor();
void InitIMU();

void UpdateIMU();
void UpdateTOFL0();
void UpdateTOFL1();

uint16_t GetL0TL();
uint16_t GetL0TR();
uint16_t GetL0BL();
uint16_t GetL0BR();
uint16_t GetL1TL();
uint16_t GetL1TR();
uint16_t GetL1BL();
uint16_t GetL1BR();

Colour_t GetColour();
int CollectorPosition();
int RampPosition();

uint8_t BlueButtonState();

#endif