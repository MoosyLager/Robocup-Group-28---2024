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
#define COLOUR_SENSOR_INTEGRATION_TIME_MS 300
#define COLOUR_SENSOR_UPDATE_PERIOD       20
#define COLOUR_THRESHOLD                  2

/**
 * Wrapper to hold RGBA colour information
 */
struct Colour_t {
    uint16_t R;
    uint16_t G;
    uint16_t B;
    uint16_t C;

    Colour_t()
    {
        R = 0;
        G = 0;
        B = 0;
        C = 0;
    }

    Colour_t(uint16_t r, uint16_t g, uint16_t b, uint16_t c)
    {
        R = r;
        G = g;
        B = b;
        C = c;
    }
};

typedef enum {
    HOME,
    ARENA,
    ENEMY
} ColourType_t;

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
#define AIO_9       9  // Inductive Sensor
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

#define TOF_L1_ROI_X          16   // Width of the ROI (Max 16)
#define TOF_L1_ROI_Y          4    // Height of the ROI (Max 16)
#define TOF_L1_ROI_CENTRE     199  // SPAD number of the centre (refer to datasheet - default 199)

/**
 * IMU
 */
#define IMU_SAMPLE_RATE_MS 100
#define IMU_ID             55
#define IMU_ADDRESS        0x28
#define IMU_WIRE           Wire1
#define IMU_BUF_SIZE       10  // Adjust the size as needed for smoothing

/**
 * Sensor thresholds
 */
#define AVOIDANCE_THRESHOLD  65
#define LONG_RANGE_THRESHOLD 1500
#define SHORT_RANGE_THRESHOLD 500
#define COLLECTION_THRESHOLD 450 // To be tested
#define DIFFERENCE_PERCENTAGE 20//% difference between the two sensors
#define DIFFERENCE_ABSOLUTE 100 //mm difference between the two sensors
#define SHORT_RANGE_MULTIPLIER 1.5
#define CENTRAL_THRESHOLD 15
#define SIDE_BIAS_LEFT 40
#define SIDE_BIAS_RIGHT 40

typedef uint16_t (*DistanceFunction)();  // Function pointer type for distance functions
extern DistanceFunction distanceFunctions[];
extern DistanceFunction distanceFunctionsTop[];

extern sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;
extern Adafruit_BNO055 bno;
extern int8_t boardTemp;

extern VL53L0X sensorsL0[NUM_TOF_L0];
extern VL53L1X sensorsL1[NUM_TOF_L1];
extern CircBuff_t sensorL0Data[NUM_TOF_L0];
extern CircBuff_t sensorL1Data[NUM_TOF_L1];

uint16_t CalculateBufferMean(CircBuff_t *buffer);
void InitSensors();
void InitTOF();
void InitIOExpander();
void InitLimitSwitch();
void InitColourSensor();
void InitIMU();
void InitCircularBuffers();

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

 
float GetOrientationPitch();
float GetOrientationRoll();
float GetOrientationYaw();
float GetAccelerationSideways();
float GetAccelerationUpwards();
float GetAccelerationForward();


Colour_t GetColour();
int CollectorPosition();
int RampPosition();

uint8_t BlueButtonState();
uint8_t ReadInductiveSensor();

bool CheckAvoidance(void);
bool detectedByPercentageDifference(uint16_t top, uint16_t bottom);
bool detectedByAbsoluteDifference(uint16_t top, uint16_t bottom);
bool detectedFarRight(void);
bool detectedFarLeft(void);
bool detectedCentreRight(void);
bool detectedCentreLeft(void);
bool weightDetected(void);
bool checkTargetHeading(int targetHeading);
bool isLinedUp(int rangeLeft, int rangeRight);
int truncateHeading(int heading);


#endif // SENSOR_H