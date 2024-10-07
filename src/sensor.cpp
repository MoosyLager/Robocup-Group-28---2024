#include "sensor.h"
#include <elapsedMillis.h>
#include "circularBuf.h"

#define IMU_BUF_SIZE 10  // Adjust the size as needed for smoothing

CircBuffFloat_t imuBufferForward;
CircBuffFloat_t imuBufferSideways;
CircBuffFloat_t imuBufferVertical;

CircBuffFloat_t imuBufferYaw;
CircBuffFloat_t imuBufferPitch;
CircBuffFloat_t imuBufferRoll;


SX1509 ioTOF;  // TOF Control - Port Expander
SX1509 ioAIO;  // AIO control - Port Expander

SX1509 io;  // Port Expander
elapsedMicros updateSensorsTimer1 = 0;
/**
 * IMU Data
 */
Adafruit_BNO055 bno = Adafruit_BNO055(IMU_ID, IMU_ADDRESS, &IMU_WIRE);
int8_t boardTempIMU;
sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;

/**
 * Colour Sensor Data
 */
Adafruit_TCS34725 colourSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

/**
 * TOF Data
 */
const uint8_t xShutPinsL0[8] = {TOF_XSHUT_L0_1, TOF_XSHUT_L0_2, TOF_XSHUT_L0_3, TOF_XSHUT_L0_4};
const uint8_t xShutPinsL1[8] = {TOF_XSHUT_L1_1, TOF_XSHUT_L1_2, TOF_XSHUT_L1_3, TOF_XSHUT_L1_4};
VL53L0X sensorsL0[NUM_TOF_L0];
VL53L1X sensorsL1[NUM_TOF_L1];
CircBuff_t sensorL0Data[NUM_TOF_L0];
CircBuff_t sensorL1Data[NUM_TOF_L1];

/**
 * Given a circular buffer, returns the mean
 */
uint16_t CalculateBufferMean(CircBuff_t *buffer)
{
    uint32_t sum = 0;
    for ( int i = 0; i < CIRCULAR_BUF_SIZE; i++ ) {
        sum = sum + CircBufRead(buffer);
    }
    uint16_t avg = ((2 * sum + CIRCULAR_BUF_SIZE) / 2 / CIRCULAR_BUF_SIZE);
    return avg;
}

/**
 * Read the value from each TOF sensor and add the reading to its circular buffer
 */
void UpdateTOFL0()
{
    for ( uint8_t i = 0; i < NUM_TOF_L0; i++ ) {
        // Read sensor data
        uint16_t sensorValue = sensorsL0[i].readRangeContinuousMillimeters() * SHORT_RANGE_MULTIPLIER;

        // Write sensor data to circular buffer
        CircBuffWrite(&sensorL0Data[i], sensorValue);

        if ( sensorsL0[i].timeoutOccurred() ) {
            Serial.print(" TIMEOUT");
        }
    }
}

/**
 * Read the value from each TOF sensor and add the reading to its circular buffer
 */
void UpdateTOFL1()
{
    for ( uint8_t i = 0; i < NUM_TOF_L1; i++ ) {
        // Read sensor data
        if ( sensorsL1[i].dataReady() ) {
            uint16_t sensorValue = sensorsL1[i].read(false);
            // Write sensor data to circular buffer
            CircBuffWrite(&sensorL1Data[i], sensorValue);
        }
        if ( sensorsL1[i].timeoutOccurred() ) {
            Serial.print(" TIMEOUT");
        }
    }
}

/**
 * Get the average TOF value from L0 Top Left
 */
uint16_t GetL0TL()
{
    uint16_t mean = CalculateBufferMean(&sensorL0Data[2]);
    if (mean > SHORT_RANGE_THRESHOLD) {
        mean =  SHORT_RANGE_THRESHOLD;
    }
    return mean;

}

/**
 * Get the average TOF value from L0 Top Right
 */
uint16_t GetL0TR()
{
    uint16_t mean = CalculateBufferMean(&sensorL0Data[0]);
    if (mean > SHORT_RANGE_THRESHOLD) {
        mean =  SHORT_RANGE_THRESHOLD;
    }
    return mean;
}

/**
 * Get the average TOF value from L0 Bottom Left
 */
uint16_t GetL0BL()
{
    uint16_t mean = CalculateBufferMean(&sensorL0Data[3]);
    if (mean > SHORT_RANGE_THRESHOLD) {
        mean =  SHORT_RANGE_THRESHOLD;
    }
    return mean;
}

/**
 * Get the average TOF value from L0 Bottom Right
 */
uint16_t GetL0BR()
{
    uint16_t mean = CalculateBufferMean(&sensorL0Data[1]);
    if (mean > SHORT_RANGE_THRESHOLD) {
        mean =  SHORT_RANGE_THRESHOLD;
    }
    return mean;
}

/**
 * Get the average TOF value from L1 Top Left
 */
uint16_t GetL1TL()
{
    uint16_t mean = CalculateBufferMean(&sensorL1Data[2]);
    if (mean > LONG_RANGE_THRESHOLD) {
        mean =  LONG_RANGE_THRESHOLD;
    }
    return mean;
}

/**
 * Get the average TOF value from L0 Top Right
 */
uint16_t GetL1TR()
{
    uint16_t mean = CalculateBufferMean(&sensorL1Data[0]);
    if (mean > LONG_RANGE_THRESHOLD) {
        mean =  LONG_RANGE_THRESHOLD;
    }
    return mean;
}

/**
 * Get the average TOF value from L1 Bottom Left
 */
uint16_t GetL1BL()
{
    uint16_t mean = CalculateBufferMean(&sensorL1Data[3]);
    if (mean > LONG_RANGE_THRESHOLD) {
        mean =  LONG_RANGE_THRESHOLD;
    }
    return mean;
}

/**
 * Get the average TOF value from L0 Bottom Right
 */
uint16_t GetL1BR()
{
    uint16_t mean =  CalculateBufferMean(&sensorL1Data[1]);
    if (mean > LONG_RANGE_THRESHOLD) {
        mean =  LONG_RANGE_THRESHOLD;
    }
    return mean;
}

/**
 * Returns 0 if the collector is at the reference position
 */
int CollectorPosition()
{
    return ioAIO.digitalRead(AIO_0);
}

/**
 * Returns 0 if the ramp is at the zero position
 */
int RampPosition()
{
    return ioAIO.digitalRead(AIO_1);
}

/**
 * Update IMU data
 */
void UpdateIMU()
{
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
    boardTempIMU = bno.getTemp();
}

/**
 * Returns the current X-rotation from the IMU
 */
float GetOrientationPitch()
{
    return orientationData.orientation.z;
}

/**
 * Returns the current Y-rotation from the IMU
 */
float GetOrientationRoll()
{
    return orientationData.orientation.y;
}

/**
 * Returns the current Z-rotation from the IMU
 */
float GetOrientationYaw()
{
    return orientationData.orientation.x;
}

float GetAccelerationForward()
{
    return linearAccelData.acceleration.y;
}

float GetAccelerationSideways()
{
    return linearAccelData.acceleration.x;
}

float GetAccelerationUpwards()
{
    return linearAccelData.acceleration.z;
}



/**
 * Reads the colours of the suface under the colour sensor
 */
Colour_t GetColour()
{
    uint16_t red, green, blue, clear;
    Colour_t colour;

    colourSensor.setInterrupt(false);
    delay(60);
    colourSensor.getRawData(&red, &green, &blue, &clear);
    colourSensor.setInterrupt(true);

    colour.R = red;
    colour.G = blue;
    colour.B = green;
    colour.C = clear;

    return colour;
}

/**
 * Initialise the circular buffers for the TOF sensor data and the IMU data
 */
void InitCircularBuffers()
{
    for ( int i = 0; i < NUM_TOF_L0; i++ ) {
        CircBuffInit(&sensorL0Data[i], CIRCULAR_BUF_SIZE);
    }
    for ( int i = 0; i < NUM_TOF_L1; i++ ) {
        CircBuffInit(&sensorL1Data[i], CIRCULAR_BUF_SIZE);
    }
    Serial.println("TOF Circular Buffers Initialised");
}


/**
 * Initialise the SX1509 IO Expander
 */
void InitIOExpander()
{
    Wire.begin();
    Wire.setClock(400000);
    ioTOF.begin(TOF_CONTROL_ADDRESS);
    ioAIO.begin(AIO_ADDRESS);
}

/**
 * Initialise any limit switches
 */
void InitLimitSwitch()
{
    ioAIO.pinMode(AIO_0, INPUT);
    ioAIO.pinMode(AIO_1, INPUT);
}

/**
 * Initialises all connected TOF sensors
 * Both VL53L0X and VL53L1X
 */
void InitTOF()
{
    // Disable/reset all sensors by driving their XSHUT pins low.
    for ( uint8_t i = 0; i < NUM_TOF_L0; i++ ) {
        ioTOF.pinMode(xShutPinsL0[i], OUTPUT);
        ioTOF.digitalWrite(xShutPinsL0[i], LOW);
    }

    // Disable/reset all sensors by driving their XSHUT pins low.
    for ( uint8_t i = 0; i < NUM_TOF_L1; i++ ) {
        ioTOF.pinMode(xShutPinsL1[i], OUTPUT);
        ioTOF.digitalWrite(xShutPinsL1[i], LOW);
    }

    // Enable, initialise, and start each sensor
    for ( uint8_t i = 0; i < NUM_TOF_L0; i++ ) {
        // Stop driving this sensor's XSHUT low. This should allow the carrier
        // board to pull it high. (We do NOT want to drive XSHUT high since it is
        // not level shifted.) Then wait a bit for the sensor to start up.
        // pinMode(xshutPins[i], INPUT);
        ioTOF.digitalWrite(xShutPinsL0[i], HIGH);
        delay(10);

        sensorsL0[i].setTimeout(500);
        if ( !sensorsL0[i].init() ) {
            Serial.print("Failed to detect and initialise sensor L0 ");
            Serial.print(i);
            while ( 1 )
                ;
        }

        // Each sensor must have its address changed to a unique value other than
        // the default of 0x29 (except for the last one, which could be left at
        // the default). To make it simple, we'll just count up from 0x2A.
        sensorsL0[i].setAddress(VL53L0X_ADDRESS_START + i);

        sensorsL0[i].startContinuous(50);
    }

    // Enable, initialise, and start each sensor
    for ( uint8_t i = 0; i < NUM_TOF_L1; i++ ) {
        // Stop driving this sensor's XSHUT low. This should allow the carrier
        // board to pull it high. (We do NOT want to drive XSHUT high since it is
        // not level shifted.) Then wait a bit for the sensor to start up.
        // pinMode(xshutPins[i], INPUT);
        ioTOF.digitalWrite(xShutPinsL1[i], HIGH);
        delay(10);

        sensorsL1[i].setTimeout(500);
        if ( !sensorsL1[i].init() ) {
            Serial.print("Failed to detect and initialise sensor L1 ");
            Serial.print(i);
            while ( 1 )
                ;
        }

        // Each sensor must have its address changed to a unique value other than
        // the default of 0x29 (except for the last one, which could be left at
        // the default). To make it simple, we'll just count up from 0x2A.
        sensorsL1[i].setAddress(VL53L1X_ADDRESS_START + i);

        // Set ROI size and centre on each sensor
        sensorsL1[i].setROISize(TOF_L1_ROI_X, TOF_L1_ROI_Y);
        sensorsL1[i].setROICenter(TOF_L1_ROI_CENTRE);

        sensorsL1[i].startContinuous(50);
    }
}

/**
 * Initialises the IMU
 */
void InitIMU()
{
    if ( !bno.begin() ) {
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    }
}

/**
 * Initialise the colour sensor
 */
void InitColourSensor()
{
    if ( !colourSensor.begin(COLOUR_SENSOR_ADDRESS, &COLOUR_SENSOR_WIRE) ) {
        Serial.println("COLOUR SENSOR NOT DETECTED");
    }
}

/**
 * Returns 1 if the blue button is pressed and 0 otherwise
 */
uint8_t BlueButtonState()
{
    return ioAIO.digitalRead(AIO_8);
}

/**
 * Initialise the IO board, I2C bus, circular buffers, and all sensors
 */
void InitSensors()
{
    InitIOExpander();

    InitCircularBuffers();

    InitTOF();
    InitColourSensor();
    InitLimitSwitch();
    InitIMU();

    ioAIO.pinMode(AIO_8, INPUT);  // Blue Button
}

/**
 * Check minimum distance for the avoidance state
 */
DistanceFunction distanceFunctions[] = {
    GetL0TL,
    GetL0TR,
    GetL0BL,
    GetL0BR,
    GetL1TL,
    GetL1TR,
    GetL1BL,
    GetL1BR
};


/**
 * Check the detection ranges for each double-sensor array
 */
bool detectedByPercentageDifference(uint16_t top, uint16_t bottom)
{
    int16_t diff = top - bottom;
    return (diff * 100) > (top * DIFFERENCE_PERCENTAGE);
}

bool detectedByAbsoluteDifference(uint16_t top, uint16_t bottom)
{
    int16_t diff = top - bottom;
    return ((diff) > DIFFERENCE_ABSOLUTE);
}

/**
 * Array checking functions
 */
bool detectedFarLeft(void)
{
    uint16_t top = GetL0TL();
    uint16_t bottom = GetL0BL();
    Serial.print("Right Top 0: ");
    Serial.print(top);
    Serial.print(" Bottom: ");
    Serial.print(bottom);
    bool detected = detectedByPercentageDifference(top, bottom);
    Serial.print(" Far Left Detected: ");
    Serial.println(detected);
    return detectedByPercentageDifference(top, bottom);
}

bool detectedFarRight(void)
{
    uint16_t top = GetL0TR();
    uint16_t bottom = GetL0BR();
    Serial.print("Left Top 0: ");
    Serial.print(top);
    Serial.print(" Bottom: ");
    Serial.print(bottom);
    bool detected = detectedByPercentageDifference(top, bottom);
    Serial.print(" Far Right Detected: ");
    Serial.println(detected);
    return detectedByPercentageDifference(top, bottom);
}

bool detectedCentreRight(void)
{
    uint16_t top = GetL1TL();
    uint16_t bottom = GetL1BL();
    Serial.print("Right Top 1: ");
    Serial.print(top);
    Serial.print(" Bottom: ");
    Serial.print(bottom);
    bool detected = detectedByAbsoluteDifference(top, bottom);
    Serial.print(" Centre Right Detected: ");
    Serial.println(detected);
    return detectedByAbsoluteDifference(top, bottom);
}

bool detectedCentreLeft(void)
{
    uint16_t top = GetL1TR();
    uint16_t bottom = GetL1BR();
    Serial.print("Left Top 1: ");
    Serial.print(top);
    Serial.print(" Bottom: ");
    Serial.print(bottom);
    bool detected = detectedByAbsoluteDifference(top, bottom);
    Serial.print(" Centre Left Detected: ");
    Serial.println(detected);
    return detectedByAbsoluteDifference(top, bottom);
}

bool weightDetected(void)
{
    return (detectedFarLeft() || detectedFarRight() || detectedCentreLeft() || detectedCentreRight());
}

/**
 * Check if the robot is lined up with the weight
 * Works as expected
 */
bool checkTargetHeading(int targetHeading)
{
    int currentHeading = GetOrientationYaw();
    int error = targetHeading - currentHeading;
    if (error > 180) {
        error = error - 360;
    } else if (error < -180) {
        error = error + 360;
    }
    if (error < 5 && error > -5) {
        Serial.println("At target heading");
    }
    return (error < 5 && error > -5);
}

bool isLinedUp(int rangeLeft, int rangeRight) {
    return ((rangeLeft - rangeRight < CENTRAL_THRESHOLD) || (rangeRight - rangeLeft < CENTRAL_THRESHOLD));
}

int truncateHeading(int heading) {
    if (heading > 360) {
        heading = heading - 360;
    } else if (heading < 0) {
        heading = heading + 360;
    }
    return heading;
}