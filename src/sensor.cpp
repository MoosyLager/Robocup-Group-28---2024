#include "sensor.h"

SX1509 io;  // Port Expander

/**
 * IMU Data
 */
Adafruit_BNO055 bno = Adafruit_BNO055(IMU_ID, IMU_ADDRESS, &IMU_WIRE);
int8_t boardTemp;
sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;

/**
 * Colour Sensor Data
 */
Adafruit_TCS34725 colourSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

/**
 * TOF Data
 */
// The Arduino pin connected to the XSHUT pin of each sensor
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
        uint16_t sensorValue = sensorsL0[i].readRangeContinuousMillimeters();

        // Write sensor data to circular buffer
        CircBuffWrite(&sensorL0Data[i], sensorValue);

        if ( sensorsL0[i].timeoutOccurred() ) {
            Serial.print(" TIMEOUT");
        }
        Serial.print('\t');
    }
}

/**
 * Read the value from each TOF sensor and add the reading to its circular buffer
 */
void UpdateTOFL1()
{
    for ( uint8_t i = 0; i < NUM_TOF_L1; i++ ) {
        // Read sensor data
        uint16_t sensorValue = sensorsL1[i].readRangeContinuousMillimeters();

        // Write sensor data to circular buffer
        CircBuffWrite(&sensorL1Data[i], sensorValue);

        if ( sensorsL0[i].timeoutOccurred() ) {
            Serial.print(" TIMEOUT");
        }
        Serial.print('\t');
    }
}

/**
 * Get the average TOF value from L0 Top Left
 */
uint16_t GetL0TL()
{
    return CalculateBufferMean(&sensorL0Data[0]);
}

/**
 * Get the average TOF value from L0 Top Right
 */
uint16_t GetL0TR()
{
    return CalculateBufferMean(&sensorL0Data[1]);
}

/**
 * Get the average TOF value from L0 Bottom Left
 */
uint16_t GetL0BL()
{
    return CalculateBufferMean(&sensorL0Data[2]);
}

/**
 * Get the average TOF value from L0 Bottom Right
 */
uint16_t GetL0BR()
{
    return CalculateBufferMean(&sensorL0Data[3]);
}

/**
 * Get the average TOF value from L1 Top Left
 */
uint16_t GetL1TL()
{
    return CalculateBufferMean(&sensorL1Data[0]);
}

/**
 * Get the average TOF value from L0 Top Right
 */
uint16_t GetL1TR()
{
    return CalculateBufferMean(&sensorL1Data[1]);
}

/**
 * Get the average TOF value from L0 Bottom Left
 */
uint16_t GetL1BL()
{
    return CalculateBufferMean(&sensorL1Data[2]);
}

/**
 * Get the average TOF value from L0 Bottom Right
 */
uint16_t GetL1BR()
{
    return CalculateBufferMean(&sensorL1Data[3]);
}

/**
 * Returns 0 if the collector is at the reference position
 */
int CollectorPosition()
{
    return io.digitalRead(AIO_0);
}

/**
 * Returns 0 if the ramp is at the zero position
 */
int RampPosition()
{
    return io.digitalRead(AIO_1);
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
    boardTemp = bno.getTemp();
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
 * Initialise the circular buffers for the TOF sensor data
 */
void InitCircularBuffers()
{
    for ( int i = 0; i < NUM_TOF_L0; i++ ) {
        CircBuffInit(&sensorL0Data[i], CIRCULAR_BUF_SIZE);
    }
    for ( int i = 0; i < NUM_TOF_L1; i++ ) {
        CircBuffInit(&sensorL1Data[i], CIRCULAR_BUF_SIZE);
    }
}


/**
 * Initialise the SX1509 IO Expander
 */
void InitIOExpander()
{
    io.begin(TOF_CONTROL_ADDRESS);
    Wire.begin();
    Wire.setClock(400000);
    // io.begin(AIO_ADDRESS);
}

/**
 * Initialise any limit switches
 */
void InitLimitSwitch()
{
    io.pinMode(AIO_0, INPUT);
    io.pinMode(AIO_1, INPUT);
}

/**
 * Initialise all connected VL53L0X Sensors
 */
void InitTOFL0()
{
    // Disable/reset all sensors by driving their XSHUT pins low.
    for ( uint8_t i = 0; i < NUM_TOF_L0; i++ ) {
        io.pinMode(xShutPinsL0[i], OUTPUT);
        io.digitalWrite(xShutPinsL0[i], LOW);
    }

    // Enable, initialise, and start each sensor
    for ( uint8_t i = 0; i < NUM_TOF_L0; i++ ) {
        // Stop driving this sensor's XSHUT low. This should allow the carrier
        // board to pull it high. (We do NOT want to drive XSHUT high since it is
        // not level shifted.) Then wait a bit for the sensor to start up.
        // pinMode(xshutPins[i], INPUT);
        io.digitalWrite(xShutPinsL0[i], HIGH);
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
}

void InitTOF()
{
    // Disable/reset all sensors by driving their XSHUT pins low.
    for ( uint8_t i = 0; i < NUM_TOF_L0; i++ ) {
        io.pinMode(xShutPinsL0[i], OUTPUT);
        io.digitalWrite(xShutPinsL0[i], LOW);
    }

    // Disable/reset all sensors by driving their XSHUT pins low.
    for ( uint8_t i = 0; i < NUM_TOF_L1; i++ ) {
        io.pinMode(xShutPinsL1[i], OUTPUT);
        io.digitalWrite(xShutPinsL1[i], LOW);
    }

    // Enable, initialise, and start each sensor
    for ( uint8_t i = 0; i < NUM_TOF_L0; i++ ) {
        // Stop driving this sensor's XSHUT low. This should allow the carrier
        // board to pull it high. (We do NOT want to drive XSHUT high since it is
        // not level shifted.) Then wait a bit for the sensor to start up.
        // pinMode(xshutPins[i], INPUT);
        io.digitalWrite(xShutPinsL0[i], HIGH);
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
        io.digitalWrite(xShutPinsL1[i], HIGH);
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

        sensorsL1[i].startContinuous(50);
    }
}

/**
 * Initialise all connected VL53L1X Sensors
 */
void InitTOFL1()
{
    // Disable/reset all sensors by driving their XSHUT pins low.
    for ( uint8_t i = 0; i < NUM_TOF_L1; i++ ) {
        io.pinMode(xShutPinsL1[i], OUTPUT);
        io.digitalWrite(xShutPinsL1[i], LOW);
    }

    // Enable, initialise, and start each sensor
    for ( uint8_t i = 0; i < NUM_TOF_L1; i++ ) {
        // Stop driving this sensor's XSHUT low. This should allow the carrier
        // board to pull it high. (We do NOT want to drive XSHUT high since it is
        // not level shifted.) Then wait a bit for the sensor to start up.
        // pinMode(xshutPins[i], INPUT);
        io.digitalWrite(xShutPinsL1[i], HIGH);
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
 * Initialise the IO board, I2C bus, circular buffers, and all sensors
 */
void InitSensors()
{
    InitIOExpander();
    InitCircularBuffers();
    InitTOFL0();
    InitTOFL1();
    InitLimitSwitch();
}

/**
 * Check minimum distance for the avoidance state
 */
bool CheckAvoidance(void)
{
    for (uint8_t i = 0; i < NUM_TOF_L0; i++) {
        uint16_t range = CalculateBufferMean(&sensorL0Data[i]);
        if (range < AVOIDANCE_THRESHOLD) {
            return true;
        }
    }
    for (uint8_t i = 0; i < NUM_TOF_L1; i++) {
        uint16_t range = CalculateBufferMean(&sensorL1Data[i]);
        if (range < AVOIDANCE_THRESHOLD) {
            return true;
        }
    }
    return false;
}

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
bool detectedFarRight(void)
{
    uint16_t top = GetL0TL();
    uint16_t bottom = GetL0BL();
    return detectedByPercentageDifference(top, bottom);
}

bool detectedFarLeft(void)
{
    uint16_t top = GetL0TR();
    uint16_t bottom = GetL0BR();
    return detectedByPercentageDifference(top, bottom);
}

bool detectedCentreRight(void)
{
    uint16_t top = GetL1TL();
    uint16_t bottom = GetL1BL();
    return detectedByAbsoluteDifference(top, bottom);
}

bool detectedCentreLeft(void)
{
    uint16_t top = GetL1TR();
    uint16_t bottom = GetL1BR();
    return detectedByAbsoluteDifference(top, bottom);
}

bool weightDetected(void)
{
    return (detectedFarLeft() || detectedFarRight() || detectedCentreLeft() || detectedCentreRight());
}



