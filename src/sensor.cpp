#include "sensor.h"
#include "circularBuf.h"

SX1509 io;  // Port Expander

/**
 * TOF Data
 */
// The Arduino pin connected to the XSHUT pin of each sensor
const uint8_t xShutPinsL0[8] = {0, 1};
const uint8_t xShutPinsL1[8] = {};
VL53L0X sensorsL0[NUM_TOF_L0];
VL53L1X sensorsL1[NUM_TOF_L1];

circularBuf_t sensorL0Data[NUM_TOF_L0];
circularBuf_t sensorL1Data[NUM_TOF_L1];

void initCircularBuffers(circularBuf_t *buffers)
{
    for (uint32_t i = 0; i < NUM_TOF_L0; i++) {
        // Initialize each buffer in the array with the given size
        circularBufTInit(&buffers[i], CIRCULAR_BUF_SIZE);
    }

    for (uint32_t i = 0; i < NUM_TOF_L1; i++) {
        // Initialize each buffer in the array with the given size
        circularBufTInit(&buffers[i], CIRCULAR_BUF_SIZE);
    }
}

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
 * Initialise the IO board, I2C bus and the sensor
 */
void InitSensors()
{
    InitIOExpander();
    InitTOFL0();
    InitTOFL1();
    InitLimitSwitch();
}

/**
 * Initialise the SX1509 IO Expander
 */
void InitIOExpander()
{
    Wire.begin();
    Wire.setClock(400000);
    io.begin(TOF_CONTROL_ADDRESS);
    io.begin(AIO_ADDRESS);
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

void ReadTOFL0()
{
    for (uint8_t i = 0; i < NUM_TOF_L0; i++) {
        // Read sensor data
        uint16_t sensorValue = sensorsL0[i].readRangeContinuousMillimeters();
        
        // Write sensor data to circular buffer
        circularBufTWrite(&sensorL0Data[i], sensorValue);
        
        // Calculate the average from the circular buffer
        uint32_t avg = findBufAvg(&sensorL0Data[i]);
        
        // Print the average value
        Serial.print("Sensor L0 ");
        Serial.print(i);
        Serial.print(" Avg: ");
        Serial.print(avg);
        
        if (sensorsL0[i].timeoutOccurred()) {
            Serial.print(" TIMEOUT");
        }
        Serial.print('\t');
    }
    Serial.println();
}

void ReadTOFL1()
{
    for (uint8_t i = 0; i < NUM_TOF_L1; i++) {
        // Read sensor data
        uint16_t sensorValue = sensorsL1[i].readRangeContinuousMillimeters();
        
        // Write sensor data to circular buffer
        circularBufTWrite(&sensorL1Data[i], sensorValue);
        
        // Calculate the average from the circular buffer
        uint32_t avg = findBufAvg(&sensorL1Data[i]);
        
        // Print the average value
        Serial.print("Sensor L1 ");
    }
}


VL53L0X *returnL0()
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
 * Returns reading from IR sensor A
 */
int IRValueA()
{
    return analogRead(IR_ADDRESS_A);
}

/**
 * Returns reading from IR sensor B
 */
int IRValueB()
{
    return analogRead(IR_ADDRESS_B);
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

Colour_t GetColour()
{
    uint16_t red, green, blue, clear;
    Colour_t colour;

    colourSensor.setInterrupt(false);
    delay(60);
    colourSensor.getRawData(&red, &green, &blue, &clear);
    colourSensor.setIntegrationTime(true);

    colour.R = red;
    colour.G = blue;
    colour.B = green;
    colour.C = clear;

    return colour;
}
