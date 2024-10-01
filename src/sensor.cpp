#include "sensor.h"

// The Arduino pin connected to the XSHUT pin of each sensor
const uint8_t xShutPinsL0[8] = {0, 1};
const uint8_t xShutPinsL1[8] = {};

SX1509 io;  // SX1509 object to use throughout
VL53L0X sensorsL0[NUM_TOF_L0];
VL53L1X sensorsL1[NUM_TOF_L1];

/**
 * Initialise the IO board, I2C bus and the sensor
 */
void InitSensors()
{
    Wire.begin();
    Wire.setClock(400000);
    io.begin(SX1509_ADDRESS);
    InitTOFL0();
    InitTOFL1();
    InitLimit();
}

/**
 * Initialise any limit switches
 */
void InitLimit()
{
    io.begin(COLLECTION_SWITCH_ADDRESS);
    io.pinMode(AIO_0, INPUT);
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
 * Returns the current state of the collector limit switch
 */
int CollectorPosition()
{
    return io.digitalRead(AIO_0);
}