#include "imu.h"
#include <Wire.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(IMU_ID, IMU_ADDRESS, &IMU_WIRE);
int8_t boardTemp;
sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;

void InitIMU()
{
    if ( !bno.begin() ) {
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    }
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