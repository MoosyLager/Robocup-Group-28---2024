#ifndef IMU_H
#define IMU_H

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>

#define IMU_SAMPLE_RATE_MS 100
#define IMU_ID             55
#define IMU_ADDRESS        0x28

extern sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;
extern Adafruit_BNO055 bno;
extern int8_t boardTemp;

void InitIMU();
void UpdateIMU();

#endif