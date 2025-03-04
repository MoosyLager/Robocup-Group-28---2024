#include "collection.h"
#include "encoder.h"
#include "motor.h"
#include "searchAlgorithm.h"
#include "sensor.h"
#include <Arduino.h>
#include <elapsedMillis.h>

elapsedMillis updateFSMTimer;
elapsedMillis updatePWMTimer;
elapsedMillis updateSensorsTimer;
elapsedMillis buttonTimer;
bool notSet = true;
RobotFSM fsm;

// elapsedMillis timer;

void setup()
{
    Serial.begin(9600);
    InitSensors();
    InitMotors();
    InitEncoders();
    initializeRobotFSM(&fsm);

    for ( int i = 0; i < 2 * CIRCULAR_BUF_SIZE; i++ ) {
        UpdateTOFL0();
        UpdateTOFL1();
    }

    while (notSet) {
        Serial.println("Calibrating IMU");
        UpdateIMU();
        float heading = GetOrientationYaw();
        if (heading != 0) {
            initialHeading = heading;
            targetHeading = initialHeading - 180;
            fsm.targetHeading = truncateHeading(targetHeading);
            notSet = false;
        }
    }

    CalibrateCollectionSystem(&fsm);

    Serial.println("Setup complete");
}

void loop()
{
    if ( BlueButtonState() && buttonTimer > 2500 && !pwmOn ) {
        pwmOn = true;
    }

    if ( updateFSMTimer > 37 ) {
        processFSM(&fsm);
        updateFSMTimer = 0;
    }

    if ( updatePWMTimer > 31 && pwmOn ) {
        PIDMotorControl(&leftMotor);
        PIDMotorControl(&rightMotor);
        updatePWMTimer = 0;
    }

    if ( updateSensorsTimer > 51 ) {
        UpdateTOFL0();
        UpdateTOFL1();
        UpdateIMU();
        updateSensorsTimer = 0;
    }
}
