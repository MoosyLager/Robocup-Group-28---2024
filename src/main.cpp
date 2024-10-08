#include "collection.h"
#include "encoder.h"
#include "motor.h"
#include "searchAlgorithm.h"
#include "sensor.h"
#include <Arduino.h>
#include <elapsedMillis.h>


RobotFSM fsm;
elapsedMicros timer;
elapsedMillis updateSensorsTimer = 0;
elapsedMillis updateFSM = 0;
elapsedMillis updatePWM = 0;

void setup()
{
    Serial.begin(9600);
    // delay(1000);
    InitSensors();
    InitMotors();
    InitEncoders();
    initializeRobotFSM(&fsm);
    delay(7000);
    Serial.println("Setup complete");
    // for ( int i = 0; i < 100; i++ ) {
    //     UpdateTOFL0();
    //     UpdateTOFL1();
    // }
}

void loop()
{
    // Serial.print("Centre Left: ");
    // Serial.print(detectedCentreLeft());
    // Serial.print(" Centre Right: ");
    // Serial.print(detectedCentreRight());
    // Serial.print(" Far Left: ");
    // Serial.print(detectedFarLeft());
    // Serial.print(" Far Right");
    // Serial.println(detectedFarRight());
    // UpdateTOFL0();
    // UpdateTOFL1();

    // Serial.print("L0TL: ");
    // Serial.print(GetL0TL());
    // Serial.print(" L0TR: ");
    // Serial.print(GetL0TR());
    // Serial.print(" L0BL: ");
    // Serial.print(GetL0BL());
    // Serial.print(" L0BR: ");
    // Serial.println(GetL0BR());
    // Serial.print("L1TL: ");
    // Serial.print(GetL1TL());
    // Serial.print(" L1TR: ");
    // Serial.print(GetL1TR());
    // Serial.print(" L1BL: ");
    // Serial.print(GetL1BL());
    // Serial.print(" L1BR: ");
    // Serial.println(GetL1BR());
    // UpdateIMU();
    // if (updateSensorsTimer > 50) {
    //     float acc = GetOrientationPitch();
    //     Serial.print("Pitch: ");
    //     Serial.print(acc);
    //     float acc2 = GetOrientationRoll();
    //     Serial.print(" Roll: ");
    //     Serial.print(acc2);
    //     float acc3 = GetOrientationYaw();
    //     Serial.print(" Yaw: ");
    //     Serial.print(acc3);
        
    //     if (checkTargetHeading(fsm.targetHeading)) {
    //         Serial.print("At target heading");
    //     }
    //     Serial.println();
    //     updateSensorsTimer = 0;
    // }

    if (updateFSM > 71) {
        processFSM(&fsm);
        updateFSM = 0;
        Serial.println("--------------------");
        // Serial.print("Current Heading: ");
        // Serial.print(GetOrientationYaw());
        // Serial.print(" Target Heading: ");
        // Serial.println(fsm.targetHeading);
    }

    // if (updatePWM > 51) {
    //     // UpdateMotors();
    //     updatePWM = 0;
    //     PIDMotorControl(&leftMotor);
    //     PIDMotorControl(&rightMotor);
    // }

    if (updateSensorsTimer > 31) {
        UpdateTOFL0();
        UpdateTOFL1();
        UpdateIMU();
        updateSensorsTimer = 0;
    }
}
