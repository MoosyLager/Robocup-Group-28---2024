#include "collection.h"
#include "encoder.h"
#include "motor.h"
#include "sensor.h"
#include <Arduino.h>

#include <elapsedMillis.h>
elapsedMillis timer;

// elapsedMillis motorDelay;
// uint16_t motorSpeed = 1100;
// elapsedMillis motorMicros;

void setup()
{
    Serial.begin(9600);
    InitSensors();
    InitMotors();
    InitEncoders();

    // CalibrateCollector();
    // delay(500);
    CalibrateCollector();
}

void loop()
{
    if ( BlueButtonState() && timer > 1500 ) {
        timer = 0;
        ActuateCollector();
    }
    UpdateCollector();
    if ( prevCollectionMotorPos != collectionMotorPos ) {
        Serial.println(collectionMotorPos);
        prevCollectionMotorPos = collectionMotorPos;
    }
    // if ( BlueButtonState() && (timer > 1000) ) {
    //     timer = 0;
    //     ActuateCollector();
    // }
    // Serial.println(collectionMotorPos);
    // Serial.println(BlueButtonState());
    // Serial.println(CollectorPosition());
    // UpdateTOFL0();
    // Serial.println(GetL0BL());
    // ReadTOFL0();
    // for ( int i = 0; i < NUM_TOF_L0; i++ ) {
    //     Serial.print("Sensor ");
    //     Serial.print(i + 1);
    //     Serial.print(": ");
    //     Serial.print(sensorL0Data[i]);
    //     Serial.print(" ");
    // }

    // for ( int i = 0; i < NUM_TOF_L0 - 1; i++ ) {
    //     for ( int j = i + 1; j < NUM_TOF_L0; j++ ) {
    //         // Calculate percentage difference between sensors
    //         float difference = abs(sensorL0Data[i] - sensorL0Data[j]);
    //         float avgValue = (sensorL0Data[i] + sensorL0Data[j]) / 2.0;

    //         if ( avgValue > 0 && (difference / avgValue) >= 0.30 ) {
    //             Serial.print("Weight Detected!");
    //             break;  // Exit the loop if a significant difference is found
    //         }
    //     }
    // }

    // Serial.println();
    // delay(200);
}
