#include <Arduino.h>
#include <elapsedMillis.h>

elapsedMillis motorDelay;
uint16_t motorSpeed = 1100;
elapsedMillis motorMicros;

void setup()
{
    Serial.begin(9600);
    delay(1000);
    InitSensors();
}

void loop()
{
    ReadTOFL0();
    for ( int i = 0; i < NUM_TOF_L0; i++ ) {
        Serial.print("Sensor ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(sensorL0Data[i]);
        Serial.print(" ");
    }

    for ( int i = 0; i < NUM_TOF_L0 - 1; i++ ) {
        for ( int j = i + 1; j < NUM_TOF_L0; j++ ) {
            // Calculate percentage difference between sensors
            float difference = abs(sensorL0Data[i] - sensorL0Data[j]);
            float avgValue = (sensorL0Data[i] + sensorL0Data[j]) / 2.0;

            if ( avgValue > 0 && (difference / avgValue) >= 0.30 ) {
                Serial.print("Weight Detected!");
                break;  // Exit the loop if a significant difference is found
            }
        }
    }

    Serial.println();
    delay(200);
}
