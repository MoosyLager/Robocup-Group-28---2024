#include "collection.h"

void CalibrateCollector()
{
    Serial.println("Calibrating Collection...");
    bool isCalibrated = false;
    while ( !isCalibrated ) {
        SetMotorSpeed(collectionMotor, MAX_MOTOR_VAL);
        if ( !CollectorPosition() ) {
            SetMotorSpeed(collectionMotor, MOTOR_STOP_VAL);
            collectionMotorPos = 0;
            prevCollectionMotorPos = 1;
            isCalibrated = true;
        }
    }
}

void CalibrateStepper()
{
}

void ActuateCollector()
{
}

void ResetCollector()
{
}
