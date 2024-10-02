#include "collection.h"

void CalibrateCollector()
{
    Serial.println("Calibrating Collector...");
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
    Serial.println("Calibraing Stepper Motor...");
    rampStepper.setSpeed(STEPPER_MAX_SPEED / 2);
    bool isCalibrated = false;
    while ( !isCalibrated ) {
        if ( !RampPosition() ) {
            rampStepper.setSpeed(0);
            rampStepper.setCurrentPosition(0);
            isCalibrated = true;
        }
        rampStepper.run();
    }
}

void ActuateCollector()
{
}

void ResetCollector()
{
}
