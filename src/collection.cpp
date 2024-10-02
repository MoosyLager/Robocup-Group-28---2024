#include "collection.h"

/**
 * Calibrate the collector encoder to a known position
 */
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

/**
 * Calibrate the ramp stepper motor to a known position
 */
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

/**
 * Set the collector jaws to an open position to allow for weights to pass through
 */
void OpenCollector()
{
}

/**
 * Set the collector jaws to a closed position to block weights from passing through
 */
void CloseCollector()
{
}

/**
 * Actuate the collector through a full rotation and move to the closed position
 */
void ActuateCollector()
{
}
