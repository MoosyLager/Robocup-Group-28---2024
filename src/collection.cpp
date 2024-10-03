#include "collection.h"

/**
 * Calibrate the collector encoder to a known position
 */
void CalibrateCollector()
{
    Serial.println("Calibrating Collector...");
    bool isCalibrated = false;
    while ( !isCalibrated ) {
        SetMotorSpeed(&collectionMotor, MAX_MOTOR_VAL);
        if ( !CollectorPosition() ) {
            SetMotorSpeed(&collectionMotor, MOTOR_STOP_VAL);
            collectionMotorPos = 0;
            prevCollectionMotorPos = 1;
            isCalibrated = true;
        }
    }
    Serial.println("Collector Calibrated!");
}

/**
 * Calibrate the ramp stepper motor to a known position
 */
void CalibrateRamp()
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

    Serial.println("Ramp Calibrated!");
}

/**
 * Set the collector jaws to an open position to allow for weights to pass through
 */
void OpenCollector()
{
    do {
        SetMotorSpeed(&collectionMotor, MAX_MOTOR_VAL);
    } while ( collectionMotorPos != COLLECTOR_OPEN_OFFSET );
}

/**
 * Set the collector jaws to a closed position to block weights from passing through
 */
void CloseCollector()
{
    do {
        SetMotorSpeed(&collectionMotor, MAX_MOTOR_VAL);
    } while ( collectionMotorPos != COLLECTOR_CLOSED_OFFSET );
}

/**
 * Actuate the collector through a full rotation and move to the closed position
 */
void ActuateCollector()
{
    // 44:16 ratio between motor and collector
    // 2.75 motor revolutions per collector revolution
    // 11 pulses per motor revolution
    Serial.println("Actuating Collector...");
    int prevPos = collectionMotorPos;
    int targetPos = prevPos - COLLECTOR_TICKS_PER_REV;
    do {
        SetMotorSpeed(&collectionMotor, MAX_MOTOR_VAL);
    } while ( collectionMotorPos > targetPos );
    SetMotorSpeed(&collectionMotor, MOTOR_STOP_VAL);
    Serial.println("Collector Actuated!");
}

/**
 * Set the ramp to the lowered position
 */
void LowerRamp()
{
}

/**
 * Set the ramp to the lifted positioned
 */
void LiftRamp()
{
}

/**
 * Calibrates the collector and ramp then sets the FSM state
 */
// void CalibrateCollectionSystem(RobotFSM* fsm)
// {
//     CalibrateCollector();
//     CalibrateRamp();
//     fsm->currentState = HUNTING;
// }