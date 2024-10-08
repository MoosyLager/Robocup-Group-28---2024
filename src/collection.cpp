#include "collection.h"
#include <elapsedMillis.h>

int collectorTarget;
int prevCollectorPos;
bool collectorActuating = false;
bool collectingWeight = false;
bool weightCollected = false;
elapsedMillis collectionWatchDog;

bool rampCalibrating = true;
/**
 * Update the collection motor speed if needed
 */
void UpdateCollector()
{
    if ( collectorActuating ) {
        int32_t wrappedCurrect = collectionMotorPos & COLLECTOR_TICKS_PER_REV;
        int32_t wrappedTarget = collectorTarget & COLLECTOR_TICKS_PER_REV;
        int32_t difference = wrappedTarget - wrappedCurrect;

        Serial.println(difference);

        if ( collectingWeight ) {
            if ( abs(difference) < COLLECTOR_STOP_THRESHOLD ) {
                SetMotorSpeed(&collectionMotor, MOTOR_STOP_VAL);
                collectorActuating = false;
                collectingWeight = false;
                Serial.println("Collection Complete!");
            } else {
                SetMotorSpeed(&collectionMotor, MAX_MOTOR_VAL);
            }
        } else {
            if ( difference > COLLECTOR_TICKS_PER_REV / 2 ) {
                difference -= COLLECTOR_TICKS_PER_REV;
            } else if ( difference < -COLLECTOR_TICKS_PER_REV / 2 ) {
                difference += COLLECTOR_TICKS_PER_REV;
            }

            if ( abs(difference) < COLLECTOR_STOP_THRESHOLD ) {
                SetMotorSpeed(&collectionMotor, MOTOR_STOP_VAL);
                Serial.println("Collection Complete!");
                collectorActuating = false;
            } else if ( difference > 0 ) {
                SetMotorSpeed(&collectionMotor, MIN_MOTOR_VAL);
            } else {
                SetMotorSpeed(&collectionMotor, MAX_MOTOR_VAL);
            }
        }
    }
}

/**
 * Calibrate the collector encoder to a known position
 */
void CalibrateCollector(void)
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
// void CalibrateRamp()
// {
//     Serial.println("Calibraing Stepper Motor...");
//     rampStepper.setSpeed(-STEPPER_MAX_SPEED);
//     bool isCalibrated = false;
//     while ( !isCalibrated ) {
//         if ( !RampPosition() ) {
//             rampStepper.setSpeed(0);
//             rampStepper.setCurrentPosition(0);
//             isCalibrated = true;
//         }
//         rampStepper.run();
//     }

//     Serial.println("Ramp Calibrated!");
// }
void CalibrateRamp()
{
    static bool once = true;
    if ( once ) {
        Serial.println("Calibrating Ramp...");
        once = false;
    }
    rampStepper.setSpeed(-STEPPER_MAX_SPEED);
    if ( !RampPosition() ) {
        rampStepper.setSpeed(0);
        rampStepper.setCurrentPosition(0);
        Serial.println("Ramp Calibrated!");
    }
    // switch ( rampCalibration ) {
    //     case CalibrationState::CALIBRATION_START:
    //         // Serial.println("Calibrating Ramp...");
    //         rampStepper.setSpeed(-STEPPER_MAX_SPEED / 2);
    //         rampCalibration = CALIBRATION_RUNNING;
    //         break;
    //     case CalibrationState::CALIBRATION_RUNNING:
    //         if ( !RampPosition() ) {
    //             rampStepper.setSpeed(0);
    //             rampStepper.setCurrentPosition(0);
    //             rampCalibration = CALIBRATION_DONE;
    //         }
    //         rampStepper.run();
    //         break;
    //     case CalibrationState::CALIBRATION_DONE:
    //         // Serial.println("Ramp Calibrated!");
    //         break;
    // }
}

/**
 * Set the collector jaws to an open position to allow for weights to pass through
 */
void OpenCollector()
{
    Serial.println("Opening Collector...");
    collectorTarget = 0 + COLLECTOR_OPEN_OFFSET;
    collectorActuating = true;
}

/**
 * Set the collector jaws to a closed position to block weights from passing through
 */
void CloseCollector()
{
    Serial.println("Closing Collector...");
    prevCollectionMotorPos = collectionMotorPos;
    collectorTarget = prevCollectionMotorPos + COLLECTOR_CLOSED_OFFSET;
    collectorActuating = true;
}

/**
 * Actuate the collector through a full rotation and move to the closed position
 */
void CollectWeight()
{
    Serial.println("Actuating Collector...");
    SetMotorSpeed(&collectionMotor, MAX_MOTOR_VAL);
    prevCollectionMotorPos = collectionMotorPos;
    collectorTarget = prevCollectionMotorPos - COLLECTOR_TICKS_PER_REV;
    collectorActuating = true;
    collectingWeight = true;
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
void CalibrateCollectionSystem(RobotFSM* fsm)
{
    CalibrateCollector();
    CalibrateRamp();
    fsm->currentState = HUNTING;
}