#include "collection.h"
#include <elapsedMillis.h>

elapsedMillis collectionWatchDog;

int collectorTarget;
int prevCollectorPos;
bool collectorActuating = false;
bool weightCollected = false;

bool collectingWeight = false;
bool closingCollector = false;
bool openingCollector = false;

bool rampCalibrating = true;
/**
 * Update the collection motor speed if needed
 */
void UpdateCollector()
{
    int32_t error = collectorTarget - (collectionMotorPos % COLLECTOR_TICKS_PER_REV);
    Serial.print("Error: ");
    Serial.print(error);
    Serial.print("  Collector Position: ");
    Serial.println(collectionMotorPos);
    if ( collectingWeight ) {
        if ( abs(error) > COLLECTOR_STOP_THRESHOLD ) {
            SetMotorSpeed(&collectionMotor, MAX_MOTOR_VAL);
        } else {
            SetMotorSpeed(&collectionMotor, MOTOR_STOP_VAL);
            collectingWeight = false;
            Serial.println("Weight Collected!");
            CloseCollector();
        }
    }
    if ( closingCollector ) {
        if ( error < -COLLECTOR_STOP_THRESHOLD ) {
            SetMotorSpeed(&collectionMotor, MAX_MOTOR_VAL);
        } else if ( error > COLLECTOR_STOP_THRESHOLD ) {
            SetMotorSpeed(&collectionMotor, MIN_MOTOR_VAL);
        } else {
            closingCollector = false;
            Serial.println("Collector Closed!");
            SetMotorSpeed(&collectionMotor, MOTOR_STOP_VAL);
        }
    }
    if ( openingCollector ) {
        if ( error < -COLLECTOR_STOP_THRESHOLD ) {
            SetMotorSpeed(&collectionMotor, MAX_MOTOR_VAL);
        } else if ( error > COLLECTOR_STOP_THRESHOLD ) {
            SetMotorSpeed(&collectionMotor, MIN_MOTOR_VAL);
        } else {
            openingCollector = false;
            Serial.println("Collector Opened!");
            SetMotorSpeed(&collectionMotor, MOTOR_STOP_VAL);
        }
    }
}

/**
 * Set the collector jaws to an open position to allow for weights to pass through
 */
void OpenCollector()
{
    Serial.println("Opening Collector...");
    collectorTarget = COLLECTOR_OPEN_OFFSET;
    openingCollector = true;
}

/**
 * Set the collector jaws to a closed position to block weights from passing through
 */
void CloseCollector()
{
    Serial.println("Closing Collector...");
    collectorTarget = COLLECTOR_CLOSED_OFFSET;
    closingCollector = true;
}

/**
 * Actuate the collector through a full rotation and move to the closed position
 */
void CollectWeight()
{
    Serial.println("Collecting Weight...");
    collectorTarget = COLLECTOR_ACTUATING_OFFSET;
    collectingWeight = true;
}

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
    static bool notLimit = true;
    

    if ( notLimit) {
        SetMotorSpeed(&collectionMotor, MAX_MOTOR_VAL);
    } else {
        
    }
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