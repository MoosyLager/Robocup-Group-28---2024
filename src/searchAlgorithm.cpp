#include <stdbool.h>
#include <stdint.h>
#include "searchAlgorithm.h"
#include "sensor.h"
#include <stdio.h>
#include <elapsedMillis.h>
#include "motor.h"
#include "collection.h"
#include "circularBuf.h"

elapsedMillis weightWatchDog;
elapsedMillis rotationCounter;
elapsedMillis moduleCounter;
elapsedMillis avoidanceTimer;
bool onLeft = false;

void initializeRobotFSM(RobotFSM* fsm) {
    fsm->currentState = HUNTING;  // Start in CALIBRATING state
    fsm->lastMainState = HUNTING; // Set last state to the initial state
    fsm->previousState = HUNTING; // Set previous state to the initial state
    fsm->huntState = SEARCH;          // Start in SEARCH state
    fsm->returnState = HOMESEEK;      // Start in HOMESEEK state
    fsm->collectedWeights = 0;        // No weights collected at the start
    fsm->calibrated = false;          // Robot is not calibrated at the start
    fsm->weightPos = NONE;            // No weight detected at the start
    fsm->avoidanceSide = NO_WALL;        // No obstacle detected at the start
    fsm->evasiveManeuverCompleted = false;  // Evasive maneuver not completed at the start
}

void checkWeightsOnboard(RobotFSM* fsm)
{
    // weightCheckFunction(fsm); changes the state to RETURNING if >= 3 weights are onboard
}

/**
 * Working function, checks whether the wall is detected by the sensors
 */


void checkCalibration(RobotFSM* fsm)
{
    // calibrationCheckFunction(fsm); changes the state to HUNTING if calibration is completed
}

// Main FSM processing function
void processFSM(RobotFSM* fsm) {
    // Checks if robot is calibrated
    // If calibrated, check if robot is too close to the wall
    // If not too close to the wall, check if there are weights onboard to start HUNTING  RETURNING
    if (fsm->currentState != fsm->previousState) {
            leftMotor.integral = 0;  // Reset left motor PID integral error
            rightMotor.integral = 0; // Reset right motor PID integral error
            fsm->previousState = fsm->currentState;  // Update the last state
        }

    if (!fsm->currentState == CALIBRATING) {
        checkWallDistances(fsm);
    }
        

    switch (fsm->currentState) {
        case CALIBRATING:
            handleCalibrating(fsm);
            break;
        case HUNTING:
            handleHunting(fsm);
            break;
        case AVOIDING:
            handleAvoiding(fsm);
            break;
        case RETURNING:
            handleReturning(fsm);
            break;
        default:
            break;
    }
}

/*
MAIN STATE FUNCTIONS
*/


// State functions
void handleCalibrating(RobotFSM* fsm) {
    // Simulate calibration completed
    // calibrationFunction();
    // if calibration is completed, change the mainstate to HUNTING
}

void handleHunting(RobotFSM* fsm) {
    switch (fsm->huntState) {
        case SEARCH:
            handleSearching(fsm);
            break;
        case CHASE:
            handleChasing(fsm);
            break;
        case COLLECT:
            // handleCollecting(fsm);
            break;
        default:
            break;
    }
}

void checkWallDistances(RobotFSM* fsm)
{
    const int numFunctions = NUM_TOF_L0 + NUM_TOF_L1;

    // Iterate through each distance function and check for obstacles
    for (int i = 0; i < numFunctions; i++) {
        uint16_t distance = distanceFunctions[i]();  // Call the function
        // Could change this to test both sensors top and bottom
        if (distance < AVOIDANCE_THRESHOLD) {
            
            fsm->evasiveManeuverCompleted = false;
            fsm->currentState = AVOIDING;  // Change the state if below threshold

            // Determine side of detection based on the function index
            if (i % 2 == 0) {
                fsm->avoidanceSide = LEFT;  // L0 corresponds to the left side
                onLeft = true;
            } else {
                fsm->avoidanceSide = RIGHT; // L1 corresponds to the right side
                onLeft = false;
            }
            return;  // Exit early since an obstacle was found
        } else {
            fsm->avoidanceSide = NO_WALL;  // Reset the avoidance side if no obstacle is found
        }
    }

    // If no obstacle is found
    // Need to be careful that the robot doesn't get stuck in this state
}

void handleAvoiding(RobotFSM* fsm) 
{
    Serial.println("Avoiding");
    if (!(fsm->evasiveManeuverCompleted)) {
        if (onLeft) {
            rotateCW(3000);
            // Or Other avoidance maneuver
        } else if (!onLeft) {
            rotateCCW(3000);
            // Or Other avoidance maneuver
        } 
    }
    // Rotation Condition Met
    if (/*RotationConditionMet &&*/ fsm->avoidanceSide == NO_WALL) {
        fsm->evasiveManeuverCompleted = true;
        fsm->currentState = fsm->lastMainState;
        avoidanceTimer = 0;
    }
}

void handleReturning(RobotFSM *fsm) {

    switch (fsm->returnState) {
        case HOMESEEK:
            handleHomeSeeking(fsm);
            break;
        case DEPOSIT:
            // handleDepositing(fsm);
            break;
        default:
            break;
    }
}


/*
SEARCHING SUB-STATE FUNCTIONS
*/

void checkWeightDetection(RobotFSM* fsm) {
    // weightDetectionFunction(fsm); // changes the state to CHASE if weight is detected
    
}

void handleSearching(RobotFSM* fsm) {
    Serial.println("Searching");

    static elapsedMillis currentCount = 0;
    bool rotationComplete = false;

    if (weightDetected()) {
        fsm->huntState = CHASE;
    }
    if (rotationCounter - currentCount > ROTATION_TIMEOUT) {
        rotateCW(3000);
        // if (rotationComplete(uint16_t targetHeading, uint16_t currentHeading)) {
        //     currentCount = rotationCounter;
        // } CURRENTLY WAITING ON THE IMU
    } else {
    moveForward(3000);
    }
}



void handleChasing(RobotFSM* fsm) {
    if (weightDetected()) {
        weightWatchDog = 0;
    } else if (weightWatchDog > LOST_WEIGHT_TIMEOUT) {
        fsm->huntState = SEARCH;
        fsm->weightPos = NONE;
        weightWatchDog = 0;
    }


    if ((detectedCentreRight() || detectedCentreLeft())) {
        fsm->weightPos = AHEAD;
        // Need to check the range
    } else if (detectedFarRight()) {
        fsm->weightPos = ON_RIGHT;
        rotateCCW(3000);
    } else if (detectedFarLeft()) {
        fsm->weightPos = ON_LEFT;
        rotateCW(3000);
    } 
    
    
    if (fsm->weightPos == AHEAD) {
        if (!detectedFarRight() && !detectedFarLeft()) {
            moveForward(1500);
        } else if (detectedCentreRight()) {
            rotateCCW(1500);
        } else if (detectedCentreLeft()) {
            rotateCW(1500);
        } else {
            moveDistance((GetL1BR() * POSITIONAL_CONVERSION) + POSITIONAL_OFFSET, &leftMotor);
            moveDistance((GetL1BL() * POSITIONAL_CONVERSION) + POSITIONAL_OFFSET, &rightMotor);
            // TO CHANGE TO THE IMU CONDITION
            // THEN CHANGE huntstate to collect
        }
    }
}

void handleCollecting(RobotFSM* fsm) {
    bool collectedWeight = false; // Need to change the actuator to move
    moveForward(0);
    ActuateCollector(); // Needs to be changes and moved into the PIDcontrol module
    if (collectedWeight) {
        fsm->collectedWeights++;
        fsm->huntState = SEARCH;
        if (fsm->collectedWeights >= 3) {
            fsm->currentState = RETURNING;
            fsm->lastMainState = RETURNING;
            fsm->returnState = HOMESEEK;
        }
        
    }
    // Need something to check the jamming of the collection motor
    /*
    if (collectionJammed()) {
        reverseToOpenJam();
    }*/
}

/*
RETURNING SUB-STATE FUNCTIONS
*/
void handleHomeSeeking(RobotFSM* fsm) {
    // homeSeekMotorFunction();
    // error = desiredHeading - currentHeading;

    // if (error > 180) {
    //     error = error - 360;
    // } else if (error < -180) {
    //     error = error + 360;
    // }

    // if (error > 0) {
    //     // Drive forward and turn right
    // } else if (error < 0) {
    //     // Drive forward and turn left
    // }

    // if (colorDetected()) {
    //     fsm->returnState = DEPOSIT;
    // }

}

void handleDepositing(RobotFSM* fsm) {
    // depositMotorFunction();
    // TIMER DELAY
    bool weightsDeposited = false;
    // Set collection acutation to open
    if (weightsDeposited) {
        fsm->currentState = HUNTING;
        fsm->lastMainState = HUNTING;
        fsm->huntState = SEARCH;
        fsm->returnState = HOMESEEK;
        fsm->collectedWeights = 0;
    }
}

/*
Currently need:

MOTOR FUNCTIONS:
- searchMotorFunction()
- chasingMotorFunction()
- collectionMotorFunction()
- homeSeekMotorFunction()
- depositMotorFunction()

SENSOR FUNCTIONS:
- weightDetectionFunction()
- checkCollected()
- calibrationFunction()
- checkWallDistance()
- weightCheckFunction()
*/