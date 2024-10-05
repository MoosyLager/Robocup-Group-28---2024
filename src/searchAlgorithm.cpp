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

void initializeRobotFSM(RobotFSM* fsm) {
    fsm->currentState = HUNTING;  // Start in CALIBRATING state
    fsm->lastMainState = HUNTING; // Set last state to the initial state
    fsm->huntState = SEARCH;          // Start in SEARCH state
    fsm->returnState = HOMESEEK;      // Start in HOMESEEK state
    fsm->collectedWeights = 0;        // No weights collected at the start
    fsm->tooCloseToWall = false;      // Robot is not near the wall at the start
    fsm->calibrated = false;          // Robot is not calibrated at the start
    fsm->weightPos = NONE;            // No weight detected at the start
    fsm->avoidanceSide = NO_WALL;        // No obstacle detected at the start
}

void checkWeightsOnboard(RobotFSM* fsm)
{
    // weightCheckFunction(fsm); changes the state to RETURNING if >= 3 weights are onboard
}

void checkWallDistances(RobotFSM* fsm)
{
    const int numFunctions = NUM_TOF_L0 + NUM_TOF_L1;

    // Iterate through each distance function and check for obstacles
    for (int i = 0; i < numFunctions; i++) {
        uint16_t distance = distanceFunctions[i]();  // Call the function
        if (distance < AVOIDANCE_THRESHOLD) {
            fsm->currentState = AVOIDING;  // Change the state if below threshold

            // Determine side of detection based on the function index
            if (i % 2 == 0) {
                fsm->avoidanceSide = LEFT;  // L0 corresponds to the left side
                Serial.println("LEFT");
            } else {
                fsm->avoidanceSide = RIGHT; // L1 corresponds to the right side
                Serial.println("RIGHT");
            }

            return;  // Exit early since an obstacle was found
        }
    }

    // If no obstacle is found, reset the state/side
    fsm->avoidanceSide = NO_WALL;
}


void checkCalibration(RobotFSM* fsm)
{
    // calibrationCheckFunction(fsm); changes the state to HUNTING if calibration is completed
}

// Main FSM processing function
void processFSM(RobotFSM* fsm) {
    // Checks if robot is calibrated
    // If calibrated, check if robot is too close to the wall
    // If not too close to the wall, check if there are weights onboard to start HUNTING  RETURNING
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
            Serial.println("Unknown state!");
            break;
    }
}

/*
MAIN STATE FUNCTIONS
*/


// State functions
void handleCalibrating(RobotFSM* fsm) {
    Serial.printf("Calibrating...\n");
    // Simulate calibration completed
    // calibrationFunction();
    // if calibration is completed, change the mainstate to HUNTING
}

void handleHunting(RobotFSM* fsm) {
    Serial.printf("Hunting for weights...\n");

    switch (fsm->huntState) {
        case SEARCH:
            // handleSearching(fsm);
            break;
        case CHASE:
            // handleChasing(fsm);
            break;
        case COLLECT:
            // handleCollecting(fsm);
            break;
        default:
            Serial.println("Unknown hunt state!");
            break;
    }
}

void handleAvoiding(RobotFSM* fsm) {
    Serial.printf("Avoiding...\n");
    if (fsm->avoidanceSide == LEFT) {
        rotateCW(3000);
    } else if (fsm->avoidanceSide == RIGHT) {
        rotateCCW(3000);
    } else if (fsm->avoidanceSide == NO_WALL) {
        fsm->currentState = HUNTING;
        fsm->huntState = SEARCH;
    } else {
        Serial.println("Unknown avoidance side!");
    }
    
}

void handleReturning(RobotFSM *fsm) {
    Serial.printf("Returning...\n");

    switch (fsm->returnState) {
        case HOMESEEK:
            // handleHomeSeeking(fsm);
            break;
        case DEPOSIT:
            // handleDepositing(fsm);
            break;
        default:
            Serial.println("Unknown return state!");
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
    static int currentCount = 0;
    if (rotationCounter - currentCount > ROTATION_TIMEOUT) {
        rotateCW(3000);
        if (rotationCounter - currentCount > (ROTATION_TIMEOUT + SPIN_TIMEOUT)) {
            currentCount = rotationCounter;
        }
    } else {
        moveForward(3000);
    }

    if (weightDetected()) {
        fsm->huntState = CHASE;
    }
}



void handleChasing(RobotFSM* fsm) {
    if (weightDetected) {
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
            fsm->returnState = HOMESEEK;
        }
        
    }
    // Need something to check the jamming of the collection motor
}

/*
RETURNING SUB-STATE FUNCTIONS
*/

void handleHomeSeeking(RobotFSM* fsm) {
    // homeSeekMotorFunction();
    // change the current return state to DEPOSIT if home is reached
}

void handleDepositing(RobotFSM* fsm) {
    // depositMotorFunction();

    // If successful, reset the collected weights to 0 also:
    // change the current mainstate to SEARCHING
    // change the huntstate to SEARCH and the returnstate to HOMESEEK
}

// int main() {
//     // Initialize the FSM
//     RobotFSM robot;
//     initializeRobotFSM(&robot);

// }

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
- checkChased()
- checkCollected()
- calibrationFunction()
- checkWallDistance()
- weightCheckFunction()
*/