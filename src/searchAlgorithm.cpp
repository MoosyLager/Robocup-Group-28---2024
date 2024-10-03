#include <stdbool.h>
#include <stdint.h>
#include "searchAlgorithm.h"
#include <stdio.h>

void initializeRobotFSM(RobotFSM* fsm) {
    fsm->currentState = CALIBRATING;  // Start in CALIBRATING state
    fsm->lastMainState = CALIBRATING; // Set last state to the initial state
    fsm->huntState = SEARCH;          // Start in SEARCH state
    fsm->returnState = HOMESEEK;      // Start in HOMESEEK state
    fsm->collectedWeights = 0;        // No weights collected at the start
    fsm->tooCloseToWall = false;      // Robot is not near the wall at the start
    fsm->calibrated = false;          // Robot is not calibrated at the start
}

void checkWeightsOnboard(RobotFSM* fsm)
{
    // weightCheckFunction(fsm); changes the state to RETURNING if >= 3 weights are onboard
}

void checkWallDistance(RobotFSM* fsm)
{
    // checks the sensor values to determine if the robot is too close to the wall
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
        checkWallDistance(fsm);
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
    printf("Calibrating...\n");
    // Simulate calibration completed
    // calibrationFunction();
    // if calibration is completed, change the mainstate to HUNTING
}

void handleHunting(RobotFSM* fsm) {
    printf("Hunting for weights...\n");

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
    // takeAvoidingAction(fsm); // Function needs to check wall detection AND completion of evading action
    fsm->huntState = SEARCH; // Reset the hunt state to SEARCH after avoiding
}

void handleReturning(RobotFSM *fsm) {
    printf("Returning...\n");
    
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
    // searchMotorFunction(); // sets the current motion to straight ahead or if rotating, the direction to rotate
    // checkWeightDetection(fsm); // Changes the state to CHASE if weight is detected
}

void handleCollecting(RobotFSM* fsm) {
    // collectionMotorFunction();
    // checkCollected(); // Changes the huntstate to SEARCH if the weight is successfully collected
    // if successul, add 1 to the collected weights also check if the robot has collected 3 weights and change the main state to RETURNING and the hunt state to SEARCH, and the return state to HOMESEEK
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

int main() {
    // Initialize the FSM
    RobotFSM robot;
    initializeRobotFSM(&robot);

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
- checkChased()
- checkCollected()
- calibrationFunction()
- checkWallDistance()
- weightCheckFunction()
*/