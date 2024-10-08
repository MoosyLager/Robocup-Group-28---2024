#include "searchAlgorithm.h"
#include "circularBuf.h"
#include "collection.h"
#include "motor.h"
#include "sensor.h"
#include <elapsedMillis.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

elapsedMillis weightWatchDog;
elapsedMillis rotationCounter;
elapsedMillis moduleCounter;
elapsedMillis avoidanceTimer;
bool onLeft = false;

void initializeRobotFSM(RobotFSM* fsm)
{
    fsm->currentState = HUNTING;            // Start in CALIBRATING state
    fsm->lastMainState = HUNTING;           // Set last state to the initial state
    fsm->previousState = HUNTING;           // Set previous state to the initial state
    fsm->huntState = SEARCH;                // Start in SEARCH state
    fsm->returnState = HOMESEEK;            // Start in HOMESEEK state
    fsm->collectedWeights = 0;              // No weights collected at the start
    fsm->calibrated = false;                // Robot is not calibrated at the start
    fsm->weightPos = NONE;                  // No weight detected at the start
    fsm->avoidanceSide = NO_WALL;           // No obstacle detected at the start
    fsm->evasiveManeuverCompleted = false;  // Evasive maneuver not completed at the start
    fsm->targetHeading = 0;
    fsm->distToTravel = 0;
}

void checkCalibration(RobotFSM* fsm)
{
    // calibrationCheckFunction(fsm); changes the state to HUNTING if calibration is completed
}

// Main FSM processing function
void processFSM(RobotFSM* fsm)
{
    // Checks if robot is calibrated
    // If calibrated, check if robot is too close to the wall
    // If not too close to the wall, check if there are weights onboard to start HUNTING  RETURNING
    if ( fsm->currentState != fsm->previousState ) {
        // State edge detection
        fsm->previousState = fsm->currentState;  // Update the last state
    }

    if ( !fsm->currentState == CALIBRATING ) {
        checkWallDistances(fsm);
    }


    switch ( fsm->currentState ) {
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
void handleCalibrating(RobotFSM* fsm)
{
    // Simulate calibration completed
    // calibrationFunction();
    // if calibration is completed, change the mainstate to HUNTING
}

void handleHunting(RobotFSM* fsm)
{
    switch ( fsm->huntState ) {
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
    for ( int i = 0; i < numFunctions; i++ ) {
        uint16_t distance = distanceFunctions[i]();  // Call the function
        // Could change this to test both sensors top and bottom
        if ( distance < AVOIDANCE_THRESHOLD ) {
            fsm->weightPos = NONE;         // Reset the weight position if an obstacle is found
            fsm->evasiveManeuverCompleted = false;
            fsm->currentState = AVOIDING;  // Change the state if below threshold
            avoidanceTimer = 0;
            // Determine side of detection based on the function index
            if ( i % 2 == 0 ) {
                fsm->avoidanceSide = LEFT;  // L0 corresponds to the left side
                fsm->targetHeading = GetOrientationYaw() + 35;
                onLeft = true;
            } else {
                fsm->avoidanceSide = RIGHT;  // L1 corresponds to the right side
                fsm->targetHeading = GetOrientationYaw() - 35;
                onLeft = false;
            }
            return;                        // Exit early since an obstacle was found
        } else {
            fsm->avoidanceSide = NO_WALL;  // Reset the avoidance side if no obstacle is found
        }
    }

    // If no obstacle is found
    // Need to be careful that the robot doesn't get stuck in this state
}

void handleAvoiding(RobotFSM* fsm)
{
    if ( !(fsm->evasiveManeuverCompleted) ) {
        if ( onLeft ) {
            move(-300, -4000);
            // Rotate clockwise
        } else if ( !onLeft ) {
            move(-4000, -300);
            // Rotate counter-clockwise
        }
        Serial.println("Avoiding");
    } else {
        fsm->currentState = fsm->lastMainState;
        fsm->huntState = SEARCH;
    }
    // Rotation Condition Met
    if ( (checkTargetHeading(fsm->targetHeading) && (fsm->avoidanceSide == NO_WALL)) ) {
        Serial.println("Evasive maneuver completed");
        fsm->evasiveManeuverCompleted = true;
        avoidanceTimer = 0;
    } else if ( avoidanceTimer > EVASIVE_MANEUVER_TIMEOUT ) {
        Serial.println("Avoidance timeout");
        fsm->evasiveManeuverCompleted = true;
        fsm->targetHeading = truncateHeading(GetOrientationYaw());
        avoidanceTimer = 0;
    }
}

void handleReturning(RobotFSM* fsm)
{

    switch ( fsm->returnState ) {
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

void checkWeightDetection(RobotFSM* fsm)
{
    // weightDetectionFunction(fsm); // changes the state to CHASE if weight is detected
}

void handleSearching(RobotFSM* fsm)
{
    Serial.println("Searching");

    static int currentCount = 0;
    static bool completeRotation = false;

    if ( weightDetected() ) {
        fsm->huntState = CHASE;
        Serial.println("Weight Detected");
    }
    if ( completeRotation && !checkTargetHeading(fsm->targetHeading) ) {
        move(4000, -4000);
        // Clockwise rotation
        if ( rotationCounter - currentCount > ROTATION_FAILURE_TIMEOUT ) {
            Serial.println("Failed to rotate, resetting");
            completeRotation = false;
            currentCount = rotationCounter;
            fsm->targetHeading = GetOrientationYaw();
        }
    } else {
        moveForward(3300);
        completeRotation = false;
    }
    if ( rotationCounter - currentCount > ROTATION_TIMEOUT ) {
        Serial.println("Rotating");
        completeRotation = true;
        currentCount = rotationCounter;
        int targetHead = GetOrientationYaw() + 345;
        fsm->targetHeading = truncateHeading(targetHead);
        // Checks whether target heading is in the range 0-360
    }
}

void handleChasing(RobotFSM* fsm)
{

    static double rangeLeft = 0;
    static double rangeRight = 0;
    static int distToTravel = 0;

    static bool linedUp = false;
    static bool calculatedDist = false;


    if ( (detectedCentreRight() || detectedCentreLeft()) ) {
        fsm->weightPos = AHEAD;
        Serial.println("Ahead");
        move(0, 0);
    } else if ( detectedFarRight() ) {
        fsm->weightPos = ON_RIGHT;
        Serial.println("Right");
        // Rotate counter-clockwise
        move(-500, 0);
    } else if ( detectedFarLeft() ) {
        fsm->weightPos = ON_LEFT;
        Serial.println("Left");
        // Rotate clockwise
        move(0, -500);
    } else {
        Serial.println("Currently no weight seen");  // Could abstract this to the weight detection function
    }

    if ( fsm->weightPos == AHEAD ) {
        if ( !detectedCentreRight() && !detectedCentreLeft() ) {
            moveForward(200);
        } else if ( detectedCentreRight() && !detectedCentreLeft() ) {
            rotateCCW(200);
        } else if ( detectedCentreLeft() && !detectedCentreRight() ) {
            rotateCW(200);
        } else {
            Serial.print("Weight in sights!: ");
            if ( !linedUp ) {
                rangeLeft = GetL1BL();
                rangeRight = GetL1BR();
                if ( rangeLeft > rangeRight ) {
                    rotateCW(200);
                    Serial.println("Rotating CW!");
                } else {
                    rotateCCW(200);
                    Serial.println("Rotating CCW!");
                }
                linedUp = isLinedUp(rangeLeft, rangeRight);
            } else {
                Serial.println("Is lined up!");
                Serial.println(isLinedUp(rangeLeft, rangeRight));
                if ( !calculatedDist ) {
                    double s = 230 + rangeLeft + rangeRight;
                    int area = sqrt(s * (s - rangeLeft) * (s - rangeRight) * (s - 230));
                    distToTravel = 2 * area / 230;
                    Serial.print("Distance to travel: ");
                    Serial.println(distToTravel);
                    leftMotor.targetMotorPos = distToTravel * ENCODER_PER_DIST + leftMotor.currentMotorPos;
                    rightMotor.targetMotorPos = distToTravel * ENCODER_PER_DIST + rightMotor.currentMotorPos;
                    fsm->distToTravel = distToTravel;
                    calculatedDist = true;
                } else {
                    moveForward(200);
                    if ( leftMotor.currentMotorPos >= leftMotor.targetMotorPos || rightMotor.currentMotorPos >= rightMotor.targetMotorPos ) {  // Might have to change to current
                        fsm->huntState = COLLECT;
                    }
                    Serial.print("Distance to travel: ");
                    Serial.println(distToTravel);
                }
            }
        }
    }

    if ( weightDetected() ) {
        weightWatchDog = 0;
    } else if ( weightWatchDog > LOST_WEIGHT_TIMEOUT ) {
        Serial.println("Lost weight");
        fsm->huntState = SEARCH;
        fsm->weightPos = NONE;
        weightWatchDog = 0;
    }
}

void handleCollecting(RobotFSM* fsm)
{
    // Need to change the actuator to move
    moveForward(0);

    if ( !collectorActuating ) {
        fsm->collectedWeights++;
        fsm->huntState = SEARCH;
        if ( fsm->collectedWeights >= 3 ) {
            fsm->currentState = RETURNING;
            fsm->lastMainState = RETURNING;
            fsm->returnState = HOMESEEK;
        }
    }

    /*
    if (collectionJammed()) {
        reverseToOpenJam();
    }*/
}

/*
RETURNING SUB-STATE FUNCTIONS
*/
void handleHomeSeeking(RobotFSM* fsm)
{
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

void handleDepositing(RobotFSM* fsm)
{
    // depositMotorFunction();
    // TIMER DELAY
    bool weightsDeposited = false;
    // Set collection acutation to open
    if ( weightsDeposited ) {
        fsm->currentState = HUNTING;
        fsm->lastMainState = HUNTING;
        fsm->huntState = SEARCH;
        fsm->returnState = HOMESEEK;
        fsm->collectedWeights = 0;
    }
}