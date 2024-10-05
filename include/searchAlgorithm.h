#ifndef SEARCHALGORITHM_H
#define SEARCHALGORITHM_H

#include <Arduino.h>
#include <stdbool.h>

extern uint8_t weightsOnboard;
extern bool finished_calibrating;

#define LOST_WEIGHT_TIMEOUT 5000
#define ROTATION_TIMEOUT 20000
#define SPIN_TIMEOUT 4000
#define EVASIVE_MANEUVER_TIMEOUT 2000

typedef enum {
    FORWARD,
    BACKWARD,
    CW,
    CCW
} Motion_t;

typedef enum {
    AHEAD,
    ON_RIGHT,
    ON_LEFT, 
    NONE
} WeightPos_t;

typedef enum {
    LEFT,
    RIGHT,
    NO_WALL
} AvoidanceSide_t;

// Main state machine
typedef enum {
    CALIBRATING,
    HUNTING,
    AVOIDING,
    RETURNING
} MainState_t;

// Sub state machine for hunting
typedef enum {
    SEARCH,
    CHASE,
    COLLECT
} HuntState_t;

// Sub state machine for returning
typedef enum {
    HOMESEEK,
    DEPOSIT
} ReturnState_t;

typedef struct {
    MainState_t currentState;
    MainState_t lastMainState;  // To track the last state (HUNTING or RETURNING)
    HuntState_t huntState;
    ReturnState_t returnState;
    int collectedWeights;
    bool calibrated;
    WeightPos_t weightPos;
    AvoidanceSide_t avoidanceSide;
    bool evasiveManeuverCompleted;
} RobotFSM;

// Function prototypes
void checkWallDistances(RobotFSM* fsm);
void processFSM(RobotFSM* fsm);
void initializeRobotFSM(RobotFSM* fsm);
void handleCalibrating(RobotFSM *fsm);
void handleSearching(RobotFSM *fsm);
void handleHunting(RobotFSM *fsm);
void handleAvoiding(RobotFSM *fsm);
void handleReturning(RobotFSM *fsm);
void checkWallProximity(RobotFSM *fsm);



#endif // SEARCHALGORITHM_H