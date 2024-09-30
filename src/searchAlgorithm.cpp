#include <stdbool.h>
#include <stdint.h>

typedef enum {
    AVOID,
    RETURN,
    EMPTY,
    COLLECT,
    CHASE,
    SEARCH,
    STOPPED
} RobotState_t;

typedef struct {
    RobotState_t currentState = STOPPED;
    RobotState_t prevState = STOPPED;
} Robot_t;

void logicLoop (void) {
    uint8_t weightsOnBoard = 0;
    /*
    bool currentlyAvoiding = false
    if (TOO CLOSE TO WALL && !currentlyAvoiding) {
        prev_state = currentState;
        currentState = AVOID;
        currentlyAvoiding = true
    } 
    if (!TOO_CLOSE_TO_WALL && currentlyAvoiding) {
        currentState = prev_state;
        currentlyAvoiding = false
    } 

    if (<15 seconds remaining || weightsOnBoard >= 3) {
        currentState = RETURN;
    }
    if (base detected && STOPPED && FULL) {
        currentState = EMPTY;
    }
    if (base detected && STOPPED && EMPTY) {
        currentState = SEARCH;
    }
    if (searching and weight detected)
        currentState = CHASE;

    if (currentState == CHASE && ready for collection) {
        currentState = COLLECT;
    }
    */
    
   ;
}