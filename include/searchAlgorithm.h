#ifndef SEARCHALGORITHM_H
#define SEARCHALGORITHM_H

#include <Arduino.h>

extern uint8_t weightsOnboard;

typedef enum {
    SEARCH,
    CHASE,
    COLLECT,
    RETURN,
    DEPOSIT,
} Mode_t;

typedef enum {
    CLOCKWISE,
    COUNTERCLOCKWISE,
    FORWARD,
    BACKWARD,
} Direction_t;

typedef struct {
    Mode_t mode;
    Direction_t direction;
    uint16_t speed;
    uint16_t distance;
} SearchAlgorithm_t;

/* 
loop
if 3 weights on board or at time limit:
    Mode = RETURN
else if (weight detected)
    Mode = CHASE


*/

#endif // SEARCHALGORITHM_H