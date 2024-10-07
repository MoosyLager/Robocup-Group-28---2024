#ifndef COLLECTION_H
#define COLLECTION_H

#include "encoder.h"
#include "motor.h"
#include "searchAlgorithm.h"
#include "sensor.h"

#define COLLECTOR_OPEN_OFFSET    -400
#define COLLECTOR_CLOSED_OFFSET  2000
#define RAMP_LOWERED_OFFSET     0

#define COLLECTOR_UPDATE_RATE_MS 10

#define COLLECTOR_STOP_THRESHOLD 10

extern bool collectorActuating;
extern bool weightCollected;

void CalibrateCollector();
void CalibrateRamp();

void CollectWeight();
void OpenCollector();
void CloseCollector();

void LowerRamp();
void LiftRamp();

void UpdateCollector();

// void CalibrateCollectionSystem(RobotFSM* fsm);

#endif