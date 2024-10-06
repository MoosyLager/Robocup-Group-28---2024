#ifndef COLLECTION_H
#define COLLECTION_H

#include "encoder.h"
#include "motor.h"
#include "searchAlgorithm.h"
#include "sensor.h"

#define COLLECTOR_OPEN_OFFSET   0
#define COLLECTOR_CLOSED_OFFSET 0
#define RAMP_LOWERED_OFFSET     0

#define COLLECTOR_UPDATE_RATE_MS 50

void CalibrateCollector();
void CalibrateRamp();

void ActuateCollector();
void OpenCollector();
void CloseCollector();

void LowerRamp();
void LiftRamp();

void UpdateCollector();

// void CalibrateCollectionSystem(RobotFSM* fsm);

#endif