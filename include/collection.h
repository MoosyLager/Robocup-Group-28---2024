#ifndef COLLECTION_H
#define COLLECTION_H

#include "encoder.h"
#include "motor.h"
#include "sensor.h"

#define COLLECTOR_OPEN_OFFSET   0
#define COLLECTOR_CLOSED_OFFSET 0
#define RAMP_DOWN_OFFSET        0

void CalibrateCollector();
void CalibrateStepper();
void ActuateCollector();
void OpenCollector();
void CloseCollector();

#endif