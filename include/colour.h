#ifndef COLOUR_H
#define COLOUR_H

#include <Adafruit_TCS34725.h>

#define COLOUR_SENSOR_ADDRESS 0x29
#define COLOUR_SENSOR_WIRE    Wire1

typedef struct {
    uint16_t R;
    uint16_t G;
    uint16_t B;
    uint16_t C;
} Colour_t;

extern Adafruit_TCS34725 colourSensor;

void InitColourSensor();
Colour_t GetColour();

#endif