#include "colour.h"

Adafruit_TCS34725 colourSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void InitColourSensor()
{
    if ( !colourSensor.begin(COLOUR_SENSOR_ADDRESS, &COLOUR_SENSOR_WIRE) ) {
        Serial.println("COLOUR SENSOR NOT DETECTED");
    }
}

Colour_t GetColour()
{
    uint16_t red, green, blue, clear;
    Colour_t colour;

    colourSensor.setInterrupt(false);
    delay(60);
    colourSensor.getRawData(&red, &green, &blue, &clear);
    colourSensor.setIntegrationTime(true);

    colour.R = red;
    colour.G = blue;
    colour.B = green;
    colour.C = clear;

    return colour;
}