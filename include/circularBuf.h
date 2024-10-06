#ifndef CIRCULARBUF_H
#define CIRCULARBUF_H

#include <stdint.h>
#include <stdlib.h>

#define CIRCULAR_BUF_SIZE 4
#define IMU_BUF_SIZE 10

typedef struct {
	uint16_t bufSize;
	uint16_t wIndex;
	uint16_t rIndex;
	uint16_t *data;
} CircBuff_t;

typedef struct {
    uint16_t wIndex;
    uint16_t rIndex;
    uint16_t bufSize;
    float *data;
} CircBuffFloat_t;


uint16_t *CircBuffInit(CircBuff_t *buffer, uint16_t size);
void CircBuffWrite(CircBuff_t *buffer, uint16_t entry);
uint16_t CircBufRead(CircBuff_t *buffer);
void CircBuffFree(CircBuff_t *buffer);

float* CircBuffFloatInit(CircBuffFloat_t *buffer, uint16_t size);
void CircBuffFloatWrite(CircBuffFloat_t *buffer, float entry);
float CircBufFloatRead(CircBuffFloat_t *buffer);
void CircBuffFloatFree(CircBuffFloat_t *buffer);
float CalculateFloatBufferMean(CircBuffFloat_t *buffer);


#endif // CIRCULARBUF_H
