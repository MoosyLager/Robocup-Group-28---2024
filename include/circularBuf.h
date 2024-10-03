#ifndef CIRCULARBUF_H
#define CIRCULARBUF_H

#include <stdint.h>

#define CIRCULAR_BUF_SIZE 10

typedef struct {
	uint32_t bufSize;
	uint32_t wIndex;
	uint32_t rIndex;
	uint32_t *data;
} CircularBuff_t;

uint32_t *circularBufTInit(CircularBuff_t *buffer, uint32_t size);
void circularBufTWrite(CircularBuff_t *buffer, uint32_t entry);
uint32_t circularBufTRead(CircularBuff_t *buffer);
void circularBufTFree(CircularBuff_t *buffer);

#endif // CIRCULARBUF_H
