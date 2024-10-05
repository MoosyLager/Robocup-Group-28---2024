#ifndef CIRCULARBUF_H
#define CIRCULARBUF_H

#include <stdint.h>

#define CIRCULAR_BUF_SIZE 4

typedef struct {
	uint16_t bufSize;
	uint16_t wIndex;
	uint16_t rIndex;
	uint16_t *data;
} CircBuff_t;

uint16_t *CircBuffInit(CircBuff_t *buffer, uint16_t size);
void CircBuffWrite(CircBuff_t *buffer, uint16_t entry);
uint16_t CircBufRead(CircBuff_t *buffer);
void CircBuffFree(CircBuff_t *buffer);

#endif // CIRCULARBUF_H
