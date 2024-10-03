#ifndef CIRCULARBUF_H
#define CIRCULARBUF_H

#include <stdint.h>

#define CIRCULAR_BUF_SIZE 10

typedef struct {
	uint32_t bufSize;
	uint32_t wIndex;
	uint32_t rIndex;
	uint32_t *data;
} circularBuf_t;

uint32_t *circularBufTInit(circularBuf_t *buffer, uint32_t size);
void circularBufTWrite(circularBuf_t *buffer, uint32_t entry);
uint32_t circularBufTRead(circularBuf_t *buffer);
void circularBufTFree(circularBuf_t *buffer);

#endif // CIRCULARBUF_H
