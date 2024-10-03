#include <stdint.h>
#include "stdlib.h"
#include "circularBuf.h"

uint32_t *circularBufTInit(circularBuf_t *buffer, uint32_t size) {
	buffer->wIndex = 0;
	buffer->rIndex = 0;
	buffer->bufSize = size;
	buffer->data = (uint32_t *)calloc(size, sizeof(uint32_t));
	return buffer->data;
}

void circularBufTWrite(circularBuf_t *buffer, uint32_t entry) {
	buffer->data[buffer->wIndex] = entry;
	buffer->wIndex++;
	if (buffer->wIndex >= buffer->bufSize)
		buffer->wIndex = 0;
}

uint32_t circularBufTRead(circularBuf_t *buffer) {
	uint32_t entry;

	entry = buffer->data[buffer->rIndex];
	buffer->rIndex++;
	if (buffer->rIndex >= buffer->bufSize)
		buffer->rIndex = 0;
	return entry;
}

void circularBufTFree(circularBuf_t *buffer) {
	buffer->wIndex = 0;
	buffer->rIndex = 0;
	buffer->bufSize = 0;
	free(buffer->data);
	buffer->data = NULL;
}


