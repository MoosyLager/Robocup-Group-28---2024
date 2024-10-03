#include "circularBuf.h"
#include <stdint.h>
#include <stdlib.h>

uint16_t *CircBuffInit(CircBuff_t *buffer, uint16_t size)
{
    buffer->wIndex = 0;
    buffer->rIndex = 0;
    buffer->bufSize = size;
    buffer->data = new uint16_t[size];
    return buffer->data;
}

void CircBuffWrite(CircBuff_t *buffer, uint16_t entry)
{
    buffer->data[buffer->wIndex] = entry;
    buffer->wIndex++;
    if ( buffer->wIndex >= buffer->bufSize )
        buffer->wIndex = 0;
}

uint16_t CircBufRead(CircBuff_t *buffer)
{
    uint16_t entry;

    entry = buffer->data[buffer->rIndex];
    buffer->rIndex++;
    if ( buffer->rIndex >= buffer->bufSize )
        buffer->rIndex = 0;
    return entry;
}

void CircBuffFree(CircBuff_t *buffer)
{
    buffer->wIndex = 0;
    buffer->rIndex = 0;
    buffer->bufSize = 0;
    delete[] buffer->data;
    buffer->data = NULL;
}
