/* 
 * File:   matt_circular_buffer.h
 * Author: satom
 *
 * Created on January 6, 2025, 9:19 AM
 */

#ifndef MATT_CIRCULAR_BUFFER_H
#define	MATT_CIRCULAR_BUFFER_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// Define the circular buffer structure
typedef struct {
    int16_t *buffer;   // Pointer to the buffer array
    uint16_t head;       // Index of the head
    uint16_t tail;       // Index of the tail
    uint16_t max;        // Max capacity of the buffer
    bool full;         // Indicator if the buffer is full
} circular_buffer_t;

void circular_buffer_init(circular_buffer_t *cb, int16_t *buffer, uint16_t size);
void circular_buffer_reset(circular_buffer_t *cb);
bool circular_buffer_full(circular_buffer_t *cb);
bool circular_buffer_empty(circular_buffer_t *cb);
uint16_t circular_buffer_size(circular_buffer_t *cb);
void advance_head(circular_buffer_t *cb);
void advance_tail(circular_buffer_t *cb);
void circular_buffer_put(circular_buffer_t *cb, int16_t data);
bool circular_buffer_get(circular_buffer_t *cb, int16_t *data);
uint16_t circular_buffer_peek(circular_buffer_t *cb, int16_t *data, uint16_t n);
void circular_buffer_delete(circular_buffer_t *cb, uint16_t n);
void circular_buffer_decrement_all(circular_buffer_t *cb);
#endif	/* MATT_CIRCULAR_BUFFER_H */

