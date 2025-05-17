/*----------------------------- Include Files -----------------------------*/
// This module
#include "matt_circular_buffer.h"


// Hardware
#include <xc.h>
//#include <proc/p32mx170f256b.h>

// Event & Services Framework
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_DeferRecall.h"
#include "ES_Port.h"
#include "terminal.h"
#include "dbprintf.h"
/*----------------------------- Module Defines ----------------------------*/

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/

/*---------------------------- Module Variables ---------------------------*/

// Initialize the circular buffer
void circular_buffer_init(circular_buffer_t *cb, int16_t *buffer, uint16_t size) {
    cb->buffer = buffer;
    cb->max = size;
    cb->head = 0;
    cb->tail = 0;
    cb->full = false;
}

// Reset the circular buffer to empty state
void circular_buffer_reset(circular_buffer_t *cb) {
    cb->head = 0;
    cb->tail = 0;
    cb->full = false;
}

// Check if the buffer is full
bool circular_buffer_full(circular_buffer_t *cb) {
    return cb->full;
}

// Check if the buffer is empty
bool circular_buffer_empty(circular_buffer_t *cb) {
    return (!cb->full && (cb->head == cb->tail));
}

// Calculate the current size of the buffer
uint16_t circular_buffer_size(circular_buffer_t *cb) {
    uint16_t size = cb->max;

    if (!cb->full) {
        if (cb->head >= cb->tail) {
            size = cb->head - cb->tail;
        } else {
            size = cb->max + cb->head - cb->tail;
        }
    }

    return size;
}

// Advance the head index
void advance_head(circular_buffer_t *cb) {
    if (cb->full) {
        cb->tail = (cb->tail + 1) % cb->max;
    }

    cb->head = (cb->head + 1) % cb->max;
    cb->full = (cb->head == cb->tail);
}

// Advance the tail index
void advance_tail(circular_buffer_t *cb) {
    cb->full = false;
    cb->tail = (cb->tail + 1) % cb->max;
}

// Add data to the buffer
void circular_buffer_put(circular_buffer_t *cb, int16_t data) {
    cb->buffer[cb->head] = data;
    advance_head(cb);
}

// Get data from the buffer
bool circular_buffer_get(circular_buffer_t *cb, int16_t *data) {
    if (circular_buffer_empty(cb)) {
        return false;  // Buffer is empty, nothing to get
    }

    *data = cb->buffer[cb->tail];
    advance_tail(cb);
    return true;
}

// Peek at n entries in the buffer without removing them
uint16_t circular_buffer_peek(circular_buffer_t *cb, int16_t *data, uint16_t n) {
    if (circular_buffer_empty(cb) || n == 0) {
        return 0;  // Buffer is empty or no entries requested
    }

    uint16_t count = 0;
    uint16_t index = cb->tail;

    // We don't check if we are accessing unfull spots!
    while (count < n) {
        data[count] = cb->buffer[index];
        index = (index + 1) % cb->max;
        count++;
    }

    return count;
}

// Delete n entries from the buffer
void circular_buffer_delete(circular_buffer_t *cb, uint16_t n) {
    if (n == 0 || circular_buffer_empty(cb)) {
        return;  // No entries to delete or buffer is empty
    }

    uint16_t delete_count = 0;

    // Advance the tail to delete n entries
    while (delete_count < n && cb->tail != cb->head) {
        advance_tail(cb);
        delete_count++;
    }
}

void circular_buffer_decrement_all(circular_buffer_t *cb) {
    if (circular_buffer_empty(cb)) {
        return;  // Buffer is empty, nothing to decrement
    }

    size_t index = cb->tail;

    while (index != cb->head || cb->full) {
        cb->buffer[index]--;
        index = (index + 1) % cb->max;

        if (index == cb->head && cb->full) {
            break;
        }
    }
}