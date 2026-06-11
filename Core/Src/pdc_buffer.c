/* Core/Src/pdc_buffer.c */
#include "pdc_buffer.h"
#include <string.h>

static pdc_sample_t s_buf[PDC_BUFFER_CAPACITY];
static volatile uint16_t s_head;        /* write idx */
static volatile uint16_t s_tail;        /* read idx */
static volatile uint16_t s_count;
static volatile uint16_t s_dropped_pending;  /* drops since last drain */

void pdc_buffer_reset(void) {
    s_head = 0; s_tail = 0; s_count = 0; s_dropped_pending = 0;
    memset(s_buf, 0, sizeof(s_buf));
}

bool pdc_buffer_push(const pdc_sample_t *sample) {
    bool dropped = false;
    if (s_count == PDC_BUFFER_CAPACITY) {
        /* drop oldest */
        s_tail = (uint16_t)((s_tail + 1) % PDC_BUFFER_CAPACITY);
        s_count--;
        s_dropped_pending++;
        dropped = true;
    }
    s_buf[s_head] = *sample;
    s_head = (uint16_t)((s_head + 1) % PDC_BUFFER_CAPACITY);
    s_count++;
    return dropped;
}

size_t pdc_buffer_drain(pdc_sample_t *out, size_t max_samples) {
    size_t n = 0;
    while (n < max_samples && s_count > 0) {
        out[n++] = s_buf[s_tail];
        s_tail = (uint16_t)((s_tail + 1) % PDC_BUFFER_CAPACITY);
        s_count--;
    }
    return n;
}

uint16_t pdc_buffer_dropped_since_last_drain(void) {
    uint16_t d = s_dropped_pending;
    s_dropped_pending = 0;
    return d;
}

size_t pdc_buffer_count(void) {
    return s_count;
}

void pdc_buffer_account_drops(uint16_t n) {
    s_dropped_pending = (uint16_t)(s_dropped_pending + n);
}
