/* Core/Inc/pdc_buffer.h */
#ifndef INC_PDC_BUFFER_H_
#define INC_PDC_BUFFER_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define PDC_BUFFER_CAPACITY 256

typedef struct __attribute__((packed)) {
    uint32_t frame_idx;
    uint16_t pdc_raw;
    uint8_t  flags;   /* bit 0 = dark_slot */
} pdc_sample_t;

#define PDC_FLAG_DARK_SLOT (1u << 0)

void     pdc_buffer_reset(void);
bool     pdc_buffer_push(const pdc_sample_t *sample);  /* drop-oldest on overflow, returns true if a drop occurred */
size_t   pdc_buffer_drain(pdc_sample_t *out, size_t max_samples);
uint16_t pdc_buffer_dropped_since_last_drain(void);
size_t   pdc_buffer_count(void);

#endif
