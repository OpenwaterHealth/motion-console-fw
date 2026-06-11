#include <assert.h>
#include <stdio.h>
#include "../Core/Inc/pdc_buffer.h"
/* pull in the source directly for the host build */
#include "../Core/Src/pdc_buffer.c"

static pdc_sample_t mk(uint32_t f) { pdc_sample_t s = { f, (uint16_t)(f & 0xFFFF), 0 }; return s; }

int main(void) {
    pdc_buffer_reset();
    assert(pdc_buffer_count() == 0);

    /* push 3, drain 3 */
    pdc_sample_t s1 = mk(1), s2 = mk(2), s3 = mk(3);
    assert(!pdc_buffer_push(&s1));
    assert(!pdc_buffer_push(&s2));
    assert(!pdc_buffer_push(&s3));
    assert(pdc_buffer_count() == 3);

    pdc_sample_t out[8];
    size_t n = pdc_buffer_drain(out, 8);
    assert(n == 3);
    assert(out[0].frame_idx == 1 && out[1].frame_idx == 2 && out[2].frame_idx == 3);
    assert(pdc_buffer_dropped_since_last_drain() == 0);

    /* overflow: push 260, capacity 256, drain should see frames 5..260 (256 samples), drop count 4 */
    pdc_buffer_reset();
    for (uint32_t i = 1; i <= 260; i++) {
        pdc_sample_t s = mk(i);
        pdc_buffer_push(&s);
    }
    assert(pdc_buffer_count() == PDC_BUFFER_CAPACITY);
    pdc_sample_t big[PDC_BUFFER_CAPACITY];
    n = pdc_buffer_drain(big, PDC_BUFFER_CAPACITY);
    assert(n == PDC_BUFFER_CAPACITY);
    assert(big[0].frame_idx == 5);    /* oldest dropped: 1..4 */
    assert(big[255].frame_idx == 260);
    assert(pdc_buffer_dropped_since_last_drain() == 4);

    /* second drain after read: dropped counter resets to 0 */
    assert(pdc_buffer_dropped_since_last_drain() == 0);

    /* partial drain */
    pdc_buffer_reset();
    for (uint32_t i = 1; i <= 10; i++) { pdc_sample_t s = mk(i); pdc_buffer_push(&s); }
    n = pdc_buffer_drain(out, 4);
    assert(n == 4);
    assert(out[0].frame_idx == 1 && out[3].frame_idx == 4);
    assert(pdc_buffer_count() == 6);

    printf("pdc_buffer host tests OK\n");
    return 0;
}
