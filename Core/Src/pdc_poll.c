/* Core/Src/pdc_poll.c */
#include "pdc_poll.h"
#include "pdc_buffer.h"
#include "trigger.h"
#include "tca9548a.h"
#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdio.h>

#define PDC_MUX_INDEX   1
#define PDC_CHANNEL     7
#define PDC_I2C_ADDR    0x41
#define PDC_REG         0x1C
#define PDC_BYTES       2
#define PDC_SETTLE_MS   1   /* FPGA averaging window after laser pulse falls */

static uint32_t s_pending_since_tick = 0;
static bool     s_have_pending = false;
static bool     s_dark_slot = false;
static uint32_t s_frame_idx = 0;
static uint32_t s_fail_count = 0;

void pdc_poll_init(void) {
    s_pending_since_tick = 0;
    s_have_pending = false;
    s_fail_count = 0;
}

void pdc_poll_tick(void) {
    /* Pick up any new pending sample from the LSYNC ISR. */
    if (!s_have_pending) {
        bool d; uint32_t f;
        if (consume_pdc_sample_pending(&d, &f)) {
            s_have_pending = true;
            s_dark_slot = d;
            s_frame_idx = f;
            s_pending_since_tick = HAL_GetTick();
        }
    }

    if (!s_have_pending) return;

    /* Wait for the FPGA peak-power averaging window to complete. */
    if ((HAL_GetTick() - s_pending_since_tick) < PDC_SETTLE_MS) return;

    uint8_t bytes[PDC_BYTES] = {0};
    int8_t rc = TCA9548A_Read_Data(PDC_MUX_INDEX, PDC_CHANNEL, PDC_I2C_ADDR,
                                   PDC_REG, PDC_BYTES, bytes);
    s_have_pending = false;   /* always consume - don't get stuck on a failing read */

    if (rc != TCA9548A_OK) {
        s_fail_count++;
        /* Log sparsely so we don't flood the UART. */
        if ((s_fail_count & 0x3F) == 1) {
            printf("pdc_poll: I2C read failed (count=%lu, rc=%d)\r\n",
                   (unsigned long)s_fail_count, (int)rc);
        }
        return;
    }

    pdc_sample_t sample = {
        .frame_idx = s_frame_idx,
        .pdc_raw   = (uint16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8)),
        .flags     = (uint8_t)(s_dark_slot ? PDC_FLAG_DARK_SLOT : 0u),
    };
    (void)pdc_buffer_push(&sample);
}
