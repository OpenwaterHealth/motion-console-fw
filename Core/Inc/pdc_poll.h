/* Core/Inc/pdc_poll.h */
#ifndef INC_PDC_POLL_H_
#define INC_PDC_POLL_H_

#include <stdint.h>

void pdc_poll_init(void);
/* Call from the main loop. Performs the I2C read of the safety FPGA peak-power
 * register and pushes a pdc_sample_t to the buffer when one is pending and the
 * post-pulse settling time has elapsed. Non-blocking otherwise. */
void pdc_poll_tick(void);

#endif
