#ifndef FPGA_REGISTER_MAP_H
#define FPGA_REGISTER_MAP_H

#include <stdint.h>

typedef enum {
    FPGA_DIR_RD = 0,
    FPGA_DIR_WR = 1,
    FPGA_DIR_RW = 2
} FpgaDirection_e;

typedef struct {
    int      muxIdx;
    int      channel;
    uint8_t  data_len; /* bytes */
    uint8_t  i2cAddr;
    uint8_t  offset;
    uint8_t  direction; /* one of FPGA_DIR_* stored as uint8_t */
    float    scale;     /* conversion scale (default 1.0) */
} FpgaRegister;

/* Returns a pointer to the FpgaRegister for the given friendly name,
   or NULL if not found. */
const FpgaRegister *fpga_register_lookup(const char *name);

#endif /* FPGA_REGISTER_MAP_H */
