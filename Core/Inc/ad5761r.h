#ifndef AD5761R_H
#define AD5761R_H

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------- AD5761/AD5761R command/address (upper 4 bits of 24-bit frame) ---- */
#define AD5761_ADDR(addr)              ((uint32_t)((addr) & 0x0F) << 16)
#define AD5761_ADDR_NOOP               0x0
#define AD5761_ADDR_DAC_WRITE          0x3
#define AD5761_ADDR_CTRL_WRITE_REG     0x4
#define AD5761_ADDR_SW_DATA_RESET      0x7
#define AD5761_ADDR_DAC_READ           0xB
#define AD5761_ADDR_CTRL_READ_REG      0xC
#define AD5761_ADDR_SW_FULL_RESET      0xF

/* CTRL register bits (matches Linux driver flags) */
#define AD5761_CTRL_USE_INTVREF        (1u << 5)
#define AD5761_CTRL_ETS                (1u << 6)   /* Enable Thermal Shutdown */

/* Output range (matches the Linux enum / m,c table) */
typedef enum {
    AD5761_VOLTAGE_RANGE_M10V_10V = 0,
    AD5761_VOLTAGE_RANGE_0V_10V   = 1,
    AD5761_VOLTAGE_RANGE_M5V_5V   = 2,
    AD5761_VOLTAGE_RANGE_0V_5V    = 3,
    AD5761_VOLTAGE_RANGE_M2V5_7V5 = 4,
    AD5761_VOLTAGE_RANGE_M3V_3V   = 5,
    AD5761_VOLTAGE_RANGE_0V_16V   = 6,
    AD5761_VOLTAGE_RANGE_0V_20V   = 7,
} AD5761R_Range;

/* Optional GPIOs (set .Port=NULL if not used) */
typedef struct {
    GPIO_TypeDef *Port;
    uint16_t      Pin;
    /* Active level is low for /SYNC, /LDAC, /CLR, /RESET on ADI parts */
} AD5761R_Gpio;

/* Driver handle */
typedef struct {
    SPI_HandleTypeDef *hspi;  /* Pre-initialized HAL SPI (e.g., &hspi1) */

    /* If you want manual chip select across multi-frame readbacks, provide CS */
    AD5761R_Gpio CSn;   /* /SYNC (chip-select). If Port==NULL, HAL NSS is used. */

    /* Reference: external in mV (2000..3000) or internal 2500mV if use_internal_ref=true */
    uint16_t vref_mV;
    bool     use_internal_ref; /* true => select internal 2.5V reference */

    /* Device config */
    AD5761R_Range range;
    uint8_t       n_bits;      /* 16 for AD5761(R), 12 for AD5721(R) */

    /* Private: cached last raw code (16-bit storage) */
    uint16_t      last_code;
} AD5761R_Handle;

/* ---------- Public API ----------------------------------------------------- */

/* Initialize device:
   - If RESET pin provided, performs a hardware reset pulse.
   - Issues a SW full reset, programs CTRL (ETS + optional internal VREF), sets output range.
   - If CLR pin is provided and held low externally, you should release it before init. */
HAL_StatusTypeDef AD5761R_Init(AD5761R_Handle *dev);

/* Software full reset (restores defaults). Range/CTRL must be re-written after this. */
HAL_StatusTypeDef AD5761R_SoftwareFullReset(AD5761R_Handle *dev);

/* Software data reset (DAC code -> midscale per datasheet). */
HAL_StatusTypeDef AD5761R_SoftwareDataReset(AD5761R_Handle *dev);

/* Set output range (programs CTRL). Keeps ETS enabled; selects internal ref if configured. */
HAL_StatusTypeDef AD5761R_SetRange(AD5761R_Handle *dev, AD5761R_Range range);

/* Write raw DAC code (right-justified, 16-bit container; driver shifts if n_bits<16). */
HAL_StatusTypeDef AD5761R_WriteRaw(AD5761R_Handle *dev, uint16_t code);

/* Read raw DAC code back (two-frame read as per datasheet). */
HAL_StatusTypeDef AD5761R_ReadRaw(AD5761R_Handle *dev, uint16_t *out_code);

/* Convenience: set output voltage in millivolts (clamped to range).
   Returns HAL_OK and writes the nearest code. */
HAL_StatusTypeDef AD5761R_SetVoltage_mV(AD5761R_Handle *dev, int32_t vout_mV);

/* Get scale (mV per LSB) as a rational: scale = (num / 2^n_bits) mV; returns num and n_bits. */
void AD5761R_GetScale_mV_per_code(const AD5761R_Handle *dev, int32_t *num_mV, uint8_t *den_pow2);

/* Get offset (LSBs) matching Linux calculation: offset = -2^n_bits * c / m */
int32_t AD5761R_GetOffset_LSB(const AD5761R_Handle *dev);

int32_t AD5761R_GetVoltage_mV(AD5761R_Handle *dev);
HAL_StatusTypeDef AD5761R_ReadControl(const AD5761R_Handle *dev, uint16_t *out_ctrl);
HAL_StatusTypeDef AD5761R_CheckComms(const AD5761R_Handle *dev);

#ifdef __cplusplus
}
#endif

#endif /* AD5761R_H */
