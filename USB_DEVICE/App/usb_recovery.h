/**
 ******************************************************************************
 * @file    usb_recovery.h
 * @brief   USB OTG_FS watchdog / recovery for EMC (IEC 61000-4-4 EFT) events.
 *
 * EFT bursts can leave the STM32H7 OTG_FS device controller in a hung state:
 *   - Host re-enumerates but the device never sees Suspend/Reset
 *   - CDC TxState stays "busy" because the IN transfer was killed mid-flight
 *   - The OTG core stops generating SOFs / receiving any traffic
 *
 * This module monitors the SOF frame-number register and, when the device is
 * supposed to be configured but has had no bus activity for a configurable
 * timeout, performs a full peripheral re-initialization (RCC force-reset +
 * USBD_DeInit + MX_USB_DEVICE_Init) to bring the link back without a board
 * reset.
 ******************************************************************************
 */
#ifndef USB_RECOVERY_H
#define USB_RECOVERY_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Time without any SOF activity (while the stack believes it is enumerated)
 * after which we consider the link hung and trigger recovery. */
#ifndef USB_RECOVERY_STALL_MS
#define USB_RECOVERY_STALL_MS   750u
#endif

/* Minimum time between two consecutive recovery attempts. */
#ifndef USB_RECOVERY_COOLDOWN_MS
#define USB_RECOVERY_COOLDOWN_MS 1500u
#endif

/* Initialise watchdog state. Call once after MX_USB_DEVICE_Init(). */
void usb_recovery_init(void);

/* Periodic poll. Call from the main super-loop (every few ms is fine). */
void usb_recovery_task(void);

/* Force an immediate recovery cycle (DeInit + RCC reset + Init). Safe to call
 * from thread context only (NOT from an ISR). Returns true on success. */
bool usb_recovery_force(void);

/* Diagnostic counter: number of recovery cycles performed since boot. */
uint32_t usb_recovery_get_count(void);

#ifdef __cplusplus
}
#endif

#endif /* USB_RECOVERY_H */
