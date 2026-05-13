/**
 ******************************************************************************
 * @file    usb_recovery.c
 * @brief   USB OTG_FS watchdog / recovery for EMC (IEC 61000-4-4 EFT) events.
 ******************************************************************************
 */

#include "usb_recovery.h"
#include "usb_device.h"
#include "usb_events.h"
#include "usbd_core.h"
#include "usbd_def.h"
#include "stm32h7xx_hal.h"

/* Globals defined in CubeMX-generated USB layer. */
extern USBD_HandleTypeDef hUsbDeviceFS;
extern PCD_HandleTypeDef  hpcd_USB_OTG_FS;
extern volatile uint8_t   usb_connected;     /* set/cleared by PCD callbacks */

/* ---------------- internal state ---------------- */
static uint32_t s_last_frame      = 0xFFFFFFFFu;
static uint32_t s_last_change_ms  = 0;
static uint32_t s_last_recover_ms = 0;
static uint32_t s_recover_count   = 0;
static uint8_t  s_initialised     = 0;

/* Read the device frame number (lower 14 bits of DSTS.FNSOF). The register is
 * updated by hardware on every received SOF whether or not SOF interrupts are
 * enabled, so it is a cheap, side-effect-free liveness signal. */
static inline uint32_t usb_get_frame_number(void)
{
    USB_OTG_DeviceTypeDef *dev =
        (USB_OTG_DeviceTypeDef *)((uint32_t)USB_OTG_FS + USB_OTG_DEVICE_BASE);
    return (dev->DSTS & USB_OTG_DSTS_FNSOF_Msk) >> USB_OTG_DSTS_FNSOF_Pos;
}

void usb_recovery_init(void)
{
    s_last_frame      = usb_get_frame_number();
    s_last_change_ms  = HAL_GetTick();
    s_last_recover_ms = s_last_change_ms;
    s_initialised     = 1;
}

uint32_t usb_recovery_get_count(void) { return s_recover_count; }

/* Perform the actual peripheral teardown / re-init. Does NOT call
 * Error_Handler() on failure - it just bumps the counter and lets the caller
 * try again on the next watchdog tick. */
bool usb_recovery_force(void)
{
    s_recover_count++;
    s_last_recover_ms = HAL_GetTick();

    /* 1. Mask the OTG IRQ while we tear down so partial state cannot fire
     *    spurious callbacks into a half-initialised stack. */
    HAL_NVIC_DisableIRQ(OTG_FS_IRQn);

    /* 2. Tell the application the link is gone so it can flush queues / drop
     *    any "TxBusy" wait loops. */
    if (usb_connected) {
        usb_connected = 0;
        usb_notify_disconnect();
    }

    /* 3. Best-effort stop + deinit of the device library. Ignore failures -
     *    the peripheral is about to be force-reset anyway. */
    (void)USBD_Stop(&hUsbDeviceFS);
    (void)USBD_DeInit(&hUsbDeviceFS);

    /* 4. Hard-reset the OTG_FS peripheral via RCC. This clears every internal
     *    state machine, FIFO pointer and stuck status bit that an EFT burst
     *    may have corrupted. A short delay between assert/release is required
     *    by the reference manual. */
    __HAL_RCC_USB_OTG_FS_FORCE_RESET();
    for (volatile uint32_t i = 0; i < 1000; ++i) { __NOP(); }
    __HAL_RCC_USB_OTG_FS_RELEASE_RESET();
    for (volatile uint32_t i = 0; i < 1000; ++i) { __NOP(); }

    /* 5. Re-initialise the full USB device stack. MX_USB_DEVICE_Init() also
     *    re-enables the OTG IRQ via HAL_PCD_MspInit(). */
    MX_USB_DEVICE_Init();

    /* 6. Reseed the watchdog so we do not immediately retrigger while the
     *    host re-enumerates. */
    s_last_frame     = usb_get_frame_number();
    s_last_change_ms = HAL_GetTick();
    return true;
}

void usb_recovery_task(void)
{
    if (!s_initialised) {
        usb_recovery_init();
        return;
    }

    uint32_t now   = HAL_GetTick();
    uint32_t frame = usb_get_frame_number();

    /* Any change in the frame number means the host is still talking to us. */
    if (frame != s_last_frame) {
        s_last_frame     = frame;
        s_last_change_ms = now;
        return;
    }

    /* Only act when the stack believes we are enumerated AND configured. In
     * any other state (default / addressed / suspended) the absence of SOFs
     * is legitimate and must not trigger a recovery. */
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) {
        s_last_change_ms = now;
        return;
    }

    /* Enforce a cool-down so we do not thrash the peripheral if recovery
     * itself takes a moment to bring the link back up. */
    if ((now - s_last_recover_ms) < USB_RECOVERY_COOLDOWN_MS) {
        return;
    }

    if ((now - s_last_change_ms) >= USB_RECOVERY_STALL_MS) {
        usb_recovery_force();
    }
}
