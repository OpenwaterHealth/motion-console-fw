#ifndef USB_EVENTS_H
#define USB_EVENTS_H

#include <stdint.h>

typedef void (*usb_disconnect_cb_t)(void);
typedef void (*usb_connect_cb_t)(void);

/* Register callback */
void usb_register_disconnect_callback(usb_disconnect_cb_t cb);
void usb_register_connect_callback(usb_connect_cb_t cb);

/* Internal use by USB ISR */
void usb_notify_disconnect(void);
void usb_notify_connect(void);

#endif
