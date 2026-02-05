#include "usb_events.h"

static usb_disconnect_cb_t disconnect_cb = 0;
static usb_connect_cb_t connect_cb = 0;

void usb_register_disconnect_callback(usb_disconnect_cb_t cb)
{
    disconnect_cb = cb;
}

void usb_register_connect_callback(usb_connect_cb_t cb)
{
    connect_cb = cb;
}

void usb_notify_disconnect(void)
{
    if (disconnect_cb)
    {
        disconnect_cb();
    }
}

void usb_notify_connect(void)
{
    if (connect_cb)
    {
        connect_cb();
    }
}
