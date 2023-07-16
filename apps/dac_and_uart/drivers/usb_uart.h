#include "usb.h"

#ifndef _USB_UART_H_
#define _USB_UART_H_


// From ataradov usb_cdc.h
#define USB_CDC_BREAK_DURATION_DISABLE   0
#define USB_CDC_BREAK_DURATION_INFINITE  0xffff
enum
{
  USB_CDC_CTRL_SIGNAL_DTE_PRESENT        = (1 << 0), // DTR
  USB_CDC_CTRL_SIGNAL_ACTIVATE_CARRIER   = (1 << 1), // RTS
};
enum
{
  USB_CDC_SERIAL_STATE_DCD     = (1 << 0),
  USB_CDC_SERIAL_STATE_DSR     = (1 << 1),
  USB_CDC_SERIAL_STATE_BREAK   = (1 << 2),
  USB_CDC_SERIAL_STATE_RING    = (1 << 3),
  USB_CDC_SERIAL_STATE_FRAMING = (1 << 4),
  USB_CDC_SERIAL_STATE_PARITY  = (1 << 5),
  USB_CDC_SERIAL_STATE_OVERRUN = (1 << 6),
};

static void tx_task(void);
static void send_buffer(void);
static void rx_task(void);
static void break_task(void);
static void uart_timer_task(void);
void usb_cdc_line_coding_updated(cdc_line_coding_t *line_coding);
void usb_cdc_control_line_state_update(int line_state);
void usb_cdc_send_break(int duration);
void usb_cdc_send_callback(void);
void usb_cdc_recv_callback(int size);
void usb_configuration_callback(int config);
#endif