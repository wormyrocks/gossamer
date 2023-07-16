// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2017-2023, Alex Taradov <alex@taradov.com>. All rights reserved.

/*- Includes ----------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "samd11.h"
#include "pins.h"
#include "sercom.h"
#include "uart_async.h"
#include "usb_uart.h"

/*- Definitions -------------------------------------------------------------*/
#define UART_BUF_SIZE            256
#define UART_SERCOM_TXPO 1
#define UART_SERCOM_RXPO 3

// Gossamer uart driver would expect 0 here, not SERCOM0
#define UART_SERCOM SERCOM0
#define UART_SERCOM_GCLK_ID      SERCOM0_GCLK_ID_CORE
#define UART_SERCOM_APBCMASK     PM_APBCMASK_SERCOM0
#define UART_SERCOM_IRQ_INDEX    SERCOM0_IRQn
#define UART_SERCOM_IRQ_HANDLER  irq_handler_sercom0


/*- Types ------------------------------------------------------------------*/
typedef struct
{
  int       wr;
  int       rd;
  uint16_t  data[UART_BUF_SIZE];
} fifo_buffer_t;

/*- Variables --------------------------------------------------------------*/
static volatile fifo_buffer_t uart_rx_fifo;
static volatile fifo_buffer_t uart_tx_fifo;
static volatile bool uart_fifo_overflow = false;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
void uart_init(cdc_line_coding_t *line_coding)
{
  int chsize, form, pmode, sbmode, baud, fp;

  HAL_GPIO_UART_TX_out();
  HAL_GPIO_UART_TX_clr();
  HAL_GPIO_UART_TX_pmuxen(HAL_GPIO_PMUX_SERCOM_ALT);

  HAL_GPIO_UART_RX_pullup();

  PM->APBCMASK.reg |= UART_SERCOM_APBCMASK;

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(UART_SERCOM_GCLK_ID) |
      GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(0);

  UART_SERCOM->USART.CTRLA.reg = SERCOM_USART_CTRLA_SWRST;
  while (UART_SERCOM->USART.CTRLA.bit.SWRST);

  uart_tx_fifo.wr = 0;
  uart_tx_fifo.rd = 0;

  uart_rx_fifo.wr = 0;
  uart_rx_fifo.rd = 0;

  uart_fifo_overflow = false;

  if (line_coding->data_bits == 5)
    chsize = 5;
  else if (line_coding->data_bits == 6)
    chsize = 6;
  else if (line_coding->data_bits == 7)
    chsize = 7;
  else
    chsize = 0;

  if (line_coding->parity == CDC_LINE_CODING_PARITY_NONE)
    form = 0;
  else
    form = 1;

  if (line_coding->parity == CDC_LINE_CODING_PARITY_EVEN)
    pmode = 0;
  else
    pmode = SERCOM_USART_CTRLB_PMODE;

  if (line_coding->stop_bits == CDC_LINE_CONDING_STOP_BITS_1)
    sbmode = 0;
  else
    sbmode = SERCOM_USART_CTRLB_SBMODE;

  baud = F_CPU / (16 * line_coding->bit_rate);
  fp = (F_CPU / line_coding->bit_rate - 16 * baud) / 2;

  UART_SERCOM->USART.CTRLA.reg =
        SERCOM_USART_CTRLA_DORD | SERCOM_USART_CTRLA_MODE_USART_INT_CLK |
      SERCOM_USART_CTRLA_FORM(form) | SERCOM_USART_CTRLA_SAMPR(1) |
      SERCOM_USART_CTRLA_RXPO(UART_SERCOM_RXPO) |
      SERCOM_USART_CTRLA_TXPO(UART_SERCOM_TXPO);

  UART_SERCOM->USART.CTRLB.reg = SERCOM_USART_CTRLB_RXEN | SERCOM_USART_CTRLB_TXEN |
      SERCOM_USART_CTRLB_CHSIZE(chsize) | pmode | sbmode;

  UART_SERCOM->USART.BAUD.reg =
      SERCOM_USART_BAUD_FRACFP_BAUD(baud) | SERCOM_USART_BAUD_FRACFP_FP(fp);

  UART_SERCOM->USART.CTRLA.reg |= SERCOM_USART_CTRLA_ENABLE;

  UART_SERCOM->USART.INTENSET.reg = SERCOM_USART_INTENSET_RXC;

  NVIC_EnableIRQ(UART_SERCOM_IRQ_INDEX);
}

//-----------------------------------------------------------------------------
void uart_close(void)
{
  UART_SERCOM->USART.CTRLA.reg = SERCOM_USART_CTRLA_SWRST;
  while (UART_SERCOM->USART.CTRLA.bit.SWRST);
}

//-----------------------------------------------------------------------------
static bool fifo_push(volatile fifo_buffer_t *fifo, int value)
{
  int next_wr = (fifo->wr + 1) % UART_BUF_SIZE;

  if (next_wr == fifo->rd)
    return false;

  fifo->data[fifo->wr] = value;
  fifo->wr = next_wr;

  return true;
}

//-----------------------------------------------------------------------------
static bool fifo_pop(volatile fifo_buffer_t *fifo, int *value)
{
  if (fifo->rd == fifo->wr)
    return false;

  *value = fifo->data[fifo->rd];
  fifo->rd = (fifo->rd + 1) % UART_BUF_SIZE;

  return true;
}

//-----------------------------------------------------------------------------
bool uart_write_byte(int byte)
{
  bool res = false;

  NVIC_DisableIRQ(UART_SERCOM_IRQ_INDEX);

  if (fifo_push(&uart_tx_fifo, byte))
  {
    UART_SERCOM->USART.INTENSET.reg = SERCOM_USART_INTENSET_DRE;
    res = true;
  }

  NVIC_EnableIRQ(UART_SERCOM_IRQ_INDEX);

  return res;
}

//-----------------------------------------------------------------------------
bool uart_read_byte(int *byte)
{
  bool res = false;

  NVIC_DisableIRQ(UART_SERCOM_IRQ_INDEX);

  if (uart_fifo_overflow)
  {
    *byte = (USB_CDC_SERIAL_STATE_OVERRUN << 8);
    uart_fifo_overflow = false;
    res = true;
  }
  else if (fifo_pop(&uart_rx_fifo, byte))
  {
    res = true;
  }

  NVIC_EnableIRQ(UART_SERCOM_IRQ_INDEX);

  return res;
}

//-----------------------------------------------------------------------------
void uart_set_break(bool brk)
{
  if (brk)
    HAL_GPIO_UART_TX_pmuxdis();
  else
    HAL_GPIO_UART_TX_pmuxen(HAL_GPIO_PMUX_SERCOM_ALT);
}

//-----------------------------------------------------------------------------
void UART_SERCOM_IRQ_HANDLER(void)
{
  int flags = UART_SERCOM->USART.INTFLAG.reg;

  if (flags & SERCOM_USART_INTFLAG_RXC)
  {
    int status = UART_SERCOM->USART.STATUS.reg;
    int byte = UART_SERCOM->USART.DATA.reg;
    int state = 0;

    UART_SERCOM->USART.STATUS.reg = status;

    if (status & SERCOM_USART_STATUS_BUFOVF)
      state |= USB_CDC_SERIAL_STATE_OVERRUN;

    if (status & SERCOM_USART_STATUS_FERR)
      state |= USB_CDC_SERIAL_STATE_FRAMING;

    if (status & SERCOM_USART_STATUS_PERR)
      state |= USB_CDC_SERIAL_STATE_PARITY;

    byte |= (state << 8);

    if (!fifo_push(&uart_rx_fifo, byte))
      uart_fifo_overflow = true;
  }

  if (flags & SERCOM_USART_INTFLAG_DRE)
  {
    int byte;

    if (fifo_pop(&uart_tx_fifo, &byte))
      UART_SERCOM->USART.DATA.reg = byte;
    else
      UART_SERCOM->USART.INTENCLR.reg = SERCOM_USART_INTENCLR_DRE;
  }
}










#if 0

/*- Definitions -------------------------------------------------------------*/
#define UART_BUF_SIZE            256

/*- Types ------------------------------------------------------------------*/
typedef struct
{
  int       wr;
  int       rd;
  uint16_t  data[UART_BUF_SIZE];
} fifo_buffer_t;

/*- Variables --------------------------------------------------------------*/
static volatile fifo_buffer_t uart_rx_fifo;
static volatile fifo_buffer_t uart_tx_fifo;
static volatile bool uart_fifo_overflow = false;
bool fifo_push(volatile fifo_buffer_t *fifo, uint8_t value);
bool fifo_pop(volatile fifo_buffer_t *fifo, uint8_t *value);

void uart_init_async(uint8_t sercom, uart_txpo_t txpo, uart_rxpo_t rxpo, uint32_t baud) {
        uart_init_custom(sercom, txpo, rxpo, baud);
        Sercom* SERCOM = SERCOM_Peripherals[sercom].sercom;
        SERCOM->USART.INTENSET.reg = SERCOM_USART_INTENSET_RXC;
        uart_tx_fifo.wr = 0;
        uart_tx_fifo.rd = 0;
        uart_rx_fifo.wr = 0;
        uart_rx_fifo.rd = 0;
        uart_fifo_overflow = false;
        NVIC_EnableIRQ(SERCOM0_IRQn + sercom);
}

bool fifo_push(volatile fifo_buffer_t *fifo, uint8_t value)
{
  int next_wr = (fifo->wr + 1) % UART_BUF_SIZE;

  if (next_wr == fifo->rd)
    return false;

  fifo->data[fifo->wr] = value;
  fifo->wr = next_wr;

  return true;
}

bool fifo_pop(volatile fifo_buffer_t *fifo, uint8_t *value)
{
  if (fifo->rd == fifo->wr)
    return false;

  *value = fifo->data[fifo->rd];
  fifo->rd = (fifo->rd + 1) % UART_BUF_SIZE;

  return true;
}

bool uart_write_async(uint8_t sercom, uint8_t byte) {
    bool res = false;
    Sercom* SERCOM = SERCOM_Peripherals[sercom].sercom;
    NVIC_DisableIRQ(SERCOM0_IRQn + sercom);
    
    if (fifo_push(&uart_tx_fifo, byte))
    {
        SERCOM->USART.INTENSET.reg = SERCOM_USART_INTENSET_DRE;
        res = true;
    }

    NVIC_EnableIRQ(SERCOM0_IRQn + sercom);

    return res;
}

bool uart_read_async (uint8_t sercom, uint8_t *byte) {
  bool res = false;
  NVIC_DisableIRQ(SERCOM0_IRQn + sercom);

  if (uart_fifo_overflow)
  {
    //*byte = (USB_CDC_SERIAL_STATE_OVERRUN << 8);
    uart_fifo_overflow = false;
    res = true;
  }
  else if (fifo_pop(&uart_rx_fifo, byte))
  {
    res = true;
  }

    NVIC_EnableIRQ(SERCOM0_IRQn + sercom);

  return res;
}
#endif