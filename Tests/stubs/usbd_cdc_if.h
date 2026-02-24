/* USB CDC interface stub */
#ifndef __USBD_CDC_IF_H
#define __USBD_CDC_IF_H

#include <stdint.h>

#define APP_RX_DATA_SIZE 256
#define CDC_RING_SIZE    256

uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);

/* CDC receive ring buffer (used by cmd_router) */
extern volatile uint8_t cdc_rx_ring[CDC_RING_SIZE];
extern volatile uint16_t cdc_rx_head;
extern volatile uint16_t cdc_rx_tail;

/* CDC ring buffer helpers (implemented in stub_helpers.c) */
uint16_t cdc_ring_available(void);
uint8_t  cdc_ring_read_byte(void);

#endif /* __USBD_CDC_IF_H */
