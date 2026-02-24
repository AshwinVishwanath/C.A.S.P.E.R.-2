/**
 * @file    radio_irq.h
 * @brief   SX1276 DIO EXTI flags — set in ISR, polled by radio_manager.
 */
#ifndef APP_RADIO_RADIO_IRQ_H
#define APP_RADIO_RADIO_IRQ_H

#include <stdint.h>

/* Volatile flags set by HAL_GPIO_EXTI_Callback, polled by radio_manager_tick() */
extern volatile uint8_t g_radio_dio0_flag;   /* DIO0: TxDone or RxDone */
extern volatile uint8_t g_radio_dio1_flag;   /* DIO1: RxTimeout        */
extern volatile uint8_t g_radio_dio3_flag;   /* DIO3: ValidHeader      */

/* Clear all DIO flags (call after handling or on radio reset) */
void radio_irq_clear_all(void);

#endif /* APP_RADIO_RADIO_IRQ_H */
