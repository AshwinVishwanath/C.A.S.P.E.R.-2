/**
 * @file    radio_irq.c
 * @brief   SX1276 DIO EXTI flags — set in ISR, polled by radio_manager.
 *
 * NO SPI transactions here. The HAL_GPIO_EXTI_Callback in main.c sets
 * these flags; the main-loop radio_manager_tick() reads and clears them.
 */
#include "radio_irq.h"

/* ---- Volatile flags polled by radio_manager_tick() ---- */
volatile uint8_t g_radio_dio0_flag = 0;  /* DIO0: TxDone or RxDone */
volatile uint8_t g_radio_dio1_flag = 0;  /* DIO1: RxTimeout        */
volatile uint8_t g_radio_dio3_flag = 0;  /* DIO3: ValidHeader      */

void radio_irq_clear_all(void)
{
    g_radio_dio0_flag = 0;
    g_radio_dio1_flag = 0;
    g_radio_dio3_flag = 0;
}
