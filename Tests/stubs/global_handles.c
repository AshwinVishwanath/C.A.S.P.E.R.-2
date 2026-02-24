/* Global peripheral handles — satisfies extern declarations from main.h and firmware */
#include "stm32h7xx_hal.h"

SPI_HandleTypeDef  hspi1, hspi2, hspi3, hspi4;
I2C_HandleTypeDef  hi2c1, hi2c2, hi2c3;
ADC_HandleTypeDef  hadc1, hadc2, hadc3;
TIM_HandleTypeDef  htim2;
UART_HandleTypeDef huart4;
QSPI_HandleTypeDef hqspi;

/* CDC ring buffer (used by cmd_router) */
volatile uint8_t cdc_rx_ring[256];
volatile uint16_t cdc_rx_head = 0;
volatile uint16_t cdc_rx_tail = 0;
