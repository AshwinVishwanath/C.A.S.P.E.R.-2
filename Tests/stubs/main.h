/* Stub main.h — provides GPIO pin defines and peripheral handle externs
 * that the firmware headers expect from Core/Inc/main.h */
#ifndef __MAIN_H
#define __MAIN_H

#include "stm32h7xx_hal.h"

/* ---- GPIO Pin Defines (from CubeMX) ---- */
#define SPI2_CS_Pin          GPIO_PIN_14
#define SPI2_CS_GPIO_Port    GPIOC
#define SPI2_INT_Pin         GPIO_PIN_15
#define SPI2_INT_GPIO_Port   GPIOC

#define SPI3_CS_Pin          GPIO_PIN_15
#define SPI3_CS_GPIO_Port    GPIOA
#define SPI3_INT_Pin         GPIO_PIN_2
#define SPI3_INT_GPIO_Port   GPIOD

#define SPI4_CS_Pin          GPIO_PIN_11
#define SPI4_CS_GPIO_Port    GPIOE

#define CONT_YN_1_Pin        GPIO_PIN_10
#define CONT_YN_1_GPIO_Port  GPIOA
#define CONT_YN_2_Pin        GPIO_PIN_14
#define CONT_YN_2_GPIO_Port  GPIOB
#define CONT_YN_3_Pin        GPIO_PIN_8
#define CONT_YN_3_GPIO_Port  GPIOE
#define CONT_YN_4_Pin        GPIO_PIN_7
#define CONT_YN_4_GPIO_Port  GPIOE

#define PYRO_1_Pin           GPIO_PIN_10
#define PYRO_1_GPIO_Port     GPIOD
#define PYRO_2_Pin           GPIO_PIN_9
#define PYRO_2_GPIO_Port     GPIOD
#define PYRO_3_Pin           GPIO_PIN_8
#define PYRO_3_GPIO_Port     GPIOD
#define PYRO_4_Pin           GPIO_PIN_15
#define PYRO_4_GPIO_Port     GPIOB

/* Pyro CubeMX aliases (casper_pyro.c uses PYx naming) */
#define PY1_Pin              PYRO_1_Pin
#define PY1_GPIO_Port        PYRO_1_GPIO_Port
#define PY2_Pin              PYRO_2_Pin
#define PY2_GPIO_Port        PYRO_2_GPIO_Port
#define PY3_Pin              PYRO_3_Pin
#define PY3_GPIO_Port        PYRO_3_GPIO_Port
#define PY4_Pin              PYRO_4_Pin
#define PY4_GPIO_Port        PYRO_4_GPIO_Port

#define GPS_NRST_Pin         GPIO_PIN_0
#define GPS_NRST_GPIO_Port   GPIOE

#define SPI1_CS_Pin          GPIO_PIN_0
#define SPI1_CS_GPIO_Port    GPIOB
#define SPI1_INT_Pin         GPIO_PIN_1
#define SPI1_INT_GPIO_Port   GPIOB

#define MAG_INT_Pin          GPIO_PIN_8
#define MAG_INT_GPIO_Port    GPIOC

/* ---- Peripheral handle externs ---- */
extern SPI_HandleTypeDef  hspi1, hspi2, hspi3, hspi4;
extern I2C_HandleTypeDef  hi2c1, hi2c2, hi2c3;
extern ADC_HandleTypeDef  hadc1, hadc2, hadc3;
extern TIM_HandleTypeDef  htim2;
extern UART_HandleTypeDef huart4;
extern QSPI_HandleTypeDef hqspi;
extern CRC_HandleTypeDef  hcrc;

/* ---- Error handler ---- */
static inline void Error_Handler(void) { }

#endif /* __MAIN_H */
