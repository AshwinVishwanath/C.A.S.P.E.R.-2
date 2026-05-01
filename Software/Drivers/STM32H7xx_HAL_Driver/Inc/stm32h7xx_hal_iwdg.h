/* Minimal HAL IWDG stub for C.A.S.P.E.R.-2 (no CubeMX IWDG peripheral) */
#ifndef STM32H7xx_HAL_IWDG_H
#define STM32H7xx_HAL_IWDG_H

#include "stm32h7xx_hal_def.h"

typedef struct {
    IWDG_TypeDef  *Instance;
    uint32_t       Prescaler;   /* IWDG_PR value (0=/4, 1=/8, 2=/16 ...) */
    uint32_t       Reload;      /* 0–0xFFF */
} IWDG_HandleTypeDef;

HAL_StatusTypeDef HAL_IWDG_Init(IWDG_HandleTypeDef *hiwdg);
HAL_StatusTypeDef HAL_IWDG_Refresh(IWDG_HandleTypeDef *hiwdg);

#endif /* STM32H7xx_HAL_IWDG_H */
