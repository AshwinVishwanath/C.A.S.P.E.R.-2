/* Minimal HAL IWDG implementation for C.A.S.P.E.R.-2.
 * IWDG1 on STM32H750, clocked from LSI (~32 kHz).
 * Key values from RM0433 Rev 8 §50.4.
 */
#include "stm32h7xx_hal.h"

#ifdef HAL_IWDG_MODULE_ENABLED

#define IWDG_KEY_RELOAD  0xAAAAUL
#define IWDG_KEY_ENABLE  0xCCCCUL
#define IWDG_KEY_WRITE   0x5555UL

HAL_StatusTypeDef HAL_IWDG_Init(IWDG_HandleTypeDef *hiwdg)
{
    if (hiwdg == NULL || hiwdg->Instance == NULL) {
        return HAL_ERROR;
    }
    /* Unlock PR and RLR */
    hiwdg->Instance->KR  = IWDG_KEY_WRITE;
    hiwdg->Instance->PR  = hiwdg->Prescaler;
    hiwdg->Instance->RLR = hiwdg->Reload;
    /* Wait for registers to update (SR bits clear when done) */
    uint32_t tickstart = HAL_GetTick();
    while ((hiwdg->Instance->SR & 0x7UL) != 0UL) {
        if ((HAL_GetTick() - tickstart) > 50UL) {
            return HAL_TIMEOUT;
        }
    }
    /* Reload and start */
    hiwdg->Instance->KR = IWDG_KEY_RELOAD;
    hiwdg->Instance->KR = IWDG_KEY_ENABLE;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_IWDG_Refresh(IWDG_HandleTypeDef *hiwdg)
{
    hiwdg->Instance->KR = IWDG_KEY_RELOAD;
    return HAL_OK;
}

#endif /* HAL_IWDG_MODULE_ENABLED */
