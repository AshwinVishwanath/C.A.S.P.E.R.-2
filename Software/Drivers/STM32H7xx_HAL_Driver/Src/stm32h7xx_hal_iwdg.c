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
    /* Order matters (RM0433 §50.4.5):
     *  1. Start the watchdog (also starts LSI clock domain)
     *  2. Unlock PR/RLR
     *  3. Write PR/RLR
     *  4. Wait for SR sync (PVU + RVU clear when LSI synchronizes)
     *  5. Refresh once to load new RLR
     *
     * The previous order (unlock -> PR/RLR -> wait -> 0xCCCC) failed because
     * (a) PR/RLR reset when IWDG starts so the configured values were lost,
     * and (b) LSI was not yet running so the SR-sync wait timed out, the
     * function returned HAL_TIMEOUT before ever writing 0xCCCC, and the
     * watchdog was never enabled. */
    hiwdg->Instance->KR  = IWDG_KEY_ENABLE;   /* 0xCCCC */
    hiwdg->Instance->KR  = IWDG_KEY_WRITE;    /* 0x5555 */
    hiwdg->Instance->PR  = hiwdg->Prescaler;
    hiwdg->Instance->RLR = hiwdg->Reload;
    uint32_t tickstart = HAL_GetTick();
    while ((hiwdg->Instance->SR & 0x7UL) != 0UL) {
        if ((HAL_GetTick() - tickstart) > 100UL) {
            return HAL_TIMEOUT;
        }
    }
    hiwdg->Instance->KR = IWDG_KEY_RELOAD;    /* 0xAAAA */
    return HAL_OK;
}

HAL_StatusTypeDef HAL_IWDG_Refresh(IWDG_HandleTypeDef *hiwdg)
{
    hiwdg->Instance->KR = IWDG_KEY_RELOAD;
    return HAL_OK;
}

#endif /* HAL_IWDG_MODULE_ENABLED */
