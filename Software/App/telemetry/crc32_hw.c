#include "crc32_hw.h"
#include "stm32h7xx_hal.h"

static CRC_HandleTypeDef *s_hcrc;

/* Reference to the global CRC handle declared in main.c */
extern CRC_HandleTypeDef hcrc;

void crc32_hw_init(void)
{
    s_hcrc = &hcrc;

    /* Reconfigure for standard CRC-32 (reflected I/O) */
    __HAL_RCC_CRC_CLK_ENABLE();

    s_hcrc->Instance = CRC;
    s_hcrc->Init.DefaultPolynomialUse    = DEFAULT_POLYNOMIAL_ENABLE;
    s_hcrc->Init.DefaultInitValueUse     = DEFAULT_INIT_VALUE_ENABLE;
    s_hcrc->Init.InputDataInversionMode  = CRC_INPUTDATA_INVERSION_BYTE;
    s_hcrc->Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_ENABLE;
    s_hcrc->InputDataFormat              = CRC_INPUTDATA_FORMAT_BYTES;

    HAL_CRC_Init(s_hcrc);
}

uint32_t crc32_hw_compute(const uint8_t *data, uint32_t len)
{
    uint32_t raw = HAL_CRC_Calculate(s_hcrc, (uint32_t *)data, len);
    /* Final XOR for standard CRC-32 */
    return raw ^ 0xFFFFFFFFu;
}

int crc32_hw_validate(const uint8_t *data, uint32_t payload_len,
                      uint32_t received_crc)
{
    uint32_t computed = crc32_hw_compute(data, payload_len);
    return (computed == received_crc) ? 1 : 0;
}
