/* Shared stub infrastructure — HAL function implementations for host testing
 *
 * Note: __attribute__((weak)) does NOT work reliably on MinGW PE/COFF.
 * GPIO Write/Read are in mock_gpio.c, ADC funcs in mock_adc.c,
 * Tick/Delay in mock_tick.c — all three are always linked via COMMON_MOCKS.
 * This file provides SPI, I2C, CRC, QSPI, CDC stubs only.
 */
#include "stm32h7xx_hal.h"
#include <string.h>

/* ---- GPIO port instances ---- */
GPIO_TypeDef gpio_stub_a, gpio_stub_b, gpio_stub_c, gpio_stub_d, gpio_stub_e;

/* ---- CRC handle (referenced by crc32_hw.c) ---- */
CRC_HandleTypeDef hcrc;

/* ---- SPI stubs ---- */
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData,
                                    uint16_t Size, uint32_t Timeout)
{
    (void)hspi; (void)pData; (void)Size; (void)Timeout;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData,
                                   uint16_t Size, uint32_t Timeout)
{
    (void)hspi; (void)Size; (void)Timeout;
    memset(pData, 0, Size);
    return HAL_OK;
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi,
                                           uint8_t *pTxData, uint8_t *pRxData,
                                           uint16_t Size, uint32_t Timeout)
{
    (void)hspi; (void)pTxData; (void)Timeout;
    memset(pRxData, 0, Size);
    return HAL_OK;
}

/* ---- I2C stubs ---- */
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
                                     uint16_t MemAddress, uint16_t MemAddSize,
                                     uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    (void)hi2c; (void)DevAddress; (void)MemAddress; (void)MemAddSize;
    (void)pData; (void)Size; (void)Timeout;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
                                    uint16_t MemAddress, uint16_t MemAddSize,
                                    uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    (void)hi2c; (void)DevAddress; (void)MemAddress; (void)MemAddSize;
    (void)Size; (void)Timeout;
    memset(pData, 0, Size);
    return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
                                           uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    (void)hi2c; (void)DevAddress; (void)pData; (void)Size; (void)Timeout;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
                                          uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    (void)hi2c; (void)DevAddress; (void)Size; (void)Timeout;
    memset(pData, 0, Size);
    return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
                                         uint32_t Trials, uint32_t Timeout)
{
    (void)hi2c; (void)DevAddress; (void)Trials; (void)Timeout;
    return HAL_OK;
}

/* ---- CRC stubs (software CRC-32 implementation for host testing) ---- */
HAL_StatusTypeDef HAL_CRC_Init(CRC_HandleTypeDef *hcrc_param)
{
    (void)hcrc_param;
    return HAL_OK;
}

static uint32_t sw_crc32_byte(uint32_t crc, uint8_t byte)
{
    crc ^= byte;
    for (int i = 0; i < 8; i++) {
        if (crc & 1)
            crc = (crc >> 1) ^ 0xEDB88320u;
        else
            crc >>= 1;
    }
    return crc;
}

uint32_t HAL_CRC_Calculate(CRC_HandleTypeDef *hcrc_param, uint32_t pBuffer[], uint32_t BufferLength)
{
    (void)hcrc_param;
    const uint8_t *data = (const uint8_t *)pBuffer;
    uint32_t crc = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < BufferLength; i++) {
        crc = sw_crc32_byte(crc, data[i]);
    }
    return crc;
}

uint32_t HAL_CRC_Accumulate(CRC_HandleTypeDef *hcrc_param, uint32_t pBuffer[], uint32_t BufferLength)
{
    return HAL_CRC_Calculate(hcrc_param, pBuffer, BufferLength);
}

/* ---- ADC extended stubs ---- */
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *hadc,
                                               uint32_t CalibrationMode,
                                               uint32_t SingleDiff)
{
    (void)hadc; (void)CalibrationMode; (void)SingleDiff;
    return HAL_OK;
}

/* ---- QSPI stubs ---- */
HAL_StatusTypeDef HAL_QSPI_Command(QSPI_HandleTypeDef *hqspi, QSPI_CommandTypeDef *cmd, uint32_t Timeout)
{
    (void)hqspi; (void)cmd; (void)Timeout;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_QSPI_Transmit(QSPI_HandleTypeDef *hqspi, uint8_t *pData, uint32_t Timeout)
{
    (void)hqspi; (void)pData; (void)Timeout;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_QSPI_Receive(QSPI_HandleTypeDef *hqspi, uint8_t *pData, uint32_t Timeout)
{
    (void)hqspi; (void)pData; (void)Timeout;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_QSPI_AutoPolling(QSPI_HandleTypeDef *hqspi, QSPI_CommandTypeDef *cmd,
                                        void *pCfg, uint32_t Timeout)
{
    (void)hqspi; (void)cmd; (void)pCfg; (void)Timeout;
    return HAL_OK;
}

/* ---- USB CDC stub ---- */
uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len)
{
    (void)Buf; (void)Len;
    return 0; /* USBD_OK */
}

/* ---- CDC ring buffer (used by cmd_router.c, flight_loop.c) ---- */
/* These reference the ring buffer defined in global_handles.c.
 * They must be functional for cmd_router tests that feed data
 * into cdc_rx_ring and expect cmd_router_process() to read it. */
#include "usbd_cdc_if.h"

uint16_t cdc_ring_available(void)
{
    return (uint16_t)((cdc_rx_head - cdc_rx_tail) & (CDC_RING_SIZE - 1));
}

uint8_t cdc_ring_read_byte(void)
{
    uint8_t byte = cdc_rx_ring[cdc_rx_tail];
    cdc_rx_tail = (cdc_rx_tail + 1) % CDC_RING_SIZE;
    return byte;
}
