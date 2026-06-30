/* ============================================================
 *  TIER:     CORE-FLIGHT
 *  MODULE:   CRC32 Hardware
 *  SUMMARY:  Wrapper for STM32 CRC peripheral; telemetry/log integrity.
 * ============================================================ */
/*
 * crc32_hw.c — thin compatibility wrapper over the portable CRC-32 seam.
 *
 * The previous implementation called the STM32H7 HAL CRC peripheral directly.
 * After the port-layer migration the real hardware logic lives in
 * board_casper2.c (casper_crc32_init / casper_crc32_compute).  This file
 * now delegates to those portable functions so that no App/ file outside the
 * board layer needs to include stm32h7xx_hal.h.
 *
 * The public API (crc32_hw_init / crc32_hw_compute) is unchanged, so all
 * callers (tlm_manager.c, cac_handler.c, etc.) rebuild without modification.
 */
#include "crc32_hw.h"
#include "casper_port.h"   /* casper_crc32_init / casper_crc32_compute */

void crc32_hw_init(void)
{
    casper_crc32_init();
}

uint32_t crc32_hw_compute(const uint8_t *data, uint32_t len)
{
    return casper_crc32_compute(data, len);
}
