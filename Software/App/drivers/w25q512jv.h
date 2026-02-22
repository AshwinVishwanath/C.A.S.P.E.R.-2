#ifndef W25Q512JV_H
#define W25Q512JV_H

#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* Return codes */
#define W25Q_OK       0
#define W25Q_ERROR   -1
#define W25Q_TIMEOUT -2

/* ------------------------------------------------------------------ */
/*  JEDEC identification                                               */
/* ------------------------------------------------------------------ */
#define W25Q512JV_MANUFACTURER_ID   0xEFu
#define W25Q512JV_MEMORY_TYPE_IQ    0x40u   /* Standard SPI (IQ variant) */
#define W25Q512JV_MEMORY_TYPE_IM    0x70u   /* DTR (IM variant) */
#define W25Q512JV_CAPACITY_ID       0x20u   /* 512Mbit = 64MB */

/* ------------------------------------------------------------------ */
/*  Memory organisation                                                */
/* ------------------------------------------------------------------ */
#define W25Q512JV_FLASH_SIZE        0x4000000u  /* 64 MB */
#define W25Q512JV_PAGE_SIZE         256u
#define W25Q512JV_SECTOR_SIZE       4096u       /* 4 KB */
#define W25Q512JV_BLOCK_32K_SIZE    32768u      /* 32 KB */
#define W25Q512JV_BLOCK_64K_SIZE    65536u      /* 64 KB */
#define W25Q512JV_SECTOR_COUNT      16384u
#define W25Q512JV_PAGE_COUNT        262144u

/* FSIZE for STM32 QUADSPI DCR register: 2^(FSIZE+1) = 64MB */
#define W25Q512JV_FSIZE             25u

/* ------------------------------------------------------------------ */
/*  Command opcodes                                                    */
/* ------------------------------------------------------------------ */

/* Identification */
#define W25Q_CMD_READ_JEDEC_ID      0x9Fu
#define W25Q_CMD_READ_UNIQUE_ID     0x4Bu

/* Read (single-line, 4-byte address) */
#define W25Q_CMD_READ_DATA_4B       0x13u
#define W25Q_CMD_FAST_READ_4B       0x0Cu

/* Read (quad output, 4-byte address) */
#define W25Q_CMD_FAST_READ_QUAD_OUT_4B  0x6Cu
#define W25Q_CMD_FAST_READ_QUAD_IO_4B   0xECu

/* Program */
#define W25Q_CMD_PAGE_PROGRAM_4B    0x12u
#define W25Q_CMD_QUAD_PAGE_PROG_4B  0x34u

/* Erase */
#define W25Q_CMD_SECTOR_ERASE_4B    0x21u   /* 4 KB */
#define W25Q_CMD_BLOCK_ERASE_64K_4B 0xDCu   /* 64 KB */
#define W25Q_CMD_CHIP_ERASE         0xC7u   /* Full chip */

/* Write control */
#define W25Q_CMD_WRITE_ENABLE       0x06u
#define W25Q_CMD_WRITE_DISABLE      0x04u
#define W25Q_CMD_VOL_SR_WRITE_EN    0x50u

/* Status registers */
#define W25Q_CMD_READ_SR1           0x05u
#define W25Q_CMD_READ_SR2           0x35u
#define W25Q_CMD_READ_SR3           0x15u
#define W25Q_CMD_WRITE_SR1          0x01u
#define W25Q_CMD_WRITE_SR2          0x31u
#define W25Q_CMD_WRITE_SR3          0x11u

/* Address mode */
#define W25Q_CMD_ENTER_4B_MODE      0xB7u
#define W25Q_CMD_EXIT_4B_MODE       0xE9u

/* Reset */
#define W25Q_CMD_ENABLE_RESET       0x66u
#define W25Q_CMD_RESET              0x99u

/* Power */
#define W25Q_CMD_POWER_DOWN         0xB9u
#define W25Q_CMD_RELEASE_PD         0xABu

/* Erase/program suspend/resume */
#define W25Q_CMD_ERASE_SUSPEND      0x75u
#define W25Q_CMD_ERASE_RESUME       0x7Au

/* Protection */
#define W25Q_CMD_GLOBAL_UNLOCK      0x98u
#define W25Q_CMD_GLOBAL_LOCK        0x7Eu

/* ------------------------------------------------------------------ */
/*  Status register bit masks                                          */
/* ------------------------------------------------------------------ */

/* SR1 (read: 0x05) */
#define W25Q_SR1_BUSY               (1u << 0)
#define W25Q_SR1_WEL                (1u << 1)
#define W25Q_SR1_BP0                (1u << 2)
#define W25Q_SR1_BP1                (1u << 3)
#define W25Q_SR1_BP2                (1u << 4)
#define W25Q_SR1_TB                 (1u << 5)
#define W25Q_SR1_SEC                (1u << 6)
#define W25Q_SR1_SRP0               (1u << 7)

/* SR2 (read: 0x35) */
#define W25Q_SR2_SRL                (1u << 0)
#define W25Q_SR2_QE                 (1u << 1)   /* Quad Enable */
#define W25Q_SR2_LB1                (1u << 3)   /* OTP — do not set! */
#define W25Q_SR2_LB2                (1u << 4)   /* OTP — do not set! */
#define W25Q_SR2_LB3                (1u << 5)   /* OTP — do not set! */
#define W25Q_SR2_CMP                (1u << 6)
#define W25Q_SR2_SUS                (1u << 7)

/* SR3 (read: 0x15) */
#define W25Q_SR3_ADP                (1u << 1)   /* Power-up address mode */
#define W25Q_SR3_WPS                (1u << 2)
#define W25Q_SR3_DRV0               (1u << 5)
#define W25Q_SR3_DRV1               (1u << 6)

/* ------------------------------------------------------------------ */
/*  Timing (maximum values from datasheet, in milliseconds)            */
/* ------------------------------------------------------------------ */
#define W25Q_TIMEOUT_PAGE_PROG      3u
#define W25Q_TIMEOUT_SECTOR_ERASE   400u
#define W25Q_TIMEOUT_BLOCK_ERASE    2000u
#define W25Q_TIMEOUT_CHIP_ERASE     400000u     /* 400 seconds */
#define W25Q_TIMEOUT_WRITE_SR       15u

/* Dummy cycles for Fast Read (single-line, 4-byte addr) */
#define W25Q_DUMMY_FAST_READ        8u

/* ------------------------------------------------------------------ */
/*  Driver struct                                                      */
/* ------------------------------------------------------------------ */
typedef struct {
    QSPI_HandleTypeDef *hqspi;
    uint8_t  jedec_id[3];       /* Manufacturer, MemType, Capacity */
    bool     initialized;
} w25q512jv_t;

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

/* Initialise: reset, read JEDEC ID, enable QE, enter 4-byte mode.
 * Returns true on success. */
bool w25q512jv_init(w25q512jv_t *dev, QSPI_HandleTypeDef *hqspi);

/* Read `len` bytes starting at `addr` into `buf`.
 * Returns W25Q_OK on success. */
int w25q512jv_read(w25q512jv_t *dev, uint32_t addr,
                   uint8_t *buf, uint32_t len);

/* Write `len` bytes from `buf` starting at `addr`.
 * Handles page-boundary splits automatically.
 * Target region MUST be erased first (all 0xFF).
 * Returns W25Q_OK on success. */
int w25q512jv_write(w25q512jv_t *dev, uint32_t addr,
                    const uint8_t *buf, uint32_t len);

/* Erase 4 KB sector containing `addr`.
 * Returns W25Q_OK on success. */
int w25q512jv_erase_sector(w25q512jv_t *dev, uint32_t addr);

/* Erase 64 KB block containing `addr`.
 * Returns W25Q_OK on success. */
int w25q512jv_erase_block(w25q512jv_t *dev, uint32_t addr);

/* Erase entire chip. WARNING: takes up to 400 seconds!
 * Returns W25Q_OK on success. */
int w25q512jv_erase_chip(w25q512jv_t *dev);

/* Write-read-verify test on sector 0.
 * Returns true if data matches. */
bool w25q512jv_test(w25q512jv_t *dev);

#endif /* W25Q512JV_H */
