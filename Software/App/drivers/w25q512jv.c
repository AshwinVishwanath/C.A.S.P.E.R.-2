/**
 * W25Q512JV QSPI NOR flash driver — bare-metal C for STM32 HAL.
 * Reference: Winbond W25Q512JV datasheet Rev G
 *
 * Configured for: 64 MB, single-line SPI (no quad yet),
 * 4-byte address mode, polling (no DMA).
 *
 * All operations use the STM32 HAL QSPI indirect mode.
 */

#include "w25q512jv.h"
#include <string.h>

/* ------------------------------------------------------------------ */
/*  Internal helpers                                                    */
/* ------------------------------------------------------------------ */

/* Send instruction-only command (no address, no data) */
static int w25q_cmd_only(w25q512jv_t *dev, uint8_t opcode)
{
    QSPI_CommandTypeDef cmd = {0};
    cmd.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    cmd.Instruction       = opcode;
    cmd.AddressMode       = QSPI_ADDRESS_NONE;
    cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    cmd.DataMode          = QSPI_DATA_NONE;
    cmd.DummyCycles       = 0;
    cmd.DdrMode           = QSPI_DDR_MODE_DISABLE;
    cmd.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    if (HAL_QSPI_Command(dev->hqspi, &cmd, 100) != HAL_OK)
        return W25Q_ERROR;
    return W25Q_OK;
}

/* Read a single status register (opcode = 0x05/0x35/0x15) */
static uint8_t w25q_read_sr(w25q512jv_t *dev, uint8_t opcode)
{
    QSPI_CommandTypeDef cmd = {0};
    cmd.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    cmd.Instruction       = opcode;
    cmd.AddressMode       = QSPI_ADDRESS_NONE;
    cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    cmd.DataMode          = QSPI_DATA_1_LINE;
    cmd.DummyCycles       = 0;
    cmd.NbData            = 1;
    cmd.DdrMode           = QSPI_DDR_MODE_DISABLE;
    cmd.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    uint8_t val = 0;
    if (HAL_QSPI_Command(dev->hqspi, &cmd, 100) != HAL_OK)
        return 0xFF;
    if (HAL_QSPI_Receive(dev->hqspi, &val, 100) != HAL_OK)
        return 0xFF;
    return val;
}

/* Write a single status register (opcode = 0x01/0x31/0x11) */
static int w25q_write_sr(w25q512jv_t *dev, uint8_t opcode, uint8_t val)
{
    QSPI_CommandTypeDef cmd = {0};
    cmd.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    cmd.Instruction       = opcode;
    cmd.AddressMode       = QSPI_ADDRESS_NONE;
    cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    cmd.DataMode          = QSPI_DATA_1_LINE;
    cmd.DummyCycles       = 0;
    cmd.NbData            = 1;
    cmd.DdrMode           = QSPI_DDR_MODE_DISABLE;
    cmd.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    if (HAL_QSPI_Command(dev->hqspi, &cmd, 100) != HAL_OK)
        return W25Q_ERROR;
    if (HAL_QSPI_Transmit(dev->hqspi, &val, 100) != HAL_OK)
        return W25Q_ERROR;
    return W25Q_OK;
}

/* Send Write Enable (0x06) and verify WEL bit is set */
static int w25q_write_enable(w25q512jv_t *dev)
{
    if (w25q_cmd_only(dev, W25Q_CMD_WRITE_ENABLE) != W25Q_OK)
        return W25Q_ERROR;

    uint8_t sr1 = w25q_read_sr(dev, W25Q_CMD_READ_SR1);
    if (!(sr1 & W25Q_SR1_WEL))
        return W25Q_ERROR;
    return W25Q_OK;
}

/* Poll SR1.BUSY until clear or timeout */
static int w25q_wait_busy(w25q512jv_t *dev, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < timeout_ms) {
        uint8_t sr1 = w25q_read_sr(dev, W25Q_CMD_READ_SR1);
        if (!(sr1 & W25Q_SR1_BUSY))
            return W25Q_OK;
    }
    return W25Q_TIMEOUT;
}

/* Page Program 4B: write up to 256 bytes within a single page */
static int w25q_page_program(w25q512jv_t *dev, uint32_t addr,
                             const uint8_t *buf, uint16_t len)
{
    QSPI_CommandTypeDef cmd = {0};
    cmd.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    cmd.Instruction       = W25Q_CMD_PAGE_PROGRAM_4B;
    cmd.AddressMode       = QSPI_ADDRESS_1_LINE;
    cmd.AddressSize       = QSPI_ADDRESS_32_BITS;
    cmd.Address           = addr;
    cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    cmd.DataMode          = QSPI_DATA_1_LINE;
    cmd.DummyCycles       = 0;
    cmd.NbData            = len;
    cmd.DdrMode           = QSPI_DDR_MODE_DISABLE;
    cmd.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    if (HAL_QSPI_Command(dev->hqspi, &cmd, 100) != HAL_OK)
        return W25Q_ERROR;
    if (HAL_QSPI_Transmit(dev->hqspi, (uint8_t *)buf, 100) != HAL_OK)
        return W25Q_ERROR;
    return W25Q_OK;
}

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

bool w25q512jv_init(w25q512jv_t *dev, QSPI_HandleTypeDef *hqspi)
{
    dev->hqspi       = hqspi;
    dev->initialized = false;
    memset(dev->jedec_id, 0, sizeof(dev->jedec_id));

    /* Software reset: Enable Reset → Reset → wait */
    w25q_cmd_only(dev, W25Q_CMD_ENABLE_RESET);
    w25q_cmd_only(dev, W25Q_CMD_RESET);
    HAL_Delay(1);

    /* Read JEDEC ID (0x9F): 3 bytes */
    {
        QSPI_CommandTypeDef cmd = {0};
        cmd.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
        cmd.Instruction       = W25Q_CMD_READ_JEDEC_ID;
        cmd.AddressMode       = QSPI_ADDRESS_NONE;
        cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
        cmd.DataMode          = QSPI_DATA_1_LINE;
        cmd.DummyCycles       = 0;
        cmd.NbData            = 3;
        cmd.DdrMode           = QSPI_DDR_MODE_DISABLE;
        cmd.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

        if (HAL_QSPI_Command(dev->hqspi, &cmd, 100) != HAL_OK)
            return false;
        if (HAL_QSPI_Receive(dev->hqspi, dev->jedec_id, 100) != HAL_OK)
            return false;
    }

    /* Verify manufacturer ID */
    if (dev->jedec_id[0] != W25Q512JV_MANUFACTURER_ID)
        return false;

    /* Ensure Quad Enable (QE) bit is set in SR2 */
    {
        uint8_t sr2 = w25q_read_sr(dev, W25Q_CMD_READ_SR2);
        if (!(sr2 & W25Q_SR2_QE)) {
            if (w25q_write_enable(dev) != W25Q_OK)
                return false;
            /* Set QE bit, preserve other bits (careful: LB bits are OTP!) */
            sr2 |= W25Q_SR2_QE;
            if (w25q_write_sr(dev, W25Q_CMD_WRITE_SR2, sr2) != W25Q_OK)
                return false;
            if (w25q_wait_busy(dev, W25Q_TIMEOUT_WRITE_SR) != W25Q_OK)
                return false;
        }
    }

    /* Enter 4-byte address mode (needed for >16MB) */
    if (w25q_write_enable(dev) != W25Q_OK)
        return false;
    if (w25q_cmd_only(dev, W25Q_CMD_ENTER_4B_MODE) != W25Q_OK)
        return false;

    dev->initialized = true;
    return true;
}

int w25q512jv_read(w25q512jv_t *dev, uint32_t addr,
                   uint8_t *buf, uint32_t len)
{
    if (len == 0)
        return W25Q_OK;

    QSPI_CommandTypeDef cmd = {0};
    cmd.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    cmd.Instruction       = W25Q_CMD_FAST_READ_4B;
    cmd.AddressMode       = QSPI_ADDRESS_1_LINE;
    cmd.AddressSize       = QSPI_ADDRESS_32_BITS;
    cmd.Address           = addr;
    cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    cmd.DataMode          = QSPI_DATA_1_LINE;
    cmd.DummyCycles       = W25Q_DUMMY_FAST_READ;
    cmd.NbData            = len;
    cmd.DdrMode           = QSPI_DDR_MODE_DISABLE;
    cmd.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    if (HAL_QSPI_Command(dev->hqspi, &cmd, 100) != HAL_OK)
        return W25Q_ERROR;
    if (HAL_QSPI_Receive(dev->hqspi, buf, 1000) != HAL_OK)
        return W25Q_ERROR;
    return W25Q_OK;
}

int w25q512jv_write(w25q512jv_t *dev, uint32_t addr,
                    const uint8_t *buf, uint32_t len)
{
    while (len > 0) {
        /* Bytes remaining in current page */
        uint32_t page_offset = addr % W25Q512JV_PAGE_SIZE;
        uint32_t chunk = W25Q512JV_PAGE_SIZE - page_offset;
        if (chunk > len)
            chunk = len;

        if (w25q_write_enable(dev) != W25Q_OK)
            return W25Q_ERROR;
        if (w25q_page_program(dev, addr, buf, (uint16_t)chunk) != W25Q_OK)
            return W25Q_ERROR;
        if (w25q_wait_busy(dev, W25Q_TIMEOUT_PAGE_PROG) != W25Q_OK)
            return W25Q_TIMEOUT;

        addr += chunk;
        buf  += chunk;
        len  -= chunk;
    }
    return W25Q_OK;
}

int w25q512jv_erase_sector(w25q512jv_t *dev, uint32_t addr)
{
    /* Align to sector boundary */
    addr &= ~(W25Q512JV_SECTOR_SIZE - 1u);

    if (w25q_write_enable(dev) != W25Q_OK)
        return W25Q_ERROR;

    QSPI_CommandTypeDef cmd = {0};
    cmd.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    cmd.Instruction       = W25Q_CMD_SECTOR_ERASE_4B;
    cmd.AddressMode       = QSPI_ADDRESS_1_LINE;
    cmd.AddressSize       = QSPI_ADDRESS_32_BITS;
    cmd.Address           = addr;
    cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    cmd.DataMode          = QSPI_DATA_NONE;
    cmd.DummyCycles       = 0;
    cmd.DdrMode           = QSPI_DDR_MODE_DISABLE;
    cmd.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    if (HAL_QSPI_Command(dev->hqspi, &cmd, 100) != HAL_OK)
        return W25Q_ERROR;

    return w25q_wait_busy(dev, W25Q_TIMEOUT_SECTOR_ERASE);
}

int w25q512jv_erase_block(w25q512jv_t *dev, uint32_t addr)
{
    /* Align to 64KB block boundary */
    addr &= ~(W25Q512JV_BLOCK_64K_SIZE - 1u);

    if (w25q_write_enable(dev) != W25Q_OK)
        return W25Q_ERROR;

    QSPI_CommandTypeDef cmd = {0};
    cmd.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    cmd.Instruction       = W25Q_CMD_BLOCK_ERASE_64K_4B;
    cmd.AddressMode       = QSPI_ADDRESS_1_LINE;
    cmd.AddressSize       = QSPI_ADDRESS_32_BITS;
    cmd.Address           = addr;
    cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    cmd.DataMode          = QSPI_DATA_NONE;
    cmd.DummyCycles       = 0;
    cmd.DdrMode           = QSPI_DDR_MODE_DISABLE;
    cmd.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    if (HAL_QSPI_Command(dev->hqspi, &cmd, 100) != HAL_OK)
        return W25Q_ERROR;

    return w25q_wait_busy(dev, W25Q_TIMEOUT_BLOCK_ERASE);
}

int w25q512jv_erase_chip(w25q512jv_t *dev)
{
    if (w25q_write_enable(dev) != W25Q_OK)
        return W25Q_ERROR;

    if (w25q_cmd_only(dev, W25Q_CMD_CHIP_ERASE) != W25Q_OK)
        return W25Q_ERROR;

    return w25q_wait_busy(dev, W25Q_TIMEOUT_CHIP_ERASE);
}

bool w25q512jv_test(w25q512jv_t *dev)
{
    /* Test pattern: 256 bytes of incrementing values */
    uint8_t tx_buf[256];
    uint8_t rx_buf[256];

    for (int i = 0; i < 256; i++)
        tx_buf[i] = (uint8_t)i;

    /* Erase sector 0 */
    if (w25q512jv_erase_sector(dev, 0x00000000) != W25Q_OK)
        return false;

    /* Write 256 bytes at address 0 */
    if (w25q512jv_write(dev, 0x00000000, tx_buf, 256) != W25Q_OK)
        return false;

    /* Read back */
    memset(rx_buf, 0, sizeof(rx_buf));
    if (w25q512jv_read(dev, 0x00000000, rx_buf, 256) != W25Q_OK)
        return false;

    /* Compare */
    return (memcmp(tx_buf, rx_buf, 256) == 0);
}
