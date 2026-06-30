/* ============================================================
 *  TIER:     CORE-FLIGHT
 *  MODULE:   W25Q512JV QSPI Flash Driver
 *  SUMMARY:  64 MB NOR flash driver, FATFS-backed mass storage.
 * ============================================================ */
/**
 * W25Q512JV QSPI NOR flash driver — board-agnostic (casper_port seam).
 * Reference: Winbond W25Q512JV datasheet Rev G
 *
 * Configured for: 64 MB, single-line SPI (no quad yet),
 * 4-byte address mode.
 *
 * All operations use casper_qspi_* (no direct HAL calls).
 * IT callbacks are dispatched by the board layer via casper_qspi_set_handler().
 */

#include "w25q512jv.h"
#include <string.h>

/* ------------------------------------------------------------------ */
/*  Internal helpers                                                    */
/* ------------------------------------------------------------------ */

/* Build a casper_qspi_cmd_t for an instruction-only frame (no address, no data). */
static casper_qspi_cmd_t make_cmd_only(uint8_t opcode)
{
    casper_qspi_cmd_t c = {0};
    c.instruction       = opcode;
    c.instruction_lines = 1;
    c.address_lines     = 0;
    c.data_lines        = 0;
    return c;
}

/* Build a casper_qspi_cmd_t for a status-register read (no address, 1 data byte). */
static casper_qspi_cmd_t make_cmd_sr(uint8_t opcode)
{
    casper_qspi_cmd_t c = {0};
    c.instruction       = opcode;
    c.instruction_lines = 1;
    c.address_lines     = 0;
    c.data_lines        = 1;
    c.data_len          = 1;
    return c;
}

/* Send instruction-only command (no address, no data) */
static int w25q_cmd_only(w25q512jv_t *dev, uint8_t opcode)
{
    casper_qspi_cmd_t cmd = make_cmd_only(opcode);
    if (casper_qspi_command(dev->bus, &cmd, 100) != CASPER_OK)
        return W25Q_ERROR;
    return W25Q_OK;
}

/* Read a single status register (opcode = 0x05/0x35/0x15) */
static uint8_t w25q_read_sr(w25q512jv_t *dev, uint8_t opcode)
{
    casper_qspi_cmd_t cmd = make_cmd_sr(opcode);
    if (casper_qspi_command(dev->bus, &cmd, 100) != CASPER_OK)
        return 0xFF;
    uint8_t val = 0;
    if (casper_qspi_receive(dev->bus, &val, 100) != CASPER_OK)
        return 0xFF;
    return val;
}

/* Write a single status register (opcode = 0x01/0x31/0x11) */
static int w25q_write_sr(w25q512jv_t *dev, uint8_t opcode, uint8_t val)
{
    casper_qspi_cmd_t cmd = make_cmd_sr(opcode);
    if (casper_qspi_command(dev->bus, &cmd, 100) != CASPER_OK)
        return W25Q_ERROR;
    if (casper_qspi_transmit(dev->bus, &val, 100) != CASPER_OK)
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
    uint32_t start = casper_millis();
    while ((casper_millis() - start) < timeout_ms) {
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
    casper_qspi_cmd_t cmd = {0};
    cmd.instruction       = W25Q_CMD_PAGE_PROGRAM_4B;
    cmd.instruction_lines = 1;
    cmd.address_lines     = 1;
    cmd.address_bytes     = 4;
    cmd.address           = addr;
    cmd.data_lines        = 1;
    cmd.dummy_cycles      = 0;
    cmd.data_len          = len;

    if (casper_qspi_command(dev->bus, &cmd, 100) != CASPER_OK)
        return W25Q_ERROR;
    if (casper_qspi_transmit(dev->bus, buf, 100) != CASPER_OK)
        return W25Q_ERROR;
    return W25Q_OK;
}

/* ------------------------------------------------------------------ */
/*  IT handler — registered on the bus during init                     */
/*                                                                      */
/*  The board layer (board_casper2.c) owns HAL_QSPI_TxCpltCallback,    */
/*  HAL_QSPI_StatusMatchCallback, and HAL_QSPI_ErrorCallback.  Those   */
/*  translate the HAL events into casper_qspi_evt_t and call this      */
/*  function via the handler pointer stored in the casper_qspi_t.      */
/*                                                                      */
/*  ctx is the w25q512jv_t* that was passed to casper_qspi_set_handler */
/*  during w25q512jv_init().                                            */
/* ------------------------------------------------------------------ */
static void w25q_it_handler(void *ctx, casper_qspi_evt_t evt)
{
    w25q512jv_t *dev = (w25q512jv_t *)ctx;

    if (evt == CASPER_QSPI_EVT_TX_DONE) {
        /* Page data transfer finished.  Now poll BUSY until the flash
         * commits the page program (WIP bit clears). */
        if (dev->it_state == W25Q_IT_WRITE_DATA) {
            casper_qspi_poll_t poll = {0};
            poll.instruction  = W25Q_CMD_READ_SR1;
            poll.match        = 0x00;
            poll.mask         = W25Q_SR1_BUSY;
            poll.poll_interval = 0x10;

            dev->it_state = W25Q_IT_WRITE_POLL;
            if (casper_qspi_autopoll_it(dev->bus, &poll) != CASPER_OK) {
                dev->it_state = W25Q_IT_IDLE;
                if (dev->on_error)
                    dev->on_error(dev->cb_ctx, false);
            }
        }
    } else if (evt == CASPER_QSPI_EVT_MATCH) {
        /* Auto-polling matched (BUSY cleared) — write or erase done. */
        dev->it_state = W25Q_IT_IDLE;
        if (dev->on_complete)
            dev->on_complete(dev->cb_ctx, true);
    } else {
        /* CASPER_QSPI_EVT_ERROR */
        dev->it_state = W25Q_IT_IDLE;
        if (dev->on_error)
            dev->on_error(dev->cb_ctx, false);
    }
}

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

bool w25q512jv_init(w25q512jv_t *dev, casper_qspi_t *bus)
{
    dev->bus         = bus;
    dev->initialized = false;
    dev->it_state    = W25Q_IT_IDLE;
    dev->on_complete = NULL;
    dev->on_error    = NULL;
    dev->cb_ctx      = NULL;
    memset(dev->jedec_id, 0, sizeof(dev->jedec_id));

    /* Register the IT event handler so the board layer can dispatch to us. */
    casper_qspi_set_handler(bus, w25q_it_handler, dev);

    /* Software reset: Enable Reset → Reset → wait 1 ms */
    w25q_cmd_only(dev, W25Q_CMD_ENABLE_RESET);
    w25q_cmd_only(dev, W25Q_CMD_RESET);
    casper_delay_ms(1);

    /* Read JEDEC ID (0x9F): 3 bytes */
    {
        casper_qspi_cmd_t cmd = {0};
        cmd.instruction       = W25Q_CMD_READ_JEDEC_ID;
        cmd.instruction_lines = 1;
        cmd.address_lines     = 0;
        cmd.data_lines        = 1;
        cmd.dummy_cycles      = 0;
        cmd.data_len          = 3;

        if (casper_qspi_command(dev->bus, &cmd, 100) != CASPER_OK)
            return false;
        if (casper_qspi_receive(dev->bus, dev->jedec_id, 100) != CASPER_OK)
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

    casper_qspi_cmd_t cmd = {0};
    cmd.instruction       = W25Q_CMD_FAST_READ_4B;
    cmd.instruction_lines = 1;
    cmd.address_lines     = 1;
    cmd.address_bytes     = 4;
    cmd.address           = addr;
    cmd.data_lines        = 1;
    cmd.dummy_cycles      = W25Q_DUMMY_FAST_READ;
    cmd.data_len          = len;

    if (casper_qspi_command(dev->bus, &cmd, 100) != CASPER_OK)
        return W25Q_ERROR;
    if (casper_qspi_receive(dev->bus, buf, 1000) != CASPER_OK)
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

    casper_qspi_cmd_t cmd = {0};
    cmd.instruction       = W25Q_CMD_SECTOR_ERASE_4B;
    cmd.instruction_lines = 1;
    cmd.address_lines     = 1;
    cmd.address_bytes     = 4;
    cmd.address           = addr;
    cmd.data_lines        = 0;
    cmd.dummy_cycles      = 0;

    if (casper_qspi_command(dev->bus, &cmd, 100) != CASPER_OK)
        return W25Q_ERROR;

    return w25q_wait_busy(dev, W25Q_TIMEOUT_SECTOR_ERASE);
}

int w25q512jv_erase_block(w25q512jv_t *dev, uint32_t addr)
{
    /* Align to 64KB block boundary */
    addr &= ~(W25Q512JV_BLOCK_64K_SIZE - 1u);

    if (w25q_write_enable(dev) != W25Q_OK)
        return W25Q_ERROR;

    casper_qspi_cmd_t cmd = {0};
    cmd.instruction       = W25Q_CMD_BLOCK_ERASE_64K_4B;
    cmd.instruction_lines = 1;
    cmd.address_lines     = 1;
    cmd.address_bytes     = 4;
    cmd.address           = addr;
    cmd.data_lines        = 0;
    cmd.dummy_cycles      = 0;

    if (casper_qspi_command(dev->bus, &cmd, 100) != CASPER_OK)
        return W25Q_ERROR;

    return w25q_wait_busy(dev, W25Q_TIMEOUT_BLOCK_ERASE);
}

/* ------------------------------------------------------------------ */
/*  Non-blocking (IT mode) API                                         */
/* ------------------------------------------------------------------ */

void w25q512jv_set_callbacks(w25q512jv_t *dev,
                             w25q_callback_t on_complete,
                             w25q_callback_t on_error,
                             void *ctx)
{
    dev->on_complete = on_complete;
    dev->on_error    = on_error;
    dev->cb_ctx      = ctx;
}

bool w25q512jv_is_idle(const w25q512jv_t *dev)
{
    return dev->it_state == W25Q_IT_IDLE;
}

int w25q512jv_write_page_it(w25q512jv_t *dev, uint32_t addr,
                            const uint8_t *buf)
{
    if (dev->it_state != W25Q_IT_IDLE)
        return W25Q_ERROR;

    /* Write Enable (blocking — ~1 us, just the command phase) */
    if (w25q_write_enable(dev) != W25Q_OK)
        return W25Q_ERROR;

    /* Page Program command header (blocking — no data yet) */
    casper_qspi_cmd_t cmd = {0};
    cmd.instruction       = W25Q_CMD_PAGE_PROGRAM_4B;
    cmd.instruction_lines = 1;
    cmd.address_lines     = 1;
    cmd.address_bytes     = 4;
    cmd.address           = addr;
    cmd.data_lines        = 1;
    cmd.dummy_cycles      = 0;
    cmd.data_len          = W25Q512JV_PAGE_SIZE;

    if (casper_qspi_command(dev->bus, &cmd, 100) != CASPER_OK)
        return W25Q_ERROR;

    /* Start async data transfer — fires w25q_it_handler(TX_DONE) on completion */
    dev->it_state = W25Q_IT_WRITE_DATA;
    if (casper_qspi_transmit_it(dev->bus, buf) != CASPER_OK) {
        dev->it_state = W25Q_IT_IDLE;
        return W25Q_ERROR;
    }

    return W25Q_OK;
}

int w25q512jv_erase_sector_it(w25q512jv_t *dev, uint32_t addr)
{
    if (dev->it_state != W25Q_IT_IDLE)
        return W25Q_ERROR;

    /* Align to sector boundary */
    addr &= ~(W25Q512JV_SECTOR_SIZE - 1u);

    /* Write Enable (blocking) */
    if (w25q_write_enable(dev) != W25Q_OK)
        return W25Q_ERROR;

    /* Sector Erase command (blocking — no data phase) */
    casper_qspi_cmd_t cmd = {0};
    cmd.instruction       = W25Q_CMD_SECTOR_ERASE_4B;
    cmd.instruction_lines = 1;
    cmd.address_lines     = 1;
    cmd.address_bytes     = 4;
    cmd.address           = addr;
    cmd.data_lines        = 0;
    cmd.dummy_cycles      = 0;

    if (casper_qspi_command(dev->bus, &cmd, 100) != CASPER_OK)
        return W25Q_ERROR;

    /* Start auto-polling for WIP (Write In Progress) bit clear.
     * Fires w25q_it_handler(MATCH) when BUSY clears. */
    casper_qspi_poll_t poll = {0};
    poll.instruction   = W25Q_CMD_READ_SR1;
    poll.match         = 0x00;
    poll.mask          = W25Q_SR1_BUSY;
    poll.poll_interval = 0x10;

    dev->it_state = W25Q_IT_ERASE_POLL;
    if (casper_qspi_autopoll_it(dev->bus, &poll) != CASPER_OK) {
        dev->it_state = W25Q_IT_IDLE;
        return W25Q_ERROR;
    }

    return W25Q_OK;
}
