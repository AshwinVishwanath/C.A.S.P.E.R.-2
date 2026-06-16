/**
 * @file  board_mock.c
 * @brief Host (gcc / MSYS2) implementation of the full casper_port interface.
 *
 * Provides an in-memory "virtual board" for unit-testing App/ modules without
 * any HAL, CMSIS, or embedded toolchain dependencies.
 *
 * Do NOT compile this file into the ARM firmware target.
 * It is compiled exclusively by test/run.ps1 into host test binaries.
 *
 * -----------------------------------------------------------------------
 *  CRC-32 variant (matches STM32H7 hardware in crc32_hw.c)
 * -----------------------------------------------------------------------
 *  Algorithm: CRC-32/ISO-HDLC  (also known as CRC-32b / zlib CRC-32)
 *    Polynomial : 0x04C11DB7
 *    Initial    : 0xFFFFFFFF
 *    RefIn      : true   (input bytes are bit-reversed before processing)
 *    RefOut     : true   (final register value is bit-reversed before XOR)
 *    XorOut     : 0xFFFFFFFF
 *    Check(123456789) : 0xCBF43926
 *
 *  The STM32 CRC unit is configured in crc32_hw.c with:
 *    InputDataInversionMode  = CRC_INPUTDATA_INVERSION_BYTE  (refin=true)
 *    OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_ENABLE (refout=true)
 *    DefaultPolynomialUse    = DEFAULT_POLYNOMIAL_ENABLE  (0x04C11DB7)
 *    DefaultInitValueUse     = DEFAULT_INIT_VALUE_ENABLE  (0xFFFFFFFF)
 *  plus a final ^ 0xFFFFFFFF in software.
 *  This software table matches that behaviour exactly so that known-answer
 *  tests are stable across host and target.
 */

#include "board_mock.h"
#include <string.h>   /* memcpy, memset */
#include <stddef.h>

/* =========================================================================
 *  Internal state
 * ========================================================================= */

/* Virtual clock */
static uint32_t s_millis = 0;

/* GPIO pin table */
static mock_GpioEntry_t s_gpio[MOCK_GPIO_MAX_PINS];

/* =========================================================================
 *  Helper: locate or allocate a GPIO table slot
 * ========================================================================= */
static mock_GpioEntry_t *gpio_find(casper_pin_t pin)
{
    /* Search for existing slot */
    for (int i = 0; i < MOCK_GPIO_MAX_PINS; i++) {
        if (s_gpio[i].valid &&
            s_gpio[i].port == pin.port &&
            s_gpio[i].pin  == pin.pin)
        {
            return &s_gpio[i];
        }
    }
    /* Allocate a new slot */
    for (int i = 0; i < MOCK_GPIO_MAX_PINS; i++) {
        if (!s_gpio[i].valid) {
            s_gpio[i].valid        = 1;
            s_gpio[i].port         = pin.port;
            s_gpio[i].pin          = pin.pin;
            s_gpio[i].out_state    = CASPER_PIN_LOW;
            s_gpio[i].in_queue     = CASPER_PIN_LOW;
            s_gpio[i].toggle_count = 0;
            return &s_gpio[i];
        }
    }
    return NULL; /* Table full — enlarge MOCK_GPIO_MAX_PINS */
}

/* =========================================================================
 *  mock_reset
 * ========================================================================= */
void mock_reset(void)
{
    s_millis = 0;
    memset(s_gpio, 0, sizeof(s_gpio));
    /* The bus/adc/pwm structs are owned by the test and reset by the test
     * (re-assign with mock_*_make()).  We only clear global state here. */
}

/* =========================================================================
 *  Virtual clock
 * ========================================================================= */
void mock_set_millis(uint32_t ms)   { s_millis = ms; }
void mock_advance_ms(uint32_t delta){ s_millis += delta; }

/* casper_time.h implementations */
uint32_t casper_millis(void)         { return s_millis; }
uint32_t casper_micros(void)         { return s_millis * 1000u; }
void     casper_delay_ms(uint32_t ms){ s_millis += ms; }

/* =========================================================================
 *  GPIO
 * ========================================================================= */
void casper_gpio_write(casper_pin_t pin, casper_pin_state_t s)
{
    mock_GpioEntry_t *e = gpio_find(pin);
    if (e) {
        e->out_state = s;
        e->in_queue  = s;   /* written state is also readable */
    }
}

casper_pin_state_t casper_gpio_read(casper_pin_t pin)
{
    mock_GpioEntry_t *e = gpio_find(pin);
    if (e) return e->in_queue;
    return CASPER_PIN_LOW;
}

void casper_gpio_toggle(casper_pin_t pin)
{
    mock_GpioEntry_t *e = gpio_find(pin);
    if (e) {
        e->out_state    = (e->out_state == CASPER_PIN_LOW) ? CASPER_PIN_HIGH : CASPER_PIN_LOW;
        e->in_queue     = e->out_state;
        e->toggle_count++;
    } else {
        /* Pin not yet seen — allocate and toggle from LOW to HIGH */
        e = gpio_find(pin); /* will allocate on second call — call again */
        if (e) {
            e->out_state    = CASPER_PIN_HIGH;
            e->in_queue     = CASPER_PIN_HIGH;
            e->toggle_count = 1;
        }
    }
}

/* GPIO test helpers */
casper_pin_state_t mock_gpio_get_state(casper_pin_t pin)
{
    mock_GpioEntry_t *e = gpio_find(pin);
    if (e) return e->out_state;
    return CASPER_PIN_LOW;
}

void mock_gpio_set_input(casper_pin_t pin, casper_pin_state_t state)
{
    mock_GpioEntry_t *e = gpio_find(pin);
    if (e) e->in_queue = state;
}

int mock_gpio_toggle_count(casper_pin_t pin)
{
    mock_GpioEntry_t *e = gpio_find(pin);
    if (e) return e->toggle_count;
    return 0;
}

/* =========================================================================
 *  SPI
 * ========================================================================= */

/* Internal: dequeue up to n bytes from the bus RX queue into dst.
 * Any byte beyond what was queued is filled with 0xFF. */
static void spi_dequeue_rx(casper_spi_t *bus, uint8_t *dst, uint16_t n)
{
    for (uint16_t i = 0; i < n; i++) {
        if (bus->rx_head < bus->rx_tail) {
            dst[i] = bus->rx_queue[bus->rx_head++];
        } else {
            dst[i] = 0xFFu; /* idle SPI byte */
        }
    }
}

void mock_spi_push_rx(casper_spi_t *bus, const uint8_t *bytes, int n)
{
    for (int i = 0; i < n && bus->rx_tail < MOCK_SPI_RX_QUEUE; i++) {
        bus->rx_queue[bus->rx_tail++] = bytes[i];
    }
}

int mock_spi_log_count(casper_spi_t *bus)          { return bus->log_count; }

const mock_SpiLog_t *mock_spi_log_get(casper_spi_t *bus, int idx)
{
    if (idx < 0 || idx >= bus->log_count) return NULL;
    return &bus->log[idx];
}

casper_status_t casper_spi_transmit(casper_spi_t *bus,
                                    const uint8_t *tx,
                                    uint16_t n,
                                    uint32_t to_ms)
{
    if (bus->log_count >= MOCK_SPI_LOG_MAX) return CASPER_ERR;
    mock_SpiLog_t *e = &bus->log[bus->log_count++];
    e->op    = MOCK_SPI_TX;
    e->len   = n;
    e->to_ms = to_ms;
    uint16_t copy = (n <= MOCK_SPI_BUF_MAX) ? n : MOCK_SPI_BUF_MAX;
    memcpy(e->tx, tx, copy);
    memset(e->rx, 0, sizeof(e->rx));
    return CASPER_OK;
}

casper_status_t casper_spi_receive(casper_spi_t *bus,
                                   uint8_t *rx,
                                   uint16_t n,
                                   uint32_t to_ms)
{
    if (bus->log_count >= MOCK_SPI_LOG_MAX) return CASPER_ERR;
    mock_SpiLog_t *e = &bus->log[bus->log_count++];
    e->op    = MOCK_SPI_RX;
    e->len   = n;
    e->to_ms = to_ms;
    memset(e->tx, 0, sizeof(e->tx));
    spi_dequeue_rx(bus, rx, n);
    uint16_t copy = (n <= MOCK_SPI_BUF_MAX) ? n : MOCK_SPI_BUF_MAX;
    memcpy(e->rx, rx, copy);
    return CASPER_OK;
}

casper_status_t casper_spi_transceive(casper_spi_t *bus,
                                      const uint8_t *tx,
                                      uint8_t *rx,
                                      uint16_t n,
                                      uint32_t to_ms)
{
    if (bus->log_count >= MOCK_SPI_LOG_MAX) return CASPER_ERR;
    mock_SpiLog_t *e = &bus->log[bus->log_count++];
    e->op    = MOCK_SPI_TRANSCEIVE;
    e->len   = n;
    e->to_ms = to_ms;
    uint16_t copy = (n <= MOCK_SPI_BUF_MAX) ? n : MOCK_SPI_BUF_MAX;
    memcpy(e->tx, tx, copy);
    spi_dequeue_rx(bus, rx, n);
    memcpy(e->rx, rx, copy);
    return CASPER_OK;
}

/* =========================================================================
 *  I2C
 * ========================================================================= */
static void i2c_dequeue_rx(casper_i2c_t *bus, uint8_t *dst, uint16_t n)
{
    for (uint16_t i = 0; i < n; i++) {
        if (bus->rx_head < bus->rx_tail) {
            dst[i] = bus->rx_queue[bus->rx_head++];
        } else {
            dst[i] = 0x00u;
        }
    }
}

void mock_i2c_push_rx(casper_i2c_t *bus, const uint8_t *bytes, int n)
{
    for (int i = 0; i < n && bus->rx_tail < MOCK_I2C_RX_QUEUE; i++) {
        bus->rx_queue[bus->rx_tail++] = bytes[i];
    }
}

int mock_i2c_log_count(casper_i2c_t *bus)          { return bus->log_count; }

const mock_I2cLog_t *mock_i2c_log_get(casper_i2c_t *bus, int idx)
{
    if (idx < 0 || idx >= bus->log_count) return NULL;
    return &bus->log[idx];
}

casper_status_t casper_i2c_mem_read(casper_i2c_t *bus,
                                    uint16_t addr8,
                                    uint16_t reg,
                                    uint16_t reg_sz,
                                    uint8_t *buf,
                                    uint16_t n,
                                    uint32_t to_ms)
{
    if (bus->log_count >= MOCK_I2C_LOG_MAX) return CASPER_ERR;
    mock_I2cLog_t *e = &bus->log[bus->log_count++];
    e->op     = MOCK_I2C_MEM_READ;
    e->addr8  = addr8;
    e->reg    = reg;
    e->reg_sz = reg_sz;
    e->len    = n;
    e->to_ms  = to_ms;
    e->trials = 0;
    i2c_dequeue_rx(bus, buf, n);
    uint16_t copy = (n <= MOCK_I2C_BUF_MAX) ? n : MOCK_I2C_BUF_MAX;
    memcpy(e->buf, buf, copy);
    return CASPER_OK;
}

casper_status_t casper_i2c_mem_write(casper_i2c_t *bus,
                                     uint16_t addr8,
                                     uint16_t reg,
                                     uint16_t reg_sz,
                                     const uint8_t *buf,
                                     uint16_t n,
                                     uint32_t to_ms)
{
    if (bus->log_count >= MOCK_I2C_LOG_MAX) return CASPER_ERR;
    mock_I2cLog_t *e = &bus->log[bus->log_count++];
    e->op     = MOCK_I2C_MEM_WRITE;
    e->addr8  = addr8;
    e->reg    = reg;
    e->reg_sz = reg_sz;
    e->len    = n;
    e->to_ms  = to_ms;
    e->trials = 0;
    uint16_t copy = (n <= MOCK_I2C_BUF_MAX) ? n : MOCK_I2C_BUF_MAX;
    memcpy(e->buf, buf, copy);
    return CASPER_OK;
}

casper_status_t casper_i2c_master_tx(casper_i2c_t *bus,
                                     uint16_t addr8,
                                     const uint8_t *buf,
                                     uint16_t n,
                                     uint32_t to_ms)
{
    if (bus->log_count >= MOCK_I2C_LOG_MAX) return CASPER_ERR;
    mock_I2cLog_t *e = &bus->log[bus->log_count++];
    e->op     = MOCK_I2C_MASTER_TX;
    e->addr8  = addr8;
    e->reg    = 0;
    e->reg_sz = 0;
    e->len    = n;
    e->to_ms  = to_ms;
    e->trials = 0;
    uint16_t copy = (n <= MOCK_I2C_BUF_MAX) ? n : MOCK_I2C_BUF_MAX;
    memcpy(e->buf, buf, copy);
    return CASPER_OK;
}

casper_status_t casper_i2c_dev_ready(casper_i2c_t *bus,
                                     uint16_t addr8,
                                     uint32_t trials,
                                     uint32_t to_ms)
{
    if (bus->log_count >= MOCK_I2C_LOG_MAX) return CASPER_ERR;
    mock_I2cLog_t *e = &bus->log[bus->log_count++];
    e->op     = MOCK_I2C_DEV_READY;
    e->addr8  = addr8;
    e->reg    = 0;
    e->reg_sz = 0;
    e->len    = 0;
    e->to_ms  = to_ms;
    e->trials = trials;
    memset(e->buf, 0, sizeof(e->buf));
    return CASPER_OK;
}

/* =========================================================================
 *  ADC
 * ========================================================================= */
void casper_adc_init_all(void)
{
    /* No-op on host. */
}

void mock_adc_push(casper_adc_t *ch, uint16_t raw)
{
    if (ch->q_tail < MOCK_ADC_QUEUE_MAX) {
        ch->queue[ch->q_tail++] = raw;
    }
}

casper_status_t casper_adc_read(casper_adc_t *ch, uint16_t *out_raw)
{
    if (ch->q_head < ch->q_tail) {
        *out_raw = ch->queue[ch->q_head++];
    } else {
        *out_raw = 0;
    }
    return CASPER_OK;
}

/* =========================================================================
 *  QSPI
 * ========================================================================= */
static void qspi_dequeue_rx(casper_qspi_t *dev, uint8_t *dst, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) {
        if (dev->rx_head < dev->rx_tail) {
            dst[i] = dev->rx_queue[dev->rx_head++];
        } else {
            dst[i] = 0xFFu;
        }
    }
}

void mock_qspi_push_rx(casper_qspi_t *dev, const uint8_t *bytes, int n)
{
    for (int i = 0; i < n && dev->rx_tail < MOCK_QSPI_RX_QUEUE; i++) {
        dev->rx_queue[dev->rx_tail++] = bytes[i];
    }
}

int mock_qspi_log_count(casper_qspi_t *dev) { return dev->log_count; }

const mock_QspiLog_t *mock_qspi_log_get(casper_qspi_t *dev, int idx)
{
    if (idx < 0 || idx >= dev->log_count) return NULL;
    return &dev->log[idx];
}

void mock_qspi_fire(casper_qspi_t *dev, casper_qspi_evt_t evt)
{
    if (dev->handler) {
        dev->handler(dev->handler_ctx, evt);
    }
}

casper_status_t casper_qspi_command(casper_qspi_t *dev,
                                    const casper_qspi_cmd_t *cmd,
                                    uint32_t to_ms)
{
    if (dev->log_count >= MOCK_QSPI_LOG_MAX) return CASPER_ERR;
    mock_QspiLog_t *e = &dev->log[dev->log_count++];
    e->op    = MOCK_QSPI_CMD;
    e->cmd   = *cmd;
    e->len   = 0;
    e->to_ms = to_ms;
    memset(e->buf, 0, sizeof(e->buf));

    /* Save as pending command for subsequent transmit/receive calls */
    dev->pending_cmd     = *cmd;
    dev->has_pending_cmd = 1;
    return CASPER_OK;
}

casper_status_t casper_qspi_transmit(casper_qspi_t *dev,
                                     const uint8_t *buf,
                                     uint32_t to_ms)
{
    if (dev->log_count >= MOCK_QSPI_LOG_MAX) return CASPER_ERR;
    mock_QspiLog_t *e = &dev->log[dev->log_count++];
    e->op    = MOCK_QSPI_TX;
    e->to_ms = to_ms;
    if (dev->has_pending_cmd) {
        e->cmd = dev->pending_cmd;
        e->len = dev->pending_cmd.data_len;
        dev->has_pending_cmd = 0;
    } else {
        e->len = 0;
    }
    uint32_t copy = (e->len <= MOCK_QSPI_BUF_MAX) ? e->len : MOCK_QSPI_BUF_MAX;
    if (buf && copy > 0) memcpy(e->buf, buf, copy);
    return CASPER_OK;
}

casper_status_t casper_qspi_receive(casper_qspi_t *dev,
                                    uint8_t *buf,
                                    uint32_t to_ms)
{
    if (dev->log_count >= MOCK_QSPI_LOG_MAX) return CASPER_ERR;
    mock_QspiLog_t *e = &dev->log[dev->log_count++];
    e->op    = MOCK_QSPI_RX;
    e->to_ms = to_ms;
    if (dev->has_pending_cmd) {
        e->cmd = dev->pending_cmd;
        e->len = dev->pending_cmd.data_len;
        dev->has_pending_cmd = 0;
    } else {
        e->len = 0;
    }
    if (buf && e->len > 0) {
        qspi_dequeue_rx(dev, buf, e->len);
        uint32_t copy = (e->len <= MOCK_QSPI_BUF_MAX) ? e->len : MOCK_QSPI_BUF_MAX;
        memcpy(e->buf, buf, copy);
    }
    return CASPER_OK;
}

casper_status_t casper_qspi_transmit_it(casper_qspi_t *dev,
                                        const uint8_t *buf)
{
    if (dev->log_count >= MOCK_QSPI_LOG_MAX) return CASPER_ERR;
    mock_QspiLog_t *e = &dev->log[dev->log_count++];
    e->op    = MOCK_QSPI_TX_IT;
    e->to_ms = 0;
    if (dev->has_pending_cmd) {
        e->cmd = dev->pending_cmd;
        e->len = dev->pending_cmd.data_len;
        dev->has_pending_cmd = 0;
    } else {
        e->len = 0;
    }
    uint32_t copy = (e->len <= MOCK_QSPI_BUF_MAX) ? e->len : MOCK_QSPI_BUF_MAX;
    if (buf && copy > 0) memcpy(e->buf, buf, copy);
    /* In the mock, IT completion is under test control via mock_qspi_fire() */
    return CASPER_OK;
}

casper_status_t casper_qspi_autopoll_it(casper_qspi_t *dev,
                                        const casper_qspi_poll_t *poll)
{
    if (dev->log_count >= MOCK_QSPI_LOG_MAX) return CASPER_ERR;
    mock_QspiLog_t *e = &dev->log[dev->log_count++];
    e->op    = MOCK_QSPI_AUTOPOLL;
    e->poll  = *poll;
    e->len   = 0;
    e->to_ms = 0;
    memset(&e->cmd, 0, sizeof(e->cmd));
    memset(e->buf, 0, sizeof(e->buf));
    /* Completion is signalled via mock_qspi_fire() */
    return CASPER_OK;
}

void casper_qspi_set_handler(casper_qspi_t *dev,
                             void (*fn)(void *ctx, casper_qspi_evt_t evt),
                             void *ctx)
{
    dev->handler     = fn;
    dev->handler_ctx = ctx;
}

/* =========================================================================
 *  PWM
 * ========================================================================= */
void casper_pwm_tone_start(casper_pwm_t *pwm)
{
    pwm->started = 1;
    pwm->call_count++;
}

void casper_pwm_tone_set(casper_pwm_t *pwm, uint32_t arr, uint32_t ccr)
{
    pwm->last_arr = arr;
    pwm->last_ccr = ccr;
    pwm->call_count++;
}

void casper_pwm_tone_off(casper_pwm_t *pwm)
{
    pwm->started = 0;
    pwm->call_count++;
}

/* PWM test helpers */
int      mock_pwm_started(casper_pwm_t *pwm)    { return pwm->started; }
uint32_t mock_pwm_last_arr(casper_pwm_t *pwm)   { return pwm->last_arr; }
uint32_t mock_pwm_last_ccr(casper_pwm_t *pwm)   { return pwm->last_ccr; }
int      mock_pwm_call_count(casper_pwm_t *pwm) { return pwm->call_count; }

/* =========================================================================
 *  CRC-32 — software implementation matching STM32H7 CRC unit
 * =========================================================================
 *
 *  Algorithm: CRC-32/ISO-HDLC
 *    Polynomial  : 0x04C11DB7
 *    Initial val : 0xFFFFFFFF
 *    RefIn=true  : each input byte is processed LSB-first
 *    RefOut=true : the final register value is reflected before XOR
 *    XorOut      : 0xFFFFFFFF
 *    Check value : CRC32("123456789") == 0xCBF43926
 *
 *  Implementation uses a 256-entry look-up table pre-computed at compile
 *  time (or rather, at first use via crc32_init) for speed.  The table is
 *  the standard reflected-polynomial table used by zlib/crc32.
 *
 *  Reflected polynomial: reflect(0x04C11DB7, 32) = 0xEDB88320
 */

static uint32_t s_crc_table[256];
static int      s_crc_ready = 0;

/* Build the reflected look-up table once. */
static void crc32_build_table(void)
{
    for (uint32_t b = 0; b < 256u; b++) {
        uint32_t crc = b;
        for (int i = 0; i < 8; i++) {
            crc = (crc & 1u) ? (0xEDB88320u ^ (crc >> 1)) : (crc >> 1);
        }
        s_crc_table[b] = crc;
    }
    s_crc_ready = 1;
}

void casper_crc32_init(void)
{
    if (!s_crc_ready) crc32_build_table();
}

uint32_t casper_crc32_compute(const uint8_t *data, uint32_t len)
{
    if (!s_crc_ready) crc32_build_table();
    uint32_t crc = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < len; i++) {
        crc = s_crc_table[(crc ^ data[i]) & 0xFFu] ^ (crc >> 8);
    }
    return crc ^ 0xFFFFFFFFu;
}
