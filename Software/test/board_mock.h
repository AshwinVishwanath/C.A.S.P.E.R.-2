/**
 * @file  board_mock.h
 * @brief Host (gcc) mock implementation of the full casper_port interface.
 *
 * Include this header in host unit-test files.  It provides:
 *   - Struct definitions for all opaque casper_*_t handles
 *   - Test-facing control/inspection API (mock_* functions)
 *   - Accessor helpers for logged transactions
 *
 * -----------------------------------------------------------------------
 *  Quick-reference API cheat-sheet (for test authors)
 * -----------------------------------------------------------------------
 *
 *  RESET / SETUP
 *    mock_reset()                          — clear all logs, queues, GPIO, clocks
 *
 *  VIRTUAL CLOCK
 *    mock_set_millis(uint32_t ms)          — set current millis tick
 *    mock_advance_ms(uint32_t delta)       — add delta ms to tick
 *    casper_millis()  → uint32_t          — returns current virtual millis
 *    casper_micros()  → uint32_t          — returns virtual_ms * 1000
 *    casper_delay_ms(ms)                  — advances virtual clock by ms
 *
 *  SPI TRANSACTIONS  (mock_SpiLog_t entries, one per call)
 *    mock_spi_push_rx(bus, bytes, n)      — queue bytes returned by next receive/transceive
 *    mock_spi_log_count(bus)  → int       — number of logged calls on this bus
 *    mock_spi_log_get(bus, idx)           — get SpiLog entry by index (returns pointer)
 *
 *  SpiLog entry fields:
 *    .op    — MOCK_SPI_TX / MOCK_SPI_RX / MOCK_SPI_TRANSCEIVE
 *    .tx[]  — bytes sent (tx/transceive; zero for rx)
 *    .rx[]  — bytes received (rx/transceive; as scripted by push_rx)
 *    .len   — transfer length
 *
 *  I2C TRANSACTIONS  (mock_I2cLog_t entries, one per call)
 *    mock_i2c_push_rx(bus, bytes, n)      — queue bytes returned by next mem_read
 *    mock_i2c_log_count(bus)  → int       — number of logged calls
 *    mock_i2c_log_get(bus, idx)           — get I2cLog entry by index
 *
 *  I2cLog entry fields:
 *    .op    — MOCK_I2C_MEM_READ / MOCK_I2C_MEM_WRITE / MOCK_I2C_MASTER_TX / MOCK_I2C_DEV_READY
 *    .addr8 — 8-bit device address (7-bit << 1)
 *    .reg   — register address
 *    .reg_sz— register address size (1 or 2)
 *    .buf[] — data bytes (written or read back)
 *    .len   — data length
 *
 *  QSPI TRANSACTIONS  (mock_QspiLog_t entries)
 *    mock_qspi_push_rx(dev, bytes, n)     — queue bytes returned by next blocking receive
 *    mock_qspi_log_count(dev)  → int
 *    mock_qspi_log_get(dev, idx)          — get QspiLog entry by index
 *    mock_qspi_fire(dev, evt)             — immediately invoke the registered handler with evt
 *
 *  QspiLog entry fields:
 *    .op    — MOCK_QSPI_CMD / MOCK_QSPI_TX / MOCK_QSPI_RX / MOCK_QSPI_TX_IT / MOCK_QSPI_AUTOPOLL
 *    .cmd   — copy of casper_qspi_cmd_t (for CMD/TX/RX/TX_IT)
 *    .poll  — copy of casper_qspi_poll_t (for AUTOPOLL)
 *    .buf[] — data bytes (transmit or received)
 *    .len   — data byte count
 *
 *  GPIO
 *    mock_gpio_get_state(pin)  → casper_pin_state_t   — last written state
 *    mock_gpio_set_input(pin, state)                  — queue input level for next read
 *    mock_gpio_toggle_count(pin)  → int               — number of toggle() calls
 *    (casper_gpio_write writes the state; casper_gpio_read returns the last
 *     queued input state, defaulting to LOW; casper_gpio_toggle flips the
 *     last-written state and increments the toggle counter)
 *
 *  ADC
 *    mock_adc_push(ch, raw)   — queue a uint16_t raw value for next casper_adc_read()
 *    (returns queued values FIFO; returns 0 if queue empty)
 *
 *  PWM
 *    mock_pwm_started(pwm)    → int        — 1 if tone_start was called, 0 after tone_off
 *    mock_pwm_last_arr(pwm)   → uint32_t   — last ARR value from tone_set
 *    mock_pwm_last_ccr(pwm)   → uint32_t   — last CCR value from tone_set
 *    mock_pwm_call_count(pwm) → int        — total tone_start + tone_set + tone_off calls
 *
 *  CRC-32
 *    casper_crc32_init()      — no-op (software table already ready)
 *    casper_crc32_compute(data, len)  → uint32_t
 *    Variant: CRC-32/ISO-HDLC  poly=0x04C11DB7, init=0xFFFFFFFF,
 *             refin=true, refout=true, xorout=0xFFFFFFFF  (matches crc32_hw.c)
 *
 * -----------------------------------------------------------------------
 *  Bus handle constructors  (declare static instances in your test)
 * -----------------------------------------------------------------------
 *  casper_spi_t  MY_SPI  = mock_spi_make("MY_SPI");
 *  casper_i2c_t  MY_I2C  = mock_i2c_make("MY_I2C");
 *  casper_qspi_t MY_QSPI = mock_qspi_make("MY_QSPI");
 *  casper_adc_t  MY_ADC  = mock_adc_make("MY_ADC");
 *  casper_pwm_t  MY_PWM  = mock_pwm_make("MY_PWM");
 *
 *  casper_pin_t  MY_PIN  = { (void*)0x1000, (uint32_t)(1u << 5) };
 *                         — port + pin are opaque; any unique values work.
 * -----------------------------------------------------------------------
 */

#ifndef BOARD_MOCK_H
#define BOARD_MOCK_H

#include <stdint.h>
#include <stddef.h>
#include "casper_port.h"   /* pulls in all casper_*.h headers */

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 *  Capacity constants — enlarge if a test needs longer sequences
 * ========================================================================= */
#define MOCK_SPI_LOG_MAX   256   /**< Max SPI transactions logged per bus.   */
#define MOCK_SPI_RX_QUEUE  256   /**< Max bytes in the SPI RX queue.         */
#define MOCK_SPI_BUF_MAX   256   /**< Max bytes per single SPI transaction.  */

#define MOCK_I2C_LOG_MAX   256
#define MOCK_I2C_RX_QUEUE  256
#define MOCK_I2C_BUF_MAX   256

#define MOCK_QSPI_LOG_MAX  256
#define MOCK_QSPI_RX_QUEUE 4096
#define MOCK_QSPI_BUF_MAX  4096

#define MOCK_ADC_QUEUE_MAX 64

/* =========================================================================
 *  SPI transaction log
 * ========================================================================= */
typedef enum {
    MOCK_SPI_TX,
    MOCK_SPI_RX,
    MOCK_SPI_TRANSCEIVE
} mock_spi_op_t;

typedef struct {
    mock_spi_op_t op;
    uint8_t       tx[MOCK_SPI_BUF_MAX];
    uint8_t       rx[MOCK_SPI_BUF_MAX];
    uint16_t      len;
    uint32_t      to_ms;
} mock_SpiLog_t;

/* =========================================================================
 *  I2C transaction log
 * ========================================================================= */
typedef enum {
    MOCK_I2C_MEM_READ,
    MOCK_I2C_MEM_WRITE,
    MOCK_I2C_MASTER_TX,
    MOCK_I2C_DEV_READY
} mock_i2c_op_t;

typedef struct {
    mock_i2c_op_t op;
    uint16_t      addr8;
    uint16_t      reg;
    uint16_t      reg_sz;
    uint8_t       buf[MOCK_I2C_BUF_MAX];
    uint16_t      len;
    uint32_t      trials;   /**< Only valid for DEV_READY op. */
    uint32_t      to_ms;
} mock_I2cLog_t;

/* =========================================================================
 *  QSPI transaction log
 * ========================================================================= */
typedef enum {
    MOCK_QSPI_CMD,
    MOCK_QSPI_TX,
    MOCK_QSPI_RX,
    MOCK_QSPI_TX_IT,
    MOCK_QSPI_AUTOPOLL
} mock_qspi_op_t;

typedef struct {
    mock_qspi_op_t     op;
    casper_qspi_cmd_t  cmd;    /**< Valid for CMD/TX/RX/TX_IT ops. */
    casper_qspi_poll_t poll;   /**< Valid for AUTOPOLL op.         */
    uint8_t            buf[MOCK_QSPI_BUF_MAX];
    uint32_t           len;
    uint32_t           to_ms;
} mock_QspiLog_t;

/* =========================================================================
 *  Opaque struct definitions — visible only to mock implementation + tests
 * ========================================================================= */

struct casper_spi_s {
    const char    *name;

    /* Transaction log */
    mock_SpiLog_t  log[MOCK_SPI_LOG_MAX];
    int            log_count;

    /* Scripted RX queue */
    uint8_t        rx_queue[MOCK_SPI_RX_QUEUE];
    int            rx_head;
    int            rx_tail;   /**< Points one past last queued byte. */
};

struct casper_i2c_s {
    const char    *name;

    mock_I2cLog_t  log[MOCK_I2C_LOG_MAX];
    int            log_count;

    uint8_t        rx_queue[MOCK_I2C_RX_QUEUE];
    int            rx_head;
    int            rx_tail;
};

struct casper_qspi_s {
    const char      *name;

    mock_QspiLog_t   log[MOCK_QSPI_LOG_MAX];
    int              log_count;

    uint8_t          rx_queue[MOCK_QSPI_RX_QUEUE];
    int              rx_head;
    int              rx_tail;

    /* Pending command (set by casper_qspi_command, consumed by tx/rx/it) */
    casper_qspi_cmd_t pending_cmd;
    int               has_pending_cmd;

    /* IT handler */
    void (*handler)(void *ctx, casper_qspi_evt_t evt);
    void  *handler_ctx;
};

struct casper_adc_s {
    const char  *name;

    uint16_t     queue[MOCK_ADC_QUEUE_MAX];
    int          q_head;
    int          q_tail;
};

struct casper_pwm_s {
    const char  *name;

    int          started;      /**< 1 after tone_start, 0 after tone_off. */
    uint32_t     last_arr;
    uint32_t     last_ccr;
    int          call_count;   /**< Total start + set + off calls.         */
};

/* =========================================================================
 *  GPIO state entry (keyed by port+pin pair)
 * ========================================================================= */
#define MOCK_GPIO_MAX_PINS 64

typedef struct {
    void              *port;
    uint32_t           pin;
    casper_pin_state_t out_state;    /**< Last written state.               */
    casper_pin_state_t in_queue;     /**< Value returned by next read().    */
    int                toggle_count;
    int                valid;        /**< 1 if this slot is in use.         */
} mock_GpioEntry_t;

/* =========================================================================
 *  Constructor macros — initialise a handle with a name and zeroed state.
 *
 *  Usage (in test file at file scope or inside a function):
 *    casper_spi_t  MY_SPI  = mock_spi_make("MY_SPI");
 *
 *  The macro expands to a designated-initializer that sets .name and leaves
 *  all other fields zero-initialised.  Using a designated initializer avoids
 *  the "braces around scalar initializer" warning that arises with positional
 *  initializers for deeply nested struct arrays.
 * ========================================================================= */
#define mock_spi_make(n)  { .name = (n) }
#define mock_i2c_make(n)  { .name = (n) }
#define mock_qspi_make(n) { .name = (n) }
#define mock_adc_make(n)  { .name = (n) }
#define mock_pwm_make(n)  { .name = (n) }

/* =========================================================================
 *  Test-control API
 * ========================================================================= */

/**
 * @brief Reset all mock state: clear logs, queues, GPIO table, virtual clock.
 *
 * Call at the start of each test (or in a setup/teardown fixture) to ensure
 * a clean slate.  Does not modify the handle structs themselves (caller
 * re-initialises those with mock_*_make() as needed).
 */
void mock_reset(void);

/* -----------------------------------------------------------------------
 *  Virtual clock
 * --------------------------------------------------------------------- */

/** Set the current millis tick to @p ms. */
void mock_set_millis(uint32_t ms);

/** Advance the virtual clock by @p delta milliseconds. */
void mock_advance_ms(uint32_t delta);

/* -----------------------------------------------------------------------
 *  SPI helpers
 * --------------------------------------------------------------------- */

/**
 * @brief Queue @p n bytes to be returned as RX data on the next SPI
 *        receive or transceive call on @p bus.
 *
 * Bytes are consumed FIFO across multiple calls.  If more bytes are
 * requested by the firmware than are queued, the mock fills the remainder
 * with 0xFF (idle SPI byte).
 */
void mock_spi_push_rx(casper_spi_t *bus, const uint8_t *bytes, int n);

/** Number of transactions logged on @p bus. */
int mock_spi_log_count(casper_spi_t *bus);

/**
 * @brief Return pointer to transaction log entry @p idx on @p bus.
 *
 * Returns NULL if @p idx is out of range.
 */
const mock_SpiLog_t *mock_spi_log_get(casper_spi_t *bus, int idx);

/* -----------------------------------------------------------------------
 *  I2C helpers
 * --------------------------------------------------------------------- */

/**
 * @brief Queue @p n bytes to be returned as read data on the next
 *        casper_i2c_mem_read() on @p bus.
 */
void mock_i2c_push_rx(casper_i2c_t *bus, const uint8_t *bytes, int n);

int mock_i2c_log_count(casper_i2c_t *bus);
const mock_I2cLog_t *mock_i2c_log_get(casper_i2c_t *bus, int idx);

/* -----------------------------------------------------------------------
 *  QSPI helpers
 * --------------------------------------------------------------------- */

/**
 * @brief Queue @p n bytes to be returned by the next casper_qspi_receive()
 *        call on @p dev.
 */
void mock_qspi_push_rx(casper_qspi_t *dev, const uint8_t *bytes, int n);

int mock_qspi_log_count(casper_qspi_t *dev);
const mock_QspiLog_t *mock_qspi_log_get(casper_qspi_t *dev, int idx);

/**
 * @brief Synchronously invoke the handler registered on @p dev with @p evt.
 *
 * Simulates the board layer firing a QSPI IT callback.  If no handler is
 * registered, this is a no-op.
 */
void mock_qspi_fire(casper_qspi_t *dev, casper_qspi_evt_t evt);

/* -----------------------------------------------------------------------
 *  GPIO helpers
 * --------------------------------------------------------------------- */

/**
 * @brief Return the last state written to @p pin via casper_gpio_write().
 *
 * Returns CASPER_PIN_LOW if the pin has never been written.
 */
casper_pin_state_t mock_gpio_get_state(casper_pin_t pin);

/**
 * @brief Queue a state that casper_gpio_read() will return for @p pin.
 *
 * The queued value persists until overwritten by another mock_gpio_set_input()
 * or a casper_gpio_write() (which also updates the readable state).
 */
void mock_gpio_set_input(casper_pin_t pin, casper_pin_state_t state);

/**
 * @brief Return the number of times casper_gpio_toggle() was called on @p pin.
 */
int mock_gpio_toggle_count(casper_pin_t pin);

/* -----------------------------------------------------------------------
 *  ADC helpers
 * --------------------------------------------------------------------- */

/**
 * @brief Queue one raw ADC reading for @p ch.
 *
 * casper_adc_read() dequeues FIFO.  Returns 0 if the queue is empty.
 */
void mock_adc_push(casper_adc_t *ch, uint16_t raw);

/* -----------------------------------------------------------------------
 *  PWM helpers
 * --------------------------------------------------------------------- */

/** 1 if casper_pwm_tone_start() was called more recently than tone_off(). */
int mock_pwm_started(casper_pwm_t *pwm);

/** Last ARR value passed to casper_pwm_tone_set(). */
uint32_t mock_pwm_last_arr(casper_pwm_t *pwm);

/** Last CCR value passed to casper_pwm_tone_set(). */
uint32_t mock_pwm_last_ccr(casper_pwm_t *pwm);

/** Total number of tone_start + tone_set + tone_off calls. */
int mock_pwm_call_count(casper_pwm_t *pwm);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_MOCK_H */
