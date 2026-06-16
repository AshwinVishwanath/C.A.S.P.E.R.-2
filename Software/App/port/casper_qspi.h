/**
 * @file  casper_qspi.h
 * @brief Portable QSPI / OSPI NOR-flash interface.
 *
 * Pure C11.  Implemented in the board layer (board_<target>.c).
 *
 * Provides three categories of operations:
 *   1. Blocking command + data I/O   (casper_qspi_command / transmit / receive)
 *   2. Non-blocking (IT) data I/O    (casper_qspi_transmit_it / autopoll_it)
 *   3. Event handler registration    (casper_qspi_set_handler)
 *
 * -----------------------------------------------------------------------
 *  casper_qspi_cmd_t field semantics
 * -----------------------------------------------------------------------
 *  instruction        : Opcode byte sent as the first byte of every
 *                       QSPI frame (e.g. 0x12 = PAGE_PROGRAM_4B).
 *  instruction_lines  : Number of data lines used for the instruction
 *                       phase.  1 = single-line SPI, 4 = quad.
 *                       On STM32 HAL maps to QSPI_INSTRUCTION_n_LINE.
 *  address_lines      : Lines for the address phase
 *                       (0 = no address phase, 1 or 4).
 *  address_bytes      : Width of the address field in bytes (3 or 4).
 *                       Maps to QSPI_ADDRESS_24BITS / _32BITS.
 *  address            : 24- or 32-bit flash byte address for this command.
 *                       Ignored when address_lines == 0.
 *  data_lines         : Lines for the data phase
 *                       (0 = no data phase, 1 or 4).
 *  dummy_cycles       : Clock cycles inserted between address and data
 *                       phases (device-dependent; e.g. 8 for Fast Read).
 *  data_len           : Number of bytes to transfer in the data phase.
 *                       Ignored when data_lines == 0.
 *
 * -----------------------------------------------------------------------
 *  Interrupt (IT) model
 * -----------------------------------------------------------------------
 *  casper_qspi_transmit_it() and casper_qspi_autopoll_it() start their
 *  respective operations and return immediately.  Completion (or error)
 *  is signalled by calling the handler registered with
 *  casper_qspi_set_handler().
 *
 *  The board layer's HAL_QSPI_TxCpltCallback / HAL_QSPI_StatusMatchCallback /
 *  HAL_QSPI_ErrorCallback implementations are responsible for translating
 *  those HAL weak callbacks into the appropriate casper_qspi_evt_t and
 *  invoking the registered handler.
 *
 *  Only one handler may be registered per casper_qspi_t instance.
 *  Calling casper_qspi_set_handler() again replaces the previous handler.
 */

#ifndef CASPER_QSPI_H
#define CASPER_QSPI_H

#include "casper_types.h"

/* -----------------------------------------------------------------------
 *  Command descriptor
 * --------------------------------------------------------------------- */
typedef struct {
    uint8_t  instruction;        /**< Opcode byte.                         */
    uint8_t  instruction_lines;  /**< Lines for instruction phase (1 or 4).*/
    uint8_t  address_lines;      /**< Lines for address phase (0, 1 or 4). */
    uint8_t  address_bytes;      /**< Address width in bytes (3 or 4).     */
    uint32_t address;            /**< Flash byte address.                  */
    uint8_t  data_lines;         /**< Lines for data phase (0, 1 or 4).    */
    uint8_t  dummy_cycles;       /**< Dummy clock cycles before data phase.*/
    uint32_t data_len;           /**< Bytes to transfer in data phase.     */
} casper_qspi_cmd_t;

/* -----------------------------------------------------------------------
 *  Auto-poll (status-match) descriptor
 *
 *  Used with casper_qspi_autopoll_it() to wait for a flash status
 *  register bit to reach a target value (e.g. WIP=0 after erase/write).
 * --------------------------------------------------------------------- */
typedef struct {
    uint8_t instruction;    /**< Status-read opcode (e.g. 0x05 = READ_SR1).  */
    uint8_t match;          /**< Expected bit pattern (after mask).           */
    uint8_t mask;           /**< Bit mask applied to the polled byte.         */
    uint8_t poll_interval;  /**< Polling interval (device-clock units, board  */
                            /**<  maps to HAL QSPI_AutoPolling.Interval).     */
} casper_qspi_poll_t;

/* -----------------------------------------------------------------------
 *  Event type delivered to the registered handler
 * --------------------------------------------------------------------- */
typedef enum {
    CASPER_QSPI_EVT_TX_DONE,   /**< casper_qspi_transmit_it() completed OK.  */
    CASPER_QSPI_EVT_MATCH,     /**< casper_qspi_autopoll_it() matched.        */
    CASPER_QSPI_EVT_ERROR      /**< Peripheral / transfer error occurred.     */
} casper_qspi_evt_t;

/* -----------------------------------------------------------------------
 *  Blocking API
 * --------------------------------------------------------------------- */

/**
 * @brief Send a QSPI command frame (no data phase, or data set up separately).
 *
 * Equivalent to HAL_QSPI_Command().  Configures the QSPI peripheral for the
 * given instruction / address / dummy-cycles settings.  If data_lines == 0,
 * the command completes here (instruction-only).  If data_lines != 0,
 * call casper_qspi_transmit() or casper_qspi_receive() immediately after.
 *
 * @param dev     Opaque QSPI handle.
 * @param cmd     Command descriptor (all fields must be valid).
 * @param to_ms   Timeout in milliseconds.
 * @return        CASPER_OK, CASPER_TIMEOUT, or CASPER_ERR.
 */
casper_status_t casper_qspi_command(casper_qspi_t *dev,
                                    const casper_qspi_cmd_t *cmd,
                                    uint32_t to_ms);

/**
 * @brief Transmit @p buf (blocking) following a casper_qspi_command() call.
 *
 * Equivalent to HAL_QSPI_Transmit().  Must be called immediately after
 * casper_qspi_command() set up a data phase with data_lines != 0 and
 * data_len > 0.
 *
 * @param dev     Opaque QSPI handle.
 * @param buf     Data to transmit (data_len bytes, from the preceding cmd).
 * @param to_ms   Timeout in milliseconds.
 * @return        CASPER_OK, CASPER_TIMEOUT, or CASPER_ERR.
 */
casper_status_t casper_qspi_transmit(casper_qspi_t *dev,
                                     const uint8_t *buf,
                                     uint32_t to_ms);

/**
 * @brief Receive @p buf (blocking) following a casper_qspi_command() call.
 *
 * Equivalent to HAL_QSPI_Receive().  Must be called immediately after
 * casper_qspi_command() set up a data phase with data_lines != 0 and
 * data_len > 0.
 *
 * @param dev     Opaque QSPI handle.
 * @param buf     Buffer to receive data into (data_len bytes).
 * @param to_ms   Timeout in milliseconds.
 * @return        CASPER_OK, CASPER_TIMEOUT, or CASPER_ERR.
 */
casper_status_t casper_qspi_receive(casper_qspi_t *dev,
                                    uint8_t *buf,
                                    uint32_t to_ms);

/* -----------------------------------------------------------------------
 *  Non-blocking (interrupt) API
 * --------------------------------------------------------------------- */

/**
 * @brief Start a non-blocking QSPI transmit (IT mode).
 *
 * Equivalent to HAL_QSPI_Transmit_IT().  Must be called immediately after
 * casper_qspi_command() configured a transmit data phase.  Returns as soon
 * as the transfer is started.
 *
 * On completion the board layer fires the handler registered with
 * casper_qspi_set_handler() with CASPER_QSPI_EVT_TX_DONE.
 * On error it fires with CASPER_QSPI_EVT_ERROR.
 *
 * @p buf must remain valid (not modified, not freed) until the handler fires.
 *
 * @param dev   Opaque QSPI handle.
 * @param buf   Data buffer (data_len bytes, from the preceding cmd).
 * @return      CASPER_OK if the transfer was started, CASPER_ERR if busy
 *              or peripheral error.
 */
casper_status_t casper_qspi_transmit_it(casper_qspi_t *dev,
                                        const uint8_t *buf);

/**
 * @brief Start non-blocking status-register polling (IT mode).
 *
 * Equivalent to HAL_QSPI_AutoPolling_IT().  The QSPI peripheral repeatedly
 * issues @p poll->instruction, applies @p poll->mask to the byte received,
 * and signals a match when (received & mask) == match.
 *
 * On match:  handler fires with CASPER_QSPI_EVT_MATCH.
 * On error:  handler fires with CASPER_QSPI_EVT_ERROR.
 *
 * Typical use: wait for WIP bit to clear after a page-program or erase:
 *   poll = { .instruction=0x05, .match=0x00, .mask=0x01, .poll_interval=16 }
 *
 * @param dev    Opaque QSPI handle.
 * @param poll   Auto-poll descriptor.
 * @return       CASPER_OK if polling was started, CASPER_ERR otherwise.
 */
casper_status_t casper_qspi_autopoll_it(casper_qspi_t *dev,
                                        const casper_qspi_poll_t *poll);

/* -----------------------------------------------------------------------
 *  Handler registration
 * --------------------------------------------------------------------- */

/**
 * @brief Register an event handler for IT-mode completions.
 *
 * Only one handler per casper_qspi_t.  Replaces any previously registered
 * handler.  The board layer stores @p fn and @p ctx in the casper_qspi_t
 * instance and calls fn(ctx, evt) from within its HAL callback
 * (HAL_QSPI_TxCpltCallback / HAL_QSPI_StatusMatchCallback /
 * HAL_QSPI_ErrorCallback) on the board layer side.
 *
 * The handler is called from an ISR context — it must be ISR-safe (no
 * blocking calls, no casper_delay_ms, no CDC transmit).
 *
 * @param dev   Opaque QSPI handle.
 * @param fn    Handler function pointer.  NULL disables the handler.
 * @param ctx   Caller-supplied context pointer passed verbatim to @p fn.
 */
void casper_qspi_set_handler(casper_qspi_t *dev,
                             void (*fn)(void *ctx, casper_qspi_evt_t evt),
                             void *ctx);

#endif /* CASPER_QSPI_H */
