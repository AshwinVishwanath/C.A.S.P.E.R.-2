/**
 * @file  casper_time.h
 * @brief Portable time / delay interface.
 *
 * Pure C11.  Implemented in the board layer (board_<target>.c).
 * On Casper 2 these wrap HAL_GetTick(), HAL_Delay(), and DWT->CYCCNT.
 */

#ifndef CASPER_TIME_H
#define CASPER_TIME_H

#include <stdint.h>

/**
 * @brief Return milliseconds elapsed since boot (wraps at 2^32 ms ≈ 49.7 days).
 *
 * Equivalent to HAL_GetTick() on STM32.  Must be callable from any context
 * (superloop, ISR, etc.) without side-effects.
 *
 * @return Millisecond tick counter.
 */
uint32_t casper_millis(void);

/**
 * @brief Blocking delay.
 *
 * Suspends execution for at least @p ms milliseconds.
 * MUST NOT be called from an ISR.
 *
 * @param ms  Duration in milliseconds (0 is a no-op).
 */
void casper_delay_ms(uint32_t ms);

/**
 * @brief Return microseconds elapsed since boot.
 *
 * On Casper 2, implemented via DWT->CYCCNT / (SystemCoreClock / 1 000 000).
 * The board layer owns the SYSCLK constant; callers must NOT hard-code
 * the clock frequency (that is why this function exists).
 *
 * Wraps at 2^32 µs ≈ 71.6 minutes.  Suitable for short-interval timing only.
 * For longer intervals use casper_millis().
 *
 * @return Microsecond tick counter.
 */
uint32_t casper_micros(void);

#endif /* CASPER_TIME_H */
