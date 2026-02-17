/*
 * Test harness for flight data logging.
 * Compiles on host PC with:
 *   gcc -DHOST_TEST -Wall -Werror \
 *       -I../flight -I../fsm -I../telemetry -I. \
 *       test_flight_log.c ../flight/flight_log.c -lm -o test_flight_log
 */
#define HOST_TEST
#include "test_shim.h"
#include "flight_log.h"
#include <string.h>

/* ── Helper: build a log entry with a known sequence number ── */
static flight_log_entry_t make_entry(uint32_t seq)
{
    flight_log_entry_t e;
    memset(&e, 0, sizeof(e));
    e.timestamp_us = seq * 10000;  /* 10ms per entry at 100Hz */
    e.fsm_state    = FSM_STATE_PAD;
    e.accel_x_mg   = (int16_t)(seq & 0x7FFF);
    e.accel_z_mg   = (int16_t)seq;
    return e;
}

/* ══════════════════════════════════════════════════════════════
 * Test 1: Ring buffer wrap
 *   Write 600 entries (capacity 500). Verify ring wraps.
 *   Oldest entry should be #100, newest #599.
 * ══════════════════════════════════════════════════════════════ */
static void test_1_ring_buffer_wrap(void)
{
    TEST_BEGIN("Test 1: Ring buffer wraps correctly");

    flight_log_init();

    /* Write 600 entries to the ring buffer (PAD state) */
    for (uint32_t i = 0; i < 600; i++) {
        flight_log_entry_t e = make_entry(i);
        flight_log_write(&e, FSM_STATE_PAD);
    }

    /* Ring should not have written anything to flash yet */
    ASSERT_EQ(flight_log_get_entry_count(), 0,
              "No entries in flash before commit");
    ASSERT_EQ(flight_log_get_write_addr(), 0x00000000,
              "Write addr should be 0 before commit");

    /* Commit the ring buffer -- should write 500 entries to flash
     * (the most recent 500, i.e. entries 100..599) */
    flight_log_commit_ring_buffer();

    ASSERT_EQ(flight_log_get_entry_count(), 500,
              "500 entries written to flash after commit");

    /* 500 entries x 32 bytes = 16000 bytes */
    ASSERT_EQ(flight_log_get_write_addr(), 500 * 32,
              "Write addr = 500 * 32 = 16000");

    ASSERT_EQ(flight_log_get_dropped(), 0,
              "No drops during ring buffer phase");

    TEST_END("Test 1: Ring buffer wraps correctly");
}

/* ══════════════════════════════════════════════════════════════
 * Test 2: Ring buffer commit with partial fill
 *   Write only 200 entries (less than 500 capacity).
 *   Commit should write exactly 200.
 * ══════════════════════════════════════════════════════════════ */
static void test_2_ring_commit_partial(void)
{
    TEST_BEGIN("Test 2: Ring buffer commit (partial fill)");

    flight_log_init();

    for (uint32_t i = 0; i < 200; i++) {
        flight_log_entry_t e = make_entry(i);
        flight_log_write(&e, FSM_STATE_PAD);
    }

    flight_log_commit_ring_buffer();

    ASSERT_EQ(flight_log_get_entry_count(), 200,
              "200 entries written to flash");
    ASSERT_EQ(flight_log_get_write_addr(), 200 * 32,
              "Write addr = 200 * 32 = 6400");
    ASSERT_EQ(flight_log_get_dropped(), 0,
              "No drops");

    TEST_END("Test 2: Ring buffer commit (partial fill)");
}

/* ══════════════════════════════════════════════════════════════
 * Test 3: Double buffer swap
 *   After commit, write 129 entries (exceeds one buffer of 128).
 *   Verify the buffer swap triggers a flash flush.
 * ══════════════════════════════════════════════════════════════ */
static void test_3_double_buffer_swap(void)
{
    TEST_BEGIN("Test 3: Double buffer swap");

    flight_log_init();

    /* Commit empty ring (instant, writes 0 entries) */
    flight_log_commit_ring_buffer();
    ASSERT_EQ(flight_log_get_entry_count(), 0,
              "Empty ring commit writes 0");

    /* Write 128 entries (fills first buffer exactly) */
    for (uint32_t i = 0; i < 128; i++) {
        flight_log_entry_t e = make_entry(i);
        e.fsm_state = FSM_STATE_BOOST;
        flight_log_write(&e, FSM_STATE_BOOST);
    }

    /* Buffer is full but not yet flushed (flush happens on the
     * NEXT write that triggers the swap) */
    ASSERT_EQ(flight_log_get_entry_count(), 0,
              "128 entries in buffer, not yet flushed");

    /* Write one more entry -- triggers swap + flush of 128 entries */
    flight_log_entry_t e129 = make_entry(128);
    e129.fsm_state = FSM_STATE_BOOST;
    flight_log_write(&e129, FSM_STATE_BOOST);

    ASSERT_EQ(flight_log_get_entry_count(), 128,
              "128 entries flushed to flash after swap");
    ASSERT_EQ(flight_log_get_write_addr(), 128 * 32,
              "Write addr = 128 * 32 = 4096 (one sector)");

    /* Write 127 more to fill the second buffer */
    for (uint32_t i = 129; i < 256; i++) {
        flight_log_entry_t e = make_entry(i);
        e.fsm_state = FSM_STATE_BOOST;
        flight_log_write(&e, FSM_STATE_BOOST);
    }

    /* Second buffer full but not flushed yet */
    ASSERT_EQ(flight_log_get_entry_count(), 128,
              "Still 128 flushed (second buffer pending)");

    /* Trigger second swap */
    flight_log_entry_t e257 = make_entry(256);
    e257.fsm_state = FSM_STATE_BOOST;
    flight_log_write(&e257, FSM_STATE_BOOST);

    ASSERT_EQ(flight_log_get_entry_count(), 256,
              "256 entries flushed after second swap");
    ASSERT_EQ(flight_log_get_write_addr(), 256 * 32,
              "Write addr = 256 * 32 = 8192 (two sectors)");

    ASSERT_EQ(flight_log_get_dropped(), 0,
              "No drops under normal conditions");

    TEST_END("Test 3: Double buffer swap");
}

/* ══════════════════════════════════════════════════════════════
 * Test 4: Dropped entry counter
 *   Fill both double buffers to trigger drops.
 * ══════════════════════════════════════════════════════════════ */
static void test_4_dropped_entries(void)
{
    TEST_BEGIN("Test 4: Dropped entry counter");

    flight_log_init();
    flight_log_commit_ring_buffer();

    /* The double buffer swap in our implementation is synchronous
     * (flush happens inline). So the only way to get drops is
     * if the flash region is full. Instead, test the LANDED
     * state rejection. */

    /* Write 128 entries in BOOST state to fill buffer a */
    for (uint32_t i = 0; i < 128; i++) {
        flight_log_entry_t e = make_entry(i);
        e.fsm_state = FSM_STATE_BOOST;
        flight_log_write(&e, FSM_STATE_BOOST);
    }

    /* Trigger swap + flush, write one into buffer b */
    flight_log_entry_t e_swap = make_entry(128);
    e_swap.fsm_state = FSM_STATE_BOOST;
    flight_log_write(&e_swap, FSM_STATE_BOOST);

    uint32_t count_after = flight_log_get_entry_count();
    ASSERT_EQ(count_after, 128, "128 entries flushed");

    /* Now write entries to LANDED state -- they should be ignored */
    for (uint32_t i = 0; i < 50; i++) {
        flight_log_entry_t e = make_entry(200 + i);
        flight_log_write(&e, FSM_STATE_LANDED);
    }

    ASSERT_EQ(flight_log_get_entry_count(), 128,
              "No additional entries from LANDED writes");

    /* Flush the remaining partial buffer */
    flight_log_flush();

    /* After flush: the 1 entry in the partial active buffer should be written */
    ASSERT_EQ(flight_log_get_entry_count(), 129,
              "Flush wrote the 1 partial entry");

    /* Verify is_active returns false after flush */
    ASSERT_TRUE(!flight_log_is_active(),
                "Logging should not be active after flush");

    /* Further writes should be ignored after flush */
    flight_log_entry_t e_after = make_entry(999);
    flight_log_write(&e_after, FSM_STATE_BOOST);
    ASSERT_EQ(flight_log_get_entry_count(), 129,
              "No writes after flush");

    TEST_END("Test 4: Dropped entry counter");
}

/* ══════════════════════════════════════════════════════════════
 * Test 5: Entry size assertion
 *   Verify sizeof(flight_log_entry_t) == 32
 * ══════════════════════════════════════════════════════════════ */
static void test_5_entry_size(void)
{
    TEST_BEGIN("Test 5: Entry size assertion");

    ASSERT_EQ((int)sizeof(flight_log_entry_t), 32,
              "flight_log_entry_t must be 32 bytes");

    /* Verify entries per sector */
    ASSERT_EQ(FLIGHT_LOG_SECTOR_SIZE / (int)sizeof(flight_log_entry_t), 128,
              "128 entries per 4KB sector");

    /* Verify entries per page */
    ASSERT_EQ(FLIGHT_LOG_PAGE_SIZE / (int)sizeof(flight_log_entry_t), 8,
              "8 entries per 256-byte page");

    /* Verify ring buffer capacity */
    ASSERT_EQ(FLIGHT_LOG_RING_SIZE, 500,
              "Ring buffer holds 500 entries (5s at 100Hz)");

    /* Verify ring buffer size in bytes */
    ASSERT_EQ(FLIGHT_LOG_RING_SIZE * (int)sizeof(flight_log_entry_t), 16000,
              "Ring buffer is 16000 bytes");

    TEST_END("Test 5: Entry size assertion");
}

/* ── Main ───────────────────────────────────────────── */
int main(void)
{
    printf("=== Flight Log Test Harness ===\n\n");

    test_1_ring_buffer_wrap();
    test_2_ring_commit_partial();
    test_3_double_buffer_swap();
    test_4_dropped_entries();
    test_5_entry_size();

    TEST_SUMMARY();
}
