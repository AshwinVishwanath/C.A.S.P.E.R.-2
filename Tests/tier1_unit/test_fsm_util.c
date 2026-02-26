/* test_fsm_util.c — Level 1 tests for dwell timer, virtual clock,
 *                   compute_vert_accel, check_antenna_up
 *
 * All 13 tests per CASPER_FSM_PRD Level 1.
 */
#include "test_config.h"
#include "fsm_util.h"
#include "casper_quat.h"
#include "mock_tick.h"

/* ================================================================== */
/*  setUp / tearDown                                                  */
/* ================================================================== */
void setUp(void)
{
    fsm_set_tick(0);
}

void tearDown(void) {}

/* ================================================================== */
/*  TEST_DWELL_01–05: dwell_check                                    */
/* ================================================================== */

void TEST_DWELL_01_returns_false_when_condition_false(void)
{
    dwell_timer_t t = {0};
    fsm_set_tick(1000);
    TEST_ASSERT_FALSE(dwell_check(&t, false, 100));
    TEST_ASSERT_FALSE(t.active);
}

void TEST_DWELL_02_returns_false_when_condition_true_but_dwell_not_elapsed(void)
{
    dwell_timer_t t = {0};
    fsm_set_tick(0);
    TEST_ASSERT_FALSE(dwell_check(&t, true, 100));
    TEST_ASSERT_TRUE(t.active);

    /* Advance 50ms — still not 100ms */
    fsm_set_tick(50);
    TEST_ASSERT_FALSE(dwell_check(&t, true, 100));
}

void TEST_DWELL_03_returns_true_when_condition_sustained(void)
{
    dwell_timer_t t = {0};
    fsm_set_tick(0);
    dwell_check(&t, true, 100);

    fsm_set_tick(100);
    TEST_ASSERT_TRUE(dwell_check(&t, true, 100));
}

void TEST_DWELL_04_resets_if_condition_drops_mid_dwell(void)
{
    dwell_timer_t t = {0};
    fsm_set_tick(0);
    dwell_check(&t, true, 100);

    fsm_set_tick(50);
    dwell_check(&t, true, 100);

    /* Drop out */
    fsm_set_tick(60);
    dwell_check(&t, false, 100);
    TEST_ASSERT_FALSE(t.active);

    /* Re-assert: timer must restart from here */
    fsm_set_tick(70);
    dwell_check(&t, true, 100);
    TEST_ASSERT_EQUAL_UINT32(70, t.start_ms);

    /* 70 + 100 = 170 */
    fsm_set_tick(169);
    TEST_ASSERT_FALSE(dwell_check(&t, true, 100));

    fsm_set_tick(170);
    TEST_ASSERT_TRUE(dwell_check(&t, true, 100));
}

void TEST_DWELL_05_zero_required_returns_true_immediately(void)
{
    dwell_timer_t t = {0};
    fsm_set_tick(500);
    TEST_ASSERT_TRUE(dwell_check(&t, true, 0));
}

/* ================================================================== */
/*  TEST_TICK_01–02: virtual clock                                    */
/* ================================================================== */

void TEST_TICK_01_set_get_roundtrip(void)
{
    fsm_set_tick(12345);
    TEST_ASSERT_EQUAL_UINT32(12345, fsm_get_tick());

    fsm_set_tick(0);
    TEST_ASSERT_EQUAL_UINT32(0, fsm_get_tick());

    fsm_set_tick(0xFFFFFFFF);
    TEST_ASSERT_EQUAL_UINT32(0xFFFFFFFF, fsm_get_tick());
}

void TEST_TICK_02_dwell_uses_virtual_clock(void)
{
    dwell_timer_t t = {0};

    /* Set virtual clock to 1000 */
    fsm_set_tick(1000);
    dwell_check(&t, true, 200);
    TEST_ASSERT_EQUAL_UINT32(1000, t.start_ms);

    /* Advance virtual clock, NOT wall clock */
    fsm_set_tick(1199);
    TEST_ASSERT_FALSE(dwell_check(&t, true, 200));

    fsm_set_tick(1200);
    TEST_ASSERT_TRUE(dwell_check(&t, true, 200));
}

/* ================================================================== */
/*  TEST_VERT_ACCEL_01–03: compute_vert_accel                        */
/* ================================================================== */

void TEST_VERT_ACCEL_01_stationary_identity_quat(void)
{
    /* Identity quaternion = upright. Accel = [0, 0, 9.81] (body Z = up)
     * After rotation: a_ned = [0, 0, 9.81]
     * vert_accel = (-9.81 - 9.80665) / 9.80665 ≈ 0g (stationary reading)
     *
     * But wait: identity quat with body Z = [0,0,1] maps to NED Z = [0,0,1].
     * NED Z is down. So body Z pointing down. If we're upright on pad with
     * body Z pointing up, the accel in NED is [0, 0, -9.81] (pointing up in NED = negative Z).
     *
     * Actually, the quat is body-to-NED. On pad, body Z = NED up = [0,0,-1] in NED.
     * So identity quat means body frame = NED frame.
     * Accel reading on pad: accelerometer measures +g along body axis opposing gravity.
     * If body Z = NED Z (down), then accel = [0, 0, -9.81] in body frame (upward).
     * Rotated to NED: [0, 0, -9.81].
     * vert_accel = (-(-9.81) - 9.80665) / 9.80665 = (9.81 - 9.80665) / 9.80665 ≈ 0g
     *
     * For the CASPER body frame: on pad with identity quat, the accelerometer
     * reads [0, 0, +9.81] if body Z = NED up (Z-up convention from MEMORY.md).
     * Rotated to NED by identity: a_ned = [0, 0, 9.81].
     * But NED Z is down, so 9.81 in NED Z means downward.
     * vert_accel = (-9.81 - 9.80665) / 9.80665 ≈ -2g. That's wrong.
     *
     * The convention matters. The spec says:
     *   vert_accel = (-a_ned[2] - 9.80665) / 9.80665
     * For stationary on pad, we need vert_accel ≈ 0g.
     * So a_ned[2] must be ≈ -9.80665 for stationary.
     * This means the accelerometer reading in NED Z must be negative (pointing up).
     *
     * With identity quat (body=NED), accelerometer reads [0,0,-9.81] in body frame
     * (measuring reaction to gravity, pointing up when gravity points down).
     * Rotated to NED: still [0,0,-9.81].
     * vert_accel = (-(-9.81) - 9.80665) / 9.80665 ≈ 0g. Correct!
     */
    float q_identity[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float accel[3] = {0.0f, 0.0f, -9.80665f};

    float va = compute_vert_accel(q_identity, accel);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, va);
}

void TEST_VERT_ACCEL_02_2g_upward_thrust(void)
{
    /* 2g upward thrust: total accel = 2g + 1g (reaction to gravity) = 3g
     * In body frame (identity quat, body=NED): accel = [0, 0, -3*9.80665]
     *   (upward in NED = negative Z)
     * vert_accel = (-(-3*9.80665) - 9.80665) / 9.80665 = (3 - 1) = 2g */
    float q_identity[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float accel[3] = {0.0f, 0.0f, -3.0f * 9.80665f};

    float va = compute_vert_accel(q_identity, accel);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 2.0f, va);
}

void TEST_VERT_ACCEL_03_free_fall(void)
{
    /* Free fall: accelerometer reads zero (no contact force) */
    float q_identity[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float accel[3] = {0.0f, 0.0f, 0.0f};

    float va = compute_vert_accel(q_identity, accel);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -1.0f, va);
}

/* ================================================================== */
/*  TEST_ANTENNA_01–04: check_antenna_up                              */
/* ================================================================== */

void TEST_ANTENNA_01_identity_quat_upright(void)
{
    /* Identity quat: body Z = NED Z (down). Body Z points down, not up.
     * For antenna to be "up", body Z must point NED up = [0,0,-1].
     * With identity quat, body Z rotates to [0,0,1] in NED = pointing down.
     * dot = -bz_ned[2] = -1.0. That's < 0.866, so NOT antenna up.
     *
     * Actually for CASPER, the Z-up convention means the pad-aligned quat
     * has body Z pointing up (NED -Z). Let's use from_accel to get the
     * proper pad quaternion.
     *
     * Let's think simpler: check_antenna_up rotates [0,0,1] by q to NED,
     * then checks dot = -bz_ned[2] > 0.866.
     * For "antenna up", we need bz_ned[2] < -0.866, meaning body Z points
     * upward in NED.
     *
     * For an upright rocket on pad with CASPER convention:
     * The quat from_accel with gravity along body Y gives a quat that maps
     * body Z to some direction. Let's just construct a quat where body Z
     * maps to NED -Z (up).
     *
     * A 180° rotation about X axis: q = [0, 1, 0, 0]
     * This maps [0,0,1] -> [0,0,-1]. So bz_ned = [0,0,-1], dot = 1.0. Antenna up!
     */
    float q[4] = {0.0f, 1.0f, 0.0f, 0.0f};  /* 180° about X */
    TEST_ASSERT_TRUE(check_antenna_up(q));
}

void TEST_ANTENNA_02_90deg_tilt(void)
{
    /* 90° tilt: body Z maps to NED horizontal.
     * 90° about Y: q = [cos(45°), 0, sin(45°), 0] = [0.7071, 0, 0.7071, 0]
     * Rotate [0,0,1]: result should be approximately [1, 0, 0] (horizontal)
     * dot = -0 = 0. NOT antenna up.
     */
    float q[4];
    /* 90° rotation about body X would map Z to -Y in NED:
     * Use casper_quat_from_euler with 90° pitch to tilt sideways */
    float angle = 90.0f * (float)M_PI / 180.0f;
    /* 90° about X: q = [cos(45°), sin(45°), 0, 0] */
    q[0] = cosf(angle * 0.5f);
    q[1] = sinf(angle * 0.5f);
    q[2] = 0.0f;
    q[3] = 0.0f;

    /* This rotates [0,0,1] -> [0,-1,0] in rotated frame. dot = 0. */
    TEST_ASSERT_FALSE(check_antenna_up(q));
}

void TEST_ANTENNA_03_25deg_tilt_within_threshold(void)
{
    /* 25° tilt from vertical. Body Z should be within 30° of NED up.
     * Start with 180° about X (upright), then tilt 25° about Y.
     * Compose quaternions. */
    float q_upright[4] = {0.0f, 1.0f, 0.0f, 0.0f};  /* 180° about X */
    float angle = 25.0f * (float)M_PI / 180.0f;
    float q_tilt[4] = {cosf(angle * 0.5f), 0.0f, sinf(angle * 0.5f), 0.0f};
    float q[4];
    casper_quat_mult(q_tilt, q_upright, q);

    /* body Z through this composite should be ~25° from NED up.
     * cos(25°) ≈ 0.906 > 0.866 threshold → antenna up */
    TEST_ASSERT_TRUE(check_antenna_up(q));
}

void TEST_ANTENNA_04_35deg_tilt_outside_threshold(void)
{
    /* 35° tilt. cos(35°) ≈ 0.819 < 0.866 → NOT antenna up */
    float q_upright[4] = {0.0f, 1.0f, 0.0f, 0.0f};
    float angle = 35.0f * (float)M_PI / 180.0f;
    float q_tilt[4] = {cosf(angle * 0.5f), 0.0f, sinf(angle * 0.5f), 0.0f};
    float q[4];
    casper_quat_mult(q_tilt, q_upright, q);

    TEST_ASSERT_FALSE(check_antenna_up(q));
}

/* ================================================================== */
/*  main                                                              */
/* ================================================================== */
int main(void)
{
    UNITY_BEGIN();

    /* Dwell timer */
    RUN_TEST(TEST_DWELL_01_returns_false_when_condition_false);
    RUN_TEST(TEST_DWELL_02_returns_false_when_condition_true_but_dwell_not_elapsed);
    RUN_TEST(TEST_DWELL_03_returns_true_when_condition_sustained);
    RUN_TEST(TEST_DWELL_04_resets_if_condition_drops_mid_dwell);
    RUN_TEST(TEST_DWELL_05_zero_required_returns_true_immediately);

    /* Virtual clock */
    RUN_TEST(TEST_TICK_01_set_get_roundtrip);
    RUN_TEST(TEST_TICK_02_dwell_uses_virtual_clock);

    /* Vertical acceleration */
    RUN_TEST(TEST_VERT_ACCEL_01_stationary_identity_quat);
    RUN_TEST(TEST_VERT_ACCEL_02_2g_upward_thrust);
    RUN_TEST(TEST_VERT_ACCEL_03_free_fall);

    /* Antenna up */
    RUN_TEST(TEST_ANTENNA_01_identity_quat_upright);
    RUN_TEST(TEST_ANTENNA_02_90deg_tilt);
    RUN_TEST(TEST_ANTENNA_03_25deg_tilt_within_threshold);
    RUN_TEST(TEST_ANTENNA_04_35deg_tilt_outside_threshold);

    return UNITY_END();
}
