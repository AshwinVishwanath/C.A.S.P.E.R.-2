#include "pyro_manager.h"
#include "casper_pyro.h"
#include "flight_fsm.h"
#include "tlm_types.h"
#include "stm32h7xx_hal.h"

/* ── Private state ──────────────────────────────────────────────── */
static casper_pyro_t pyro;
static bool s_armed[PYRO_MGR_NUM_CHANNELS];
static bool s_test_mode;

void pyro_mgr_init(ADC_HandleTypeDef *hadc1,
                    ADC_HandleTypeDef *hadc2,
                    ADC_HandleTypeDef *hadc3)
{
    casper_pyro_init(&pyro, hadc1, hadc2, hadc3);
    for (int i = 0; i < PYRO_MGR_NUM_CHANNELS; i++) {
        s_armed[i] = false;
    }
    s_test_mode = false;
}

void pyro_mgr_tick(void)
{
    /* Delegate to low-level driver for ADC reads, LED updates, auto-stop */
    casper_pyro_tick(&pyro);
}

int pyro_mgr_set_arm(uint8_t channel, bool armed)
{
    if (channel < 1 || channel > PYRO_MGR_NUM_CHANNELS) {
        return -1;
    }
    uint8_t idx = channel - 1;

    if (armed) {
        /* Precondition: must have continuity to arm */
        if (!pyro.continuity[idx]) {
            return -1;
        }
    } else {
        /* Disarming: stop any active fire on this channel */
        if (pyro.firing[idx]) {
            casper_pyro_stop(&pyro, idx);
        }
    }

    s_armed[idx] = armed;
    return 0;
}

bool pyro_mgr_has_continuity(uint8_t channel)
{
    if (channel < 1 || channel > PYRO_MGR_NUM_CHANNELS) {
        return false;
    }
    return pyro.continuity[channel - 1];
}

int pyro_mgr_fire(uint8_t channel, uint16_t duration_ms)
{
    if (channel < 1 || channel > PYRO_MGR_NUM_CHANNELS) {
        return -1;
    }
    uint8_t idx = channel - 1;

    /* Must be armed */
    if (!s_armed[idx]) {
        return -1;
    }

    /* Must have continuity */
    if (!pyro.continuity[idx]) {
        return -1;
    }

    /* Must be in test mode or past PAD */
    if (!s_test_mode && flight_fsm_get_state() == FSM_STATE_PAD) {
        return -1;
    }

    /* Cap duration */
    uint32_t dur = duration_ms;
    if (dur > PYRO_MAX_FIRE_MS) {
        dur = PYRO_MAX_FIRE_MS;
    }

    /* In test mode, cap to 50ms for safety */
    if (s_test_mode && dur > 50) {
        dur = 50;
    }

    /* Fire via low-level driver (0-indexed) */
    if (!casper_pyro_fire(&pyro, idx, dur)) {
        return -1;
    }

    return 0;
}

uint8_t pyro_mgr_get_arm_bitmap(void)
{
    uint8_t bm = 0;
    for (int i = 0; i < PYRO_MGR_NUM_CHANNELS; i++) {
        if (s_armed[i]) {
            bm |= (uint8_t)(1u << i);
        }
    }
    return bm;
}

uint8_t pyro_mgr_get_cont_bitmap(void)
{
    uint8_t bm = 0;
    for (int i = 0; i < PYRO_MGR_NUM_CHANNELS; i++) {
        if (pyro.continuity[i]) {
            bm |= (uint8_t)(1u << i);
        }
    }
    return bm;
}

bool pyro_mgr_is_firing(void)
{
    for (int i = 0; i < PYRO_MGR_NUM_CHANNELS; i++) {
        if (pyro.firing[i]) {
            return true;
        }
    }
    return false;
}

void pyro_mgr_disarm_all(void)
{
    casper_pyro_stop_all(&pyro);
    for (int i = 0; i < PYRO_MGR_NUM_CHANNELS; i++) {
        s_armed[i] = false;
    }
}

void pyro_mgr_set_test_mode(bool enable)
{
    s_test_mode = enable;
    if (!enable) {
        pyro_mgr_disarm_all();
    }
}

bool pyro_mgr_is_test_mode(void)
{
    return s_test_mode;
}
