#include "casper_pyro.h"
#include <string.h>
#ifdef HIL_MODE
#include "hil_aux_handler.h"
#endif

/* ── Public API ── */

void casper_pyro_init(casper_pyro_t *p,
                      casper_adc_t  *adc[PYRO_NUM_CHANNELS],
                      casper_pin_t   fire[PYRO_NUM_CHANNELS],
                      casper_pin_t   led[PYRO_NUM_CHANNELS])
{
    memset(p, 0, sizeof(*p));

    /* Store board-provided tables */
    for (int i = 0; i < PYRO_NUM_CHANNELS; i++) {
        p->cont[i] = adc[i];
        p->fire[i] = fire[i];
        p->led[i]  = led[i];
    }

    /* Force all pyro outputs LOW */
    for (int i = 0; i < PYRO_NUM_CHANNELS; i++)
        casper_gpio_write(p->fire[i], CASPER_PIN_LOW);
}

bool casper_pyro_fire(casper_pyro_t *p, uint8_t ch, uint32_t duration_ms)
{
    if (ch >= PYRO_NUM_CHANNELS)
        return false;

    p->firing[ch]          = true;
    p->fire_start_ms[ch]   = casper_millis();
    p->fire_duration_ms[ch] = duration_ms;

    casper_gpio_write(p->fire[ch], CASPER_PIN_HIGH);
    return true;
}

void casper_pyro_stop(casper_pyro_t *p, uint8_t ch)
{
    if (ch >= PYRO_NUM_CHANNELS)
        return;

    casper_gpio_write(p->fire[ch], CASPER_PIN_LOW);
    p->firing[ch] = false;
}

void casper_pyro_stop_all(casper_pyro_t *p)
{
    for (uint8_t i = 0; i < PYRO_NUM_CHANNELS; i++)
        casper_pyro_stop(p, i);
}

void casper_pyro_tick(casper_pyro_t *p)
{
#ifdef HIL_MODE
    /* HIL: continuity comes from the host's aux bitmap, not the
     * unwired ADC. adc_raw stays zero (no real reading). LEDs and
     * the auto-stop timer still run so the simulated firing windows
     * close on time. */
    for (uint8_t i = 0; i < PYRO_NUM_CHANNELS; i++) {
        p->adc_raw[i]    = 0;
        p->continuity[i] = (g_hil_aux.cont_bitmap & (1u << i)) != 0u;
    }
#else
    /* ── 1. Read all continuity ADCs ── */
    for (uint8_t i = 0; i < PYRO_NUM_CHANNELS; i++) {
        uint16_t raw = 0;
        casper_adc_read(p->cont[i], &raw);
        p->adc_raw[i]    = raw;
        p->continuity[i] = (raw > PYRO_CONTINUITY_THRESHOLD);
    }
#endif

    /* ── 2. Update continuity LEDs ── */
    for (uint8_t i = 0; i < PYRO_NUM_CHANNELS; i++) {
        casper_gpio_write(p->led[i],
                          p->continuity[i] ? CASPER_PIN_HIGH : CASPER_PIN_LOW);
    }

    /* ── 3. Auto-stop expired fires ── */
    uint32_t now = casper_millis();
    for (uint8_t i = 0; i < PYRO_NUM_CHANNELS; i++) {
        if (p->firing[i] && (now - p->fire_start_ms[i] >= p->fire_duration_ms[i]))
            casper_pyro_stop(p, i);
    }
}
