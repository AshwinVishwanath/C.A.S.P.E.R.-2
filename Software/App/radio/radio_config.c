/**
 * @file radio_config.c
 * @brief Radio profile definitions (shared between flight and ground builds).
 */

#include "radio_config.h"

const radio_profile_t RADIO_PROFILE_A = {
    .sf = 7, .bw_hz = 250000, .cr = 5,
    .sync_word = 0x12, .preamble = 8,
    .tx_power_dbm = 20, .freq_hz = 868000000
};

const radio_profile_t RADIO_PROFILE_B = {
    .sf = 8, .bw_hz = 250000, .cr = 5,
    .sync_word = 0x12, .preamble = 8,
    .tx_power_dbm = 20, .freq_hz = 868000000
};
