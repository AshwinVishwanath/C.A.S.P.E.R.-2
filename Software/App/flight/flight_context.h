#ifndef FLIGHT_CONTEXT_H
#define FLIGHT_CONTEXT_H

#include <stdint.h>
#include <stdbool.h>
#include "tlm_types.h"  /* for fsm_state_t and FSM_STATE_* defines */

/* Do NOT redefine FSM states -- use FSM_STATE_* from tlm_types.h */

typedef struct {
    /* FSM */
    fsm_state_t state;
    uint32_t    state_entry_ms;
    uint32_t    mission_start_ms;
    bool        mission_started;

    /* Motor tracking */
    uint8_t     motor_count;           /* burnouts detected so far */
    uint8_t     motors_expected;       /* 1 or 2, from config */
    uint32_t    last_burnout_ms;

    /* Detection flags */
    bool        launch_detected;
    bool        apogee_detected;
    uint32_t    launch_timestamp_ms;
    uint32_t    apogee_timestamp_ms;
    uint32_t    landing_timestamp_ms;

    /* Peak tracking */
    float       max_altitude_m;
    float       max_velocity_mps;
    float       max_accel_mps2;
    float       peak_tilt_deg;         /* max tilt from vertical since launch */

    /* Baro reference */
    float       baro_ref_m;

    /* Apogee voting state */
    float       max_ekf_alt_m;         /* independent EKF altitude peak tracker */
    float       max_baro_alt_m;        /* independent baro altitude peak tracker */

    /* Debounce timers (internal) */
    uint32_t    launch_debounce_start_ms;
    bool        launch_debouncing;
    uint32_t    burnout_debounce_start_ms;
    bool        burnout_debouncing;
    uint32_t    apogee_debounce_start_ms;
    bool        apogee_debouncing;
    uint32_t    landing_debounce_start_ms;
    bool        landing_debouncing;
    float       landing_ref_alt_m;
} flight_context_t;

/* Snapshot of sensor data passed to FSM each tick */
typedef struct {
    float    accel_body_ms2[3];   /* body frame accelerometer, m/s^2 */
    float    ekf_alt_m;           /* EKF altitude AGL */
    float    ekf_vel_mps;         /* EKF vertical velocity, +up */
    float    baro_alt_agl_m;      /* raw baro altitude AGL (independent of EKF) */
    float    tilt_deg;            /* current tilt from vertical, degrees */
    uint32_t timestamp_ms;        /* current time (HAL_GetTick or test injection) */
    bool     baro_updated;        /* true if baro has new data this tick */
} sensor_input_t;

#endif /* FLIGHT_CONTEXT_H */
