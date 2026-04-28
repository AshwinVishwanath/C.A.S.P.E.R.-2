/* ============================================================
 *  TIER:     CORE-FLIGHT
 *  MODULE:   Flight Loop
 *  SUMMARY:  Top-level superloop: schedules sensor reads, EKF, FSM, telemetry.
 * ============================================================ */
#ifndef APP_FLIGHT_FLIGHT_LOOP_H
#define APP_FLIGHT_FLIGHT_LOOP_H

void flight_loop_init(void);
void flight_loop_tick(void);

#endif
