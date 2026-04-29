#ifndef APP_FLIGHT_FLIGHT_LOOP_H
#define APP_FLIGHT_FLIGHT_LOOP_H

#include <stdbool.h>

void flight_loop_init(void);
void flight_loop_tick(void);

/* True once the 30-second pad-mode calibration has completed. */
bool flight_loop_is_cal_done(void);

#endif
