# C.A.S.P.E.R.-2 Flight Readiness: Passive Ride-Along Test

## Test Profile

- **Passive ride-along** — no pyro firing, motor eject recovery
- **Validate:** INS/EKF navigation accuracy, LoRa two-way radio datalink, QSPI flight logging
- **No GPS** (antenna not populated)
- **Branch:** `logging-development`

## Risk Context

With no pyro and motor eject, the consequences of firmware bugs are **data loss only** — not vehicle loss. A superloop hang loses logging/telemetry data but the rocket comes down safely. This dramatically lowers the bar vs. a pyro-armed flight.

**Pyro system is self-protecting:** No ematches → `pyro.continuity[] == false` → `auto_arm_flight()` is a no-op → `auto_fire()` returns -1 at apogee/main. The FSM will transition through all states normally; the pyro calls silently fail. No code changes needed.

---

## SHOWSTOPPER (Will Lose Test Objective)

### S1. CAC Command Responses Never Sent Over Radio
**Files:** `Software/App/command/cac_handler.c`, `Software/App/telemetry/tlm_manager.c:195-202`, `Software/App/radio/radio_manager.c:603-616`

**The bug:** All `tlm_send_response()` calls → `cobs_encode_and_send()` → `CDC_Transmit_FS()`. Responses go to USB CDC only. The ground station sends a command over LoRa, FC processes it correctly, but the ACK/NACK goes out the USB port instead of back over radio.

`radio_send_response()` exists with a working priority queue (highest priority in `select_and_build_packet()`). It's just never called.

**Impact for this test:** Ground station cannot complete any CAC handshake over radio. TESTMODE, POLL, SIM commands all get processed but confirmations vanish. Two-way radio validation fails.

**Fix:** In `tlm_send_response()`, also call `radio_send_response()` when radio is active:
```c
int tlm_send_response(const uint8_t *data, int len) {
    // existing CDC path...
    int ok = cobs_encode_and_send(s_raw_buf, len);
    // ADD: also send over radio
    if (radio_is_active())
        radio_send_response(data, (uint8_t)len);
    return ok;
}
```

### S2. FSM Events Not Transmitted Over Radio
**Files:** `Software/App/telemetry/tlm_manager.c:156-188`, `Software/App/radio/radio_manager.c:591-601`

**The bug:** `tlm_queue_event()` sends events (BURNOUT, APOGEE, STATE transitions) over CDC only. `radio_queue_event()` exists and works but is only called for radio profile switches (`radio_manager.c:125`).

**Impact for this test:** Ground station sees FC_MSG_FAST packets (alt/vel/quat) at 10Hz but **never sees** BOOST, BURNOUT, COAST, APOGEE state transitions. The ground display can't show flight phase. This is a primary test objective.

**Fix:** Add one line to `tlm_queue_event()`:
```c
int tlm_queue_event(uint8_t type, uint16_t data) {
    // existing CDC path...
    radio_queue_event(type, data);  // ADD
    return cobs_encode_and_send(...);
}
```

---

## CRITICAL (Data Quality / Reliability)

### C1. Radio Init Failure is Non-Fatal
**File:** `Software/Core/Src/main.c:440-459`

If `radio_manager_init()` fails, firmware blinks LEDs 10x then enters the flight loop with no telemetry. On a ride-along you'd have no idea until post-flight flash analysis.

**Fix:** Make radio init failure a hard stop (infinite LED blink + buzzer). If radio is the primary test objective, don't fly without it.

### C2. IMU EXTI Disabled — Polling Fallback
**Files:** `Software/Core/Src/main.c:364`, `Software/App/flight/flight_loop.c:332-336`

LSM6DSO32 INT2 on PC15 is disabled. The superloop polls `imu.data_ready` flag directly, with a 5ms register-read fallback. This works at normal superloop speeds but under heavy load (flash erase + radio TX + logging all in one iteration), jitter could cause missed IMU samples.

**Impact:** EKF predict assumes uniform 416Hz sampling. Jitter degrades navigation accuracy — which is exactly what this test is measuring.

**Recommendation:** Add timing instrumentation (log max/min loop period) to quantify jitter. If >5% jitter observed on bench, fix the EXTI hang before flight.

### C3. No Watchdog
**File:** `Software/Core/Src/main.c`

No IWDG configured. A superloop hang (flash erase stall, SPI lockup) means total data loss for the rest of the flight.

**Impact (passive ride-along):** Rocket comes down safely on motor eject. You lose nav data and radio telemetry from the hang point onward. Not catastrophic, but wastes a flight opportunity.

**Fix:** Enable IWDG (~200ms timeout), kick in `flight_loop_tick()`. A reset mid-flight loses some data but at least logging restarts.

### C4. Self-Test Not Called at Boot
**File:** `Software/Core/Src/main.c`

No automatic sensor health check at startup. If IMU or baro SPI bus is dead, the EKF runs on stale/zero data and the entire flight log is garbage.

**Fix:** Call `self_test_run_all()` after sensor init. Halt with LED error code + buzzer if IMU or baro WHO_AM_I fails. Don't fly a data-collection mission with dead sensors.

### C5. `radio_send_gps()` Never Called
**File:** `Software/App/flight/flight_loop.c:506-514`

GPS data is logged to flash but never sent over radio. Out of scope for this no-GPS test, but if the GPS antenna happens to get a fix, ground won't see it.

**Fix (1 line, optional):** `radio_send_gps(&gps);` after `flight_logger_summary_gps()`.

---

## IMPORTANT (Should Fix, Won't Lose Data)

### I1. Flash Dump Blocks Superloop
**File:** `Software/App/flight/flight_loop.c:257-311`

`flash_dump_over_cdc()` blocks the entire loop for minutes. Triggered by a CDC command — unlikely in flight, but if someone sends it accidentally over radio during flight, nav stack freezes.

**Fix (1 line):** Guard with FSM state check:
```c
if (cmd_router_dump_requested() && flight_fsm_get_state() >= FSM_STATE_LANDED) {
```

### I2. Battery Voltage Hardcoded
**File:** `Software/App/flight/flight_loop.c:543`

`tstate.batt_v = 7.4f` — ground always sees 7.4V. Can't detect low battery on the pad.

**Impact:** Minor for a single short flight. Good to fix for situational awareness.

### I3. Baro Calibration No Minimum Sample Check
**File:** `Software/App/flight/flight_loop.c:393-405`

30-second cal period with no check that enough samples were collected. If MS5611 is intermittent, baro_ref could be based on very few samples.

**Impact:** Bad altitude reference → EKF altitude offset for entire flight. Data is still logged but analysis requires manual correction.

### I4. Mag Saturation Not Checked
**File:** `Software/App/nav/casper_attitude.c:275-282`

No magnitude range check before mag normalization. Motor burn ferrous interference could briefly corrupt attitude heading. Mitigated by 10Hz correction rate and low Kp in flight mode.

### I5. Empty FSM Case Handlers
**File:** `Software/App/fsm/flight_fsm.c`

DROGUE, RECOVERY, COAST_1, SUSTAIN, COAST_2, TUMBLE fall to `default: break;`. Functionally fine but add explicit `case` + `break` for compiler cleanliness and logger state tracking.

---

## What's Already Flight-Ready (No Changes Needed)

| Subsystem | Status | Notes |
|-----------|--------|-------|
| EKF core | SOLID | Joseph form, Mach gate, ZUPT, bias estimation, un-gate all correct |
| Attitude estimator | SOLID | Mahony→RK4, mag gating, quaternion ops all correct |
| SX1276 radio driver | SOLID | TX/RX/profile switch/DIO polling/error recovery working |
| Radio TX scheduler | SOLID | 10Hz cadence, priority queue, packet builders |
| Radio RX + dispatch | SOLID | CRC validation, CAC handler dispatch correct |
| CAC protocol | SOLID | 3-way handshake, nonce, timeout, interlocks |
| Pyro system | N/A | Self-protecting (no continuity → no arm → no fire) |
| Flight logger | SOLID | 3-stream QSPI, ring buffers, erase-ahead, summary |
| Sensor drivers | SOLID | MS5611, LSM6DSO32, ADXL372, MMC5983MA, MAX-M10M |
| FSM transitions | WORKS | PAD→BOOST→COAST→APOGEE→MAIN→LANDED path functional |
| Build | SOLID | 93KB / 128KB, clean compile |

---

## Fix Priority for Passive Ride-Along

| # | Issue | Effort | Required? |
|---|-------|--------|-----------|
| **S1** | Route CAC responses to radio | 30 min | **YES — test objective** |
| **S2** | Route events to radio | 15 min | **YES — test objective** |
| C1 | Radio init failure → hard stop | 15 min | **YES — don't fly blind** |
| C4 | Self-test at boot, halt on fail | 1 hr | **YES — don't fly with dead sensors** |
| C3 | Enable IWDG watchdog | 45 min | Strongly recommended |
| C2 | IMU jitter instrumentation | 30 min | Recommended (quantify EKF quality) |
| I1 | Guard flash dump with FSM check | 5 min | Quick win |
| I5 | Empty FSM case handlers | 10 min | Quick win |
| I3 | Baro cal minimum sample check | 15 min | Nice-to-have |

**Minimum to fly: S1 + S2 + C1 + C4 = ~2 hours of work.**
**Recommended full set: ~4-5 hours.**

---

## Verification Plan

1. **Bench (TEST_MODE=2):** Type state commands over CDC, verify FSM transitions + LED/buzzer responses
2. **Radio loopback:** Ground station sends POLL command over LoRa → verify ACK_CFG comes back **over LoRa** (not just CDC). This validates S1.
3. **Event check:** Run `flight_fsm_sim_start()` → verify ground station receives FC_EVT_STATE events for each transition over LoRa. This validates S2.
4. **Logger check:** After sim flight, use `flash_dump_over_cdc()` or USB_MODE=2 to read back flash. Run `tools/casper_decode.py` to verify HR/LR/ADXL streams contain valid data.
5. **Self-test:** Power cycle with IMU CS disconnected → verify board halts with error LED (validates C4)
6. **Shake test:** Mount board, run TEST_MODE=1, physically shake → verify EKF tracks attitude, baro cal completes, logger records data, radio transmits

---

## Implementation Order

1. **S1:** Add `radio_send_response()` call in `tlm_send_response()` — `tlm_manager.c`
2. **S2:** Add `radio_queue_event()` call in `tlm_queue_event()` — `tlm_manager.c`
3. **C1:** Change radio init fail to hard stop — `main.c`
4. **C4:** Call self-test at boot, halt on IMU/baro fail — `main.c`
5. **C3:** Enable IWDG, kick in `flight_loop_tick()` — `main.c`, `flight_loop.c`
6. **Quick wins:** FSM case handlers, flash dump guard — `flight_fsm.c`, `flight_loop.c`
