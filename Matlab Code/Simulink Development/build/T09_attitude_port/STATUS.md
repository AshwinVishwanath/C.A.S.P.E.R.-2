# T09 Attitude Port — STATUS

## Files created
- casper_quat_ops.m
- casper_attitude_state_new.m
- casper_attitude_static_init.m
- casper_attitude_gyro_lpf.m
- casper_attitude_predict_rk4.m
- casper_attitude_mahony.m
- casper_attitude_mag_correct_flight.m
- casper_attitude_tick.m
- build_attitude_block.m
- test_attitude_port.m
- attitude_block.slx
- plots/attitude_pad_initialization.png
- plots/attitude_flight_tracking.png
- plots/attitude_error_euler.png

## Acceptance criteria
| # | Criterion | Status | Notes |
|---|---|---|---|
| 1 | Static init (final err <1 deg, init <=10s) | PASS | final err 0.035 deg (lim 1.0), init at 4.79 s (lim 10.0) |
| 2 | Gyro-only drift over 60 s, <0.5 deg per axis | PASS | |q angle| = 0.0000 deg; max axis = 0.0000 deg (lim 0.5) |
| 3 | Quaternion unit norm |q|-1| < 1e-5 | PASS | max ||q||-1 = 2.220e-16 (lim 1e-5) |
| 4 | Mahony converges from 5 deg perturb within 5 s | PASS | converged to <0.5 deg at 0.23 s (lim 5.0) |
| 5 | Flight mag correction fires at 10 Hz | PASS | fires=18 (exp 20), gap_mean=0.1056 s (target 0.100) |
| 6 | Heading sigma >= floor always | PASS | min heading_sigma = 0.010000 (floor 0.010000) |
| 7 | Tilt RMS <1 deg (powered), <2 deg (coast) | PASS | RMS powered 0.132 deg (lim 1.0), RMS coast 0.135 deg (lim 2.0) |
| 8 | Determinism (same seed → byte-identical quats) | PASS | max |Δq| between runs = 0.000e+00 (must be 0) |
| 9 | No NaN/Inf | PASS | all finite = 1 |
| 10 | Performance (<30s wall for 549s sim) | PASS | measured 1.33s for 30s sim → projected 24.39s for 549s (lim 30s) |

**Summary**: 10 PASS, 0 FAIL

## Stripped items (per ARCHITECTURE.md §7)
- gyro temperature compensation: not implemented (Phase 0 stripped)
- online gyro EMA bias gate: not implemented (Phase 0 stripped, bias is static-init only)
- ignition-gated mag corrections: not implemented (no FSM in Phase 0)
- flight FSM-driven mode switching: replaced with simple boolean 'mode_pad' input

## Deviations from spec
- Flight-mode mag correction uses firmware's full 3D cross product (R' * m_ref vs m_meas) instead of the alternative tilt-projection formulation in spec §9.1. Reason: CLAUDE.md 'firmware is canonical'.
- Kp_MagPad and Kp_MagFlight come from T02 ``Attitude.Kp_*`` which mirror main.c live values (BOTH 0). Code path is exercised but the correction is a no-op for omega until firmware changes the gains.
- RK4 sub-steps do NOT renormalize between k1..k4 (matches firmware); only final aggregation is normalized.
