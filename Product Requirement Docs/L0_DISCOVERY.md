# L0 Discovery — Mahony Hardening Workstream

**Spec:** `MAHONY_HARDENING_PRD.md`
**Branch:** `Simulink-Auto-Build-2026-05-20`
**Date:** 2026-05-25

This document answers every bullet in PRD §5 Level 0 before any L1+ work
begins. No edits were made to the codebase during discovery.

---

## 1. Mahony filter location

| Item | Path / Reference |
|---|---|
| Mahony PAD-phase function | `Matlab Code/Simulink Development/nav/attitude/casper_attitude_mahony.m` |
| Flight-phase mag correction | `Matlab Code/Simulink Development/nav/attitude/casper_attitude_mag_correct_flight.m` |
| Per-tick dispatcher | `Matlab Code/Simulink Development/nav/attitude/casper_attitude_tick.m` |
| Filter state factory | `Matlab Code/Simulink Development/nav/attitude/casper_attitude_state_new.m` |
| Static-init (initial quat + bias seed) | `Matlab Code/Simulink Development/nav/attitude/casper_attitude_static_init.m` |
| Gyro LPF | `Matlab Code/Simulink Development/nav/attitude/casper_attitude_gyro_lpf.m` |
| RK4 quaternion propagator | `Matlab Code/Simulink Development/nav/attitude/casper_attitude_predict_rk4.m` |

### Simulink artefacts wrapping the filter

| Block / model | Path |
|---|---|
| Standalone attitude subsystem | `nav/attitude/attitude_block.slx` (built by `build_attitude_block.m`) |
| Integration top-level model | `integration/casper_sim_phase0.slx` (built by `build_casper_sim_phase0.m`) |
| MATLAB-Function wrapper inside the top model | `integration/attitude_step_helper.m` (owns the launch latch + transient drop) |
| Legacy non-Simulink driver | `integration/casper_phase0_run.m` (canonical byte-exact reference) |

---

## 2. Mahony parameter values (from `params/casper_sensor_params.m`, pre-L2 state)

Read from main.c att_cfg initializer (firmware-canonical, see file
comments).

| Parameter | Pre-L2 value | Source |
|---|---|---|
| `Kp_Grav` | 10.0 | main.c att_cfg.Kp_grav |
| `Kp_MagPad` | 0.0 | main.c att_cfg.Kp_mag_pad |
| `Kp_MagFlight` | 0.0 | main.c att_cfg.Kp_mag_flight |
| `Ki` | **0.1** | main.c att_cfg.Ki — DIVERGES from PRD §2 default; will change to 0 in L2.2 |
| `GyroLpfCutoff_Hz` | 50.0 | main.c att_cfg.gyro_lpf_cutoff_hz |
| `MagUpdateRate_Hz` | 10.0 | main.c att_cfg.mag_update_hz |
| `LaunchAccel_g` | 3.0 | main.c att_cfg.launch_accel_g |
| `StaticInitSamples` | 500 | casper_attitude.c STATIC_INIT_MAG_SAMPLES |
| `StaticInitTimeout_s` | 10.0 | casper_attitude.c STATIC_INIT_TIMEOUT_S |
| `HeadingSigmaFloor_rad` | 0.01 | casper_attitude.c HEADING_SIGMA_FLOOR |
| `BiasGyroThresh_radps` | 0.035 | casper_attitude.c BIAS_GYRO_THRESH |
| `BiasEmaInvTau_perS` | 0.2 | casper_attitude.c BIAS_EMA_INV_TAU |
| `GyroArw_radSqrtS` | `[6.08e-5; 4.92e-5; 6.73e-5]` | casper_attitude.c att_init() |
| `GravGate_WindowHalfWidth_g` | NOT FOUND | — to be added in L2.1 |
| `PadCalibDuration_s` | NOT FOUND | — to be added in L2.4 |

### Gyro-bias estimation method

- **INIT phase**: arithmetic mean of the first 100 raw gyro samples, captured
  inside `casper_attitude_tick.m` lines 51-54 (NOT in the firmware; this is
  a Phase 0 spec §5.3 addition).
- **POST-INIT (steady state)**: bias is FROZEN at the INIT mean. The
  firmware has an EMA-style refinement gated on `|omega| < BiasGyroThresh`,
  but `casper_attitude_tick.m` does NOT port that path. So pre-L2 the
  effective behaviour is "freeze at init, never refine". L2.4 will change
  this to keep refining for the entire pad window.

### PAD ↔ FLIGHT transition

Owned by the CALLER (`attitude_step_helper.m` and `casper_phase0_run.m`),
NOT the attitude module. Both use an identical latch:

```
mode_pad_latch = true initially
if mode_pad_latch && norm(a_fw) > Attitude.LaunchAccel_g * 9.80665
    mode_pad_latch = false
```

Latch is one-way (no re-entry into pad mode after launch).

---

## 3. Magnetometer noise block

| Item | Path | Value |
|---|---|---|
| Noise generator | `sensors/mag/casper_mag_noise.m` | AR(1) coloured noise |
| Per-axis 1-σ | `Mag.NoiseStd_uT` in `params/casper_sensor_params.m` | **0.5 µT** |
| Correlation time τ | `Mag.NoiseTauSec` | 0.160 s |
| Hard-iron offset | `Mag.HardIron_uT` | `[-9.083; -23.521; -18.100]` (from `mag_cal.c`) |
| Soft-iron matrix | `Mag.SoftIron` | 3×3, from `mag_cal.c` static array |
| Axis-flip sign | `Mag.AxisFlipSign` | `[-1; -1; -1]` (firmware-canonical) |
| Radio-TX interference | `sensors/mag/casper_mag_radio_interference.m` | ±10 µT spike on 100 ms TX cadence |

### Implication for PRD §9 open item

`Mag.NoiseStd_uT = 0.5 µT` is **~100× smaller** than the Python validation's
placeholder `σ = 50 µT`. In principle this supports a non-zero `Kp_mag_pad`
default. **The PRD's directive to default both Kp_mag knobs to 0 is
nevertheless kept**: until a real-data SNR characterisation against this
0.5 µT noise floor (plus the radio-TX spike) is produced, the conservative
0 default avoids the risk of mag-corrections injecting heading drift during
boost. Raised explicitly so it can be revisited at L3.

---

## 4. Baseline 4-state EKF subsystem

| Item | Path |
|---|---|
| EKF state factory | `nav/eskf/casper_eskf_state.m` |
| EKF predict | `nav/eskf/casper_eskf_predict.m` |
| Baro update | `nav/eskf/casper_eskf_update_baro.m` |
| ZUPT update | `nav/eskf/casper_eskf_update_zupt.m` |
| Mach gate | `nav/eskf/casper_eskf_mach_gate.m` |
| Simulink subsystem | `nav/eskf/eskf_block.slx` (built by `build_eskf_block.m`) |
| Top-level integration | `integration/casper_sim_phase0.slx` row 5 (built into `casper_phase0_run.m` as well) |

EKF state: `[altitude_m, velocity_m_s, accel_bias_m_s2, baro_bias_m]`.
Predict rate 416 Hz (snapped to 500 Hz in the visual model). PRD §1 confirms
this layer is OUT OF SCOPE for the Mahony hardening workstream.

---

## 5. Existing test harness / run scripts

| Script | Purpose |
|---|---|
| `nav/attitude/test_attitude_port.m` | Existing 10-criterion attitude AC suite |
| `nav/attitude/test_mahony_hardening.m` | NEW — added in this workstream, 4 L2 unit tests |
| `integration/run_phase0_trustgate.m` | End-to-end trust-gate (byte-exact regression + plots) |
| `integration/run_pad_only_test.m` | 5 s stationary smoke test |
| `integration/run_determinism_check.m` | Two-run hash compare |
| `casper.m` | Top-level entry: paths + params + truth + tuning knobs |

---

## 6. PRD §9 open-item resolutions

### 6.1 Magnetometer noise floor (PRD §9 bullet 1)

See §3 above. `Mag.NoiseStd_uT = 0.5 µT` ≪ placeholder 50 µT. Default
`Kp_mag_pad = 0` is kept per PRD; documented for future L3 follow-up.

### 6.2 Gyro bias freeze-at-launch (PRD §9 bullet 2)

Current Simulink behaviour: bias is frozen at the END OF INIT (not at
launch). The Python port freezes at launch. The behaviours differ.

L2.4 addresses this by porting the Python "freeze at launch" semantics:
- Bias accumulates over the entire PAD window until either
  `PadCalibDuration_s` elapses or `mode_pad → false`.
- After the latch, bias is frozen.

This is a deliberate behaviour change consistent with the PRD §2 validated
configuration. Flagged here per PRD §9 requirement.

### 6.3 Mahony body-frame convention vs Z-up nav (PRD §9 bullet 3)

The Simulink attitude module uses the **firmware-canonical** initial-attitude
formula in `casper_attitude_static_init.m` lines 100-115:

```
pitch = atan2(-ax, sqrt(ay^2 + az^2))
roll  = atan2( ay, az)
yaw   = tilt-compensated mag
```

This matches `casper_attitude.c` lines 152-171. The CASPER-2 nav-frame is
Z-UP (per `casper_phase0_run.m` line ~210 and `nav/frame_switch`
documentation). The formula above assumes body-Z is up at rest, which
matches the CASPER-2 "nose-up on pad" convention because the firmware
frame switch puts the rocket's body-Z along the launch-rail axis.

EKF_SPEC.md §3.2.1's alternative formula (referenced in PRD §9) is NOT used
here. No bug is introduced or fixed by this workstream — the existing
formula is correct for the CASPER-2 frame convention. Raised explicitly
per the PRD's instruction.

---

## 7. Files identified for L2 modification

Read-only L0 lists the files L2 will touch (no edits in this doc):

- `nav/attitude/casper_attitude_mahony.m`
- `nav/attitude/casper_attitude_mag_correct_flight.m`
- `nav/attitude/casper_attitude_tick.m`
- `nav/attitude/casper_attitude_state_new.m`
- `nav/attitude/casper_attitude_grav_correct.m` (NEW — shared gate helper)
- `params/casper_sensor_params.m` (Attitude defaults)
- `casper.m` (tuning knobs)
- `integration/attitude_step_helper.m` (gate launch on `pad_calib_complete`)
- `integration/casper_phase0_run.m` (same)
- `nav/attitude/test_mahony_hardening.m` (NEW — L2 unit tests)
