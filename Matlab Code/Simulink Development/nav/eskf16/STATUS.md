# nav/eskf16 — 16-state error-state EKF (Simulink Phase 0)

## What this is

A Simulink visual-model port of the 16-state error-state EKF that the user
already verified in bench tests (`Matlab Code/EKF Dev/`). It runs in
parallel with the existing 4-state vertical ESKF inside
`casper_sim_phase0.slx` so the two estimators can be compared side-by-side
over a full apogee profile.

## State vector

```
x[0:2]  = attitude error (body-frame rotation vector, rad)
x[3:5]  = velocity error (NED, m/s)
x[6:8]  = position error (NED, m)
x[9:11] = gyro bias error (body, rad/s)
x[12:14]= accel bias error (body, m/s^2)
x[15]   = baro bias error (m)
```

Reference state propagates with Joseph-form scalar updates and an
attitude reset on every error-state injection (mirrors EKF16Verify.m §7).

## Files in this directory

| File | Purpose |
| --- | --- |
| `casper_eskf16_load_symbolic.m` | Loads `casper_ekf16_symbolic.mat` and compiles `matlabFunction` handles for F (16x16), Q (16x16), H_MAG (3x16). Cached in base workspace. |
| `casper_eskf16_helper.m` | Persistent-state Simulink shim. Mirrors `EKF16Verify.m` §7 main loop one tick at a time. |
| `casper_eskf16_body_fw_to_zup.m` | Constant body-frame swap (firmware Y-nose body -> EKF16 body-Zup). |
| `build_eskf16_block.m` | Programmatic builder for `eskf16_block.slx`. |
| `eskf16_block.slx` | Library with `eskf16_visual_block` subsystem. Linked from `integration/casper_sim_phase0.slx`. |
| `test_eskf16_isolation.m` | Closed-loop synthetic O5500X 1-DOF flight, asserts apogee within 5%, vel RMS < 5 m/s. |

## Frame note (IMPORTANT)

`EKF16Verify.m` uses "body Z = up at pad, NED nav frame". On the pad
`q_pad = [0;0;1;0]` and the accelerometer reads `[0;0;+g]` along body Z.

CASPER-2 firmware uses Y-nose body / Z-up nav. The `eskf16_visual_block`
subsystem rotates `gyro_body_fw`, `accel_body_fw`, `mag_body_fw` into
EKF16's body-Zup at its inputs (constant swap X<->Y, keep Z). The
algorithm itself is byte-faithful to EKF16Verify; only the I/O frame
boundary is converted.

The output `att_quat` is therefore in EKF16's `body-Zup -> NED`
convention. Report-side conversion to firmware frame for display lives
in `generate_ekf_comparison_report.m`.

## How to run

```matlab
cd "Matlab Code/Simulink Development"
casper                                  % apogee profile, StopTime=85 s

% Both EKFs run in parallel inside the visual model:
sim('casper_sim_phase0')

% Generate side-by-side comparison report:
cd integration
generate_ekf_comparison_report          % writes plots + .md report
```

## Isolation test

```matlab
cd "Matlab Code/Simulink Development/nav/eskf16"
test_eskf16_isolation                   % must PASS (apogee within 5%)
```

## Reference

- `Matlab Code/EKF Dev/EKF_Symbolic_Dev.m` — F, Q, H derivations + numerical sanity.
- `Matlab Code/EKF Dev/EKF16Verify.m` — closed-loop verification on synthetic O5500X.
- `Flight Images and Raw Data/casper_ekf16_symbolic.mat` — read-only symbolic workspace.

## Implementation notes

### Hand-coded F and Q (not symbolic at runtime)

The deliverables include `casper_eskf16_load_symbolic.m` which loads the
symbolic .mat and compiles MATLAB `matlabFunction` handles, but the
helper itself uses **hand-coded numeric F and Q** (`build_F_` and
`build_Q_` local functions inside `casper_eskf16_helper.m`) for speed.
The hand-coded versions are byte-equivalent to the symbolic derivation
(see `EKF_Symbolic_Dev.m` §4 + §6) and the helper validates against the
synthetic O5500X test in `test_eskf16_isolation.m` to within 0.5 m on a
16 km flight (apogee err ~ 0.00 %, alt RMS 0.44 m).

### Visual model accuracy

Both the existing 4-state ESKF and the new 16-state EKF run inside
`casper_sim_phase0.slx` from the same FrameSwitch_Accel/_Gyro/_Mag
streams. The Phase 0 visual model's sensor chain (imuSensor + sign-flip
+ casper_frame_switch_body) produces "+g on Z" on the pad — i.e., the
EKF16 body-Zup convention is the IDENTITY of body-fw at the
FrameSwitch_Accel output. So the chart's `body_fw_to_zup` block is a
copy, not a swap. See the long comment in
`casper_eskf16_body_fw_to_zup.m`.

### Comparison report

`integration/generate_ekf_comparison_report.m` writes
`EKF_COMPARISON_REPORT.md` and 5 plots into `integration/plots/ekf_comparison/`.
If a sim is running headless and matplotlib-like figure rendering hangs,
set env var `EKF_REPORT_SKIP_PLOTS=1` to bypass plot generation and
write the markdown report only.
