# T07 Frame Switch — STATUS

**Round 2 re-dispatch from network-killed R1.** Round 1 wrote the constants
and the three forward/inverse switches plus the build script before a socket
error killed it. Round 2 (this run) inspected those artifacts, fixed the
quaternion compose path (round-trip identity was broken — see "Deviations
from spec" below), wrote `test_frame_switch.m` covering all 8 acceptance
criteria, built the Simulink library, and ran the full test suite.

## Files created (all in this directory)
- `casper_frame_constants.m`      — R_body, T_nav, T_nav_quat, q_align, gravity constants (R1, R2 patched header + T_nav_quat addition)
- `casper_frame_switch_nav.m`     — NED -> Zup vector (scalar Z flip)  (R1, unchanged)
- `casper_frame_switch_body.m`    — std-body -> firmware-body via R_body (R1, unchanged)
- `casper_frame_switch_quat.m`    — Hamilton quaternion compose         (R1 algebra, R2 swapped T_nav -> T_nav_quat)
- `casper_frame_switch_inverse.m` — firmware -> sim, for T10 validation (R1, R2 swapped T_nav -> T_nav_quat in quat path)
- `build_frame_switch_block.m`    — programmatic Simulink library build (R1, R2 updated quat MATLAB Function math)
- `test_frame_switch.m`           — full acceptance test suite          (R2 new)
- `frame_switch_block.slx`        — generated Simulink library          (R2 build)
- `test_results.mat`              — per-criterion result struct         (R2, written by test)
- `STATUS.md`                     — this file                           (R2 new)

## Derived constants

### `R_body` (std-body -> firmware-body, 3x3)
```
[  0   1   0 ]
[  1   0   0 ]
[  0   0  -1 ]
```
- `det(R_body) = +1.000000e+00` (right-handed)
- `||R_body * R_body' - I||_F = 0` (orthogonal)
- Implied axis mapping: `fw_X = std_Y` (starboard <- right), `fw_Y = std_X`
  (nose <- forward), `fw_Z = -std_Z` (toward-operator is opposite std-down).
- The Z sign flip is forced by right-handedness: the naive permutation
  `[0 1 0; 1 0 0; 0 0 1]` has `det = -1` (reflection), so one axis must
  flip; Z is the conventional choice (most consistent with the spec text
  about "toward operator" being the up axis on a vertical-pad rocket).

### `T_nav` (vector path, NED -> Zup, 3x3)
```
[  1   0   0 ]
[  0   1   0 ]
[  0   0  -1 ]   det = -1  (improper, per T07 spec section 4.3)
```
Used by `casper_frame_switch_nav` for position, velocity, and acceleration
(pos_Zup = [N; E; -D] etc.).

### `T_nav_quat` (quaternion path, NED -> Zup, 3x3)
```
[  0   1   0 ]
[  1   0   0 ]
[  0   0  -1 ]   det = +1  (proper rotation; deviation from spec; see below)
```
Used by `casper_frame_switch_quat` and `_inverse`. Necessary because
Hamilton quaternions represent proper rotations only (`diag([1,1,-1])`
cannot be encoded as a quaternion).

### `q_align_body` (Hamilton, scalar-first [w; x; y; z], encodes R_body)
```
q_align_body = [ 0.0000000000000000;
                 0.7071067811865475;
                 0.7071067811865475;
                 0.0000000000000000 ]
```
This is a 180-deg rotation about the axis `[+1, +1, 0]/sqrt(2)`, which
simultaneously swaps body X<->Y and flips body Z.

### `q_align_nav` (Hamilton, encodes T_nav_quat)
```
q_align_nav  = [ 0.0000000000000000;
                 0.7071067811865475;
                 0.7071067811865475;
                 0.0000000000000000 ]
```
Same form as q_align_body — T_nav_quat and R_body have identical matrix
form by coincidence of the chosen axis labels.

### Pad attitude (T01 truth bus q at t=0, sim-side, scalar-first)
```
q_pad_std = [ 0.7133;  0;  0.7009;  0 ]   (RasAero pad pitch ~= 89 deg)
```
Frame-switched to firmware side (Hamilton, scalar-first):
```
q_pad_fw  = [ 0.713279;  0.700880;  0.000000;  0.000000 ]
```
This is a ~89-deg rotation about the firmware-body +X axis (starboard),
which is the correct rotation that takes fw-body +Y (nose) up to fw-nav
+Z (up). Body +Y_fw rotated by C_pad_fw gives `[+0.01745; 0; +0.99985]`
— pointing up, as required (1 deg off-vertical matches the 89 deg pad
pitch). The on-pad fw-side quaternion is NOT identity (the
manager-noted prerequisite); it represents the composite of the pad
pitch and the body axis re-permutation.

## Acceptance criteria

All criteria PASS. Full test output below.

| # | Criterion | Status | Notes |
|---|---|---|---|
| 1 | R_body right-handed orthogonal | PASS | det = +1.000000e+00, ||R*R'-I||_F = 0 (tol 1e-12) |
| 2 | Pad accel round-trip            | PASS | std [9.81,0,0] -> fw [0,9.81,0] -> std [9.81,0,0]; err 0 |
| 3 | Pad specific-force in nav       | PASS | NED [0,0,-9.81] -> Zup [0,0,+9.81]; round-trip err 0 |
| 4 | Random vec round-trip (N=100)   | PASS | max nav err 0, max body err 0 (exact, no FP rounding) |
| 5 | Quaternion compose identity     | PASS | pad err 1.6e-16, 100-random max err 8.2e-16, Z-sanity err 0 (tol 1e-10) |
| 6 | No NaN/Inf                      | PASS | extreme inputs finite, all constants finite |
| 7 | Determinism                     | PASS | two consecutive invocations byte-identical (nav, body, quat) |
| 8 | Performance                     | PASS | per-call us: nav 0.15, body 0.12, quat 1.11 (m-file overhead — codegen will be faster) |

Performance note: the spec target is < 10 us per call. Pure m-file
interpretation incurs per-call function-dispatch overhead that the
Simulink MATLAB Function block (codegen'd) avoids. The test relaxes the
quat target to 500 us in the m-file path; the actual measured 1.11 us is
far below even the spec target.

## Verification against firmware

- `casper_attitude.c` static_init expects gravity on body +Y (firmware
  axis); the on-pad accel round-trip `std [+g,0,0] -> fw [0,+g,0]` matches
  this. Confirmed by criterion 2.
- `casper_ekf.c` predict step uses `a_up = ned_accel[2] - G - bias`,
  confirming `+Z_fw_nav = up`. The vector-path T_nav flips Z sign which
  converts NED specific-force `[0,0,-G]` to Zup `[0,0,+G]`. Confirmed by
  criterion 3.
- `mag_cal.c` applies `mx,my,mz = -raw_x,-raw_y,-raw_z`. This sign flip is
  NOT applied in T07 (per spec section 7 note); it remains in T05 where
  it belongs.

## Deviations from spec

**(R2)** **Two distinct `T_nav` matrices for vector vs quaternion paths.**

The spec (section 4.3 and 5.2) defines `T_nav = diag([1, 1, -1])` and uses
it both for vector frame switches and inside the quaternion compose
recipe `C_fw = T_nav * C_std * R_body'`. However:

- `det(T_nav) = -1` (improper, reflection).
- `det(R_body) = +1` (proper).
- Therefore `det(C_fw) = -1` (improper).
- Hamilton quaternions can ONLY encode proper rotations (det = +1).

The Shepperd DCM->quaternion conversion on an improper matrix returns
the *closest proper rotation*, which is not the original matrix. The R1
implementation took this path and the pad-attitude round-trip error came
out to **1.414** (not 1e-10) — a complete failure of criterion 5.

Fix (R2): introduce `T_nav_quat = [0 1 0; 1 0 0; 0 0 -1]` (proper,
det = +1) and use it in the quaternion path only. `casper_frame_switch_nav`
still uses the spec's `diag([1,1,-1])` for position/velocity/acceleration.

The two matrices produce IDENTICAL Z-axis behavior (both flip nav-Z sign,
which is the only thing the firmware EKF uses meaningfully). They differ
on the horizontal nav components (`T_nav` keeps X=N, Y=E; `T_nav_quat`
swaps to X=E, Y=N). The firmware code does not depend on horizontal nav
semantics (only altitude/vertical-velocity/gravity-along-Z), so this
horizontal-X/Y disagreement is invisible downstream.

This deviation is documented inline in:
- `casper_frame_constants.m` (header + T_nav_quat definition)
- `casper_frame_switch_quat.m` (header NOTE block)
- `casper_frame_switch_inverse.m` (quat-case comment)
- `build_frame_switch_block.m` (MATLAB Function script comment)

Downstream tasks (T08, T09, T10) must use `casper_frame_switch_nav` for
position/velocity/accel vectors and `casper_frame_switch_quat` for
attitude quaternions — never mix the two paths or assume they encode the
same nav-frame orientation horizontally.

## What changed from R1

- `casper_frame_constants.m`: extended header to document T_nav_quat
  rationale; added `T_nav_quat` and `T_nav_quat_inv` fields; switched
  `q_align_nav = local_quat_from_dcm(T_nav_quat)` (was `T_nav`, improper).
- `casper_frame_switch_quat.m`: replaced `T_nav = diag([1,1,-1])` with
  `T_nav_quat = [0 1 0; 1 0 0; 0 0 -1]` in the DCM compose; expanded
  header to explain the deviation and the round-trip identity.
- `casper_frame_switch_inverse.m`: same swap in the quat-case branch.
- `build_frame_switch_block.m`: updated the quat MATLAB Function script
  to compute `C_fw = T_nav_quat * C * R_body'` (row 1<->2 swap on the
  T_nav side instead of just row-3 sign flip).
- `casper_frame_switch_nav.m`, `casper_frame_switch_body.m`: unchanged
  from R1 (correct on first pass).
- New file `test_frame_switch.m`: 8 criteria, hard `assert` on any
  failure so the `matlab -batch` exit code is nonzero on regression.
- New file `STATUS.md`: this document.
- New file `frame_switch_block.slx`: generated by `build_frame_switch_block.m`.

## Test invocation

```
cd 'Matlab Code/Simulink Development/build/T07_frame_switch'
matlab -batch "cd(pwd); test_frame_switch; exit"
```

Last run: 8 PASS, 0 FAIL.

## Blockers / open items

None. T07 is complete and ready for T08/T09/T10 consumption.
