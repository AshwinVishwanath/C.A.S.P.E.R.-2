# T07 — Sim↔Firmware Frame Switch (HIL Boundary)

## 1. Goal

Build the single, isolated MATLAB Function block that converts every sim-side signal (NED nav frame, standard-aircraft body frame: X-fwd, Y-right, Z-down) into the firmware-side signal (Z-up local-level nav frame, Y-nose body frame: Y=nose, X=stbd, Z=toward-operator).

This block is the HIL boundary. Everything upstream is in sim-native (Aerospace Blockset) conventions; everything downstream is in firmware-native conventions. T08 and T09 estimator ports consume the output of this block exclusively.

Get this block wrong and the estimator sees garbage. Get it right once and every downstream task becomes trivial.

## 2. Inputs

| Input | Source |
|---|---|
| Truth bus (sim-native) | T01 |
| Sensor outputs (sim-native) | T03 IMU+ADXL, T05 mag (each output is in std-body frame) |
| Baro/GPS scalars | T04, T06 (frame-invariant, pass through) |

## 3. Outputs

All in `Software/Sim/build/T07_frame_switch/`:

| File | Purpose |
|---|---|
| `casper_frame_constants.m` | Defines `R_body`, `q_align`, and the NED↔Zup sign vector once |
| `casper_frame_switch_nav.m` | MATLAB Function: NED→Zup for position/velocity/accel (scalar Z-flip) |
| `casper_frame_switch_body.m` | MATLAB Function: std-body→firmware-body via constant `R_body` |
| `casper_frame_switch_quat.m` | MATLAB Function: compose quaternion through both frame changes |
| `casper_frame_switch_inverse.m` | MATLAB Function: reverse direction (firmware→sim), for validation only |
| `build_frame_switch_block.m` | Simulink build script |
| `test_frame_switch.m` | Sanity round-trip + on-pad accel test |
| `frame_switch_block.slx` (or library) | Subsystem artifact |
| `STATUS.md` | Acceptance status |

## 4. Conventions recap

### 4.1 Sim-side (input)

- **Nav frame: NED.** +X = North, +Y = East, +Z = Down. Gravity in nav: `[0, 0, +9.80665]`.
- **Body frame: standard-aircraft.** +X = forward (nose direction), +Y = right (starboard), +Z = down (relative to aircraft).
- **Quaternion: scalar-first `[q0, q1, q2, q3]`, body-to-nav.** Aerospace Blockset 6-DOF Quaternion block convention.
- On the pad (rocket vertical, nose up), body +X points to NED -Z (i.e., up). The truth quaternion `q_std_pad` is the rotation taking body axes to NED axes with this orientation.

### 4.2 Firmware-side (output)

- **Nav frame: Z-up local-level.** +X = East (or +X = some local frame X; the firmware doesn't strongly use horizontal), +Y = North (likewise), +Z = up. Gravity in nav: `[0, 0, +9.80665]` — same sign as NED because Z is now up, so the gravity vector points up away from Earth's center? **No** — gravity is the *reaction* the accelerometer feels. On the pad, the body experiences specific force +9.81 along the up axis. In Z-up frame, this is `[0, 0, +9.81]`. See `casper_ekf.c` predict: `a_up = ned_accel[2] - G_ACCEL - bias`. On the pad, `ned_accel[2] ≈ +9.81`, `a_up ≈ 0`. Verified.
- **Body frame: Y-nose.** +Y = nose direction, +X = starboard, +Z = toward operator (looking forward from the launch site).
- **Quaternion: Hamilton `[w, x, y, z]`, body-to-nav.** From `casper_quat.h`. Scalar-first.

### 4.3 The two transformations to compose

**Transformation 1 — Nav frame: NED → Z-up.**
The two frames share +N, +E directions but flip Z. So:
```
pos_Zup = [pos_NED.N; pos_NED.E; -pos_NED.D]
vel_Zup = [vel_NED.N; vel_NED.E; -vel_NED.D]
accel_Zup = [accel_NED.N; accel_NED.E; -accel_NED.D]
```

In matrix form: `T_nav = diag([1, 1, -1])`. This flips only the Z axis.

**Transformation 2 — Body frame: std (X-fwd, Y-right, Z-down) → firmware (Y-nose, X-stbd, Z-op).**
The std forward axis (+X) becomes firmware nose (+Y). The std right axis (+Y) becomes firmware starboard (+X). The std down axis (+Z) becomes firmware operator (+Z)? Let's verify:
- std body X = forward = up on pad
- std body Y = right = starboard on pad (if launching North-pointing, right = East)
- std body Z = down = down relative to aircraft = toward operator? Depends on where operator stands. By convention, "operator" in the firmware = "where the cable comes out" = aft = standard -X axis? Or it's "Z component" = into the page when viewing from the side = roughly down-and-toward-base-of-rocket.

The cleanest empirical determination of `R_body`: pick the orientation that makes the on-pad accelerometer reading round-trip correctly.

#### Pad accel round-trip test

On pad, rocket is vertical, nose up:
- The accelerometer feels specific force `+9.81` in the up direction (reaction to gravity).
- In std body frame: up = forward = +X. So `accel_std = [+9.81, 0, 0]`.
- In firmware body frame: up = nose = +Y. So `accel_fw = [0, +9.81, 0]`.

Therefore `R_body * [1; 0; 0] = [0; 1; 0]`.

For full 3×3 determination, we need two more axis mappings. The user-specified mapping is:
- std +X (forward = up) → firmware +Y (nose). ✓
- std +Y (right) → firmware +X (starboard). 
- std +Z (down) → firmware +Z (toward operator).

So:
```
R_body = [ 0  1  0 ;     row 1: firmware X = std Y
           1  0  0 ;     row 2: firmware Y = std X
           0  0  1 ]     row 3: firmware Z = std Z
```

Verify `det(R_body) = -1` → this is a reflection, not a rotation. A reflection is not a valid orthogonal change-of-basis for a right-handed frame. **This is a problem** — it means one of the axis assumptions above is inconsistent with right-handedness.

Resolution: either the firmware body frame is left-handed (unlikely for a sane convention) or one of the axis directions is opposite. The likely fix: firmware +Z is *opposite* the std +Z, i.e., firmware +Z = -std +Z. Let's check: if firmware Z points *toward* the operator and the operator stands behind the launch pad with the rocket pointing North, the operator's eye looks at the rocket from -X NED direction. The rocket's "rear" (bottom, fins) is +X NED direction. So "toward operator" = -X NED at base = -X std body (since std body X = forward). So in body frame, "toward operator" = backward = -X std body, NOT +Z std body.

Updated mapping:
- std +X (forward = up) → firmware +Y (nose)
- std -X (backward = down on pad, toward operator) → firmware +Z (toward operator)?? This doesn't work either; +Z perpendicular to nose, not parallel.

The honest answer is that **the firmware body frame's Z axis direction needs to be confirmed by inspection of `casper_attitude.c` and the IMU mounting drawing**. Two candidate `R_body`:

**Candidate A** (firmware Z = std Z = down on pad):
```
R_body = [ 0  1  0 ;
           1  0  0 ;
           0  0  1 ]   det = -1, BAD (reflection)
```

**Candidate B** (firmware Z = std -Z = up on pad, with firmware Y = std X = forward = up):
That makes firmware Y and Z parallel, BAD.

**Candidate C** (firmware Z = std Y, firmware X = std Z):
```
R_body = [ 0  0  1 ;     firmware X = std Z = down
           1  0  0 ;     firmware Y = std X = forward
           0  1  0 ]     firmware Z = std Y = right
det = ?
```
`det = 1·(0·0 - 0·1) - 0 + 0 = 0`... no.

The implementation sub-agent owns deriving the correct 3×3. Steps:

1. **Read** `Software/App/drivers/lsm6dso32.c` and the IMU mounting comments. Identify which IMU axis is reported as which firmware-body axis.
2. **Read** `Software/App/cal/mag_cal.c` and check the ×-1 sign flip — this tells you how mag axes are remapped.
3. **Cross-reference** with the `casper_attitude.c` pad-state assumptions: in `casper_attitude_static_init`, the code averages accel and assumes the average is gravity. Which axis does the firmware expect gravity on, on the pad? Find the line.
4. **Derive `R_body`** that maps std-body to firmware-body, satisfying:
   - On-pad gravity round-trip: `R_body * [9.81; 0; 0] = [0; 9.81; 0]` (or whichever axis the firmware expects).
   - Right-handed: `det(R_body) = +1`.
   - Orthogonal: `R_body * R_body' = I`.
5. **Bake** into `casper_frame_constants.m` as a literal 3×3.
6. **Verify** with the round-trip test.

For the trial implementation, start with this guess (consistent with right-hand rule and "Y=nose, X=stbd, Z=operator"):

```
firmware_X = +std_Y     (std Y = right = starboard ✓)
firmware_Y = +std_X     (std X = forward = nose ✓ on a vertical rocket)
firmware_Z = +std_Z     (std Z = down = wait... this conflicts with rocket vertical = down on a vertical rocket means down toward Earth's center = toward base; "operator" if standing at the base looking up = same direction = ✓)
```

Then `R_body = [0 1 0; 1 0 0; 0 0 1]`. `det = -1`. ✗

If "operator" is actually the *opposite* of std-body Z (i.e., looking from above, std-body Z=down, but operator stands at base looking up = +std-body Z, so firmware Z = +std-body Z is correct, but then we lose right-handedness).

**Implementation note (T07 sub-agent)**: there is no way to make a Y-nose firmware body frame right-handed AND match all three axis-direction stories simultaneously with just permutations. One axis must be sign-flipped. The most likely candidate (based on the project's mag-flip pattern, where firmware applies ×-1 to all mag axes for a sign-convention fix): the firmware-body frame's `Z` axis is *opposite* to a naively-permuted std `Z`.

Use:
```
R_body = [ 0  1  0 ;
           1  0  0 ;
           0  0 -1 ]      det = +1, right-handed ✓
```

Verify on-pad accel: `R_body * [9.81; 0; 0] = [0; 9.81; 0]` ✓.

This is the recommended starting `R_body`. The sub-agent must verify against the on-pad accel firmware reading; if the firmware's `casper_attitude_static_init` expects gravity on a different axis or different sign, escalate to the manager.

## 5. Quaternion composition

Given a quaternion `q_std` (Hamilton, scalar-first, body-std to NED), produce `q_fw` (Hamilton, scalar-first, body-fw to Zup).

### 5.1 The composition

```
v_fw = R_body * v_std                   (body-frame vector rotation)
v_Zup = T_nav * v_NED                   (nav-frame Z flip)
q_fw_body_to_fw_nav = q_align_nav ⊗ q_std ⊗ q_align_body
```

where:
- `q_align_body` is the Hamilton quaternion encoding `R_body^T` (rotation from firmware-body back to std-body, applied to the body-side of `q_std`).
- `q_align_nav` is the Hamilton quaternion encoding `T_nav` (the diag([1,1,-1]) sign flip on the nav side).

Both `q_align_body` and `q_align_nav` are constants. Compute them once in `casper_frame_constants.m`.

### 5.2 Practical recipe

For Phase 0, use this simpler formulation:

1. Convert `q_std` to a 3×3 rotation matrix `C_std` (body-std to NED).
2. Compute the firmware-frame DCM: `C_fw = T_nav * C_std * R_body^T`.
3. Convert `C_fw` back to a Hamilton quaternion `q_fw`.

This is mathematically equivalent to the quaternion compose but easier to debug.

### 5.3 Quaternion sign convention

Hamilton quaternion has the property that `q` and `-q` represent the same rotation. The conversion from DCM to quaternion has a sign ambiguity. Adopt: `q.w >= 0`, flip sign of all four components if not. Match the firmware's convention (see `casper_quat.c` `quat_from_dcm`).

## 6. Inverse transformation (for validation)

T10 validation needs to compare estimator output (in firmware frame) against truth (in sim frame). To do so, transform the estimator output *back* to sim frame:

```
pos_NED = [pos_Zup.X; pos_Zup.Y; -pos_Zup.Z]
vel_NED = [vel_Zup.X; vel_Zup.Y; -vel_Zup.Z]
q_std = q_align_nav^{-1} ⊗ q_fw ⊗ q_align_body^{-1}
```

Implement in `casper_frame_switch_inverse.m`. T10 uses this.

## 7. Sensor signals through the boundary

| Signal | Type | Transformation |
|---|---|---|
| IMU accel (m/s² or g, body) | std-body 3×1 | `accel_fw = R_body * accel_std` |
| IMU gyro (dps, body) | std-body 3×1 | `gyro_fw = R_body * gyro_std` |
| ADXL accel (g, body) | std-body 3×1 | `accel_fw = R_body * accel_std` |
| Mag (µT, body) | std-body 3×1 | `mag_fw = R_body * mag_std` (note: firmware applies ×-1 later, so do NOT pre-flip here) |
| Baro pressure | scalar | pass through |
| Baro temperature | scalar | pass through |
| Baro altitude | scalar | pass through (frame-invariant — altitude is up-positive in both nav frames) |
| GPS lat/lon | 2 scalars | pass through (geodetic, frame-invariant) |
| GPS altitude MSL | scalar | pass through |
| GPS velocity NED | 3×1 NED | `vel_fw = T_nav * vel_NED` |
| Truth quaternion (for validation) | std body-to-NED Hamilton | full compose |
| Truth position | NED 3×1 | `T_nav * pos_NED` |
| Truth velocity | NED 3×1 | `T_nav * vel_NED` |
| Truth accel | NED 3×1 | `T_nav * accel_NED` |
| Truth body rates | std-body 3×1 | `R_body * omega_std` |

## 8. Acceptance criteria

1. **`R_body` is right-handed orthogonal**: `det(R_body) = +1 ± 1e-12` and `R_body * R_body' = I ± 1e-12`.
2. **Pad accel round-trip**: with std-body input `[+9.81, 0, 0]`, output is `[0, +9.81, 0]` to within 1e-12.
3. **Pad gravity in nav**: with NED input `[0, 0, +9.80665]` (gravity vector pointing down), output Z-up `[0, 0, -9.80665]` (gravity vector still pointing down, but now negative in Z-up — wait, that's wrong; gravity acceleration is *toward Earth*, which is -Z-up). **Confirm with sub-agent: gravity in Z-up should be `[0, 0, -9.80665]`** because the gravity force vector points down (Z-down direction = -Z-up direction).

   But the accelerometer specific-force reading on the pad is **+9.80665 in the up axis** (reaction force from the pad). The accel reading is *NOT* the gravity vector itself; it's the specific force opposing gravity.

   So the transformation T_nav = diag([1,1,-1]) is correct for *vectors* in the nav frame, but the *meaning* of those vectors changes interpretation. The accel reading on the pad in NED is `[0, 0, -9.81]` (specific force opposes gravity = points up = -Z in NED). Transformed: `[0, 0, +9.81]` in Z-up (specific force points up = +Z in Z-up). ✓

   So the criterion: NED specific-force `[0, 0, -9.81]` → Z-up `[0, 0, +9.81]`. Verify.

4. **Round-trip identity**: `frame_switch_inverse(frame_switch(x)) == x` for 100 random vectors, to within 1e-12.

5. **Quaternion compose identity**: starting with a known `q_std` (e.g., pad attitude), going to `q_fw` and back recovers the original to within 1e-10 component-wise (and modulo sign).

6. **No NaN/Inf**: ever.

7. **Determinism**: this block has no random state; running twice produces byte-identical outputs.

8. **Performance**: a single block call must complete in < 10 µs. No loops, no allocations.

## 9. Anti-goals

- Do NOT compose multiple frame transforms in sequence at run-time — bake `q_align_nav ⊗ q_align_body` once at `casper_frame_constants.m` load time.
- Do NOT call DCM↔quaternion conversion functions from `Aerospace Blockset` inside a tight Simulink loop. They are written for clarity, not speed. Inline the math.
- Do NOT introduce any "tunable" parameters in this block. Frame constants are constants.
- Do NOT pre-apply the mag ×-1 sign flip here. The mag-flip is sensor-specific and lives in T05.
- Do NOT modify any firmware files to "fix" axis conventions. The firmware is the source of truth; the sim adapts.

## 10. Hand-off

T08 (EKF) and T09 (attitude) consume the frame-switched outputs as if they were real firmware-side sensor readings. T10 (validation) uses `casper_frame_switch_inverse.m` to compare estimator output against truth.

If `R_body` is wrong, the entire downstream stack diverges within milliseconds. Triple-check the on-pad accel round-trip test before declaring T07 complete.

## 11. STATUS.md template

```
# T07 Frame Switch — STATUS

## Files created
- casper_frame_constants.m
- casper_frame_switch_nav.m
- casper_frame_switch_body.m
- casper_frame_switch_quat.m
- casper_frame_switch_inverse.m
- build_frame_switch_block.m
- test_frame_switch.m

## R_body chosen
```
[  0   1   0 ]
[  1   0   0 ]
[  0   0  -1 ]
```
det = +1, right-handed ✓

## Acceptance criteria
| # | Criterion | Status | Notes |
|---|---|---|---|
| 1 | R_body orthogonal | PASS | det = 1.0000 |
| 2 | Pad accel round-trip | PASS | (1e-15 error) |
| 3 | Pad gravity in nav | PASS | |
| 4 | Round-trip identity | PASS | max 4e-15 |
| 5 | Quat compose identity | PASS | max 8e-15 |
| ... | | | |

## Verification against firmware
- casper_attitude.c static_init expects gravity on body +Y: confirmed.
- mag_cal.c ×-1 sign flip: not duplicated here (correct).

## Deviations from spec
- (any)
```
