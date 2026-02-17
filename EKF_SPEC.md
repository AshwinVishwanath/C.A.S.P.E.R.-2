# EKF_SPEC.md — Navigation Stack Architecture

**Ground-truth reference for the C.A.S.P.E.R.-2 inertial navigation system.**
Covers the 4-state Extended Kalman Filter, the attitude estimator, and the gyro integrator.

---

## 1. System Overview

The navigation stack provides real-time altitude, velocity, and attitude estimates by fusing data from the IMU (LSM6DSO32), barometer (MS5611), magnetometer (MMC5983MA), and GPS (MAX-M10M).

```
         LSM6DSO32 (833 Hz)
             │
    ┌────────┴────────┐
    │ Axis Remap      │  sensor[Z,X,Y] → body[X,Y,Z]
    │ (main.c)        │  See ORIENTATION_SPEC.md
    └───────┬─────────┘
            │  accel_ms2[3], gyro_rads[3]
            │
    ┌───────┴──────────────────┐
    │  casper_attitude         │  Mahony filter (pad) / RK4 + mag (flight)
    │  → body-to-NED quat     │  833 Hz update rate
    └───────┬──────────────────┘
            │  att.q[4] (quaternion)
            │  accel_ms2[3]
    ┌───────┴──────────────────┐
    │  casper_ekf_predict()    │  Rotate accel to NED, propagate state + P
    │  833 Hz                  │  State: [alt, vel, accel_bias, baro_bias]
    └───────┬──────────────────┘
            │
    ┌───────┴──────────────────┐
    │  casper_ekf_update_baro()│  Async (~100 Hz, non-blocking ms5611_tick)
    │  Baro alt AGL            │  Gated during transonic flight
    └──────────────────────────┘
```

**Files:**

| File | Role |
|------|------|
| [casper_ekf.c](Software/Core/Src/casper_ekf.c) | 4-state vertical EKF |
| [casper_ekf.h](Software/Core/Inc/casper_ekf.h) | EKF types and API |
| [casper_attitude.c](Software/Core/Src/casper_attitude.c) | Mahony + RK4 attitude estimator |
| [casper_attitude.h](Software/Core/Inc/casper_attitude.h) | Attitude types and API |
| [casper_gyro_int.c](Software/Core/Src/casper_gyro_int.c) | Pure gyro dead-reckoning (legacy) |
| [casper_gyro_int.h](Software/Core/Inc/casper_gyro_int.h) | Gyro integrator types |
| [casper_quat.c](Software/Core/Src/casper_quat.c) | Quaternion math library |
| [casper_quat.h](Software/Core/Inc/casper_quat.h) | Quaternion API |

---

## 2. EKF — 4-State Vertical Channel Filter

### 2.1 State Vector

```
x[0] = altitude          (m, positive up, AGL)
x[1] = vertical velocity (m/s, positive up)
x[2] = accel bias        (m/s^2, vertical component)
x[3] = baro bias         (m)
```

All states are initialized to zero. The barometer reference altitude (`baro_ref`) is set at the end of the 30-second calibration period to anchor `x[0] = 0` on the pad.

### 2.2 Process Model

**State transition matrix Phi (constant for fixed dt):**

```
        [1   dt   -0.5*dt^2   0]
Phi =   [0   1    -dt         0]
        [0   0     1          0]
        [0   0     0          1]
```

**State propagation:**

```
a_up = f_ned[2] - g - x[2]         (NED vertical accel minus gravity minus bias)
x[0] += x[1]*dt + 0.5*a_up*dt^2    (altitude)
x[1] += a_up*dt                     (velocity)
x[2] unchanged                      (accel bias: random walk)
x[3] unchanged                      (baro bias: random walk)
```

Where `f_ned = R(q) * accel_body` — the body-frame accelerometer reading rotated into NED using the attitude quaternion.

**Covariance propagation (CMSIS-DSP):**

```
P = Phi * P * Phi' + Q
```

Implemented using `arm_mat_mult_f32()` and `arm_mat_add_f32()` from CMSIS-DSP. Symmetry enforced after each propagation via `P = 0.5*(P + P')`.

### 2.3 Process Noise Q

Derived from MATLAB Allan variance analysis of LSM6DSO32 sensor data:

| Parameter | Symbol | Value | Unit | Source |
|-----------|--------|-------|------|--------|
| Accel velocity random walk | `ACCEL_VRW` | 2.162545e-03 | m/s/sqrt(s) | Allan variance |
| Accel bias instability | `ACCEL_BI_SIGMA` | 1.953783e-04 | m/s^2/sqrt(s) | Allan variance |
| Baro bias instability | `BARO_BI_SIGMA` | 1.000000e-03 | m/sqrt(s) | Allan variance |

Q matrix construction (continuous-to-discrete):

```
qa  = ACCEL_VRW^2
qab = ACCEL_BI_SIGMA^2
qbb = BARO_BI_SIGMA^2

Q[0][0] = qa * dt^3 / 3       (altitude process noise)
Q[0][1] = qa * dt^2 / 2       (alt-vel cross term)
Q[1][0] = qa * dt^2 / 2       (symmetric)
Q[1][1] = qa * dt              (velocity process noise)
Q[2][2] = qab * dt             (accel bias random walk)
Q[3][3] = qbb * dt             (baro bias random walk)
```

All other Q elements are zero.

### 2.4 Barometer Measurement Update

**Measurement model:**

```
H = [1, 0, 0, 1]
z = baro_alt_m (AGL)
innovation = z - (x[0] + x[3])     (altitude + baro_bias)
```

**Measurement noise:**

| Parameter | Value | Unit |
|-----------|-------|------|
| `R_BARO` | 0.08 | m^2 (sigma ~ 0.28 m) |

**Scalar Kalman update:**

```
S = H * P * H' + R = P[0][0] + P[0][3] + P[3][0] + P[3][3] + R_baro
K = P * H' / S     (4x1 vector: K[i] = (P[i][0] + P[i][3]) / S)
x = x + K * innovation
```

**Joseph form covariance update** (numerically stable):

```
I_KH = I - K * H
P = I_KH * P * I_KH' + K * R * K'
```

This avoids the standard `P = (I-KH)*P` which can lose positive-definiteness.

### 2.5 Transonic Barometer Gating

During transonic flight, barometric pressure readings are unreliable due to shockwave effects. The EKF gates (disables) baro updates using estimated Mach number:

```
mach = |velocity| / speed_of_sound(altitude)
speed_of_sound = sqrt(1.4 * 287.058 * T_K)
T_K = 288.15 - 0.0065 * altitude
```

| Parameter | Value | Description |
|-----------|-------|-------------|
| `MACH_GATE_ON` | 0.40 | Gate baro above Mach 0.4 |
| `MACH_GATE_OFF` | 0.35 | Un-gate below Mach 0.35 |

Hysteresis prevents chattering near the threshold. While gated, the EKF relies solely on the IMU (dead-reckoning) for altitude/velocity.

### 2.6 GPS Measurement Updates (STUBS)

Two GPS update functions are defined but not yet implemented:

**GPS Altitude:**
```
H = [1, 0, 0, 0]
R = 100.0 m^2 (sigma = 10.0 m, u-blox MAX-M10M accuracy)
```

**GPS Vertical Velocity:**
```
H = [0, 1, 0, 0]
R = 1.0 (m/s)^2 (sigma = 1.0 m/s)
```

These will use the same Joseph form update as the baro update.

### 2.7 Initial Covariance P0

| State | P0 | Sigma | Unit |
|-------|-----|-------|------|
| Altitude | 0.1 | 0.32 m | m^2 |
| Velocity | 0.001 | 0.032 m/s | (m/s)^2 |
| Accel bias | 0.025 | 0.158 m/s^2 | (m/s^2)^2 |
| Baro bias | 0.75 | 0.87 m | m^2 |

### 2.8 Timing

| Parameter | Value |
|-----------|-------|
| `EKF_DT` | 0.0012 s (833 Hz) |
| Predict rate | Every IMU sample (833 Hz) |
| Baro update rate | ~100 Hz (non-blocking ms5611_tick) |
| GPS update rate | 10 Hz (when implemented) |

### 2.9 CMSIS-DSP Usage

The EKF uses ARM CMSIS-DSP for 4x4 matrix operations:

- `arm_mat_init_f32()` — Initialize matrix instances pointing to float arrays
- `arm_mat_mult_f32()` — Matrix multiplication (Phi*P, result*PhiT)
- `arm_mat_add_f32()` — Matrix addition (+ Q)
- `arm_matrix_instance_f32` — Matrix descriptor struct

All matrices are 4x4 row-major float arrays (16 elements). Working memory (`tmp_a`, `tmp_b`) avoids dynamic allocation.

---

## 3. Attitude Estimator — `casper_attitude`

The attitude estimator produces a body-to-NED quaternion for the EKF's coordinate rotation and for telemetry. See ORIENTATION_SPEC.md for the complete frame convention.

### 3.1 Configuration

```c
typedef struct {
    float Kp_grav;            /* Gravity correction gain (pad phase)    */
    float Kp_mag_pad;         /* Mag correction gain (pad phase)        */
    float Kp_mag_flight;      /* Mag correction gain (flight phase)     */
    float Ki;                 /* Integral gain (both phases)            */
    float gyro_lpf_cutoff_hz; /* Gyro LPF cutoff frequency (50 Hz)     */
    float mag_update_hz;      /* Mag correction rate in flight (10 Hz)  */
    float launch_accel_g;     /* Launch detection threshold (3g)        */
} casper_att_config_t;
```

### 3.2 Phases

The estimator operates in three phases:

```
Power-on → [Static Init] → [Pad Phase] → [Flight Phase]
           0-5 seconds      5-30 seconds   30s+ (launched)
```

#### Phase 1: Static Initialization (0-5 seconds)

**Goal:** Compute initial quaternion and magnetometer reference vector.

1. Accumulate accel samples at 833 Hz and mag samples at ~93 Hz
2. Wait for 500 mag samples (about 5.4 seconds) OR 10-second timeout
3. Average all samples:
   ```
   accel_avg = sum(accel) / count
   mag_avg   = sum(mag) / count
   ```
4. Compute roll and pitch from gravity:
   ```
   pitch = atan2(-ax, sqrt(ay^2 + az^2))
   roll  = atan2(ay, az)
   ```
5. Compute tilt-compensated magnetic heading:
   ```
   mx_h = mx*cos(pitch) + my*sin(roll)*sin(pitch) + mz*cos(roll)*sin(pitch)
   my_h = my*cos(roll) - mz*sin(roll)
   yaw  = atan2(-my_h, mx_h)
   ```
6. Build initial quaternion from ZYX Euler angles: `q = from_euler(roll, pitch, yaw)`
7. Store mag reference in NED: `m_ref_ned = R(q) * mag_avg`

**Fallback:** If 10 seconds pass without 500 mag samples, initialize from gravity only (yaw = 0, `mag_available = false`).

#### Phase 2: Pad Phase (5-30 seconds)

Mahony complementary filter at 833 Hz:

1. **Gyro LPF:** First-order IIR, 50 Hz cutoff
   ```
   alpha = dt / (dt + 1/(2*pi*cutoff))
   gyro_filtered = alpha * gyro_raw + (1-alpha) * gyro_filtered
   ```

2. **Gyro bias estimation:** Running average of all raw gyro samples (double precision accumulator to prevent drift)

3. **Gravity correction:**
   ```
   a_hat  = normalize(accel)
   g_pred = R(q)' * [0, 0, 1]    (predicted gravity in body frame)
   e_grav = cross(a_hat, g_pred)
   ```

4. **Magnetometer correction:**
   ```
   m_hat  = normalize(mag_cal)
   m_pred = R(q)' * m_ref_ned     (predicted mag in body frame)
   e_mag  = cross(m_hat, normalize(m_pred))
   ```

5. **Integral accumulation:**
   ```
   e_int += (e_grav + e_mag) * dt
   ```

6. **Corrected angular rate:**
   ```
   omega = gyro_filtered - gyro_bias + Kp_grav*e_grav + Kp_mag_pad*e_mag + Ki*e_int
   ```

7. **RK4 quaternion propagation** (see Section 3.4)

8. **Launch detection:** If `|accel| > launch_accel_g * 9.80665`, transition to flight phase. Integral error is reset to zero.

#### Phase 3: Flight Phase (after launch)

1. **Gyro bias frozen** — no further estimation
2. **Integral error reset** at launch transition
3. **10 Hz mag correction** (timer-decimated):
   - Same cross-product error computation as pad phase
   - Uses `Kp_mag_flight` gain (lower than pad)
   - **Ignition gating:** Mag corrections disabled during motor burn windows (configurable via `casper_att_add_gate()`, up to 4 gates)
   - Timer only resets on successful correction — if mag data is NULL or gated, correction fires next tick with valid data
4. **RK4 quaternion propagation** at 833 Hz
5. **Uncertainty tracking:**
   ```
   att_sigma[i] = sqrt(att_sigma[i]^2 + gyro_arw[i]^2 * dt)
   heading_sigma = sqrt(heading_sigma^2 + gyro_arw[2]^2 * dt)
   ```
   Heading sigma reduces after mag corrections:
   ```
   heading_sigma = heading_sigma * (1 - alpha) + HEADING_SIGMA_FLOOR * alpha
   ```
   Where `alpha = Kp_mag_flight * dt_corr`, `HEADING_SIGMA_FLOOR = 0.01 rad (~0.6 deg)`

### 3.3 Gyro ARW Values (from MATLAB Allan variance)

LSM6DSO32 characterization results:

| Axis | ARW (rad/sqrt(s)) |
|------|-------------------|
| X | 6.73e-05 |
| Y | 6.08e-05 |
| Z | 4.92e-05 |

### 3.4 RK4 Quaternion Propagation

The quaternion derivative:
```
qdot = 0.5 * q (x) [0, omega_x, omega_y, omega_z]
```

Where `(x)` is Hamilton quaternion multiplication.

Four-stage Runge-Kutta:
```
k1 = qdot(q, omega)
k2 = qdot(q + k1*dt/2, omega)
k3 = qdot(q + k2*dt/2, omega)
k4 = qdot(q + k3*dt, omega)
q_new = q + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)
normalize(q_new)
```

RK4 working memory is pre-allocated in the struct (`k1`, `k2`, `k3`, `k4`, `q_tmp`) to avoid stack allocation in the tight 833 Hz loop.

### 3.5 Ignition Gating

Up to 4 ignition gates can be configured via `casper_att_add_gate(start_s, duration_s)`. During each gate window, magnetometer corrections are completely suppressed to prevent motor exhaust plume / iron casing from corrupting heading.

```c
casper_att_add_gate(&att, 0.0f, 5.0f);    /* First stage: 0-5 seconds */
casper_att_add_gate(&att, 10.0f, 3.0f);   /* Second stage: 10-13 seconds */
```

---

## 4. Gyro Integrator — `casper_gyro_int` (Legacy)

The standalone gyro integrator was the original attitude module before `casper_attitude` was developed. It provides pure gyro dead-reckoning without magnetometer corrections.

### 4.1 Architecture

- Identical RK4 propagation and IIR LPF as `casper_attitude`
- No gravity correction, no mag correction
- Timed calibration (25000 samples = ~30 seconds at 833 Hz) instead of launch detection
- Initial quaternion from gravity only (`casper_quat_from_accel`)
- Gyro bias frozen after calibration sample count reached

### 4.2 Differences from casper_attitude

| Feature | casper_gyro_int | casper_attitude |
|---------|----------------|-----------------|
| Gravity correction | No | Yes (pad only) |
| Mag correction | No | Yes (pad + flight) |
| Initial heading | 0 (unknown) | Tilt-compensated mag |
| Bias estimation | Timed (30s sample count) | Running average + freeze at launch |
| Launch detection | None (timed transition) | Accel threshold |
| Ignition gating | N/A | Up to 4 gates |
| Uncertainty tracking | ARW growth only | ARW growth + mag reduction |

---

## 5. Quaternion Library — `casper_quat`

Utility functions used by both attitude modules and the EKF.

| Function | Description |
|----------|-------------|
| `casper_quat_mult(a, b, r)` | Hamilton product r = a (x) b |
| `casper_quat_normalize(q)` | In-place normalize to unit length |
| `casper_quat_to_rotmat(q, R)` | Body-to-NED rotation matrix (3x3, row-major) |
| `casper_quat_from_accel(accel, q)` | Gravity-only quaternion (no heading) |
| `casper_quat_from_euler(roll, pitch, yaw, q)` | ZYX Euler angles to quaternion |
| `casper_quat_to_euler(q, euler)` | Quaternion to [roll, pitch, yaw] in degrees |

**Convention:** `q[4] = [w, x, y, z]`, scalar-first, Hamilton product.

---

## 6. Superloop Integration

In the main loop (USB_MODE=1), the navigation stack runs as follows:

```
1. Poll LSM6DSO32 data-ready flag (833 Hz interrupt-driven)
2. If new IMU data:
   a. Read accel + gyro
   b. Remap sensor → body frame (see ORIENTATION_SPEC.md)
   c. Read mag if DRDY (100 Hz), negate + calibrate
   d. If static init not complete:
      → casper_att_static_init(accel, mag)
   e. Else if on pad (calibrating):
      → casper_att_update(gyro, accel, mag, dt)
   f. Else (flight):
      → casper_att_update(gyro, accel, mag, dt)
      → casper_ekf_predict(&ekf, &att, accel_ms2, dt)
3. If ms5611_tick() returns new baro data:
   → casper_ekf_update_baro(&ekf, baro_alt - baro_ref)
4. If max_m10m_tick() returns new GPS fix:
   → casper_ekf_update_gps_alt/vel (stubs, not yet active)
5. Every 100 ms: send FC_MSG_FAST telemetry with EKF state + quaternion
```

### 6.1 Baro Reference

The barometer reference altitude (`baro_ref`) is set once at the end of the 30-second calibration period:

```c
baro_ref = ms5611_get_altitude(&baro, 1013.25f);
```

All subsequent baro updates use `baro_alt - baro_ref` to provide altitude AGL (above ground level).

---

## 7. Known Limitations

1. **GPS update not implemented** — The EKF stubs exist but GPS altitude/velocity updates are not yet functional. The filter relies solely on IMU + barometer.

2. **No horizontal state estimation** — The EKF is vertical-only (altitude + velocity). Horizontal position uses raw GPS deltas, not a filtered estimate.

3. **Single-frequency barometer** — The MS5611 operates at a single OSR. No multi-rate fusion is implemented.

4. **Euler gimbal lock** — The Euler angle extraction has a singularity at pitch = +/-90 degrees (the pad condition). The quaternion itself has no singularity. See ORIENTATION_SPEC.md Section 8.3.

5. **Linear process model** — The EKF uses a linearized (constant Phi) model. This is adequate for vertical flight but may need augmentation for highly dynamic maneuvers.
