# EKF_SPEC.md -- Navigation Stack Architecture

**Ground-truth reference for the C.A.S.P.E.R.-2 inertial navigation system.**
Covers the 4-state Extended Kalman Filter, the attitude estimator, and the gyro integrator.

This is the consolidated specification. It replaces EKF_SPEC v1, EKF_SPEC_v2, and EKF_SPEC_v2.1.

---

## 1. System Overview

The navigation stack provides real-time altitude, velocity, and attitude estimates by fusing data from the IMU (LSM6DSO32), barometer (MS5611), magnetometer (MMC5983MA), and GPS (MAX-M10M).

```
         LSM6DSO32 (833 Hz)
             |
    +--------+--------+
    | Gyro Temp Comp  |  (if GYRO_TEMP_COMP defined)
    | Identity Map    |  body[X,Y,Z] = sensor[X,Y,Z]
    | (flight_loop.c) |
    +-------+---------+
            |  accel_ms2[3], gyro_rads[3]
            |
    +-------+------------------+
    |  casper_attitude         |  Mahony filter (pad) / RK4 + mag (flight)
    |  -> body-to-NED quat    |  833 Hz update rate
    +-------+------------------+
            |  att.q[4] (quaternion)
            |  accel_ms2[3]
    +-------+------------------+
    |  Body->NED rotation      |  833 Hz: R(q) * accel -> ned_accel
    |  Trapezoidal accumulator |  Average 2 NED samples -> 416 Hz
    +-------+------------------+
            |
    +-------+------------------+
    |  casper_ekf_predict()    |  Propagate state + P with NED accel
    |  416 Hz (EKF_DT=0.0024) |  State: [alt, vel, accel_bias, baro_bias]
    +-------+------------------+
            |
    +-------+------------------+
    |  casper_ekf_update_zupt()|  Zero-velocity pseudo-measurement
    |  416 Hz, PAD / LANDED    |  Clamps velocity when stationary
    +-------+------------------+
            |
    +-------+------------------+
    |  casper_ekf_update_baro()|  Async (~100 Hz, non-blocking ms5611_tick)
    |  Baro alt AGL            |  Gated during transonic flight
    +--------+-----------------+
```

**Sign convention (Z-UP local-level frame, NOT true NED):**
The attitude estimator uses `g_ned = {0, 0, +1}` in `casper_attitude.c`, meaning the Z axis points UP. On the pad, `ned_accel_ms2[2] ~ +9.81`. The EKF computes `a_up = ned_accel_ms2[2] - G_ACCEL - bias`. Positive `a_up` means upward acceleration. This is consistent throughout the stack -- do not change without checking `casper_attitude.c`.

**Body frame convention (identity sensor-to-body mapping):**
Body X = sensor X (starboard), Body Y = sensor Y (nose, up on pad), Body Z = sensor Z (toward operator). No axis permutation is applied.

**Files:**

| File | Role |
|------|------|
| [casper_ekf.c](Software/App/nav/casper_ekf.c) | 4-state vertical EKF |
| [casper_ekf.h](Software/App/nav/casper_ekf.h) | EKF types and API |
| [casper_attitude.c](Software/App/nav/casper_attitude.c) | Mahony + RK4 attitude estimator |
| [casper_attitude.h](Software/App/nav/casper_attitude.h) | Attitude types and API |
| [casper_gyro_int.c](Software/App/nav/casper_gyro_int.c) | Pure gyro dead-reckoning (legacy) |
| [casper_gyro_int.h](Software/App/nav/casper_gyro_int.h) | Gyro integrator types |
| [casper_quat.c](Software/App/nav/casper_quat.c) | Quaternion math library |
| [casper_quat.h](Software/App/nav/casper_quat.h) | Quaternion API |
| [flight_loop.c](Software/App/flight/flight_loop.c) | Flight loop (nav stack caller) |
| [app_globals.h](Software/App/flight/app_globals.h) | Extern declarations for sensor globals |
| [temp_cal_coeffs.h](Software/App/nav/temp_cal_coeffs.h) | Gyro temperature compensation coefficients |

---

## 2. EKF -- 4-State Vertical Channel Filter

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
a_up = ned_accel_ms2[2] - G_ACCEL - x[2]
x[0] += x[1]*dt + 0.5*a_up*dt^2    (altitude)
x[1] += a_up*dt                     (velocity)
x[2] unchanged                      (accel bias: random walk)
x[3] unchanged                      (baro bias: random walk)
```

The caller rotates body-frame accel to NED before calling predict. The EKF receives NED accel directly (see Section 6.2 for the accumulation scheme).

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
| Baro bias instability | `BARO_BI_SIGMA` | 1.000000e-03 | m/sqrt(s) | Tuned |

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

All other Q elements are zero. Q is parameterised by `dt`, so it scales correctly when the predict rate changes (416 Hz predict uses `dt = 0.0024`).

### 2.4 Generic Joseph-Form Scalar Update

All measurement updates (baro, ZUPT, future GPS) use a shared Joseph-form scalar update function:

```c
static void joseph_scalar_update(float x[4], float P[16],
                                 const float H[4], float z,
                                 float R, float gate_k2,
                                 float tmp_a[16], float tmp_b[16]);
```

**Parameters:**
- `H[4]` -- measurement matrix row vector
- `z` -- measurement value
- `R` -- measurement noise variance (scalar)
- `gate_k2` -- chi-squared gate threshold. Pass `INFINITY` to disable gating (e.g. ZUPT).

**Algorithm:**

```
innovation = z - H . x
S = H . P . H' + R                    (innovation covariance, scalar)

/* Innovation gate (NaN-safe) */
if (!(innovation^2 <= gate_k2 * S))    (inverted logic rejects NaN)
    return

K = P . H' / S                         (Kalman gain, 4x1)
x += K * innovation                    (state update)

/* Joseph form covariance update */
I_KH = I - K * H
P = I_KH * P * I_KH' + K * R * K'
symmetrize(P)
```

The Joseph form avoids the standard `P = (I-KH)*P` which can lose positive-definiteness. The two 4x4 matrix multiplies use CMSIS-DSP (`arm_mat_mult_f32`).

**NaN-safe innovation gate:** IEEE 754 NaN comparisons always return false. The gate uses `!(innovation * innovation <= gate_k2 * S)` so that NaN inputs are rejected (NaN <= X is false, !false = true). The standard form `innovation^2 > threshold` would pass NaN through because `NaN > X` evaluates to false.

### 2.5 Barometer Measurement Update

**Measurement model:**

```
H = [1, 0, 0, 1]
z = baro_alt_m (AGL)
innovation = z - (x[0] + x[3])     (altitude + baro_bias)
```

**Measurement noise:**

| Parameter | Value | Unit |
|-----------|-------|------|
| `R_BARO` | 0.5 | m^2 (sigma ~ 0.7 m) |

Uses `joseph_scalar_update()` with `gate_k2 = BARO_GATE_K2` (25.0, 5-sigma).

**No baro-derived velocity:** Velocity correction comes through the cross-covariance in P via the predict step. A baro-derived velocity measurement was considered but removed because it is correlated with the baro altitude measurement and adds complexity without clear benefit given the P coupling already provides indirect velocity observability.

### 2.6 Innovation Gating

A 5-sigma chi-squared gate rejects outlier baro measurements:

```
gate_test = innovation^2 / S
if gate_test > BARO_GATE_K2 (= 25.0), reject update
```

| Parameter | Value | Description |
|-----------|-------|-------------|
| `BARO_GATE_K2` | 25.0 | Squared threshold (5^2) |

This is a hard reject -- if the innovation exceeds 5 sigma, the update is discarded entirely. The ZUPT measurement passes `INFINITY` for `gate_k2`, meaning it is never gated.

### 2.7 NaN/Inf Input Rejection

Before any measurement processing, the baro altitude input is checked:

```c
if (!isfinite(baro_alt_m))
    return;
```

**Root cause:** The MS5611 can occasionally return corrupt SPI reads that produce a negative pressure value. `powf(negative, 0.190284)` returns NaN, which propagates through the innovation into the state vector. The driver also clamps pressure to a minimum of 0.01 hPa as a defense-in-depth measure (see SENSOR_SPEC.md Section 4.5).

### 2.8 ZUPT -- Zero-Velocity Pseudo-Measurement

When the system detects stationary conditions, a pseudo-measurement constraining velocity to zero is injected:

```
H = [0, 1, 0, 0]
z = 0.0
innovation = 0.0 - x[1]
R = R_ZUPT
```

| Parameter | Value | Unit | Description |
|-----------|-------|------|-------------|
| `R_ZUPT` | 6.15e-06 | (m/s)^2 | Measurement noise |
| `EKF_ZUPT_THRESHOLD` | 0.3 | m/s^2 | Stationarity threshold (in casper_ekf.h) |

**Stationarity detection (in flight_loop.c):**

```c
float accel_mag = |accel| in m/s^2;
bool stationary = fabsf(accel_mag - G_ACCEL) < EKF_ZUPT_THRESHOLD;
```

The threshold of 0.3 m/s^2 is tuned for the LSM6DSO32 at +/-32g range, which has a relatively high noise floor.

**Flight-phase guard:** ZUPT is only active when the FSM is in `FSM_STATE_PAD` or `FSM_STATE_LANDED`. It is disabled during all flight states (BOOST through RECOVERY/TUMBLE). Without this guard, high-altitude descent with thin air and near-1g deceleration can false-trigger ZUPT, slamming velocity to zero mid-flight.

**Gating:** ZUPT passes `INFINITY` for `gate_k2` in the Joseph-form update, meaning it is never rejected by the innovation gate.

**Rate:** ZUPT fires at the EKF predict rate (416 Hz) when active.

**Why ZUPT matters:** ZUPT directly constrains velocity to zero and, through the P cross-terms, also corrects altitude and accel bias. This eliminates velocity drift on the pad and prevents altitude random walk from accumulating before launch.

### 2.9 Baro Bias Covariance Floor

After each baro measurement update, the baro bias variance is clamped to a minimum:

```c
if (P[3][3] < P_FLOOR_BARO_BIAS)
    P[3][3] = P_FLOOR_BARO_BIAS;
```

| Parameter | Value | Description |
|-----------|-------|-------------|
| `P_FLOOR_BARO_BIAS` | 0.01 | m^2 (sigma >= 0.1 m) |

**Why:** Without this floor, the baro bias variance collapses to near zero after convergence. When a large baro disturbance occurs (e.g., pressure port transient), the filter assigns near-zero Kalman gain to the bias state and pushes the innovation into velocity instead, causing divergence. The floor keeps the filter adaptable to bias shifts.

Only `P[3][3]` has a floor. No other diagonal element is clamped.

### 2.10 Transonic Barometer Gating

During transonic flight, barometric pressure readings are unreliable due to shockwave effects. The EKF gates (disables) baro updates using estimated Mach number:

```
mach = |velocity| / speed_of_sound(altitude)
speed_of_sound = sqrt(1.4 * 287.058 * T_K)
T_K = 288.15 - 0.0065 * alt_clamp
```

**Temperature floor clamp:** `T_K` is clamped to a minimum of 216.65 K (tropopause temperature). Without this, `T_K` goes negative at high altitudes, producing NaN from the square root.

```c
float alt_clamp = ekf->x[0] > 0.0f ? ekf->x[0] : 0.0f;
float T_K = 288.15f - 0.0065f * alt_clamp;
if (T_K < 216.65f) T_K = 216.65f;
```

| Parameter | Value | Description |
|-----------|-------|-------------|
| `MACH_GATE_ON` | 0.40 | Gate baro above Mach 0.4 |
| `MACH_GATE_OFF` | 0.35 | Un-gate below Mach 0.35 |

Hysteresis prevents chattering near the threshold. While gated, the EKF relies solely on the IMU (dead-reckoning) for altitude/velocity.

The Mach gate state is updated inside `casper_ekf_predict()` every predict cycle. The `baro_gated` flag is checked at the top of `casper_ekf_update_baro()`.

### 2.11 Bias Reset and P Inflation at Un-Gate

When the Mach gate opens (transition from gated to ungated), the EKF has been dead-reckoning on IMU alone. Both bias states may have drifted to non-physical values. The filter resets them to give freedom for reconvergence.

**Triggered once, on the gate-open edge:**

```c
/* Detect gate-open transition */
if (ekf->baro_prev_gated) {
    ekf->baro_prev_gated = false;
    ekf->ungate_count = 0;

    /* Reset bias states */
    ekf->x[2] = 0.0f;   /* accel bias */
    ekf->x[3] = 0.0f;   /* baro bias  */

    /* Inflate P diagonals for bias states */
    ekf->P[2][2] = P_UNGATE_ACCEL_BIAS;  /* 1.0 (m/s^2)^2 */
    ekf->P[3][3] = P_UNGATE_BARO_BIAS;   /* 10.0 m^2      */

    /* Zero cross-covariance for rows/cols 2 and 3 */
    for (j = 0..3):
        if j != 2: P[2][j] = P[j][2] = 0
        if j != 3: P[3][j] = P[j][3] = 0
}
```

| Parameter | Value | Unit | Description |
|-----------|-------|------|-------------|
| `P_UNGATE_ACCEL_BIAS` | 1.0 | (m/s^2)^2 | Inflated accel bias variance |
| `P_UNGATE_BARO_BIAS` | 10.0 | m^2 | Inflated baro bias variance |

### 2.12 Step-Inflated R Post Un-Gate

After the gate opens, baro measurement noise is inflated for a fixed number of updates, then returns to nominal. This is a **step function**, not an exponential decay.

```
if ungate_count < N_UNGATE_STEPS:
    R = R_BARO_UNGATE       (50.0 m^2)
    ungate_count++
else:
    R = R_BARO               (0.5 m^2)
```

| Parameter | Value | Unit | Description |
|-----------|-------|------|-------------|
| `R_BARO_UNGATE` | 50.0 | m^2 | Inflated R for first N updates post-gate |
| `N_UNGATE_STEPS` | 10 | updates | Number of updates at inflated R |

After 10 updates at R = 50.0, the filter switches directly to nominal R = 0.5. The inflated R prevents the state from snapping violently when baro re-enters with a potentially large innovation after a long dead-reckoning period. The bias reset (Section 2.11) gives the filter freedom to move; the inflated R prevents it from moving too fast.

### 2.13 GPS Measurement Updates (STUBS)

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

These will use the same `joseph_scalar_update()` function as baro and ZUPT.

### 2.14 Initial Covariance P0

| State | P0 | Sigma | Unit |
|-------|-----|-------|------|
| Altitude | 0.1 | 0.32 m | m^2 |
| Velocity | 0.001 | 0.032 m/s | (m/s)^2 |
| Accel bias | 0.025 | 0.158 m/s^2 | (m/s^2)^2 |
| Baro bias | 0.75 | 0.87 m | m^2 |

### 2.15 Timing

| Parameter | Value |
|-----------|-------|
| `EKF_DT` | 0.0024 s (416 Hz) |
| Predict rate | Every 2nd IMU sample (416 Hz) |
| Baro update rate | ~100 Hz (non-blocking ms5611_tick, OSR_1024) |
| ZUPT rate | Every predict step (416 Hz, when stationary + PAD/LANDED) |
| GPS update rate | 10 Hz (when implemented) |

The predict rate was reduced from 833 Hz to 416 Hz to halve CPU load on the covariance propagation (three 4x4 CMSIS-DSP matrix multiplies per predict). The attitude estimator still runs at 833 Hz. See Section 6.2 for the NED accumulation scheme that preserves high-rate attitude information.

### 2.16 CMSIS-DSP Usage

The EKF uses ARM CMSIS-DSP for 4x4 matrix operations:

- `arm_mat_init_f32()` -- Initialize matrix instances pointing to float arrays
- `arm_mat_mult_f32()` -- Matrix multiplication (Phi*P, result*PhiT, Joseph form)
- `arm_mat_add_f32()` -- Matrix addition (+ Q)
- `arm_matrix_instance_f32` -- Matrix descriptor struct

All matrices are 4x4 row-major float arrays (16 elements). Working memory (`tmp_a`, `tmp_b`) is pre-allocated in the struct to avoid dynamic allocation. The Joseph-form update reuses the same working arrays.

### 2.17 Complete Tuning Parameter Summary

| Parameter | Symbol | Value | Unit |
|-----------|--------|-------|------|
| Gravity constant | `G_ACCEL` | 9.80665 | m/s^2 |
| Predict timestep | `EKF_DT` | 0.0024 | s |
| P0 altitude | `P0_ALT` | 0.1 | m^2 |
| P0 velocity | `P0_VEL` | 0.001 | (m/s)^2 |
| P0 accel bias | `P0_ACCEL_BIAS` | 0.025 | (m/s^2)^2 |
| P0 baro bias | `P0_BARO_BIAS` | 0.75 | m^2 |
| Accel VRW | `ACCEL_VRW` | 2.162545e-03 | m/s/sqrt(s) |
| Accel bias instability | `ACCEL_BI_SIGMA` | 1.953783e-04 | m/s^2/sqrt(s) |
| Baro bias instability | `BARO_BI_SIGMA` | 1.0e-03 | m/sqrt(s) |
| Baro measurement noise | `R_BARO` | 0.5 | m^2 |
| ZUPT measurement noise | `R_ZUPT` | 6.15e-06 | (m/s)^2 |
| ZUPT stationarity threshold | `EKF_ZUPT_THRESHOLD` | 0.3 | m/s^2 |
| Innovation gate threshold | `BARO_GATE_K2` | 25.0 | (5-sigma)^2 |
| Baro bias P floor | `P_FLOOR_BARO_BIAS` | 0.01 | m^2 |
| Mach gate on | `MACH_GATE_ON` | 0.40 | Mach |
| Mach gate off | `MACH_GATE_OFF` | 0.35 | Mach |
| Un-gate inflated R | `R_BARO_UNGATE` | 50.0 | m^2 |
| Un-gate step count | `N_UNGATE_STEPS` | 10 | updates |
| Un-gate P accel bias | `P_UNGATE_ACCEL_BIAS` | 1.0 | (m/s^2)^2 |
| Un-gate P baro bias | `P_UNGATE_BARO_BIAS` | 10.0 | m^2 |

---

## 3. Attitude Estimator -- `casper_attitude`

The attitude estimator produces a body-to-NED quaternion for the EKF's coordinate rotation and for telemetry.

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

**Note:** `Kp_mag_pad` and `Kp_mag_flight` are currently set to 0.0 (magnetometer corrections disabled for bench testing, pending field calibration).

### 3.2 Phases

The estimator operates in three phases:

```
Power-on -> [Static Init] -> [Pad Phase] -> [Flight Phase]
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

1. **Gyro bias frozen** -- no further estimation
2. **Integral error reset** at launch transition
3. **10 Hz mag correction** (timer-decimated):
   - Same cross-product error computation as pad phase
   - Uses `Kp_mag_flight` gain (lower than pad)
   - **Ignition gating:** Mag corrections disabled during motor burn windows (configurable via `casper_att_add_gate()`, up to 4 gates)
   - Timer only resets on successful correction -- if mag data is NULL or gated, correction fires next tick with valid data
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

## 4. Gyro Integrator -- `casper_gyro_int` (Legacy)

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

## 5. Quaternion Library -- `casper_quat`

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

## 6. Flight Loop Integration

The navigation stack runs in `flight_loop_tick()` ([flight_loop.c](Software/App/flight/flight_loop.c)), called from the main superloop when USB_MODE=1. Sensor globals (baro, imu, ekf, att, gps, mag) are defined in `main.c` and accessed via extern declarations in [app_globals.h](Software/App/flight/app_globals.h).

### 6.1 Execution Flow

```
flight_loop_tick():
1. Poll magnetometer at 100 Hz: mmc5983ma_read(), negate + calibrate
2. IMU interrupt watchdog: poll STATUS if EXTI hasn't fired in 5 ms
3. If new IMU data (833 Hz):
   a. Read accel + gyro, apply gyro temp compensation (if GYRO_TEMP_COMP)
   b. Identity map: body[X,Y,Z] = sensor[X,Y,Z]  (no axis permutation)
   c. If static init not complete:
      -> casper_att_static_init(accel, mag)
   d. Else if calibrating (< 30 s):
      -> casper_att_update(gyro, accel, mag, dt)
      -> accumulate baro for ground reference
   e. Else (cal done, flight-ready):
      -> casper_att_update(gyro, accel, mag, dt)   [833 Hz]
      -> rotate accel to NED: R(q) * accel         [833 Hz]
      -> trapezoidal accumulation (see 6.2)
      -> casper_ekf_predict(ned_avg, dt*2)          [416 Hz]
      -> casper_ekf_update_zupt() if PAD/LANDED + stationary  [416 Hz]
4. If ms5611_tick() returns new baro data:
   -> casper_ekf_update_baro(baro_alt - baro_ref)
5. If max_m10m_tick() returns new GPS fix with 3D lock:
   -> casper_ekf_update_gps_alt/vel (stubs, not yet active)
6. pyro_mgr_tick()
7. Build telemetry state + FSM tick
8. TEST_MODE=1: COBS binary telemetry (10 Hz via tlm_tick)
   TEST_MODE=2: ASCII serial plotter output (50 Hz, Teleplot format)
```

### 6.2 416 Hz EKF Predict with NED Accumulation

The body-to-NED rotation runs at 833 Hz using the full-rate quaternion. Two consecutive NED-frame accel vectors are accumulated via trapezoidal averaging before calling predict:

```c
/* Every IMU sample (833 Hz): */
float ned_accel[3];
casper_quat_to_rotmat(att.q, R);
ned_accel[i] = R * accel;    /* body -> NED */

if (imu_subsample_count == 0) {
    ned_accel_accum = ned_accel;     /* store first sample */
    imu_subsample_count = 1;
} else {
    /* Trapezoidal average of 2 NED-frame samples */
    ned_avg[i] = 0.5 * (ned_accel_accum[i] + ned_accel[i]);
    casper_ekf_predict(&ekf, ned_avg, 2 * EKF_DT);

    /* ZUPT at predict rate */
    if (FSM == PAD || FSM == LANDED):
        if (|accel_mag - G| < EKF_ZUPT_THRESHOLD):
            casper_ekf_update_zupt(&ekf);

    imu_subsample_count = 0;
}
```

**Why trapezoidal in NED, not body:** The attitude (quaternion) changes between the two IMU samples. Averaging in body frame and then rotating would use a stale rotation matrix for the first sample. Trapezoidal averaging in NED preserves the high-rate attitude information where it matters most.

**`casper_ekf_predict()` signature:** The predict function receives NED-frame accel directly. It does not depend on `casper_attitude.h`. The rotation is the caller's responsibility.

```c
void casper_ekf_predict(casper_ekf_t *ekf,
                        const float ned_accel_ms2[3],
                        float dt);
```

### 6.3 Baro Reference

The barometer reference altitude (`baro_ref`) is computed as the average of all baro samples during the calibration period (0-30 s). If no samples were accumulated (unlikely), it falls back to a single `ms5611_get_altitude()` call:

```c
if (baro_cal_count > 0)
    baro_ref = (float)(baro_cal_sum / (double)baro_cal_count);
else
    baro_ref = ms5611_get_altitude(&baro, 1013.25f);
```

All subsequent baro updates use `baro_alt - baro_ref` to provide altitude AGL (above ground level).

### 6.4 TEST_MODE=2 Diagnostic Output

When built with `TEST_MODE=2`, the flight loop outputs Teleplot-format ASCII at 50 Hz over USB CDC:

```
>alt:%.2f,vel:%.2f,ab:%.4f,bb:%.2f,baro:%.2f,r:%.1f,p:%.1f,y:%.1f,fsm:%d,nz:%.3f,zupt:%d
```

The `nz` field is the latest NED-Z accel (useful for verifying sign convention and gravity subtraction). The `zupt` field is 1 if ZUPT fired on the most recent predict cycle, 0 otherwise.

---

## 7. Known Limitations

1. **GPS update not implemented** -- The EKF stubs exist but GPS altitude/velocity updates are not yet functional. The filter relies solely on IMU + barometer.

2. **No horizontal state estimation** -- The EKF is vertical-only (altitude + velocity). Horizontal position uses raw GPS deltas, not a filtered estimate.

3. **Single-frequency barometer** -- The MS5611 operates at a single OSR. No multi-rate fusion is implemented.

4. **Euler gimbal lock** -- The Euler angle extraction has a singularity at pitch = +/-90 degrees (the pad condition). The quaternion itself has no singularity.

5. **Linear process model** -- The EKF uses a linearized (constant Phi) model. This is adequate for vertical flight but may need augmentation for highly dynamic maneuvers.

6. **MS5611 corrupt reads** -- Rare SPI read corruption can produce negative pressure values. Defense-in-depth: (a) pressure clamped to >= 0.01 hPa in the driver, (b) `isfinite()` guard in EKF update, (c) NaN-safe innovation gate logic. See Sections 2.6-2.7.

7. **Magnetometer corrections currently disabled** -- `Kp_mag_pad` and `Kp_mag_flight` are set to 0.0, pending field calibration of the MMC5983MA hard/soft iron model. The attitude estimator is running on gyro + gravity only.

8. **ZUPT threshold tuned for +/-32g noise floor** -- The `EKF_ZUPT_THRESHOLD` of 0.3 m/s^2 is relatively large because the LSM6DSO32 at +/-32g range has a coarse LSB (~0.97 mg). A lower-range accelerometer would permit a tighter threshold.
