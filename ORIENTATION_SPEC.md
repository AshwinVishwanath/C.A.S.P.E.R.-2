# ORIENTATION_SPEC.md — Quaternion Encoding & Frame Conventions

**Ground-truth reference for the MC Electron app.**
Describes exactly how the FC firmware represents orientation, maps sensor axes to the vehicle body frame, encodes the quaternion into telemetry, and how the MC must decode it for accurate 3D visualization.

---

## 1. Coordinate Frame Definitions

### 1.1 NED (North-East-Down) — The World Frame

The FC uses the **NED** (North-East-Down) frame as its inertial reference:

```
       N (North, +X)
       ^
       |
       |
 (West)-----> E (East, +Y)
       |
       v
       D (Down, +Z)
```

- **+X** = Geographic North
- **+Y** = Geographic East
- **+Z** = Down (toward center of Earth)
- Gravity vector in NED = `[0, 0, +g]` (points down)
- Magnetic North is in the +X direction (at zero declination)

### 1.2 Vehicle Body Frame

The vehicle body frame is fixed to the rocket airframe. **+Y is the thrust/nose axis:**

```
         +Y (nose, thrust axis, UP on pad)
         ^
         |
         |
         +-------> +X (starboard)
        /
       v
      +Z (toward operator, completes RH frame)
```

- **+X** = Starboard (right side when looking from behind the rocket toward the nose)
- **+Y** = Nose of the rocket (thrust axis, pointing UP when on the pad)
- **+Z** = Toward operator (completes the right-hand coordinate system)
- On the pad: body +Y points up (against gravity), body -Y points toward ground

The body frame is identical to the IMU sensor frame (identity mapping, no axis permutation).

### 1.3 LSM6DSO32 IMU Sensor Frame (Physical Mounting)

The LSM6DSO32 is mounted on the PCB with:

- **Sensor +Y** = toward nose (UP on pad)
- **Sensor +X** = starboard
- **Sensor +Z** = toward operator (forward face of rocket)

Raw driver outputs:
- `imu.accel_g[0..2]` = Sensor X, Y, Z acceleration in g
- `imu.gyro_dps[0..2]` = Sensor X, Y, Z angular rate in deg/s

### 1.4 ADXL372 High-G Accelerometer Sensor Frame

Same orientation as the LSM6DSO32:

- **Sensor +Y** = toward nose (UP on pad)
- **Sensor +X** = starboard

### 1.5 MMC5983MA Magnetometer Sensor Frame (Physical Mounting)

The magnetometer is mounted with OPPOSITE polarity to the IMU:

- **Sensor -Y** = toward nose (UP on pad) — i.e., sensor +Y points AWAY from nose
- **Sensor -X** = starboard — i.e., sensor +X points PORT (left)

This is why the firmware negates all three mag axes before use (Section 2.2).

---

## 2. Sensor-to-Body Frame Mapping

### 2.1 IMU (LSM6DSO32) Mapping

The sensor frame and vehicle body frame are identical — no axis permutation is needed:

```
Vehicle Body X  =  Sensor X  (starboard)
Vehicle Body Y  =  Sensor Y  (nose, up on pad)
Vehicle Body Z  =  Sensor Z  (toward operator)
```

In code (`flight_loop.c`):

```c
/* Accel: sensor g → body m/s² */
float accel_ms2[3] = {
    imu.accel_g[0] * 9.80665,   /* Body X = Sensor X (starboard)       */
    imu.accel_g[1] * 9.80665,   /* Body Y = Sensor Y (nose, up on pad) */
    imu.accel_g[2] * 9.80665    /* Body Z = Sensor Z (toward operator) */
};

/* Gyro: sensor deg/s → body rad/s */
float gyro_rads[3] = {
    gyro_sensor[0],              /* Body X = Sensor X (starboard)       */
    gyro_sensor[1],              /* Body Y = Sensor Y (nose)            */
    gyro_sensor[2]               /* Body Z = Sensor Z (toward operator) */
};
```

This is an identity mapping (no axis permutation needed).

### 2.2 Magnetometer (MMC5983MA) Mapping

The magnetometer axes are opposite to the IMU, so the firmware negates all three axes to align them with the IMU sensor frame before applying calibration:

```c
/* Negate to align mag with IMU sensor frame, then apply hard/soft-iron cal */
float mag_raw[3] = {-mag.mag_ut[0], -mag.mag_ut[1], -mag.mag_ut[2]};
mag_cal_apply(mag_raw, mag_cal_ut);
```

After negation:
- `mag_raw[0]` = `-Mag_X` = starboard (matches IMU sensor X)
- `mag_raw[1]` = `-Mag_Y` = nose/up (matches IMU sensor Y)
- `mag_raw[2]` = `-Mag_Z`

The `mag_cal_apply()` function applies hard-iron offset subtraction and soft-iron matrix correction. The magnetometer data (after negation + calibration) is in the sensor/body frame (they are identical). No separate remap step is needed.

### 2.3 What the MC Sees

The MC never sees raw sensor data — it only sees the quaternion. The key facts for MC:

- The quaternion rotates from **vehicle body frame** to **NED**
- Vehicle body **+Y = nose** (thrust axis, UP on the pad)
- Vehicle body **+X = starboard**
- If the MC's 3D model has a different nose axis, apply a fixed rotation to the model mesh

---

## 3. The Quaternion

### 3.1 Convention

| Property | Value |
|----------|-------|
| **Format** | `q[4] = [w, x, y, z]` (scalar-first) |
| **Product** | Hamilton (standard, non-commutative) |
| **Rotation** | Body-to-NED |
| **Identity** | `[1, 0, 0, 0]` = body frame aligned with NED (never happens on pad) |

**Body-to-NED** means: given a vector `v_body` in the body frame, the corresponding vector in the NED frame is:

```
v_ned = q ⊗ [0, v_body] ⊗ q*
```

Or equivalently, using the rotation matrix `R = quat_to_rotmat(q)`:

```
v_ned = R × v_body
```

### 3.2 Rotation Matrix from Quaternion

The rotation matrix `R` (body-to-NED, row-major) is constructed as:

```
R = | 1-2(y²+z²)   2(xy-wz)    2(xz+wy)  |
    | 2(xy+wz)    1-2(x²+z²)   2(yz-wx)   |
    | 2(xz-wy)    2(yz+wx)    1-2(x²+y²)  |
```

Where `w = q[0], x = q[1], y = q[2], z = q[3]`.

### 3.3 On the Pad

When the rocket is sitting vertical on the pad with the nose pointing up:

- Body +Y (nose) points UP = NED **-Z** direction (NED Z is down, up = -Z)
- Body -Y (tail) points DOWN = NED +Z direction
- Accelerometer reads reaction to gravity in body frame = `[0, +g, 0]` (positive along nose/up direction)

The **pad quaternion is NOT identity**. It must encode the rotation from a frame where +Y = nose (up) to NED where +Z = down. This is approximately a **-90° rotation about the starboard axis (body X)** (plus yaw for heading).

The firmware computes the exact pad quaternion from:
- **Pitch and roll** from the averaged gravity vector (accelerometer)
- **Yaw/heading** from the tilt-compensated magnetometer

On a perfectly vertical rocket pointing North:
- The body Y-axis (nose) must map to NED -Z (up)
- The body X-axis (starboard) must map to NED +Y (east) if heading is North

### 3.4 Hamilton Product

For reference, the Hamilton quaternion product `r = a ⊗ b` is:

```
r.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z
r.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y
r.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x
r.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
```

---

## 4. Attitude Estimator Pipeline

### 4.1 Static Initialization (0–5 seconds)

1. Accumulate 500 magnetometer samples + accel samples (takes ~5s at 100 Hz mag rate)
2. Average all samples to get mean accel and mean mag vectors
3. Compute roll and pitch from gravity (using body-frame accel):
   ```
   pitch = atan2(-ax, sqrt(ay² + az²))
   roll  = atan2(ay, az)
   ```
   Where `ax, ay, az` are the averaged body-frame accelerometer values.
4. Compute tilt-compensated magnetic heading:
   ```
   mx_h = mx·cos(pitch) + my·sin(roll)·sin(pitch) + mz·cos(roll)·sin(pitch)
   my_h = my·cos(roll) - mz·sin(roll)
   yaw  = atan2(-my_h, mx_h)
   ```
5. Build initial quaternion from ZYX Euler angles `(roll, pitch, yaw)`
6. Store magnetometer reference vector in NED frame: `m_ref_ned = R × mag_avg`

**Fallback:** If 10 seconds pass without 500 mag samples, initialize from gravity only (yaw = 0).

### 4.2 Complementary Filter — Pad Phase (5–30 seconds)

Mahony-style complementary filter running at 833 Hz:

1. **Gravity correction:** `e_grav = cross(accel_hat, R' × [0,0,1])`
2. **Mag correction:** `e_mag = cross(mag_hat, R' × m_ref_ned_hat)`
3. **Integral accumulation:** `e_int += (e_grav + e_mag) × dt`
4. **Corrected gyro:** `omega = gyro_filtered - gyro_bias + Kp_grav·e_grav + Kp_mag·e_mag + Ki·e_int`
5. **RK4 quaternion propagation** with corrected omega
6. **Gyro bias estimation:** Running average (frozen at transition to flight)

This phase converges the gyro bias estimate and refines the initial quaternion.

### 4.3 Flight Phase (after 30-second init)

At the 30s mark, the firmware transitions to flight mode:
- `att.launched = true`
- Integral error reset to zero
- Gyro bias frozen (no further estimation)

Flight update (833 Hz):

1. **Gyro LPF:** First-order IIR, 50 Hz cutoff
2. **Bias subtraction:** `omega = gyro_filtered - gyro_bias`
3. **10 Hz mag correction** (timer-decimated, with ignition gating):
   - Same cross-product error as pad, but lower gain (`Kp_mag_flight`)
   - Disabled during motor burns via configurable time windows
4. **RK4 quaternion integration** with corrected omega
5. **Uncertainty tracking:** Heading sigma grows with gyro ARW, shrinks with mag corrections

### 4.4 RK4 Quaternion Propagation

The quaternion derivative is:

```
qdot = 0.5 × q ⊗ [0, ωx, ωy, ωz]
```

Four-stage Runge-Kutta:
```
k1 = qdot(q, ω)
k2 = qdot(q + k1·dt/2, ω)
k3 = qdot(q + k2·dt/2, ω)
k4 = qdot(q + k3·dt, ω)
q_new = q + (dt/6)·(k1 + 2·k2 + 2·k3 + k4)
normalize(q_new)
```

---

## 5. Telemetry Quaternion Encoding

### 5.1 Smallest-Three Compression

The unit quaternion `[w, x, y, z]` has the constraint `w² + x² + y² + z² = 1`, so only 3 of the 4 components need to be transmitted. The fourth is reconstructed from the unit-norm constraint.

**Encoding steps (firmware `quat_pack.c`):**

1. **Find the largest component** by absolute value. Call its index `drop` (0=w, 1=x, 2=y, 3=z).

2. **If the dropped component is negative, negate the entire quaternion.** Since `q` and `-q` represent the same rotation, this ensures the dropped component is always positive, so the decoder can reconstruct it with `sqrt(...)` without a sign ambiguity.

3. **Extract the three remaining components** in ascending index order. Call them A, B, C.

   | drop | A | B | C |
   |------|---|---|---|
   | 0 (w dropped) | x | y | z |
   | 1 (x dropped) | w | y | z |
   | 2 (y dropped) | w | x | z |
   | 3 (z dropped) | w | x | y |

4. **Quantize to 12-bit signed integers:**

   ```
   A_raw = round(A × 4096.0)    clamped to [-2048, 2047]
   B_raw = round(B × 4096.0)    clamped to [-2048, 2047]
   C_raw = round(C × 4096.0)    clamped to [-2048, 2047]
   ```

   > **IMPORTANT:** The scale factor is **4096.0**, NOT 2895.27 as written in INTERFACE_SPEC.md §5.5.
   > The INTERFACE_SPEC value is incorrect. This document is the ground truth.

5. **Mask to 12 bits** (two's complement): `A_u12 = A_raw & 0x0FFF`, etc.

### 5.2 Byte Packing (Little-Endian)

The 40-bit payload is packed into 5 bytes in **little-endian** order (byte 0 = LSB, byte 4 = MSB):

```
Byte 0:  C[7:0]                           (low 8 bits of C)
Byte 1:  B[3:0] | C[11:8]                 (low 4 of B, high 4 of C)
Byte 2:  B[11:4]                           (high 8 bits of B)
Byte 3:  A[7:0]                            (low 8 bits of A)
Byte 4:  drop[1:0] | rsvd[1:0] | A[11:8]  (drop index, 2 reserved bits, high 4 of A)
```

> **NOTE:** This is the REVERSE byte order from INTERFACE_SPEC.md §5.5 which shows byte 0
> as the drop-index byte. The firmware transmits **LSB first** (byte 0 = C low bits).
> This document is the ground truth.

Visually, the 40 bits from MSB to LSB:

```
bit 39    bit 0
[dd][rr][AAAA][AAAAAAAA][BBBBBBBB][BBBB][CCCC][CCCCCCCC]
 ↑    ↑    ↑       ↑        ↑       ↑     ↑       ↑
drop rsvd A[11:8] A[7:0]  B[11:4] B[3:0] C[11:8] C[7:0]
byte 4              byte 3  byte 2   byte 1        byte 0
```

### 5.3 Position in FC_MSG_FAST Packet

The quaternion occupies bytes 7–11 of the 20-byte FC_MSG_FAST packet:

```
Offset  Field           Size    Encoding
──────  ──────────────  ──────  ────────────────────────
0       msg_id          1       0x01
1–2     status          2       FC_TLM_STATUS bitmap
3–4     altitude        2       u16 LE, metres (×1.0)
5–6     velocity        2       i16 LE, dm/s (×0.1)
7–11    quaternion       5       Smallest-three (see §5.2)
12–13   flight_time     2       u16 LE, deciseconds (×0.1)
14      battery         1       u8, 6.0 + raw×0.012 volts
15      seq             1       u8, wrapping counter
16–19   crc32           4       CRC-32 over bytes [0–15]
```

---

## 6. MC Decoding — Step by Step

### 6.1 Extract Packed Fields from Bytes 7–11

Given 5 bytes `b[0]..b[4]` from the quaternion field:

```javascript
// Extract 12-bit unsigned values
const C_u12 = (b[0]) | ((b[1] & 0x0F) << 8);
const B_u12 = ((b[1] >> 4) & 0x0F) | (b[2] << 4);
const A_u12 = (b[3]) | ((b[4] & 0x0F) << 8);
const drop  = (b[4] >> 6) & 0x03;
```

### 6.2 Sign-Extend 12-Bit to Signed Integer

Each 12-bit value is two's complement. Sign-extend to a standard integer:

```javascript
function signExtend12(val) {
    return (val >= 2048) ? (val - 4096) : val;
}

const A_i12 = signExtend12(A_u12);
const B_i12 = signExtend12(B_u12);
const C_i12 = signExtend12(C_u12);
```

### 6.3 Convert to Float

```javascript
const QUAT_SCALE = 4096.0;

const A = A_i12 / QUAT_SCALE;
const B = B_i12 / QUAT_SCALE;
const C = C_i12 / QUAT_SCALE;
```

### 6.4 Reconstruct Full Quaternion

```javascript
// Dropped component is always positive (encoder negated q if needed)
const dropped = Math.sqrt(Math.max(0, 1.0 - A*A - B*B - C*C));

// Place components back by drop index
// A, B, C are in ascending index order (skipping 'drop')
let w, x, y, z;
switch (drop) {
    case 0: w = dropped; x = A; y = B; z = C; break;
    case 1: w = A; x = dropped; y = B; z = C; break;
    case 2: w = A; x = B; y = dropped; z = C; break;
    case 3: w = A; x = B; y = C; z = dropped; break;
}
```

The result is a unit quaternion `[w, x, y, z]` representing the **body-to-NED** rotation.

### 6.5 Extract Euler Angles (ZYX Convention)

These are the ZYX intrinsic Euler angles as computed by the firmware. Because the body frame has **Y = nose** and **X = starboard**, the physical meaning of each angle is described below.

```javascript
// "Roll" — rotation about body X (starboard)
const sinr = 2.0 * (w*x + y*z);
const cosr = 1.0 - 2.0 * (x*x + y*y);
const euler_x_deg = Math.atan2(sinr, cosr) * (180 / Math.PI);

// "Pitch" — rotation about body Y (nose/thrust axis)
let sinp = 2.0 * (w*y - z*x);
sinp = Math.max(-1, Math.min(1, sinp));
const euler_y_deg = Math.asin(sinp) * (180 / Math.PI);

// "Yaw" — rotation about body Z (toward operator)
const siny = 2.0 * (w*z + x*y);
const cosy = 1.0 - 2.0 * (y*y + z*z);
const euler_z_deg = Math.atan2(siny, cosy) * (180 / Math.PI);
```

**Physical meaning of each angle for a rocket (Y = nose, X = starboard):**

| Firmware Name | Body Axis | Physical Meaning (Rocket) | On Pad (vertical, heading North) |
|---------------|-----------|--------------------------|----------------------------------|
| euler_x | X (starboard) | Nose up/down tilt (aerospace pitch) | ~±90° (gimbal lock) |
| euler_y | Y (nose) | Spin about thrust axis (aerospace roll) | ~0° |
| euler_z | Z (toward operator) | Heading/yaw | ~heading |

> **Mapping to standard aerospace terms:**
> - **Aerospace pitch** (nose above/below horizon) ≈ `euler_x`
> - **Aerospace roll** (spin about longitudinal/nose axis) ≈ `euler_y`
> - **Aerospace yaw** (heading change) ≈ `euler_z`
>
> For a 3D visualization, **use the quaternion directly** (Section 7) rather than
> Euler angles. The Euler decomposition is only useful for simple numeric displays.

---

## 7. MC 3D Visualization Guide

### 7.1 The Quaternion is Body-to-NED

The quaternion `q = [w, x, y, z]` rotates vectors FROM the body frame INTO NED:

```
v_ned = R(q) × v_body
```

For a 3D visualization, you need to orient a rocket model in your scene's coordinate system using this quaternion.

### 7.2 Applying to a Three.js Scene (Y-Up)

Three.js uses a **Y-up, right-handed** coordinate system. NED is Z-down. You need a frame conversion.

**NED to Three.js mapping:**
```
Three.js X =  NED Y (East)
Three.js Y = -NED Z (Up = -Down)
Three.js Z =  NED X (North)
```

**Step-by-step:**

1. Decode quaternion `[w, x, y, z]` as described in Section 6.

2. Convert the body-to-NED quaternion to a Three.js quaternion. The recommended approach is to build the rotation matrix from the quaternion, apply the NED→Three.js axis swap to the matrix, and set the model's quaternion from that matrix. Alternatively, the component-swap shortcut:

   ```javascript
   // Body-to-NED quaternion from FC
   // q_bn = [w, x, y, z]

   // Convert to Three.js coordinate system
   // NED (X=N, Y=E, Z=D) → Three.js (X=E, Y=Up, Z=N)
   // This swaps: NED_x→TJS_z, NED_y→TJS_x, NED_z→-TJS_y
   const q_display = new THREE.Quaternion(
       y,     // Three.js qx ← NED qy
       -z,    // Three.js qy ← -NED qz
       x,     // Three.js qz ← NED qx
       w      // scalar unchanged
   );
   ```

3. **Model mesh alignment:** Your rocket 3D model's mesh must have its nose pointing along **+Y in the body frame** (since body +Y = nose). If your model was built with +Z as nose (common in some 3D modeling tools), pre-rotate the mesh geometry by +90° about X to align it before applying the telemetry quaternion.

4. **Verify with pad condition:**
   - On the pad, the rocket nose (+Y body) points UP in the scene
   - Heading should match: if heading is North, the nose-up rocket should face the Three.js +Z direction (which = NED North)

### 7.3 2D Gauges

For simple 2D attitude displays:

| Gauge | Value | Range | On Pad (vertical) |
|-------|-------|-------|-----|
| Pitch (nose up/down) | `euler_x` | -90° to +90° | ~±90° (gimbal lock) |
| Roll (spin about nose) | `euler_y` | -180° to +180° | ~0° |
| Heading/yaw | `euler_z` | -180° to +180° | ~heading |

> **Gimbal lock warning:** When the rocket is vertical (nose straight up or down),
> `euler_x ≈ ±90°` and `euler_y` / `euler_z` become degenerate. This is inherent
> to Euler angles — the quaternion itself is always well-defined.
> Use the quaternion for 3D display; use Euler angles only for numeric readouts.

---

## 8. Precision & Limitations

### 8.1 Quantization Error

Each component is quantized to 12 bits with scale 4096:
- **Resolution:** `1/4096 ≈ 0.000244` per LSB
- **Angular resolution:** ~0.028° (0.5 milliradians) for small rotations

### 8.2 Range Clipping

The 12-bit signed range is [-2048, 2047], representing component values [-0.500, 0.4998] after dividing by 4096. However, the three remaining components of a unit quaternion can each be up to `1/√2 ≈ 0.707` when the dropped component is exactly equal to another.

Components in the range (0.500, 0.707] will be **clipped to 0.4998**. In practice this causes a worst-case angular error of ~5° at the specific orientations where two quaternion components are equal in magnitude (45° rotations about certain axes). For most flight orientations this clipping does not occur.

### 8.3 Gimbal Lock

The Euler angle extraction has a singularity when `euler_x = ±90°` (which is the **pad condition** for a vertical rocket). At exactly ±90°, `euler_y` and `euler_z` become degenerate. The firmware clamps `sin(pitch)` to [-1, 1] to prevent NaN. **The quaternion itself has no singularity** — only the Euler angle representation does. For 3D visualization, always use the quaternion directly (Section 7.2), not Euler angles.

---

## 9. INTERFACE_SPEC.md Errata

This document supersedes INTERFACE_SPEC.md §5.5 for the following:

| Field | INTERFACE_SPEC Value | Actual FC Value | Impact |
|-------|---------------------|-----------------|--------|
| Scale factor | 2895.27 (`2047×√2`) | **4096.0** | MC must divide raw int12 by 4096, not 2895.27 |
| Byte order | Byte 0 = drop index (MSB first) | Byte 0 = C_lo (**LSB first**) | MC must parse bytes in little-endian order per §5.2 above |

Using the INTERFACE_SPEC values will produce **incorrect quaternion decoding** and broken 3D visualization.

---

## Appendix A: Quick Reference

**FC sends:** Body-to-NED quaternion `[w, x, y, z]`, Hamilton, scalar-first.

**Body frame:** +Y = nose (up on pad), +X = starboard, +Z = toward operator.

**Scale:** 4096.0 (NOT 2895.27).

**Byte order:** Little-endian (byte 0 = LSB = C_lo, byte 4 = MSB = drop index).

**On the pad:** Quaternion is NOT identity. Nose (body +Y) maps to NED -Z (up). Euler `euler_x ≈ ±90°` (gimbal lock — use quaternion for visualization, not Euler).
