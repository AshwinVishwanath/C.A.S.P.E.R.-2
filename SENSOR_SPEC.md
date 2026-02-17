# SENSOR_SPEC.md — Sensor Configuration & Driver Reference

**Ground-truth reference for all C.A.S.P.E.R.-2 sensor drivers.**
Covers register configuration, initialization sequences, data conversion, APIs, and bus assignments.

---

## 1. Sensor Inventory

| Sensor | Part | Measures | Bus | Bus Freq | Files |
|--------|------|----------|-----|----------|-------|
| IMU | LSM6DSO32 | Accel + Gyro | SPI2 | 5.25 MHz | `lsm6dso32.c/h` |
| High-G Accel | ADXL372 | +-200g Accel | SPI3 | TBD | `adxl372.c/h` |
| Barometer | MS5611 | Pressure + Temp | SPI4 | 13.5 MHz | `ms5611.c/h` |
| Flash | W25Q512JV | 64 MB NOR | QUADSPI | 54 MHz | `w25q512jv.c/h` |
| GPS | MAX-M10M | Position + Vel | I2C1 | 400 kHz | `max_m10m.c/h` |
| Magnetometer | MMC5983MA | 3-axis Mag | I2C3 | 400 kHz | `mmc5983ma.c/h` |

---

## 2. LSM6DSO32 — 6-axis IMU (SPI2)

### 2.1 Overview

| Parameter | Value |
|-----------|-------|
| Part | STMicroelectronics LSM6DSO32 |
| Accel range | +-32g |
| Gyro range | +-2000 dps |
| Accel ODR | 833 Hz |
| Gyro ODR | 833 Hz |
| WHO_AM_I | 0x6C |
| SPI mode | Mode 3 (CPOL=1, CPHA=1) |
| CS pin | PC14 |
| INT2 pin | PC15 (accel data-ready) |

### 2.2 Bus Configuration

| Parameter | Value |
|-----------|-------|
| SPI peripheral | SPI2 |
| SCK | PD3 |
| MOSI | PC1 |
| MISO | PC2_C |
| CS | PC14 (software, active low) |
| INT2 | PC15 (EXTI, rising edge) |
| Prescaler | 16 (84 MHz / 16 = 5.25 MHz) |
| Data size | 8-bit |
| CPOL | HIGH (Mode 3) |
| CPHA | 2nd edge (Mode 3) |

**Critical:** CubeMX defaults SPI2 to Mode 0 — must override to Mode 3 in USER CODE SPI2_Init 2. Also `MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE` and `NSSPMode = SPI_NSS_PULSE_DISABLE`.

### 2.3 Register Configuration

| Register | Address | Value | Description |
|----------|---------|-------|-------------|
| CTRL1_XL | 0x10 | 0x78 | Accel: 833 Hz ODR, +-32g FS |
| CTRL2_G | 0x11 | 0x7C | Gyro: 833 Hz ODR, +-2000 dps |
| CTRL3_C | 0x12 | 0x44 | BDU=1, IF_INC=1 (block data update + auto-increment) |
| INT2_CTRL | 0x0E | 0x01 | Accel data-ready on INT2 |

### 2.4 Data Format

**Accelerometer:**
- Raw: 16-bit signed, left-justified
- Sensitivity: 0.976 mg/LSB (+-32g range)
- Conversion: `accel_g = raw * 0.000976f`

**Gyroscope:**
- Raw: 16-bit signed, left-justified
- Sensitivity: 70 mdps/LSB (+-2000 dps range)
- Conversion: `gyro_dps = raw * 0.070f`

**Temperature:**
- Raw: 16-bit signed
- Sensitivity: 256 LSB/deg C
- Offset: 0 LSB = 25 deg C
- Conversion: `temp_c = 25.0f + raw / 256.0f`

### 2.5 Driver API

```c
bool lsm6dso32_init(lsm6dso32_t *dev, SPI_HandleTypeDef *hspi,
                     GPIO_TypeDef *cs_port, uint16_t cs_pin);
int  lsm6dso32_read(lsm6dso32_t *dev);      /* Reads accel_g + gyro_dps + temp_c */
int  lsm6dso32_read_raw(lsm6dso32_t *dev);  /* Reads raw int16 only */
void lsm6dso32_irq_handler(lsm6dso32_t *dev); /* Call from EXTI callback */
```

### 2.6 Interrupt-Driven Reads

The data-ready interrupt on INT2 (PC15) sets `dev->data_ready = true`. The main loop polls this flag and calls `lsm6dso32_read()` when set. This ensures synchronization with the sensor's 833 Hz ODR.

### 2.7 Axis Mapping

See ORIENTATION_SPEC.md for the full body frame mapping:
- Sensor Y = vehicle nose (up on pad)
- Sensor X = vehicle starboard
- Firmware remaps: `body[X,Y,Z] = sensor[Z,X,Y]`

---

## 3. ADXL372 — High-G Accelerometer (SPI3)

### 3.1 Overview

| Parameter | Value |
|-----------|-------|
| Part | Analog Devices ADXL372 |
| Range | +-200g |
| Resolution | 12-bit |
| Sensitivity | 100 mg/LSB |
| Max ODR | 6400 Hz |
| DEVID | 0xFA |
| ADI_DEVID | 0xAD |
| MST_DEVID | 0x1D |
| CS pin | PA15 |
| INT pin | PD2 |

### 3.2 Bus Configuration

| Parameter | Value |
|-----------|-------|
| SPI peripheral | SPI3 |
| SCK | PC10 |
| MOSI | PC12 |
| MISO | PC11 |
| CS | PA15 (software, active low) |
| INT | PD2 (EXTI) |

### 3.3 Register Configuration (Init Sequence)

1. Soft reset: Write `0x52` to SRESET (0x41), wait 10 ms
2. Verify DEVID (0x02) = 0xFA
3. Configure:

| Register | Address | Value | Description |
|----------|---------|-------|-------------|
| TIMING | 0x3D | ODR bits | Output data rate (400/800/1600/3200/6400 Hz) |
| MEASURE | 0x3E | BW + low noise | Bandwidth selection + low noise enable |
| POWER_CTL | 0x3F | 0x03 | Full bandwidth measurement mode |

### 3.4 FIFO Mode

For data collection, the ADXL372 is configured in FIFO streaming mode:

```c
void adxl372_fifo_init(adxl372_t *dev, uint8_t odr_bits);
uint16_t adxl372_fifo_entries(adxl372_t *dev);  /* Returns sample count */
int adxl372_fifo_read(adxl372_t *dev);          /* Read one XYZ triplet */
```

FIFO stores up to 512 samples (individual axis readings, so 170 XYZ triplets).

### 3.5 Data Format

- Raw: 12-bit signed, left-justified in 16-bit register pair
- Data registers: `X_DATA_H` (bits [11:4]), `X_DATA_L` (bits [3:0] in upper nibble)
- Sensitivity: 100 mg/LSB
- Conversion: `accel_g = raw / 10.0f` (where raw = sign-extended 12-bit value)

### 3.6 Driver API

```c
bool     adxl372_init(adxl372_t *dev, SPI_HandleTypeDef *hspi,
                      GPIO_TypeDef *cs_port, uint16_t cs_pin);
int      adxl372_read(adxl372_t *dev);           /* Single-shot XYZ read */
void     adxl372_fifo_init(adxl372_t *dev, uint8_t odr_bits);
uint16_t adxl372_fifo_entries(adxl372_t *dev);
int      adxl372_fifo_read(adxl372_t *dev);      /* One FIFO triplet */
```

---

## 4. MS5611 — Barometer (SPI4)

### 4.1 Overview

| Parameter | Value |
|-----------|-------|
| Part | TE Connectivity MS5611-01BA03 |
| Pressure range | 10-1200 mbar |
| Resolution | 24-bit ADC |
| Altitude resolution | ~0.012 m (OSR_4096) |
| CS pin | PE11 |

### 4.2 Bus Configuration

| Parameter | Value |
|-----------|-------|
| SPI peripheral | SPI4 |
| SCK | PE12 |
| MOSI | PE14 |
| MISO | PE13 |
| CS | PE11 (software, active low) |
| Prescaler | 8 (108 MHz / 8 = 13.5 MHz, max spec 20 MHz) |
| SPI mode | Mode 0 (CPOL=0, CPHA=0) |

### 4.3 SPI Commands

| Command | Opcode | Description |
|---------|--------|-------------|
| Reset | 0x1E | Software reset |
| Convert D1 | 0x40 + OSR*2 | Start pressure conversion |
| Convert D2 | 0x50 + OSR*2 | Start temperature conversion |
| Read ADC | 0x00 | Read 24-bit conversion result |
| Read PROM | 0xA0 + addr*2 | Read calibration coefficient (7 words) |

### 4.4 Oversampling Rates

| OSR | Enum | Conversion Time |
|-----|------|----------------|
| 256 | `MS5611_OSR_256` | 0.6 ms |
| 512 | `MS5611_OSR_512` | 1.2 ms |
| 1024 | `MS5611_OSR_1024` | 2.3 ms |
| 2048 | `MS5611_OSR_2048` | 4.6 ms |
| 4096 | `MS5611_OSR_4096` | 9.1 ms |

Default for navigation: OSR_1024 (~100 Hz effective rate with non-blocking tick).

### 4.5 Calibration

The MS5611 stores 7 factory calibration coefficients in PROM (128 bits). These are read at init and used in the temperature-compensated pressure calculation.

**Compensation algorithm** (2nd order, from datasheet):

```
dT    = D2 - C[5] * 256
TEMP  = 2000 + dT * C[6] / 8388608
OFF   = C[2] * 65536 + (C[4] * dT) / 128
SENS  = C[1] * 32768 + (C[3] * dT) / 256
P     = (D1 * SENS / 2097152 - OFF) / 32768
```

Second-order correction applied when TEMP < 2000 (below 20 deg C).

**Altitude formula (barometric/hypsometric):**

```
altitude_m = 44330.0 * (1.0 - pow(pressure_hPa / sea_level_hPa, 0.190284))
```

### 4.6 Non-Blocking State Machine

```
ms5611_tick() state machine:
  IDLE → start D1 conversion → CONVERTING_D1
  CONVERTING_D1 → (wait conversion time) → read D1, start D2 → CONVERTING_D2
  CONVERTING_D2 → (wait conversion time) → read D2, compute T+P → IDLE
  Returns 1 when new data available, 0 otherwise.
```

### 4.7 Driver API

```c
bool  ms5611_init(ms5611_t *dev, SPI_HandleTypeDef *hspi,
                  GPIO_TypeDef *cs_port, uint16_t cs_pin);
int   ms5611_read(ms5611_t *dev);              /* Blocking read */
int   ms5611_tick(ms5611_t *dev);              /* Non-blocking, returns 1 on new data */
float ms5611_get_temperature(const ms5611_t *dev);   /* deg C */
float ms5611_get_pressure(const ms5611_t *dev);      /* mbar/hPa */
float ms5611_get_altitude(const ms5611_t *dev, float sea_level_hPa);  /* metres */
void  ms5611_set_oversampling(ms5611_t *dev, ms5611_osr_t osr);
```

---

## 5. W25Q512JV — QSPI Flash (QUADSPI)

### 5.1 Overview

| Parameter | Value |
|-----------|-------|
| Part | Winbond W25Q512JV |
| Capacity | 64 MB (512 Mbit) |
| Interface | Quad SPI (4-line) |
| Page size | 256 bytes |
| Sector size | 4 KB |
| Block size | 64 KB |
| Sector count | 16384 |
| JEDEC ID | 0xEF, 0x40/0x70, 0x20 |

### 5.2 Bus Configuration

| Parameter | Value |
|-----------|-------|
| Peripheral | QUADSPI |
| CLK | PB2 |
| BK1_NCS | PB6 |
| BK1_IO0 | PD11 |
| BK1_IO1 | PD12 |
| BK1_IO2 | PD13 |
| BK1_IO3 | PE2 |
| FSIZE | 25 (2^26 = 64 MB) |

### 5.3 Init Sequence

1. Reset: Enable Reset (0x66) + Reset (0x99), wait 100 ms
2. Read JEDEC ID (0x9F) — verify manufacturer 0xEF
3. Enable Quad Enable (QE) bit in Status Register 2
4. Enter 4-byte address mode (0xB7) — required for >16 MB
5. Global unlock (0x98) — remove write protection

### 5.4 Operations

| Operation | Command | Time |
|-----------|---------|------|
| Page program | 0x12 (4B addr) | 3 ms max |
| Sector erase (4 KB) | 0x21 (4B addr) | 400 ms max |
| Block erase (64 KB) | 0xDC (4B addr) | 2000 ms max |
| Chip erase | 0xC7 | 400 seconds max |
| Fast read (quad) | 0x6C (4B addr) | 8 dummy cycles |

### 5.5 FATFS Integration

The W25Q512JV serves as the storage backend for FATFS. The `user_diskio.c` bridge maps FATFS read/write/ioctl to `w25q512jv_read/write/erase_sector`.

### 5.6 USB MSC Integration

In USB_MODE=2, the W25Q512JV appears as a USB Mass Storage device. The `usbd_msc_storage_if.c` bridge provides block-level access for the USB host.

### 5.7 Driver API

```c
bool w25q512jv_init(w25q512jv_t *dev, QSPI_HandleTypeDef *hqspi);
int  w25q512jv_read(w25q512jv_t *dev, uint32_t addr, uint8_t *buf, uint32_t len);
int  w25q512jv_write(w25q512jv_t *dev, uint32_t addr, const uint8_t *buf, uint32_t len);
int  w25q512jv_erase_sector(w25q512jv_t *dev, uint32_t addr);  /* 4 KB */
int  w25q512jv_erase_block(w25q512jv_t *dev, uint32_t addr);   /* 64 KB */
int  w25q512jv_erase_chip(w25q512jv_t *dev);                   /* Full chip, ~400s */
bool w25q512jv_test(w25q512jv_t *dev);                          /* Write-read-verify */
```

---

## 6. MAX-M10M — GPS Receiver (I2C1)

### 6.1 Overview

| Parameter | Value |
|-----------|-------|
| Part | u-blox MAX-M10M |
| Protocol | UBX binary over I2C DDC |
| Fix rate | 10 Hz (GPS-only) |
| I2C address | 0x42 (7-bit) |
| Reset pin | PE15 (NRST_GPS, active low) |
| INT pin | PE0 (I2C1_INT, not yet enabled) |

### 6.2 Bus Configuration

| Parameter | Value |
|-----------|-------|
| I2C peripheral | I2C1 |
| SCL | PB8 |
| SDA | PB9 |
| Speed | 400 kHz (Fast Mode) |

### 6.3 UBX Configuration (at init)

The driver configures the GPS via UBX-CFG-VALSET messages:

1. **Disable all NMEA messages** on I2C:
   - GGA, GLL, GSA, GSV, RMC, VTG → rate 0

2. **Enable UBX-NAV-PVT** on I2C:
   - `CFG_MSGOUT_UBX_NAV_PVT_I2C` → rate 1

3. **Set measurement rate** to 100 ms (10 Hz):
   - `CFG_RATE_MEAS` → 100

4. **GPS-only mode** (disable other constellations for 10 Hz):
   - `CFG_SIGNAL_GPS_ENA` → 1
   - `CFG_SIGNAL_GAL_ENA` → 0
   - `CFG_SIGNAL_BDS_ENA` → 0
   - `CFG_SIGNAL_QZSS_ENA` → 0
   - `CFG_SIGNAL_GLO_ENA` → 0

### 6.4 UBX-NAV-PVT Payload (92 bytes)

The driver parses the following fields from NAV-PVT:

| Offset | Field | Type | Unit |
|--------|-------|------|------|
| 0 | iTOW | u32 | ms (GPS time of week) |
| 4 | year | u16 | |
| 6 | month | u8 | |
| 7 | day | u8 | |
| 8 | hour | u8 | |
| 9 | min | u8 | |
| 10 | sec | u8 | |
| 11 | valid | u8 | Validity flags |
| 20 | fixType | u8 | 0=none, 2=2D, 3=3D |
| 23 | numSV | u8 | Satellites used |
| 24 | lon | i32 | degrees * 1e-7 |
| 28 | lat | i32 | degrees * 1e-7 |
| 36 | hMSL | i32 | mm above MSL |
| 40 | hAcc | u32 | mm horizontal accuracy |
| 44 | vAcc | u32 | mm vertical accuracy |
| 48 | velN | i32 | mm/s north |
| 52 | velE | i32 | mm/s east |
| 56 | velD | i32 | mm/s down |
| 76 | pDOP | u16 | * 0.01 |

### 6.5 Non-Blocking Tick Architecture

```
max_m10m_tick() state machine:
  IDLE → check poll interval (25 ms) → READ_AVAIL
  READ_AVAIL → I2C read bytes-available register (0xFD-0xFE) → READ_DATA
  READ_DATA → I2C read data stream, feed through UBX parser → IDLE
  Returns 1 when new NAV-PVT parsed, 0 otherwise.
```

The UBX parser is a byte-by-byte state machine:
```
SYNC1 → SYNC2 → CLASS → ID → LEN1 → LEN2 → PAYLOAD → CK_A → CK_B
```

Checksum: Fletcher-8 over class + ID + length + payload.

### 6.6 Convenience Floats

After parsing NAV-PVT, the driver computes:
```
lat_deg   = lat_deg7 * 1e-7
lon_deg   = lon_deg7 * 1e-7
alt_msl_m = h_msl_mm * 0.001
vel_d_m_s = vel_d_mm_s * 0.001
```

### 6.7 EKF Integration

When a 3D fix is obtained, the main loop calls:
```c
casper_ekf_update_gps_alt(&ekf, gps.alt_msl_m);  /* stub */
casper_ekf_update_gps_vel(&ekf, gps.vel_d_m_s);  /* stub */
```

For telemetry, GPS position is encoded as delta from pad origin in millimetres:
```c
fc_gps_state_t gps_state;
gps_state.dlat_mm = (int32_t)((gps.lat_deg - pad_lat) * 111320000.0);
gps_state.dlon_mm = (int32_t)((gps.lon_deg - pad_lon) * 111320000.0 * cos(pad_lat * DEG2RAD));
```

### 6.8 Driver API

```c
bool max_m10m_init(max_m10m_t *dev, I2C_HandleTypeDef *hi2c,
                   GPIO_TypeDef *nrst_port, uint16_t nrst_pin);
int  max_m10m_tick(max_m10m_t *dev);           /* Non-blocking, returns 1 on new PVT */
bool max_m10m_has_3d_fix(const max_m10m_t *dev);
```

---

## 7. MMC5983MA — Magnetometer (I2C3)

### 7.1 Overview

| Parameter | Value |
|-----------|-------|
| Part | MEMSIC MMC5983MA |
| Range | +-8 Gauss |
| Resolution | 18-bit |
| Continuous rate | 100 Hz |
| I2C address | 0x30 (7-bit) |
| Product ID | 0x30 |
| DRDY pin | PC8 (EXTI9_5, rising edge) |

### 7.2 Bus Configuration

| Parameter | Value |
|-----------|-------|
| I2C peripheral | I2C3 |
| SCL | PA8 |
| SDA | PC9 |
| DRDY | PC8 (EXTI, rising edge) |
| Speed | 400 kHz (Fast Mode) |

**Critical:** PC8 is configured as GPIO output by CubeMX. The firmware overrides it to EXTI rising-edge input in USER CODE BEGIN 2.

### 7.3 Register Configuration (Init Sequence)

1. Software reset: Write `SW_RST` (bit 7) to CTRL1 (0x0A), wait 15 ms
2. Verify Product ID (0x2F) = 0x30
3. Configure CTRL0 (0x09): Enable auto SET/RESET + DRDY interrupt
   ```
   AUTO_SR_EN | INT_MEAS_DONE_EN = 0x24
   ```
4. Configure CTRL1 (0x0A): Bandwidth = 800 Hz
   ```
   BW1 | BW0 = 0x03
   ```
5. Configure CTRL2 (0x0B): Continuous mode at 100 Hz + periodic SET
   ```
   CMM_EN | CM_100HZ | EN_PRD_SET | PRD_SET_1 = 0xAD
   ```

### 7.4 Data Format

- 18-bit unsigned per axis, spread across 7 registers:
  - `X_OUT_0` (bits [17:10]), `X_OUT_1` (bits [9:2]), `XYZ_OUT_2` bits [7:6] (bits [1:0])
- Midpoint: 131072 (2^17) = 0 Gauss
- Scale: 16384 counts/Gauss (2^18 / 16 Gauss full range)
- Conversion:
  ```
  field_gauss = (raw_18bit - 131072) / 16384.0
  field_ut    = field_gauss * 100.0    (1 Gauss = 100 microtesla)
  ```

### 7.5 Axis Mapping

The magnetometer is mounted with **opposite polarity** to the IMU. The firmware negates all three axes before calibration:

```c
float mag_raw[3] = {-mag.mag_ut[0], -mag.mag_ut[1], -mag.mag_ut[2]};
```

After negation, the mag axes align with the IMU sensor frame. See ORIENTATION_SPEC.md Section 2.2.

### 7.6 Hard/Soft-Iron Calibration

After negation, the `mag_cal_apply()` function applies:
1. Hard-iron offset subtraction
2. Soft-iron matrix correction (3x3)

Calibration parameters are stored in `mag_cal.h`.

### 7.7 Driver API

```c
bool mmc5983ma_init(mmc5983ma_t *dev, I2C_HandleTypeDef *hi2c);
int  mmc5983ma_read(mmc5983ma_t *dev);         /* Read 18-bit field, convert to Gauss + uT */
int  mmc5983ma_read_temp(mmc5983ma_t *dev);    /* Read die temperature */
void mmc5983ma_irq_handler(mmc5983ma_t *dev);  /* EXTI callback */
```

---

## 8. Data Collection Mode (USB_MODE=3)

In data collection mode, each sensor is logged to a CSV file on the QSPI flash at its full rate:

| Phase | Sensor | ODR | Duration | Output File |
|-------|--------|-----|----------|-------------|
| 1 | LSM6DSO32 (accel+gyro) | 833 Hz | 300 s | IMU.CSV |
| 2 | ADXL372 (FIFO streaming) | 800 Hz | 300 s | ADXL.CSV |
| 3 | MS5611 (baro OSR_4096) | ~110 Hz | 300 s | BARO_HI.CSV |
| 4 | MS5611 (baro OSR_1024) | ~430 Hz | 120 s | BARO_MD.CSV |
| 5 | MS5611 (baro OSR_256) | ~1650 Hz | 120 s | BARO_LO.CSV |

Select phase with `-DDATA_PHASE=N` in Makefile. After collection, rebuild with USB_MODE=2 to mount flash as USB drive and copy CSVs to PC for MATLAB analysis.
