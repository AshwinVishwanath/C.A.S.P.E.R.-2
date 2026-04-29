#include "flight_logger.h"
#include "hamming.h"
#include "quat_pack.h"
#include "crc32_hw.h"
#include "tlm_types.h"
#include "stm32h7xx_hal.h"
#include <string.h>
#include <math.h>

/* ═══════════════════════════════════════════════════════════════════════
 *  Ring buffer memory — placed in AXI SRAM (RAM region, 0x24000000)
 *
 *  NOTE: The linker script must have a section for RAM_D1 or these
 *  arrays must be placed via __attribute__((section(".bss"))) if the
 *  default .bss is in AXI SRAM. Currently the default .bss is in
 *  DTCMRAM which is only 128 KB. These arrays need a linker section
 *  update to place them in the 512 KB AXI SRAM (RAM).
 *
 *  Total: 300*256 + 40*256 + 32*256 = 76,800 + 10,240 + 8,192 = 95,232 bytes
 * ═══════════════════════════════════════════════════════════════════════ */
static uint8_t hr_ring[300 * LOG_PAGE_SIZE]   __attribute__((section(".axi_sram"), aligned(4)));
static uint8_t lr_ring[40 * LOG_PAGE_SIZE]    __attribute__((section(".axi_sram"), aligned(4)));
static uint8_t adxl_ring[32 * LOG_PAGE_SIZE]  __attribute__((section(".axi_sram"), aligned(4)));

/* Ring page counts */
#define HR_RING_PAGES    300
#define LR_RING_PAGES    40
#define ADXL_RING_PAGES  32

/* Pre-launch data retention (pages) */
#define HR_PRELAUNCH_PAGES    188   /* ~3.1 s at 150 Hz (4 rec/page) */
#define LR_PRELAUNCH_PAGES    10    /* ~1.25 s at 1 Hz (8 rec/page) */
#define ADXL_PRELAUNCH_PAGES  16    /* ~0.5 s at 800 Hz (25 rec/page) */

/* ═══════════════════════════════════════════════════════════════════════
 *  QSPI IT-mode callback wrappers (match w25q_callback_t signature)
 * ═══════════════════════════════════════════════════════════════════════ */

static void qspi_complete_cb(void *ctx, bool success)
{
    (void)success;
    flight_logger_qspi_complete((flight_logger_t *)ctx);
}

static void qspi_error_cb(void *ctx, bool success)
{
    (void)success;
    flight_logger_qspi_error((flight_logger_t *)ctx);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  Helpers
 * ═══════════════════════════════════════════════════════════════════════ */

static void write_summary_to_flash(flight_logger_t *log, uint8_t type)
{
    log->summary.magic          = SUMMARY_MAGIC;
    log->summary.format_version = SUMMARY_VERSION;
    log->summary.summary_type   = type;

    /* Fill diagnostics from streams */
    log->summary.hr_records_written   = log->hr.records_written;
    log->summary.lr_records_written   = log->lr.records_written;
    log->summary.adxl_records_written = log->adxl.records_written;
    log->summary.total_drop_count     = log->hr.drop_count + log->lr.drop_count + log->adxl.drop_count;
    log->summary.flash_err_count      = log->hr.err_count + log->lr.err_count + log->adxl.err_count;

    /* CRC32 over bytes [0..251] (everything except the CRC field itself) */
    log->summary.crc32 = crc32_hw_compute((const uint8_t *)&log->summary,
                                           sizeof(flight_summary_t) - 4);

    /* Write to summary pool. Each flight gets 2 pages (partial + final).
     * Address = FLASH_SUMMARY_BASE + (flight_id - 1) * 2 * PAGE_SIZE + page_offset
     * Sector is pre-erased in flight_logger_launch(). */
    uint32_t base_addr = FLASH_SUMMARY_BASE
                       + (uint32_t)(log->index.current_flight - 1) * 2 * LOG_PAGE_SIZE;
    uint32_t addr;
    if (type == SUMMARY_PARTIAL)
        addr = base_addr;
    else
        addr = base_addr + LOG_PAGE_SIZE;

    /* Blocking write — called at APOGEE (partial) and LANDED (final) */
    w25q512jv_write(log->index.flash, addr,
                    (const uint8_t *)&log->summary, sizeof(flight_summary_t));
}

/* ═══════════════════════════════════════════════════════════════════════
 *  flight_logger_init
 * ═══════════════════════════════════════════════════════════════════════ */
bool flight_logger_init(flight_logger_t *log, w25q512jv_t *flash)
{
    memset(log, 0, sizeof(*log));

    if (!log_index_init(&log->index, flash))
        return false;

    /* Retrieve flash write addresses from index */
    uint32_t hr_addr, lr_addr, adxl_addr;
    log_index_get_write_addrs(&log->index, &hr_addr, &lr_addr, &adxl_addr);

    /* Initialize streams */
    log_stream_init(&log->hr, hr_ring, HR_RING_PAGES, HR_PRELAUNCH_PAGES,
                    HR_REC_SIZE, HR_RECS_PER_PAGE, hr_addr, FLASH_HR_END);

    log_stream_init(&log->lr, lr_ring, LR_RING_PAGES, LR_PRELAUNCH_PAGES,
                    LR_REC_SIZE, LR_RECS_PER_PAGE, lr_addr, FLASH_LR_END);

    log_stream_init(&log->adxl, adxl_ring, ADXL_RING_PAGES, ADXL_PRELAUNCH_PAGES,
                    ADXL_REC_SIZE, ADXL_RECS_PER_PAGE, adxl_addr, FLASH_ADXL_END);

    /* Summary init */
    memset(&log->summary, 0, sizeof(log->summary));
    log->summary.min_temp_avg_c = 999.0f;

    /* PAD-rate dividers (push called at 833 Hz superloop rate) */
    log->hr_tick_div  = 6;    /* 833/6 ≈ 139 Hz */
    log->lr_tick_div  = 83;   /* 833/83 ≈ 10 Hz  */
    log->qspi_state   = QSPI_IDLE;

    /* Register QSPI IT callbacks for non-blocking flash operations */
    w25q512jv_set_callbacks(flash, qspi_complete_cb, qspi_error_cb, log);

    return true;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  flight_logger_start  — begin pre-launch recording
 * ═══════════════════════════════════════════════════════════════════════ */
void flight_logger_start(flight_logger_t *log)
{
    log_stream_start(&log->hr);
    log_stream_start(&log->lr);
    log_stream_start(&log->adxl);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  flight_logger_launch  — transition to drain mode
 * ═══════════════════════════════════════════════════════════════════════ */
void flight_logger_launch(flight_logger_t *log)
{
    log_stream_launch(&log->hr);
    log_stream_launch(&log->lr);
    log_stream_launch(&log->adxl);

    /* Wait for any in-progress IT erase to complete before blocking ops.
     * The ring buffers absorb records during this spin (~400 ms worst case). */
    while (!w25q512jv_is_idle(log->index.flash)) { /* spin */ }
    log->qspi_state    = QSPI_IDLE;
    log->active_stream = NULL;

    log_index_start_flight(&log->index, HAL_GetTick());

    /* Pre-erase summary sector if this flight starts a new 4 KB sector.
     * Each flight uses 2 pages (512B), so 8 flights per sector.
     * Blocking erase is OK here — ring buffer absorbs the ≤400 ms delay. */
    {
        uint32_t summary_addr = FLASH_SUMMARY_BASE
                              + (uint32_t)(log->index.current_flight - 1) * 2 * LOG_PAGE_SIZE;
        if ((summary_addr & (W25Q512JV_SECTOR_SIZE - 1)) == 0)
            w25q512jv_erase_sector(log->index.flash, summary_addr);
    }

    log->summary.launch_tick = HAL_GetTick();
    log->launched = true;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  flight_logger_tick  — QSPI dispatch (call every superloop iteration)
 * ═══════════════════════════════════════════════════════════════════════ */
void flight_logger_tick(flight_logger_t *log)
{
    if (log->qspi_state != QSPI_IDLE)
        return;

    if (log->finalized)
        return;

    if (!log->launched) {
        /* PAD mode: non-blocking erase-ahead via IT mode.
         * Demand-based: erase the pool with the least runway first. */
        log_stream_t *streams[3] = { &log->hr, &log->lr, &log->adxl };

        int best = -1;
        uint32_t min_runway = 0xFFFFFFFF;
        for (int i = 0; i < 3; i++) {
            log_stream_t *s = streams[i];
            if (s->erased_up_to < s->flash_end) {
                uint32_t runway = s->erased_up_to - s->flash_addr;
                if (runway < min_runway) {
                    min_runway = runway;
                    best = i;
                }
            }
        }

        if (best >= 0) {
            log_stream_t *s = streams[best];
            if (w25q512jv_erase_sector_it(log->index.flash,
                                           s->erased_up_to) == W25Q_OK) {
                log->qspi_state    = QSPI_ERASING;
                log->active_stream = s;
            }
        }
        return;
    }

    /* DRAIN mode: non-blocking page writes. Priority: HR > ADXL > LR */
    log_stream_t *priority[3] = { &log->hr, &log->adxl, &log->lr };

    for (int i = 0; i < 3; i++) {
        log_stream_t *s = priority[i];

        if (!log_stream_has_page(s))
            continue;

        uint32_t addr = log_stream_flash_addr(s);

        /* Frontier check: if we hit the erase frontier, stop this stream.
         * PAD erase-ahead should have prepared enough runway. */
        if (addr >= s->erased_up_to) {
            s->err_count++;
            continue;
        }

        /* Non-blocking page write via IT mode */
        const uint8_t *page = log_stream_peek_page(s);
        if (w25q512jv_write_page_it(log->index.flash, addr, page) == W25Q_OK) {
            log->qspi_state    = QSPI_WRITING;
            log->active_stream = s;
        }

        /* One operation per tick */
        return;
    }
}

/* ═══════════════════════════════════════════════════════════════════════
 *  flight_logger_set_rate
 * ═══════════════════════════════════════════════════════════════════════ */
void flight_logger_set_rate(flight_logger_t *log, uint8_t fsm_state)
{
    /* Push functions called at 833 Hz superloop rate.
     * HR: 139 Hz (BOOST–DROGUE) → 10 Hz (MAIN–LANDED)
     * LR: 10 Hz (PAD) → 49 Hz (BOOST–DROGUE) → 10 Hz (MAIN–LANDED) */
    switch (fsm_state) {
    case FSM_STATE_BOOST:
    case FSM_STATE_COAST:
    case FSM_STATE_COAST_1:
    case FSM_STATE_SUSTAIN:
    case FSM_STATE_COAST_2:
    case FSM_STATE_APOGEE:
    case FSM_STATE_DROGUE:
        log->hr_tick_div = 6;   /* 833/6  ≈ 139 Hz */
        log->lr_tick_div = 17;  /* 833/17 ≈  49 Hz */
        break;

    case FSM_STATE_MAIN:
    case FSM_STATE_RECOVERY:
    case FSM_STATE_LANDED:
        log->hr_tick_div = 83;  /* 833/83 ≈ 10 Hz */
        log->lr_tick_div = 83;  /* 833/83 ≈ 10 Hz */
        break;

    default: /* PAD */
        log->hr_tick_div = 6;   /* 833/6  ≈ 139 Hz */
        log->lr_tick_div = 83;  /* 833/83 ≈  10 Hz */
        break;
    }
}

/* ═══════════════════════════════════════════════════════════════════════
 *  flight_logger_push_hr
 * ═══════════════════════════════════════════════════════════════════════ */
#ifdef LOGGER_SANITY
/* Call-rate diagnostics: count every entry and every divider-passed push. */
static volatile uint32_t s_hr_calls = 0;
static volatile uint32_t s_hr_pushes = 0;
static volatile uint32_t s_lr_calls = 0;
static volatile uint32_t s_lr_pushes = 0;
/* IMU IRQ / watchdog diagnostic counters — defined here, accessed via
 * extern declarations from main.c (EXTI ISR) and flight_loop.c (watchdog). */
volatile uint32_t g_imu_exti_fires = 0;
volatile uint32_t g_imu_wd_polls   = 0;
volatile uint32_t g_imu_wd_hits    = 0;
uint32_t flight_logger_diag_hr_calls(void)  { return s_hr_calls; }
uint32_t flight_logger_diag_hr_pushes(void) { return s_hr_pushes; }
uint32_t flight_logger_diag_lr_calls(void)  { return s_lr_calls; }
uint32_t flight_logger_diag_lr_pushes(void) { return s_lr_pushes; }
#endif

void flight_logger_push_hr(flight_logger_t *log,
                           const lsm6dso32_t *imu, const ms5611_t *baro,
                           const adxl372_t *adxl, const mmc5983ma_t *mag_sensor,
                           const casper_ekf_t *ekf, const casper_attitude_t *att,
                           uint8_t fsm_state, uint8_t flags,
                           uint16_t sustain_ms)
{
#ifdef LOGGER_SANITY
    s_hr_calls++;
#endif
    /* Rate divider */
    log->hr_tick_count++;
    if (log->hr_tick_count < log->hr_tick_div)
        return;
    log->hr_tick_count = 0;
#ifdef LOGGER_SANITY
    s_hr_pushes++;
#endif

    hr_record_t rec;
    memset(&rec, 0, sizeof(rec));

    rec.timestamp_ms = HAL_GetTick();

    /* Baro: pressure in Pa, encode as 2 Pa/LSB */
    rec.baro_pressure = (uint16_t)(baro->pressure / 2);

    /* IMU raw values */
    rec.lsm6_accel[0] = imu->raw_accel[0];
    rec.lsm6_accel[1] = imu->raw_accel[1];
    rec.lsm6_accel[2] = imu->raw_accel[2];
    rec.lsm6_gyro[0]  = imu->raw_gyro[0];
    rec.lsm6_gyro[1]  = imu->raw_gyro[1];
    rec.lsm6_gyro[2]  = imu->raw_gyro[2];

    /* ADXL372: raw_accel is 12-bit, scale is 100 mg/LSB */
    rec.adxl372[0] = adxl->raw_accel[0];
    rec.adxl372[1] = adxl->raw_accel[1];
    rec.adxl372[2] = adxl->raw_accel[2];

    /* Magnetometer: 18-bit raw -> shift to 16-bit */
    rec.mmc[0] = (uint16_t)(mag_sensor->raw_mag[0] >> 2);
    rec.mmc[1] = (uint16_t)(mag_sensor->raw_mag[1] >> 2);
    rec.mmc[2] = (uint16_t)(mag_sensor->raw_mag[2] >> 2);

    /* EKF state */
    rec.ekf_alt_m   = ekf->x[0];
    rec.ekf_vel_mps = ekf->x[1];

    /* Quaternion: smallest-three packing */
    quat_pack_smallest_three(rec.quat_packed, att->q);

    rec.fsm_state = fsm_state;
    rec.flags     = flags;

    /* EKF biases: scale to integer */
    rec.ekf_accel_bias = (int16_t)(ekf->x[2] * 1000.0f);  /* 0.001 m/s^2 / LSB */
    rec.ekf_baro_bias  = (int16_t)(ekf->x[3] * 100.0f);   /* 0.01 m / LSB */

    /* Temperatures */
    rec.imu_temp  = (int16_t)(imu->temp_c * 100.0f);     /* 0.01 C / LSB */
    rec.baro_temp = (int16_t)(baro->temperature);         /* Already 0.01 C */

    rec.seq_num            = log->hr_seq++;
    rec.sustain_counter_ms = sustain_ms;

    /* Hamming SECDED over bytes [0:59] */
    rec.hamming_secded = hamming_encode((const uint8_t *)&rec, 60);

    /* CRC16-CCITT over bytes [0:61] */
    rec.crc16 = crc16_ccitt((const uint8_t *)&rec, 62);

    log_stream_push(&log->hr, &rec);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  flight_logger_push_lr
 * ═══════════════════════════════════════════════════════════════════════ */
void flight_logger_push_lr(flight_logger_t *log,
                           uint8_t pyro_state, const uint16_t pyro_cont[4],
                           int8_t rssi, int8_t snr,
                           uint16_t tx_count, uint16_t rx_count, uint16_t fail_count,
                           const max_m10m_t *gps)
{
#ifdef LOGGER_SANITY
    s_lr_calls++;
#endif
    /* Rate divider */
    log->lr_tick_count++;
    if (log->lr_tick_count < log->lr_tick_div)
        return;
    log->lr_tick_count = 0;
#ifdef LOGGER_SANITY
    s_lr_pushes++;
#endif

    lr_record_t rec;
    memset(&rec, 0, sizeof(rec));

    rec.timestamp_ms     = HAL_GetTick();
    rec.pyro_state       = pyro_state;
    rec.pyro_cont_adc[0] = pyro_cont[0];
    rec.pyro_cont_adc[1] = pyro_cont[1];
    rec.pyro_cont_adc[2] = pyro_cont[2];
    rec.pyro_cont_adc[3] = pyro_cont[3];
    rec.radio_rssi       = rssi;
    rec.radio_snr        = snr;
    rec.radio_tx_count   = tx_count;
    rec.radio_rx_count   = rx_count;
    rec.radio_fail_count = fail_count;

    /* GPS delta position */
    if (gps) {
        /* dlat/dlon in mm (int32 from telemetry convention) */
        rec.gps_dlat_mm   = gps->lat_deg7;  /* raw 1e-7 deg stored as-is for now */
        rec.gps_dlon_mm   = gps->lon_deg7;
        rec.gps_alt_msl_m = (int16_t)(gps->alt_msl_m);
        rec.gps_fix_sats  = (uint8_t)((gps->fix_type << 4) | (gps->num_sv & 0x0F));
    }

    log_stream_push(&log->lr, &rec);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  flight_logger_push_adxl
 * ═══════════════════════════════════════════════════════════════════════ */
void flight_logger_push_adxl(flight_logger_t *log, const int16_t samples[][3],
                             uint16_t count)
{
    uint32_t now = HAL_GetTick();

    for (uint16_t i = 0; i < count; i++) {
        adxl_record_t rec;
        rec.timestamp_ms = now;
        rec.accel_x      = samples[i][0];
        rec.accel_y      = samples[i][1];
        rec.accel_z      = samples[i][2];

        log_stream_push(&log->adxl, &rec);
    }
}

/* ═══════════════════════════════════════════════════════════════════════
 *  Summary update functions
 * ═══════════════════════════════════════════════════════════════════════ */

void flight_logger_summary_imu(flight_logger_t *log, float accel_mag_g,
                               float tilt_deg, float roll_rate_dps,
                               uint8_t fsm_state)
{
    if (fsm_state == FSM_STATE_BOOST && accel_mag_g > log->summary.peak_accel_boost_g)
        log->summary.peak_accel_boost_g = accel_mag_g;

    if (fsm_state == FSM_STATE_SUSTAIN && accel_mag_g > log->summary.peak_accel_sustain_g)
        log->summary.peak_accel_sustain_g = accel_mag_g;

    if (tilt_deg > log->summary.max_tilt_deg)
        log->summary.max_tilt_deg = tilt_deg;

    if (roll_rate_dps > log->summary.max_roll_rate_dps)
        log->summary.max_roll_rate_dps = roll_rate_dps;
}

void flight_logger_summary_ekf(flight_logger_t *log, float vel_mps,
                               float alt_m, float baro_pa, float temp_k)
{
    uint32_t now = HAL_GetTick();

    /* Peak velocity */
    if (vel_mps > log->summary.peak_velocity_mps) {
        log->summary.peak_velocity_mps = vel_mps;
        log->summary.peak_vel_tick = now;
    }

    /* Mach number */
    if (temp_k > 100.0f) {  /* sanity check */
        float a_sound = sqrtf(1.4f * 287.058f * temp_k);
        float mach    = vel_mps / a_sound;
        if (mach > log->summary.peak_mach)
            log->summary.peak_mach = mach;

        /* Dynamic pressure: q = 0.5 * rho * v^2, rho = P / (R * T) */
        float rho = baro_pa / (287.058f * temp_k);
        float q   = 0.5f * rho * vel_mps * vel_mps;
        if (q > log->summary.max_q_pa) {
            log->summary.max_q_pa    = q;
            log->summary.max_q_alt_m = alt_m;
            log->summary.max_q_tick  = now;
        }
    }

    /* Max descent rate (positive = downward) */
    if (vel_mps < 0.0f) {
        float descent = -vel_mps;
        if (descent > log->summary.max_descent_rate_mps)
            log->summary.max_descent_rate_mps = descent;
    }

    /* Temperature extremes (convert K to C) */
    float temp_c = temp_k - 273.15f;
    if (temp_c > log->summary.peak_temp_avg_c)
        log->summary.peak_temp_avg_c = temp_c;
    if (temp_c < log->summary.min_temp_avg_c)
        log->summary.min_temp_avg_c = temp_c;
}

void flight_logger_summary_event(flight_logger_t *log, uint8_t fsm_state,
                                 uint8_t prev_state)
{
    uint32_t now = HAL_GetTick();

    log->summary.num_fsm_transitions++;

    switch (fsm_state) {
    case FSM_STATE_COAST:
    case FSM_STATE_COAST_1:
        /* First burnout */
        if (log->summary.burnout1_tick == 0)
            log->summary.burnout1_tick = now;
        break;

    case FSM_STATE_COAST_2:
        /* Second burnout (after sustain) */
        log->summary.burnout2_tick = now;
        break;

    case FSM_STATE_APOGEE:
        log->summary.apogee_tick    = now;
        log->summary.time_to_apogee = now - log->summary.launch_tick;
        break;

    case FSM_STATE_MAIN:
        log->summary.main_deploy_tick = now;
        break;

    case FSM_STATE_LANDED:
        log->summary.landing_tick      = now;
        log->summary.total_flight_tick = now - log->summary.launch_tick;
        break;

    default:
        break;
    }

    (void)prev_state;
}

void flight_logger_summary_gps(flight_logger_t *log, const max_m10m_t *gps)
{
    if (!gps || gps->fix_type < 2)
        return;

    /* Store launch position (first valid fix) */
    if (log->summary.launch_lat_1e7 == 0 && log->summary.launch_lon_1e7 == 0) {
        log->summary.launch_lat_1e7 = gps->lat_deg7;
        log->summary.launch_lon_1e7 = gps->lon_deg7;
    }

    /* Always update landing position (last known fix) */
    log->summary.landing_lat_1e7 = gps->lat_deg7;
    log->summary.landing_lon_1e7 = gps->lon_deg7;

    /* Max GPS altitude */
    if (gps->alt_msl_m > log->summary.gps_max_alt_msl_m)
        log->summary.gps_max_alt_msl_m = gps->alt_msl_m;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  Summary write to flash
 * ═══════════════════════════════════════════════════════════════════════ */

void flight_logger_write_partial_summary(flight_logger_t *log)
{
    write_summary_to_flash(log, SUMMARY_PARTIAL);
}

void flight_logger_write_final_summary(flight_logger_t *log)
{
    write_summary_to_flash(log, SUMMARY_FINAL);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  flight_logger_finalize  — drain remaining pages, write summary
 * ═══════════════════════════════════════════════════════════════════════ */
void flight_logger_finalize(flight_logger_t *log)
{
    if (log->finalized)
        return;

    /* Wait for any in-progress IT operation to complete before
     * switching to blocking mode. Timeout ~500 ms max (sector erase). */
    while (!w25q512jv_is_idle(log->index.flash)) { /* spin */ }
    log->qspi_state   = QSPI_IDLE;
    log->active_stream = NULL;

    /* Finalize all streams (pad partial pages) */
    log_stream_finalize(&log->hr);
    log_stream_finalize(&log->lr);
    log_stream_finalize(&log->adxl);

    /* Drain remaining pages with blocking writes (at LANDED, timing is relaxed) */
    log_stream_t *streams[3] = { &log->hr, &log->adxl, &log->lr };

    for (int i = 0; i < 3; i++) {
        log_stream_t *s = streams[i];
        while (log_stream_has_page(s)) {
            uint32_t addr = log_stream_flash_addr(s);

            /* Erase ahead if needed */
            while (addr >= s->erased_up_to) {
                w25q512jv_erase_sector(log->index.flash, s->erased_up_to);
                s->erased_up_to += W25Q512JV_SECTOR_SIZE;
            }

            const uint8_t *page = log_stream_peek_page(s);
            int ret = w25q512jv_write(log->index.flash, addr, page, LOG_PAGE_SIZE);
            if (ret == W25Q_OK)
                log_stream_page_done(s);
            else
                log_stream_page_fail(s);
        }
    }

    /* Write final summary */
    flight_logger_write_final_summary(log);

    /* Close flight index entry */
    log_index_end_flight(&log->index, HAL_GetTick(),
                         log->hr.flash_addr,
                         log->lr.flash_addr,
                         log->adxl.flash_addr);

    log->finalized = true;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  QSPI callbacks (ISR context — keep minimal)
 * ═══════════════════════════════════════════════════════════════════════ */

void flight_logger_qspi_complete(flight_logger_t *log)
{
    if (log->qspi_state == QSPI_ERASING) {
        if (log->active_stream)
            log->active_stream->erased_up_to += W25Q512JV_SECTOR_SIZE;
    } else if (log->qspi_state == QSPI_WRITING || log->qspi_state == QSPI_POLLING) {
        if (log->active_stream)
            log_stream_page_done(log->active_stream);
    }

    log->qspi_state   = QSPI_IDLE;
    log->active_stream = NULL;
}

void flight_logger_qspi_error(flight_logger_t *log)
{
    if (log->qspi_state == QSPI_WRITING || log->qspi_state == QSPI_POLLING) {
        if (log->active_stream)
            log_stream_page_fail(log->active_stream);
    }

    log->qspi_state   = QSPI_IDLE;
    log->active_stream = NULL;
}
