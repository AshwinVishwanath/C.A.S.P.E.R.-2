/* ============================================================
 *  TIER:     SAFETY-CRITICAL
 *  MODULE:   Self Test
 *  SUMMARY:  Power-on integrity checks; gates entry to ARMED.
 * ============================================================ */
#include "self_test.h"
#include "crc32_hw.h"
#include "tlm_manager.h"
#include "lsm6dso32.h"
#include "mmc5983ma.h"
#include "ms5611.h"
#include "casper_ekf.h"
#include "casper_attitude.h"
#include "w25q512jv.h"
#include "log_types.h"
#include <math.h>
#include <string.h>

/* ── Extern sensor handles from main.c ──────────────────────────── */
extern lsm6dso32_t  imu;
extern mmc5983ma_t  mag;
extern ms5611_t     baro;

/* ── Extern nav handles from app_globals ────────────────────────── */
extern casper_ekf_t      ekf;
extern casper_attitude_t att;
extern w25q512jv_t       flash;

/* ── Forward declaration ────────────────────────────────────────── */
extern uint32_t cfg_get_active_hash(void);

/* ── Helpers ────────────────────────────────────────────────────── */
static void put_le16(uint8_t *dst, uint16_t val)
{
    dst[0] = (uint8_t)(val & 0xFF);
    dst[1] = (uint8_t)((val >> 8) & 0xFF);
}

static void put_le32(uint8_t *dst, uint32_t val)
{
    dst[0] = (uint8_t)(val & 0xFF);
    dst[1] = (uint8_t)((val >> 8) & 0xFF);
    dst[2] = (uint8_t)((val >> 16) & 0xFF);
    dst[3] = (uint8_t)((val >> 24) & 0xFF);
}

/* ── Self-test implementations ──────────────────────────────────── */

int self_test_run_all(diag_result_t *results)
{
    int n = 0;

    /* Test 0: IMU WHO_AM_I = 0x6C */
    results[n].test_id = 0;
    results[n].result  = (imu.device_id == 0x6C) ? 1 : 0;
    results[n].detail  = (uint16_t)imu.device_id;
    n++;

    /* Test 1: Magnetometer product ID = 0x30 */
    results[n].test_id = 1;
    results[n].result  = (mag.product_id == 0x30) ? 1 : 0;
    results[n].detail  = (uint16_t)mag.product_id;
    n++;

    /* Test 2: Baro PROM calibration coefficients read OK.
     * prom[0] is a manufacturer/factory word that is legitimately 0 on many
     * MS5611 parts — the actual calibration coefficients are prom[1..6] (C1..C6).
     * A working sensor never has all of C1..C6 zero. */
    {
        results[n].test_id = 2;
        bool any_nonzero = false;
        for (int k = 1; k <= 6; k++) {
            if (baro.prom[k] != 0) { any_nonzero = true; break; }
        }
        results[n].result = any_nonzero ? 1 : 0;
        results[n].detail = baro.prom[1];  /* C1 = pressure sensitivity */
    }
    n++;

    /* Test 3: EKF init — P[0] non-zero means casper_ekf_init() ran */
    {
        results[n].test_id = 3;
        results[n].result  = (ekf.P[0] > 0.0f) ? 1 : 0;
        /* detail: upper 16 bits of P[0] raw bits (via memcpy to avoid aliasing) */
        uint32_t p0bits = 0;
        memcpy(&p0bits, &ekf.P[0], sizeof(p0bits));
        results[n].detail  = (uint16_t)(p0bits >> 16);
    }
    n++;

    /* Test 4: Attitude quaternion norm ≈ 1.0 (±0.01)
     * If init not complete, result=0, detail=0xFF (not-ready marker). */
    {
        results[n].test_id = 4;
        if (!att.init_complete) {
            results[n].result = 0;
            results[n].detail = 0xFF;
        } else {
            float n2 = att.q[0]*att.q[0] + att.q[1]*att.q[1]
                     + att.q[2]*att.q[2] + att.q[3]*att.q[3];
            float norm = sqrtf(n2);
            float err  = norm - 1.0f;
            if (err < 0.0f) err = -err;
            results[n].result = (err < 0.01f) ? 1 : 0;
            /* detail: norm scaled to uint16 (1.0 → 0x8000) */
            results[n].detail = (uint16_t)(norm * 32768.0f);
        }
    }
    n++;

    /* Test 5: Flash write/read — erase second-to-last 4KB sector,
     * program 256 bytes of 0xA5/0x5A, read back and verify.
     * Sector at 0x3FFE000 is within FLASH_HR region but near the end;
     * the logger write pointer starts at FLASH_HR_BASE (0x01091000) and
     * advances forward, so this sector is safe at power-on self-test time. */
    {
        static const uint32_t SCRATCH_ADDR = 0x3FFE000u; /* second-to-last 4KB sector */
        static uint8_t wbuf[256];
        static uint8_t rbuf[256];
        for (int i = 0; i < 256; i++) {
            wbuf[i] = (i & 1) ? 0x5Au : 0xA5u;
        }
        int ok = 0;
        if (flash.initialized) {
            if (w25q512jv_erase_sector(&flash, SCRATCH_ADDR) == W25Q_OK) {
                if (w25q512jv_write(&flash, SCRATCH_ADDR, wbuf, 256) == W25Q_OK) {
                    if (w25q512jv_read(&flash, SCRATCH_ADDR, rbuf, 256) == W25Q_OK) {
                        ok = (memcmp(wbuf, rbuf, 256) == 0) ? 1 : 0;
                    }
                }
            }
        }
        results[n].test_id = 5;
        results[n].result  = (uint8_t)ok;
        results[n].detail  = flash.initialized ? 0x0001u : 0x0000u;
    }
    n++;

    /* Test 6: Config hash non-zero */
    {
        uint32_t hash = cfg_get_active_hash();
        results[n].test_id = 6;
        results[n].result  = (hash != 0) ? 1 : 0;
        results[n].detail  = (uint16_t)(hash & 0xFFFF);
    }
    n++;

    return n;
}

int self_test_run_and_send(void)
{
    diag_result_t results[7];
    int num = self_test_run_all(results);

    /* Build response:
     * [0xC2][num:u8][{id:u8,result:u8,detail:u16}×N][CRC-32:u32] */
    uint8_t pkt[32];
    int pos = 0;

    pkt[pos++] = MSG_ID_DIAG;   /* 0xC2 */
    pkt[pos++] = (uint8_t)num;

    for (int i = 0; i < num; i++) {
        pkt[pos++] = results[i].test_id;
        pkt[pos++] = results[i].result;
        put_le16(&pkt[pos], results[i].detail);
        pos += 2;
    }

    /* Append CRC-32 over bytes 0..(pos-1) */
    uint32_t crc = crc32_hw_compute(pkt, (uint32_t)pos);
    put_le32(&pkt[pos], crc);
    pos += 4;

    return tlm_send_response(pkt, pos);
}
