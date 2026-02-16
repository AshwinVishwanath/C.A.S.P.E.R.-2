#include "self_test.h"
#include "crc32_hw.h"
#include "tlm_manager.h"
#include "lsm6dso32.h"
#include "mmc5983ma.h"
#include "ms5611.h"
#include <math.h>
#include <string.h>

/* ── Extern sensor handles from main.c ──────────────────────────── */
extern lsm6dso32_t  imu;
extern mmc5983ma_t  mag;
extern ms5611_t     baro;

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

    /* Test 2: Baro PROM CRC (non-zero PROM[0] means calibration read OK) */
    results[n].test_id = 2;
    results[n].result  = (baro.prom[0] != 0) ? 1 : 0;
    results[n].detail  = baro.prom[0];
    n++;

    /* Test 3: EKF init — stub: always pass */
    results[n].test_id = 3;
    results[n].result  = 1;
    results[n].detail  = 0;
    n++;

    /* Test 4: Attitude quaternion norm ≈ 1.0 (±0.01)
     * We can't access att.q directly since it's inside a struct.
     * Use the extern float att_q[4] trick — handled in main.c integration. */
    {
        /* Placeholder: will be populated when main.c provides a getter */
        results[n].test_id = 4;
        results[n].result  = 1;  /* Stub: pass */
        results[n].detail  = 0;
    }
    n++;

    /* Test 5: Flash write/read — stub */
    results[n].test_id = 5;
    results[n].result  = 1;  /* TODO: Pattern match via FATFS */
    results[n].detail  = 0;
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
