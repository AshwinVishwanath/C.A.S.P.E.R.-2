/* ============================================================
 *  TIER:     CORE-FLIGHT
 *  MODULE:   Log Index
 *  SUMMARY:  On-flash index of log records; supports flash dump protocol.
 * ============================================================ */
#include "log_index.h"
#include <string.h>

/* Sector size for alignment */
#define SECTOR_SIZE  W25Q512JV_SECTOR_SIZE  /* 4096 */

/* Round address up to next sector boundary */
static uint32_t sector_align_up(uint32_t addr)
{
    return (addr + SECTOR_SIZE - 1) & ~(SECTOR_SIZE - 1);
}

/* Wrap address within a pool */
static uint32_t pool_wrap(uint32_t addr, uint32_t base, uint32_t end)
{
    if (addr >= end)
        return base;
    return addr;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  log_index_init
 * ═══════════════════════════════════════════════════════════════════════ */
bool log_index_init(log_index_t *idx, w25q512jv_t *flash)
{
    memset(idx, 0, sizeof(*idx));
    idx->flash = flash;

    /* Default: start each pool from its base (sector-aligned) */
    idx->hr_next_addr   = FLASH_HR_BASE;
    idx->lr_next_addr   = FLASH_LR_BASE;
    idx->adxl_next_addr = FLASH_ADXL_BASE;

    /*
     * Scan flight index entries. Each entry is 36 bytes.
     * FLASH_INDEX_SIZE (4 KB) / 36 = 113 entries max (MAX_FLIGHTS = 113).
     * An empty slot has flight_id == 0xFFFF (erased NOR flash).
     */
    flight_index_entry_t entry;
    uint16_t count = 0;

    for (uint16_t i = 0; i < MAX_FLIGHTS; i++) {
        uint32_t addr = FLASH_INDEX_BASE + (uint32_t)i * sizeof(flight_index_entry_t);
        int ret = w25q512jv_read(flash, addr, (uint8_t *)&entry, sizeof(entry));
        if (ret != W25Q_OK)
            return false;

        if (entry.flight_id == 0xFFFF) {
            /* Empty slot — end of used entries */
            break;
        }

        count++;

        /* Compute next write addresses from this entry */
        if (entry.hr_end_addr != INDEX_DIRTY) {
            idx->hr_next_addr = sector_align_up(entry.hr_end_addr);
            idx->hr_next_addr = pool_wrap(idx->hr_next_addr, FLASH_HR_BASE, FLASH_HR_END);
        } else {
            /*
             * Dirty flight (no clean shutdown). Use hr_start_addr and
             * estimate: we don't know how far writing got, so start from
             * the next sector after start. This wastes some space but
             * avoids corrupting unfinished data.
             */
            idx->hr_next_addr = sector_align_up(entry.hr_start_addr + SECTOR_SIZE);
            idx->hr_next_addr = pool_wrap(idx->hr_next_addr, FLASH_HR_BASE, FLASH_HR_END);
        }

        if (entry.lr_end_addr != INDEX_DIRTY) {
            idx->lr_next_addr = sector_align_up(entry.lr_end_addr);
            idx->lr_next_addr = pool_wrap(idx->lr_next_addr, FLASH_LR_BASE, FLASH_LR_END);
        } else {
            idx->lr_next_addr = sector_align_up(entry.lr_start_addr + SECTOR_SIZE);
            idx->lr_next_addr = pool_wrap(idx->lr_next_addr, FLASH_LR_BASE, FLASH_LR_END);
        }

        if (entry.adxl_end_addr != INDEX_DIRTY) {
            idx->adxl_next_addr = sector_align_up(entry.adxl_end_addr);
            idx->adxl_next_addr = pool_wrap(idx->adxl_next_addr, FLASH_ADXL_BASE, FLASH_ADXL_END);
        } else if (entry.adxl_start_addr != INDEX_DIRTY) {
            /* Dirty flight — end not recorded; skip at least one sector to
             * avoid overwriting partial data. */
            idx->adxl_next_addr = sector_align_up(entry.adxl_start_addr + SECTOR_SIZE);
            idx->adxl_next_addr = pool_wrap(idx->adxl_next_addr, FLASH_ADXL_BASE, FLASH_ADXL_END);
        }
    }

    idx->flight_count   = count;
    idx->current_flight = count;  /* Next flight will be count + 1 */

    return true;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  log_index_start_flight
 * ═══════════════════════════════════════════════════════════════════════ */
bool log_index_start_flight(log_index_t *idx, uint32_t start_tick)
{
    if (idx->flight_count >= MAX_FLIGHTS) {
        /* Index full — erase and restart from 0 */
        int ret = w25q512jv_erase_sector(idx->flash, FLASH_INDEX_BASE);
        if (ret != W25Q_OK)
            return false;
        idx->flight_count = 0;
    }

    idx->current_flight = idx->flight_count + 1;

    flight_index_entry_t entry;
    memset(&entry, 0xFF, sizeof(entry));  /* Start with all 0xFF */

    entry.flight_id       = idx->current_flight;
    entry.start_tick_ms   = start_tick;
    entry.end_tick_ms     = INDEX_DIRTY;
    entry.hr_start_addr   = idx->hr_next_addr;
    entry.hr_end_addr     = INDEX_DIRTY;
    entry.lr_start_addr   = idx->lr_next_addr;
    entry.lr_end_addr     = INDEX_DIRTY;
    entry.adxl_start_addr = idx->adxl_next_addr;
    entry.adxl_end_addr   = INDEX_DIRTY;
    entry.flags           = 0xFFFF;  /* Dirty — bit 0 will be cleared on clean shutdown */

    uint32_t addr = FLASH_INDEX_BASE + (uint32_t)idx->flight_count * sizeof(flight_index_entry_t);
    int ret = w25q512jv_write(idx->flash, addr, (const uint8_t *)&entry, sizeof(entry));
    if (ret != W25Q_OK)
        return false;

    idx->flight_count++;
    return true;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  log_index_end_flight
 * ═══════════════════════════════════════════════════════════════════════ */
bool log_index_end_flight(log_index_t *idx, uint32_t end_tick,
                          uint32_t hr_end, uint32_t lr_end, uint32_t adxl_end)
{
    /*
     * NOR flash 1->0 trick: we can clear bits without erasing.
     * Re-write the entry with actual end addresses (clearing the
     * 0xFFFFFFFF dirty markers to real values).
     */
    flight_index_entry_t entry;
    uint32_t entry_addr = FLASH_INDEX_BASE
                        + (uint32_t)(idx->flight_count - 1) * sizeof(flight_index_entry_t);

    /* Read existing entry */
    int ret = w25q512jv_read(idx->flash, entry_addr, (uint8_t *)&entry, sizeof(entry));
    if (ret != W25Q_OK)
        return false;

    /* Update fields (1->0 is always valid on NOR) */
    entry.end_tick_ms  = end_tick;
    entry.hr_end_addr  = hr_end;
    entry.lr_end_addr  = lr_end;
    entry.adxl_end_addr = adxl_end;
    entry.flags        = entry.flags & ~0x0001u;  /* Clear bit 0: clean_shutdown = 0 */

    /* Re-write the entire entry */
    ret = w25q512jv_write(idx->flash, entry_addr, (const uint8_t *)&entry, sizeof(entry));
    if (ret != W25Q_OK)
        return false;

    /* Update next addresses for potential future flights */
    idx->hr_next_addr   = sector_align_up(hr_end);
    idx->hr_next_addr   = pool_wrap(idx->hr_next_addr, FLASH_HR_BASE, FLASH_HR_END);
    idx->lr_next_addr   = sector_align_up(lr_end);
    idx->lr_next_addr   = pool_wrap(idx->lr_next_addr, FLASH_LR_BASE, FLASH_LR_END);
    idx->adxl_next_addr = sector_align_up(adxl_end);
    idx->adxl_next_addr = pool_wrap(idx->adxl_next_addr, FLASH_ADXL_BASE, FLASH_ADXL_END);

    return true;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  log_index_get_write_addrs
 * ═══════════════════════════════════════════════════════════════════════ */
void log_index_get_write_addrs(const log_index_t *idx,
                               uint32_t *hr, uint32_t *lr, uint32_t *adxl)
{
    if (hr)   *hr   = idx->hr_next_addr;
    if (lr)   *lr   = idx->lr_next_addr;
    if (adxl) *adxl = idx->adxl_next_addr;
}
