#include "log_stream.h"
#include <string.h>

/* ═══════════════════════════════════════════════════════════════════════
 *  log_stream_init
 * ═══════════════════════════════════════════════════════════════════════ */
void log_stream_init(log_stream_t *s, uint8_t *ring_mem,
                     uint16_t ring_pages, uint16_t prelaunch_pages,
                     uint8_t rec_size, uint8_t recs_per_page,
                     uint32_t flash_base, uint32_t flash_end)
{
    memset(s, 0, sizeof(*s));

    s->ring_buf        = ring_mem;
    s->ring_pages      = ring_pages;
    s->prelaunch_pages = prelaunch_pages;
    s->rec_size        = rec_size;
    s->recs_per_page   = recs_per_page;

    s->flash_base      = flash_base;
    s->flash_addr      = flash_base;
    s->flash_end       = flash_end;
    s->erased_up_to    = flash_base;

    s->state           = LOG_IDLE;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  log_stream_start  (IDLE -> PAD)
 * ═══════════════════════════════════════════════════════════════════════ */
void log_stream_start(log_stream_t *s)
{
    s->head_page       = 0;
    s->drain_page      = 0;
    s->rec_idx         = 0;
    s->drop_count      = 0;
    s->err_count       = 0;
    s->records_written = 0;
    s->state           = LOG_PAD;

    /* Zero ring buffer so partial pages are deterministic */
    memset(s->ring_buf, 0xFF, (uint32_t)s->ring_pages * LOG_PAGE_SIZE);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  log_stream_push
 * ═══════════════════════════════════════════════════════════════════════ */
void log_stream_push(log_stream_t *s, const void *record)
{
    if (s->state == LOG_IDLE || s->state == LOG_FINAL)
        return;

    /* Write record into current slot in the ring page */
    uint16_t ring_idx = (uint16_t)(s->head_page % s->ring_pages);
    uint32_t offset   = (uint32_t)ring_idx * LOG_PAGE_SIZE
                      + (uint32_t)s->rec_idx * s->rec_size;
    memcpy(&s->ring_buf[offset], record, s->rec_size);
    s->rec_idx++;
    s->records_written++;

    /* Check if page is full */
    if (s->rec_idx >= s->recs_per_page) {
        s->rec_idx = 0;

        if (s->state == LOG_PAD) {
            /* Ring wraps freely in PAD mode */
            s->head_page++;
            /* Pad unused bytes in the page (for ADXL: 250 of 256 used) */
            uint16_t used = (uint16_t)s->recs_per_page * s->rec_size;
            if (used < LOG_PAGE_SIZE) {
                uint16_t next_ring_idx = (uint16_t)((s->head_page - 1) % s->ring_pages);
                uint32_t pad_off = (uint32_t)next_ring_idx * LOG_PAGE_SIZE + used;
                memset(&s->ring_buf[pad_off], 0xFF, LOG_PAGE_SIZE - used);
            }
        } else {
            /* LOG_DRAIN: linear advance */
            /* Overrun check: if head laps the drain pointer */
            if (s->head_page + 1 - s->drain_page >= s->ring_pages) {
                /* Ring overrun — lose this page of records */
                s->drop_count += s->recs_per_page;
                /* Don't advance head_page; data in ring is stale */
                return;
            }
            /* Pad unused bytes in page */
            uint16_t used = (uint16_t)s->recs_per_page * s->rec_size;
            if (used < LOG_PAGE_SIZE) {
                uint16_t cur_idx = (uint16_t)(s->head_page % s->ring_pages);
                uint32_t pad_off = (uint32_t)cur_idx * LOG_PAGE_SIZE + used;
                memset(&s->ring_buf[pad_off], 0xFF, LOG_PAGE_SIZE - used);
            }
            s->head_page++;
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════
 *  log_stream_launch  (PAD -> DRAIN)
 * ═══════════════════════════════════════════════════════════════════════ */
void log_stream_launch(log_stream_t *s)
{
    /*
     * We want the last prelaunch_pages COMPLETE pages before head_page.
     * head_page is the next page to be written (current partial page).
     *
     * If the ring has wrapped (head_page >= ring_pages), we have
     * ring_pages of data. Otherwise we have head_page pages.
     */
    uint32_t available;
    if (s->head_page >= s->ring_pages)
        available = s->ring_pages;
    else
        available = s->head_page;

    uint32_t keep = s->prelaunch_pages;
    if (keep > available)
        keep = available;

    s->drain_page = s->head_page - keep;
    s->state      = LOG_DRAIN;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  log_stream_has_page
 * ═══════════════════════════════════════════════════════════════════════ */
bool log_stream_has_page(const log_stream_t *s)
{
    if (s->state != LOG_DRAIN && s->state != LOG_FINAL)
        return false;

    return s->drain_page < s->head_page;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  log_stream_peek_page
 * ═══════════════════════════════════════════════════════════════════════ */
const uint8_t *log_stream_peek_page(const log_stream_t *s)
{
    uint16_t ring_idx = (uint16_t)(s->drain_page % s->ring_pages);
    return &s->ring_buf[(uint32_t)ring_idx * LOG_PAGE_SIZE];
}

/* ═══════════════════════════════════════════════════════════════════════
 *  log_stream_flash_addr
 * ═══════════════════════════════════════════════════════════════════════ */
uint32_t log_stream_flash_addr(const log_stream_t *s)
{
    return s->flash_addr;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  log_stream_page_done
 * ═══════════════════════════════════════════════════════════════════════ */
void log_stream_page_done(log_stream_t *s)
{
    s->drain_page++;
    s->flash_addr += LOG_PAGE_SIZE;

    /* Pool wrap */
    if (s->flash_addr >= s->flash_end)
        s->flash_addr = s->flash_base;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  log_stream_page_fail
 * ═══════════════════════════════════════════════════════════════════════ */
void log_stream_page_fail(log_stream_t *s)
{
    s->err_count++;
    /* Skip the page — don't retry, advance to next */
    s->drain_page++;
    s->flash_addr += LOG_PAGE_SIZE;

    if (s->flash_addr >= s->flash_end)
        s->flash_addr = s->flash_base;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  log_stream_finalize
 * ═══════════════════════════════════════════════════════════════════════ */
void log_stream_finalize(log_stream_t *s)
{
    if (s->state == LOG_IDLE || s->state == LOG_FINAL)
        return;

    /* If there's a partial page (rec_idx > 0), pad and advance */
    if (s->rec_idx > 0) {
        uint16_t ring_idx = (uint16_t)(s->head_page % s->ring_pages);
        uint32_t used     = (uint32_t)s->rec_idx * s->rec_size;
        uint32_t offset   = (uint32_t)ring_idx * LOG_PAGE_SIZE + used;
        memset(&s->ring_buf[offset], 0xFF, LOG_PAGE_SIZE - used);
        s->head_page++;
        s->rec_idx = 0;
    }

    s->state = LOG_FINAL;
}
