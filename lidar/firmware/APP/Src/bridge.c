/* RAW_BYTES bridge: original UART2 bytes, 240 B max / 10 ms flush. */
#include "bridge.h"

#include <string.h>

void bridge_init(Bridge *b, uint32_t now_us)
{
    memset(b, 0, sizeof *b);
    b->last_flush_us = now_us;
}

static uint32_t emit_chunk(Bridge *b, uint32_t now_us, uint8_t flags,
                           BridgeChunk *out, uint32_t cap)
{
    if (cap == 0U || b->n == 0U) return 0U;
    out->rx_end_us = b->last_byte_us;
    out->chunk_id = b->chunk_id++;
    out->flags = (uint8_t)(flags | 0x01U); /* bit0: rx_end_us is estimated */
    out->n = (uint8_t)b->n;
    memcpy(out->data, b->buf, b->n);
    if (b->n >= RAW_CHUNK_MAX) b->pending_flags = 0U;
    b->n = 0U;
    b->last_flush_us = now_us;
    return 1U;
}

static uint32_t flush_full(Bridge *b, uint32_t now_us, BridgeChunk *out, uint32_t cap)
{
    uint32_t made = 0;
    while (b->n >= RAW_CHUNK_MAX && made < cap) {
        made += emit_chunk(b, now_us, b->pending_flags, &out[made], cap - made);
        /* A discontinuity applies once: following chunks are contiguous. */
        b->pending_flags &= (uint8_t)~0x02U;
    }
    return made;
}

uint32_t bridge_feed(Bridge *b, const uint8_t *data, uint32_t len, uint32_t now_us,
                     bool discontinuity_before, bool uart_error,
                     BridgeChunk *out, uint32_t cap)
{
    uint32_t made = 0;
    if (discontinuity_before) b->pending_flags |= 0x02U;
    if (uart_error) b->pending_flags |= 0x04U;

    for (uint32_t i = 0; i < len; i++) {
        b->last_byte_us = now_us - (uint32_t)(((uint64_t)(len-i-1U)*10000000U)/LIDAR_BAUD);
        b->buf[b->n++] = data[i];
        if (b->n >= RAW_CHUNK_MAX) {
            made += flush_full(b, now_us, out + made, cap - made);
            if (made >= cap) break;
        }
    }
    if (b->n == 0U) {
        /* nothing pending; a pure discontinuity flag waits for the next bytes */
    }
    return made;
}

uint32_t bridge_tick(Bridge *b, uint32_t now_us, BridgeChunk *out)
{
    if (b->n == 0U) return 0U;
    if (app_us_diff(now_us, b->last_flush_us) < RAW_CHUNK_FLUSH_US) return 0U;
    return emit_chunk(b, now_us, b->pending_flags, out, 1U);
}

void bridge_mark_drop(Bridge *b, uint32_t bytes, bool uart_error)
{
    b->drop_bytes += bytes;
    b->pending_flags |= 0x02U;
    if (uart_error) b->pending_flags |= 0x04U;
}
