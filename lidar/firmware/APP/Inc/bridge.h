/* RAW_BYTES bridge: unmodified UART2 bytes in <=240 byte chunks, 10 ms flush. */
#ifndef BRIDGE_H
#define BRIDGE_H

#include <stdbool.h>
#include <stdint.h>
#include "app_config.h"

typedef struct {
    uint32_t rx_end_us;
    uint32_t chunk_id;
    uint8_t flags;                 /* bit0 estimated, bit1 dirscontinuity, bit2 UART error */
    uint8_t n;
    uint8_t data[RAW_CHUNK_MAX];
} BridgeChunk;

typedef struct {
    uint8_t buf[RAW_CHUNK_MAX];
    uint16_t n;
    uint32_t chunk_id;
    uint32_t last_flush_us;
    uint32_t last_byte_us;
    uint32_t drop_bytes;
    uint8_t pending_flags;
} Bridge;

void bridge_init(Bridge *b, uint32_t now_us);
/* Feed raw bytes; writes up to cap completed chunks, returns chunk count. */
uint32_t bridge_feed(Bridge *b, const uint8_t *data, uint32_t len, uint32_t now_us,
                     bool discontinuity_before, bool uart_error,
                     BridgeChunk *out, uint32_t cap);
/* 10 ms inactivity flush; returns 1 and fills chunk when one is due. */
uint32_t bridge_tick(Bridge *b, uint32_t now_us, BridgeChunk *out);
/* Drop accounting for DMA overrun / discontinuity (never silently hides loss). */
void bridge_mark_drop(Bridge *b, uint32_t bytes, bool uart_error);
#endif
