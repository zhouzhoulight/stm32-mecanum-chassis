/* Runtime counters / STATUS payload assembly (pure C). */
#ifndef STATUS_H
#define STATUS_H

#include <stdbool.h>
#include <stdint.h>
#include "app_config.h"
#include "protocol.h"

typedef struct {
    uint32_t rx_bytes;
    uint32_t valid_ld_packets;
    uint32_t ld_crc_errors;
    uint32_t parser_resyncs;
    uint32_t dma_overruns;
    uint32_t bridge_drop_bytes;
    uint32_t scan_count;
    uint16_t last_scan_points;
    uint16_t scan_hz_x100;
    uint32_t processing_max_us;
    uint32_t last_packet_us;
    bool capture_discontinuous;
    bool last_scan_partial;
    uint32_t start_us;
    uint32_t now_us;
    bool extrinsics_valid;
    bool pose_valid;
    bool deskew_enabled;
} StatusCounters;

void status_init(StatusCounters *s, uint32_t start_us);
void status_set_packet(StatusCounters *s, uint32_t now_us);
void status_set_scan(StatusCounters *s, uint16_t points, bool partial);
void status_build(const StatusCounters *s, StatusPayload *out);
#endif
