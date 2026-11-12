/* Runtime counters -> STATUS payload (pure C). */
#include "status.h"

#include <string.h>

void status_init(StatusCounters *s, uint32_t start_us)
{
    memset(s, 0, sizeof *s);
    s->start_us = start_us;
    s->last_packet_us = start_us;
}

void status_set_packet(StatusCounters *s, uint32_t now_us)
{
    s->last_packet_us = now_us;
}

void status_set_scan(StatusCounters *s, uint16_t points, bool partial)
{
    s->scan_count++;
    s->last_scan_points = points;
    s->last_scan_partial = partial;
}

void status_build(const StatusCounters *s, StatusPayload *out)
{
    memset(out, 0, sizeof *out);
    out->uptime_ms = s->now_us / 1000U;
    out->rx_bytes = s->rx_bytes;
    out->valid_ld_packets = s->valid_ld_packets;
    out->ld_crc_errors = s->ld_crc_errors;
    out->parser_resyncs = s->parser_resyncs;
    out->dma_overruns = s->dma_overruns;
    out->bridge_drop_bytes = s->bridge_drop_bytes;
    out->scan_count = s->scan_count;
    out->last_scan_points = s->last_scan_points;
    out->scan_hz_x100 = s->scan_hz_x100;
    out->processing_max_us = s->processing_max_us;
    uint16_t flags = 0;
    if (s->now_us - s->last_packet_us > 500000U) flags |= ST_FLAG_NO_PACKET_500MS;
    if (s->capture_discontinuous) flags |= ST_FLAG_DISCONTINUOUS;
    if (s->last_scan_partial) flags |= ST_FLAG_PARTIAL_SCAN;
    if (s->extrinsics_valid) flags |= ST_FLAG_EXTRINSICS_VALID;
    if (s->pose_valid) flags |= ST_FLAG_POSE_VALID;
    if (s->deskew_enabled) flags |= ST_FLAG_DESKEW;
    out->flags = flags;
    out->reserved = 0;
}
