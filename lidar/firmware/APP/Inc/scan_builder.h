/* Scan assembly: per-point zero-crossing segmentation (packets may span scans). */
#ifndef SCAN_BUILDER_H
#define SCAN_BUILDER_H

#include <stdbool.h>
#include <stdint.h>
#include "app_config.h"
#include "ld19_parser.h"

#define SCAN_INVALID_HZ 0U

typedef struct {
    uint16_t angle_cdeg;
    uint16_t distance_mm;
    uint8_t intensity;
    uint32_t t_us;       /* estimated point receive time (see builder notes) */
} ScanPoint;

typedef struct {
    uint32_t scan_id;
    uint32_t n;
    bool partial;
    uint16_t speed_dps;
    uint32_t t0_us;
    uint32_t t1_us;
    ScanPoint points[SCAN_MAX_POINTS];
} Ld19Scan;

typedef struct {
    Ld19Scan current;
    bool have_last_angle;
    uint16_t last_angle;
    uint32_t next_scan_id;
    uint32_t completed_scans;
    uint32_t hz_x100;
    uint32_t last_scan_t1_us;
    bool force_partial;
    uint32_t dropped_points;
    bool first_scan;
} ScanBuilder;

void scan_builder_init(ScanBuilder *b);
/* Feed one valid LD19 packet received at rx_end_us. Returns true and fills
 * *completed when a scan boundary was crossed by this packet. A packet may
 * close the previous scan and start the next one; points are always split at
 * the exact angle wrap, never per packet. */
bool scan_builder_feed(ScanBuilder *b, const Ld19Packet *pkt, uint32_t rx_end_us,
                       Ld19Scan *completed);
/* Close a partial scan on capture stop/discontinuity; returns true if any
 * points were pending. */
bool scan_builder_flush(ScanBuilder *b, Ld19Scan *completed);
uint32_t scan_builder_hz_x100(const ScanBuilder *b);
#endif
