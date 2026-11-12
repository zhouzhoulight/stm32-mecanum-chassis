/* Scan assembly: per-point zero-crossing splitting (a packet may span 2 scans). */
#include "scan_builder.h"

#include <math.h>
#include <string.h>

void scan_builder_init(ScanBuilder *b)
{
    memset(b, 0, sizeof *b);
    b->current.partial = true; /* power-on half scan is honest */
    b->next_scan_id = 1U;
    b->first_scan = true;
}

static uint32_t pkt_duration_us(const Ld19Packet *pkt)
{
    uint32_t delta = ((uint32_t)pkt->end_cdeg + 36000U - (uint32_t)pkt->start_cdeg) % 36000U;
    if (pkt->speed_dps == 0U) return 0U;
    /* span_deg / speed_dps seconds; delta is 0.01 deg -> ms resolution safe */
    uint32_t us = (uint32_t)(((uint64_t)delta * 10000U + pkt->speed_dps/2U) / pkt->speed_dps);
    return us > 1000000U ? 1000000U : us;
}

static void start_scan(ScanBuilder *b, const Ld19Packet *pkt, uint32_t t0_us)
{
    memset(&b->current, 0, sizeof b->current);
    b->current.partial = b->first_scan || b->force_partial;
    b->current.speed_dps = pkt->speed_dps;
    b->current.t0_us = t0_us;
    b->force_partial = false;
}

static bool finish_scan(ScanBuilder *b, Ld19Scan *out)
{
    if (b->current.n == 0U) return false;
    b->current.scan_id = b->next_scan_id++;
    b->current.t1_us = b->current.points[b->current.n - 1U].t_us;
    b->completed_scans++;
    if (b->last_scan_t1_us != 0U) {
        uint32_t dt = app_us_diff(b->current.t1_us, b->last_scan_t1_us);
        if (dt >= 10000U && dt <= 1000000U) b->hz_x100 = 100000000U / dt;
    }
    b->last_scan_t1_us = b->current.t1_us;
    if (out) *out = b->current;
    b->first_scan = false;
    b->current.n = 0U;
    return true;
}

bool scan_builder_feed(ScanBuilder *b, const Ld19Packet *pkt, uint32_t rx_end_us,
                       Ld19Scan *completed)
{
    uint32_t dur = pkt_duration_us(pkt);
    uint32_t step = dur / (LD19_POINTS_PER_PACKET - 1U);
    bool done = false;

    for (uint32_t i = 0; i < LD19_POINTS_PER_PACKET; i++) {
        uint16_t angle = ld19_point_angle_cdeg(pkt, i);
        uint32_t t = rx_end_us - (uint32_t)(LD19_POINTS_PER_PACKET - 1U - i) * step;

        if (b->have_last_angle && angle < b->last_angle) {
            /* Angle decreased: previous scan ends exactly at the wrap. */
            done |= finish_scan(b, completed);
            start_scan(b, pkt, t);
        } else if (b->current.n == 0U) {
            start_scan(b, pkt, t);
        }
        b->have_last_angle = true;
        b->last_angle = angle;

        if (b->current.n >= SCAN_MAX_POINTS) {
            b->dropped_points++; /* bounded buffer: never write past the array */
            continue;
        }
        ScanPoint *pt = &b->current.points[b->current.n++];
        pt->angle_cdeg = angle;
        pt->distance_mm = pkt->points[i].distance_mm;
        pt->intensity = pkt->points[i].intensity;
        pt->t_us = t;
    }
    return done;
}

bool scan_builder_flush(ScanBuilder *b, Ld19Scan *completed)
{
    if (b->current.n == 0U) return false;
    b->current.partial = true;
    bool ok = finish_scan(b, completed);
    b->have_last_angle = false;
    b->force_partial = true;
    return ok;
}

uint32_t scan_builder_hz_x100(const ScanBuilder *b)
{
    return b->hz_x100;
}
