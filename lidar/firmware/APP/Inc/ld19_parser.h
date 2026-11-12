/* LD19 47-byte packet byte-stream parser (CRC8 0x4D, sliding resync). */
#ifndef LD19_PARSER_H
#define LD19_PARSER_H

#include <stdbool.h>
#include <stdint.h>
#include "app_config.h"

typedef struct {
    uint16_t distance_mm;
    uint8_t intensity;
} Ld19Point;

typedef struct {
    uint8_t raw[LD19_PACKET_SIZE];
    bool crc_ok;
    uint16_t speed_dps;      /* degrees/second, NOT rpm */
    uint16_t start_cdeg;     /* 0.01 degree */
    uint16_t end_cdeg;
    uint16_t device_ms;      /* manufacturer resets at 30000 */
    uint16_t end_in_feed;    /* byte offset just after packet in last feed */
    Ld19Point points[LD19_POINTS_PER_PACKET];
} Ld19Packet;

typedef struct {
    uint32_t valid;          /* CRC-correct packets */
    uint32_t crc_error;      /* 47-byte candidates with bad CRC */
    uint32_t resync;         /* forced byte-slide recovery events */
    uint32_t ignored_bytes;  /* bytes dropped while searching for 0x54 2C */
} Ld19ParserStats;

typedef struct {
    uint8_t window[LD19_PACKET_SIZE];
    uint8_t used;
    Ld19ParserStats stats;
} Ld19Parser;

uint8_t ld19_crc8(const uint8_t *data, uint32_t len);
uint8_t ld19_crc8_bitwise(const uint8_t *data, uint32_t len);

void ld19_parser_init(Ld19Parser *p);
void ld19_parser_reset(Ld19Parser *p);
/* Feed arbitrary bytes; returns number of valid packets written to out[] (cap).
 * CRC-bad candidate packets are counted but not returned; callers keep the raw
 * byte stream separately (RAW_BYTES bridge). */
uint32_t ld19_parser_feed(Ld19Parser *p, const uint8_t *data, uint32_t len,
                          Ld19Packet *out, uint32_t cap);
/* Point i angle in 0.01 deg: delta=(end-start+36000)%36000;
 * angle_i=(start+delta*i/11)%36000 computed with 64-bit intermediates. */
uint16_t ld19_point_angle_cdeg(const Ld19Packet *pkt, uint32_t i);
/* Device millisecond unwrap helper (30000 wrap). */
static inline int32_t ld19_device_ms_diff(uint16_t newer, uint16_t older)
{
    int32_t d = (int32_t)newer - (int32_t)older;
    if (d > (int32_t)(LD19_DEVICE_MS_WRAP / 2)) d -= (int32_t)LD19_DEVICE_MS_WRAP;
    if (d < -(int32_t)(LD19_DEVICE_MS_WRAP / 2)) d += (int32_t)LD19_DEVICE_MS_WRAP;
    return d;
}
#endif
