/* Exact LD19 47-byte parser: CRC8 0x4D, byte-stream with sliding resync. */
#include "ld19_parser.h"

#include <string.h>

static uint8_t crc_table[256];
static int crc_table_ready;

static void crc_table_init(void)
{
    /* MSB-first polynomial 0x4D, init 0, no final xor: identical to the vendor
     * lipkg.cpp 256-entry table (verified by the native test). */
    for (uint32_t i = 0; i < 256U; i++) {
        uint8_t crc = (uint8_t)i;
        for (uint32_t b = 0; b < 8U; b++) {
            crc = (uint8_t)((crc & 0x80U) ? ((uint8_t)(crc << 1) ^ 0x4DU)
                                          : (uint8_t)(crc << 1));
        }
        crc_table[i] = crc;
    }
    crc_table_ready = 1;
}

uint8_t ld19_crc8(const uint8_t *data, uint32_t len)
{
    uint8_t crc = 0;
    if (!crc_table_ready) crc_table_init();
    for (uint32_t i = 0; i < len; i++) crc = crc_table[(uint8_t)(crc ^ data[i])];
    return crc;
}

uint8_t ld19_crc8_bitwise(const uint8_t *data, uint32_t len)
{
    uint8_t crc = 0;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint32_t b = 0; b < 8U; b++)
            crc = (uint8_t)((crc & 0x80U) ? ((uint8_t)(crc << 1) ^ 0x4DU)
                                          : (uint8_t)(crc << 1));
    }
    return crc;
}

static uint16_t rd16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

uint16_t ld19_point_angle_cdeg(const Ld19Packet *pkt, uint32_t i)
{
    uint32_t delta = ((uint32_t)pkt->end_cdeg + 36000U - (uint32_t)pkt->start_cdeg) % 36000U;
    uint64_t acc = (uint64_t)pkt->start_cdeg * 11ULL + (uint64_t)delta * i;
    return (uint16_t)((acc / 11ULL) % 36000ULL);
}

static void packet_parse(Ld19Packet *pkt)
{
    pkt->speed_dps = rd16(&pkt->raw[2]);
    pkt->start_cdeg = rd16(&pkt->raw[4]);
    for (uint32_t i = 0; i < LD19_POINTS_PER_PACKET; i++) {
        pkt->points[i].distance_mm = rd16(&pkt->raw[6U + 3U * i]);
        pkt->points[i].intensity = pkt->raw[8U + 3U * i];
    }
    pkt->end_cdeg = rd16(&pkt->raw[42]);
    pkt->device_ms = rd16(&pkt->raw[44]);
    pkt->crc_ok = true;
}

void ld19_parser_init(Ld19Parser *p)
{
    memset(p, 0, sizeof *p);
}

void ld19_parser_reset(Ld19Parser *p)
{
    p->used = 0;
}

/* Drop bytes until the window starts with a plausible 0x54 0x2C candidate. */
static void resync_window(Ld19Parser *p)
{
    while (p->used > 0U) {
        if (p->used >= 2U && p->window[0] == LD19_HEADER && p->window[1] == LD19_VER_LEN)
            return;
        if (p->used == 1U && p->window[0] == LD19_HEADER)
            return; /* need the next byte to decide */
        memmove(&p->window[0], &p->window[1], (size_t)(p->used - 1U));
        p->used--;
        p->stats.ignored_bytes++;
    }
}

uint32_t ld19_parser_feed(Ld19Parser *p, const uint8_t *data, uint32_t len,
                          Ld19Packet *out, uint32_t cap)
{
    uint32_t produced = 0;

    for (uint32_t idx = 0; idx < len; idx++) {
        uint8_t byte = data[idx];

        if (p->used == 0U) {
            if (byte == LD19_HEADER) p->window[p->used++] = byte;
            else p->stats.ignored_bytes++;
            continue;
        }
        if (p->used == 1U) {
            if (byte == LD19_VER_LEN) {
                p->window[p->used++] = byte;
            } else {
                p->stats.ignored_bytes++;
                p->used = 0;
                if (byte == LD19_HEADER) p->window[p->used++] = byte;
            }
            continue;
        }

        p->window[p->used++] = byte;
        if (p->used < LD19_PACKET_SIZE) continue;

        uint8_t crc = ld19_crc8(p->window, LD19_PACKET_SIZE - 1U);
        if (crc == p->window[LD19_PACKET_SIZE - 1U]) {
            p->stats.valid++;
            if (produced < cap) {
                Ld19Packet *pkt = &out[produced++];
                memcpy(pkt->raw, p->window, LD19_PACKET_SIZE);
                packet_parse(pkt);
                pkt->end_in_feed = (uint16_t)(idx + 1U);
            }
            p->used = 0;
        } else {
            p->stats.crc_error++;
            p->stats.resync++;
            /* Slide by one byte (never blindly skip 47): the true header may
             * live inside the rejected candidate. */
            memmove(&p->window[0], &p->window[1], LD19_PACKET_SIZE - 1U);
            p->used = LD19_PACKET_SIZE - 1U;
            resync_window(p);
        }
    }
    return produced;
}
