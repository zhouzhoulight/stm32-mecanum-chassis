/* Host protocol v1: AA55 envelope, commands, payload builders (pure C). */
#include "protocol.h"

#include <stdio.h>
#include <string.h>

void pb_decoder_init(PbDecoder *d)
{
    memset(d, 0, sizeof *d);
}

void pb_decoder_reset(PbDecoder *d)
{
    d->state = 0;
    d->got = 0;
}

static void decoder_restart(PbDecoder *d)
{
    d->state = 0;
    d->got = 0;
}

uint32_t pb_decoder_feed(PbDecoder *d, const uint8_t *data, uint32_t len,
                         PbFrame *out, uint32_t cap)
{
    uint32_t produced = 0;
    for (uint32_t i = 0; i < len; i++) {
        uint8_t b = data[i];
        switch (d->state) {
        case 0:
            if (b == 0xAA) d->state = 1;
            else d->dropped_bytes++;
            break;
        case 1:
            if (b == 0x55) d->state = 2;
            else { d->dropped_bytes++; d->state = (b == 0xAA) ? 1U : 0U; }
            break;
        case 2:
            d->cmd = b; d->state = 3; break;
        case 3:
            d->seq = b; d->state = 4; break;
        case 4:
            d->len = b;
            d->got = 0;
            if (d->len > PB_PAYLOAD_MAX) {
                d->length_errors++;
                decoder_restart(d);
            } else if (d->len == 0U) {
                d->state = 6; /* straight to XOR */
            } else {
                d->state = 5;
            }
            break;
        case 5:
            d->payload[d->got++] = b;
            if (d->got >= d->len) d->state = 6;
            break;
        case 6: {
            uint8_t x = (uint8_t)(0xAAU ^ 0x55U ^ d->cmd ^ d->seq ^ d->len);
            for (uint8_t k = 0; k < d->len; k++) x ^= d->payload[k];
            x ^= b;
            if (x == 0U) {
                d->frames++;
                if (produced < cap) {
                    PbFrame *f = &out[produced++];
                    memset(f, 0, sizeof *f);
                    f->cmd = d->cmd;
                    f->seq = d->seq;
                    f->len = d->len;
                    memcpy(f->payload, d->payload, d->len);
                    f->valid = true;
                }
            } else {
                d->xor_errors++;
            }
            decoder_restart(d);
            break;
        }
        default:
            decoder_restart(d);
            break;
        }
    }
    return produced;
}

uint32_t pb_encode(uint8_t cmd, uint8_t seq, const uint8_t *payload, uint16_t len,
                   uint8_t *out, uint32_t cap)
{
    if (len > PB_PAYLOAD_MAX) return 0U;
    uint32_t total = 6U + len;
    if (total > cap) return 0U;
    out[0] = 0xAA; out[1] = 0x55; out[2] = cmd; out[3] = seq; out[4] = (uint8_t)len;
    if (len) memcpy(&out[5], payload, len);
    uint8_t x = 0;
    for (uint32_t i = 0; i < total - 1U; i++) x ^= out[i];
    out[total - 1U] = x;
    return total;
}

FilterConfig filter_defaults(void)
{
    FilterConfig f;
    f.r_min_mm = 20; f.r_max_mm = 4000; f.cluster_gap_mm = 80;
    f.intensity_min = 0; f.min_points = 2; f.confirm_scans = 3; f.flags = 0;
    return f;
}

bool filter_validate(const FilterConfig *f)
{
    if (!f) return false;
    if (f->r_min_mm > 12000U) return false;
    if (f->r_max_mm < 20U || f->r_max_mm > 12000U) return false;
    if (f->r_min_mm >= f->r_max_mm) return false;
    if (f->cluster_gap_mm < 10U || f->cluster_gap_mm > 500U) return false;
    if (f->min_points < 1U || f->min_points > 12U) return false;
    if (f->confirm_scans < 1U || f->confirm_scans > 20U) return false;
    if (f->flags != 0U) return false;
    return true;
}

bool filter_from_payload(const uint8_t *p10, FilterConfig *out)
{
    if (!p10 || !out) return false;
    FilterConfig f;
    f.r_min_mm = (uint16_t)((uint16_t)p10[0] | ((uint16_t)p10[1] << 8));
    f.r_max_mm = (uint16_t)((uint16_t)p10[2] | ((uint16_t)p10[3] << 8));
    f.intensity_min = p10[4];
    f.cluster_gap_mm = (uint16_t)((uint16_t)p10[5] | ((uint16_t)p10[6] << 8));
    f.min_points = p10[7];
    f.confirm_scans = p10[8];
    f.flags = p10[9];
    if (!filter_validate(&f)) return false;
    *out = f;
    return true;
}

void filter_to_payload(const FilterConfig *f, uint8_t p10[10])
{
    p10[0] = (uint8_t)(f->r_min_mm & 0xFFU); p10[1] = (uint8_t)(f->r_min_mm >> 8);
    p10[2] = (uint8_t)(f->r_max_mm & 0xFFU); p10[3] = (uint8_t)(f->r_max_mm >> 8);
    p10[4] = f->intensity_min;
    p10[5] = (uint8_t)(f->cluster_gap_mm & 0xFFU); p10[6] = (uint8_t)(f->cluster_gap_mm >> 8);
    p10[7] = f->min_points; p10[8] = f->confirm_scans; p10[9] = f->flags;
}

static void wr16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v & 0xFFU); p[1] = (uint8_t)(v >> 8); }
static void wr32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFU); p[1] = (uint8_t)((v >> 8) & 0xFFU);
    p[2] = (uint8_t)((v >> 16) & 0xFFU); p[3] = (uint8_t)((v >> 24) & 0xFFU);
}
static uint16_t rd16(const uint8_t *p) { return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8)); }
static uint32_t rd32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

uint32_t info_build_payload(const InfoContext *ctx, uint8_t *out, uint32_t cap)
{
    if (!ctx || !out) return 0U;
    int n = snprintf((char *)out, cap,
        "{\"fw\":\"%s\",\"protocol_version\":%d,\"mcu\":\"%s\","
        "\"hse_hz\":%lu,\"sys_hz\":%lu,\"host_baud\":%lu,"
        "\"board_profile\":\"%s\",\"caps\":%u,\"uptime_ms\":%lu}",
        ctx->fw, PROTOCOL_VERSION, MCU_NAME,
        (unsigned long)ctx->hse_hz, (unsigned long)ctx->sys_hz,
        (unsigned long)ctx->host_baud, ctx->board_profile, (unsigned)ctx->caps,
        (unsigned long)ctx->uptime_ms);
    if (n <= 0 || (uint32_t)n >= cap || (uint32_t)n > 240U) return 0U;
    return (uint32_t)n;
}

uint32_t storage_build_payload(bool w25q_ok, const uint8_t jedec[3], bool eeprom_ok,
                               uint8_t eeprom_byte, uint8_t *out, uint32_t cap)
{
    char jedec_txt[8] = "??";
    if (w25q_ok && jedec)
        snprintf(jedec_txt, sizeof jedec_txt, "%02X%02X%02X", jedec[0], jedec[1], jedec[2]);
    int n = snprintf((char *)out, cap,
        "{\"storage_version\":1,\"w25q_present\":%s,\"w25q_jedec\":\"%s\","
        "\"eeprom_present\":%s,\"eeprom_addr\":\"0x50\",\"eeprom_byte0\":%u,"
        "\"write_policy\":\"read_only\"}",
        w25q_ok ? "true" : "false", jedec_txt,
        eeprom_ok ? "true" : "false", (unsigned)eeprom_byte);
    if (n <= 0 || (uint32_t)n >= cap || (uint32_t)n > 240U) return 0U;
    return (uint32_t)n;
}

uint32_t ack_build_payload(uint8_t req_cmd, uint8_t status, uint32_t scan_id,
                           uint8_t out[8])
{
    out[0] = req_cmd; out[1] = status; wr32(&out[2], scan_id); wr16(&out[6], 0);
    return 8U;
}

uint32_t status_build_payload(const StatusPayload *st, uint8_t out[44])
{
    if (!st || !out) return 0U;
    wr32(&out[0], st->uptime_ms);
    wr32(&out[4], st->rx_bytes);
    wr32(&out[8], st->valid_ld_packets);
    wr32(&out[12], st->ld_crc_errors);
    wr32(&out[16], st->parser_resyncs);
    wr32(&out[20], st->dma_overruns);
    wr32(&out[24], st->bridge_drop_bytes);
    wr32(&out[28], st->scan_count);
    wr16(&out[32], st->last_scan_points);
    wr16(&out[34], st->scan_hz_x100);
    wr32(&out[36], st->processing_max_us);
    wr16(&out[40], st->flags);
    wr16(&out[42], st->reserved);
    return 44U;
}

bool status_parse_payload(const uint8_t in[44], StatusPayload *st)
{
    if (!in || !st) return false;
    st->uptime_ms = rd32(&in[0]);
    st->rx_bytes = rd32(&in[4]);
    st->valid_ld_packets = rd32(&in[8]);
    st->ld_crc_errors = rd32(&in[12]);
    st->parser_resyncs = rd32(&in[16]);
    st->dma_overruns = rd32(&in[20]);
    st->bridge_drop_bytes = rd32(&in[24]);
    st->scan_count = rd32(&in[28]);
    st->last_scan_points = rd16(&in[32]);
    st->scan_hz_x100 = rd16(&in[34]);
    st->processing_max_us = rd32(&in[36]);
    st->flags = rd16(&in[40]);
    st->reserved = rd16(&in[42]);
    return true;
}

uint32_t raw_build_payload(uint32_t rx_end_us, uint32_t chunk_id, uint8_t flags,
                           const uint8_t *data, uint32_t n, uint8_t *out, uint32_t cap)
{
    if (!data || n < 1U || n > RAW_CHUNK_MAX) return 0U;
    if (cap < 10U + n) return 0U;
    wr32(&out[0], rx_end_us);
    wr32(&out[4], chunk_id);
    out[8] = flags;
    out[9] = (uint8_t)n;
    memcpy(&out[10], data, n);
    return 10U + n;
}

uint32_t features_build_payload(uint32_t t_ms, uint32_t scan_id,
                                const PbFeature *f, uint32_t count,
                                uint8_t *out, uint32_t cap)
{
    if (count > FEATURE_MAX) return 0U;
    uint32_t need = 9U + 14U * count;
    if (need > cap || need > PB_PAYLOAD_MAX) return 0U;
    wr32(&out[0], t_ms);
    wr32(&out[4], scan_id);
    out[8] = (uint8_t)count;
    for (uint32_t i = 0; i < count; i++) {
        uint8_t *p = &out[9 + 14U * i];
        wr16(&p[0], f[i].id);
        p[2] = f[i].kind;
        p[3] = f[i].npoints;
        wr16(&p[4], (uint16_t)f[i].x_mm);
        wr16(&p[6], (uint16_t)f[i].y_mm);
        wr16(&p[8], f[i].radius_mm);
        p[10] = f[i].intensity;
        p[11] = f[i].flags;
        wr16(&p[12], f[i].fit_error_mm);
    }
    return need;
}

/* ---------------- command dispatch (testable) ---------------- */
void protocol_context_init(ProtocolContext *ctx, PbSendFn send, void *user)
{
    memset(ctx, 0, sizeof *ctx);
    ctx->send = send;
    ctx->user = user;
    ctx->filter = filter_defaults();
    ctx->stream = 0;
    ctx->next_scan_id = PB_NO_SCAN_ID;
    ctx->info.fw = APP_VERSION;
    ctx->info.hse_hz = HSE_HZ;
    ctx->info.sys_hz = SYS_HZ;
    ctx->info.host_baud = HOST_BAUD;
    ctx->info.board_profile = BOARD_PROFILE;
    ctx->info.caps = APP_CAPS;
}

static void reply_ack(ProtocolContext *ctx, const PbFrame *f, uint8_t status)
{
    uint8_t payload[8];
    (void)ack_build_payload(f->cmd, status, ctx->next_scan_id, payload);
    if (ctx->send) (void)ctx->send(ctx->user, MSG_ACK, f->seq, payload, 8U);
}

void protocol_handle_frame(ProtocolContext *ctx, const PbFrame *f)
{
    if (!ctx || !f || !f->valid) return;
    switch (f->cmd) {
    case CMD_INFO: {
        if (f->len != 0U) { reply_ack(ctx, f, ACK_BAD_LENGTH); return; }
        uint8_t payload[PB_PAYLOAD_MAX];
        uint32_t n = info_build_payload(&ctx->info, payload, sizeof payload);
        if (n == 0U) { reply_ack(ctx, f, ACK_BUSY); return; }
        if (ctx->send) (void)ctx->send(ctx->user, MSG_INFO_REPLY, f->seq, payload, (uint16_t)n);
        return;
    }
    case CMD_STREAM: {
        if (f->len != 1U) { reply_ack(ctx, f, ACK_BAD_LENGTH); return; }
        if (f->payload[0] > 2U) { reply_ack(ctx, f, ACK_OUT_OF_RANGE); return; }
        ctx->stream = f->payload[0];
        reply_ack(ctx, f, ACK_OK);
        return;
    }
    case CMD_FILTER: {
        if (f->len != 10U) { reply_ack(ctx, f, ACK_BAD_LENGTH); return; }
        FilterConfig cfg;
        if (!filter_from_payload(f->payload, &cfg)) { reply_ack(ctx, f, ACK_OUT_OF_RANGE); return; }
        ctx->filter = cfg; /* applied at next scan boundary by the caller */
        ctx->filter_revision++;
        reply_ack(ctx, f, ACK_OK);
        return;
    }
    case CMD_STORAGE_INFO: {
        if (f->len != 0U) { reply_ack(ctx, f, ACK_BAD_LENGTH); return; }
        if (!ctx->storage_json) { reply_ack(ctx, f, ACK_NOT_SUPPORTED); return; }
        uint8_t payload[PB_PAYLOAD_MAX];
        uint32_t n = ctx->storage_json(ctx->storage_user, payload, sizeof payload);
        if (n == 0U) { reply_ack(ctx, f, ACK_NOT_SUPPORTED); return; }
        if (ctx->send) (void)ctx->send(ctx->user, MSG_STORAGE, f->seq, payload, (uint16_t)n);
        return;
    }
    case CMD_PWM:
        if (f->len != 3U) { reply_ack(ctx, f, ACK_BAD_LENGTH); return; }
        /* Phase 1: PWM control is not implemented and PB8 is not initialised. */
        reply_ack(ctx, f, ACK_NOT_SUPPORTED);
        return;
    case CMD_SHADOW:
        if(f->len != 1U) { reply_ack(ctx,f,ACK_BAD_LENGTH); return; }
        if(f->payload[0]>1U) { reply_ack(ctx,f,ACK_OUT_OF_RANGE); return; }
        if(!ctx->shadow_control) { reply_ack(ctx,f,ACK_NOT_SUPPORTED); return; }
        ctx->shadow_control(ctx->user,f->payload[0]!=0);
        reply_ack(ctx,f,ACK_OK); return;
    case CMD_MOTION:
        if (f->len != 20U) { reply_ack(ctx, f, ACK_BAD_LENGTH); return; }
        if (!ctx->motion_input) { reply_ack(ctx, f, ACK_NOT_SUPPORTED); return; }
        reply_ack(ctx, f, ctx->motion_input(ctx->user, f->payload) ? ACK_OK : ACK_OUT_OF_RANGE);
        return;
    case CMD_TIME: {
        if (f->len != 4U) { reply_ack(ctx, f, ACK_BAD_LENGTH); return; }
        if (!ctx->clock_us) { reply_ack(ctx, f, ACK_NOT_SUPPORTED); return; }
        uint8_t payload[12];
        uint32_t received_us = ctx->clock_us(ctx->user);
        memcpy(payload, f->payload, 4);
        wr32(payload + 4, received_us);
        wr32(payload + 8, ctx->clock_us(ctx->user));
        if (ctx->send) (void)ctx->send(ctx->user, MSG_TIME, f->seq, payload, 12);
        return;
    }
    default:
        reply_ack(ctx, f, ACK_NOT_SUPPORTED);
        return;
    }
}
