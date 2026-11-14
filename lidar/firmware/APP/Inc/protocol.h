/* LD19 bench host protocol v1: AA55 envelope, commands and payload builders.
 * Pure C (no HAL) so the native test compiles this exact source on the host. */
#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>
#include "app_config.h"

enum {
    CMD_INFO         = 0x60,
    CMD_STREAM       = 0x61,
    CMD_FILTER       = 0x62,
    CMD_STORAGE_INFO = 0x63,
    CMD_PWM          = 0x64,
    CMD_MOTION       = 0x65,
    CMD_TIME         = 0x66,
    CMD_SHADOW       = 0x67,
    MSG_INFO_REPLY   = 0xE0,
    MSG_ACK          = 0xE1,
    MSG_STORAGE      = 0xE3,
    MSG_RAW          = 0x70,
    MSG_FEATURES     = 0x71,
    MSG_STATUS       = 0x72,
    MSG_TRACKS       = 0x73,
    MSG_SHADOW       = 0x74,
    MSG_SEGMENTS     = 0x76, /* protocol2 additive, same-scan surface capsules */
    MSG_TIME         = 0xE6
};

enum {
    ACK_OK                = 0,
    ACK_BAD_LENGTH        = 1,
    ACK_OUT_OF_RANGE      = 2,
    ACK_BUSY              = 3,
    ACK_NOT_SUPPORTED     = 4,
    ACK_PERIPHERAL_ERROR  = 5
};

#define PB_NO_SCAN_ID 0xFFFFFFFFU

typedef struct {
    uint8_t cmd;
    uint8_t seq;
    uint8_t len;
    uint8_t payload[PB_PAYLOAD_MAX];
    bool valid;
} PbFrame;

typedef struct {
    uint8_t state;   /* 0=AA,1=55,2=CMD,3=SEQ,4=LEN,5=payload,6=XOR */
    uint8_t cmd, seq, len, got;
    uint8_t payload[PB_PAYLOAD_MAX];
    uint32_t frames;
    uint32_t xor_errors;
    uint32_t length_errors;
    uint32_t dropped_bytes;
} PbDecoder;

typedef struct {
    uint16_t r_min_mm;
    uint16_t r_max_mm;
    uint16_t cluster_gap_mm;
    uint8_t  intensity_min;
    uint8_t  min_points;
    uint8_t  confirm_scans;
    uint8_t  flags;
} FilterConfig;

typedef struct {
    uint32_t uptime_ms;
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
    uint16_t flags;
    uint16_t reserved;
} StatusPayload;

typedef struct {
    uint16_t id;
    uint8_t  kind;
    uint8_t  npoints;
    int16_t  x_mm;
    int16_t  y_mm;
    uint16_t radius_mm;
    uint8_t  intensity;
    uint8_t  flags;
    uint16_t fit_error_mm;
} PbFeature;

typedef struct {
    const char *fw;
    uint32_t hse_hz;
    uint32_t sys_hz;
    uint32_t host_baud;
    const char *board_profile;
    uint8_t  caps;
    uint32_t uptime_ms;
} InfoContext;

/* decoder */
void pb_decoder_init(PbDecoder *d);
void pb_decoder_reset(PbDecoder *d);
uint32_t pb_decoder_feed(PbDecoder *d, const uint8_t *data, uint32_t len,
                         PbFrame *out, uint32_t cap);
/* encoder: returns total frame bytes, 0 on overflow */
uint32_t pb_encode(uint8_t cmd, uint8_t seq, const uint8_t *payload, uint16_t len,
                   uint8_t *out, uint32_t cap);

FilterConfig filter_defaults(void);
bool filter_validate(const FilterConfig *f);
bool filter_from_payload(const uint8_t *p10, FilterConfig *out); /* range-checked */
void filter_to_payload(const FilterConfig *f, uint8_t p10[10]);

/* payload builders; all return payload length (0 on overflow/no data) */
uint32_t info_build_payload(const InfoContext *ctx, uint8_t *out, uint32_t cap);
uint32_t storage_build_payload(bool w25q_ok, const uint8_t jedec[3], bool eeprom_ok,
                               uint8_t eeprom_byte, uint8_t *out, uint32_t cap);
uint32_t ack_build_payload(uint8_t req_cmd, uint8_t status, uint32_t scan_id,
                           uint8_t out[8]);
uint32_t status_build_payload(const StatusPayload *st, uint8_t out[44]);
bool   status_parse_payload(const uint8_t in[44], StatusPayload *st);
uint32_t raw_build_payload(uint32_t rx_end_us, uint32_t chunk_id, uint8_t flags,
                           const uint8_t *data, uint32_t n, uint8_t *out, uint32_t cap);
uint32_t features_build_payload(uint32_t t_ms, uint32_t scan_id,
                                const PbFeature *f, uint32_t count,
                                uint8_t *out, uint32_t cap);

/* Testable command dispatch. send() must transmit one complete envelope.
 * storage_json() builds the 0xE3 JSON payload; may be NULL (NOT_SUPPORTED). */
typedef bool (*PbSendFn)(void *user, uint8_t cmd, uint8_t seq,
                         const uint8_t *payload, uint16_t len);
typedef uint32_t (*PbStorageJsonFn)(void *user, uint8_t *out, uint32_t cap);
typedef bool (*PbMotionFn)(void *user, const uint8_t *payload);
typedef uint32_t (*PbClockFn)(void *user);
typedef void (*PbShadowFn)(void *user, bool anchor);
typedef struct {
    PbSendFn send;
    void *user;
    PbStorageJsonFn storage_json;
    void *storage_user;
    PbMotionFn motion_input;
    PbClockFn clock_us;
    PbShadowFn shadow_control;
    FilterConfig filter;
    uint8_t stream;          /* 0,1,2 */
    uint32_t next_scan_id;   /* 0xFFFFFFFF when unknown */
    uint32_t filter_revision;
    InfoContext info;
} ProtocolContext;

void protocol_context_init(ProtocolContext *ctx, PbSendFn send, void *user);
void protocol_handle_frame(ProtocolContext *ctx, const PbFrame *f);
#endif
