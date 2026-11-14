/* Application orchestration: non-blocking main-loop pipeline.
 * ISR/DMA only move bytes and sample time; parsing, scan building, clustering,
 * protocol and storage diagnostics all run here. */
#include "app.h"

#include <string.h>

#include "board_clock.h"
#include "board_config.h"
#include "bridge.h"
#include "features.h"
#include "shadow_localizer.h"
#include "ld19_parser.h"
#include "protocol.h"
#include "scan_builder.h"
#include "status.h"
#include "storage_diag.h"
#include "timebase.h"
#include "usart1.h"
#include "usart2.h"
#include "vehicle_uart.h"

typedef struct {
    BoardClockInfo clock;
    Ld19Parser parser;
    PbDecoder decoder, vehicle_decoder;
    bool vehicle_enabled;
    uint32_t vehicle_drops;
    ScanBuilder builder;
    Bridge bridge;
    FeatureTracker tracker;
    MotionHistory motion;
    ShadowLocalizer shadow;
    StatusCounters status;
    ProtocolContext pctx;
    uint32_t next_status_us;
    uint32_t start_us;
    uint32_t proc_max_us;
    uint8_t raw_seq, status_seq, feature_seq, track_seq, shadow_seq;
    uint8_t prev_stream;
    uint32_t applied_filter_revision;
} AppState;

static AppState s;
/* Front150deg centred on native270deg:195..345. Reapply after every reset. */
static void installed_features_init(void){
 features_init_installed(&s.tracker);
}

static bool app_send_message(uint8_t cmd, uint8_t seq, const uint8_t *payload, uint16_t len)
{
    uint8_t frame[PB_FRAME_MAX];
    uint32_t n = pb_encode(cmd, seq, payload, len, frame, sizeof frame);
    if (n == 0U) return false;
    return usart1_send(frame, n);
}

static bool vehicle_send(uint8_t cmd,uint8_t seq,const uint8_t *p,uint16_t len){
 uint8_t frame[PB_FRAME_MAX];uint32_t n=pb_encode(cmd,seq,p,len,frame,sizeof frame);
 bool priority=cmd==MSG_TIME||cmd==0xE8;
 bool ok=n&&(priority?vehicle_uart_send_priority(frame,n):vehicle_uart_send(frame,n));
 if(!ok)s.vehicle_drops++;
 return ok;
}
static bool app_pb_send(void *user, uint8_t cmd, uint8_t seq,
                        const uint8_t *payload, uint16_t len)
{
    (void)user;
    return app_send_message(cmd, seq, payload, len);
}

static uint32_t app_storage_json(void *user, uint8_t *out, uint32_t cap)
{
    (void)user;
    const StorageDiag *d = storage_diag_get();
    if (!d || !d->fresh) return 0U;
    return storage_build_payload(d->w25q_ok, d->w25q_jedec, d->eeprom_ok,
                                 d->eeprom_byte0, out, cap);
}

static void app_send_raw(const BridgeChunk *c)
{
    uint8_t payload[PB_PAYLOAD_MAX];
    uint32_t n = raw_build_payload(c->rx_end_us, c->chunk_id, c->flags,
                                   c->data, c->n, payload, sizeof payload);
    if (n == 0U) return;
    if (!app_send_message(MSG_RAW, s.raw_seq++, payload, (uint16_t)n))
        bridge_mark_drop(&s.bridge,c->n,false);
}

static void app_send_status(uint32_t now_us)
{
    StatusPayload sp;
    s.status.now_us = now_us;
    s.status.scan_hz_x100 = (uint16_t)scan_builder_hz_x100(&s.builder);
    s.status.processing_max_us = s.proc_max_us;
    s.status.ld_crc_errors = s.parser.stats.crc_error;
    s.status.parser_resyncs = s.parser.stats.resync;
    s.status.rx_bytes = usart2_stats()->rx_bytes;
    s.status.dma_overruns = usart2_stats()->overrun_bytes;
    s.status.bridge_drop_bytes = s.bridge.drop_bytes;
    status_build(&s.status, &sp);
    sp.uptime_ms=HAL_GetTick();
    uint8_t payload[44];
    (void)status_build_payload(&sp, payload);
    (void)app_send_message(MSG_STATUS, s.status_seq++, payload, 44U);
}

static void app_on_scan(const Ld19Scan *scan, uint32_t now_us)
{
    status_set_scan(&s.status, (uint16_t)(scan->n > 65535U ? 65535U : scan->n), scan->partial);
    if (s.pctx.stream != 2U && !s.vehicle_enabled) return;
    if(s.applied_filter_revision!=s.pctx.filter_revision){
        installed_features_init();shadow_reset(&s.shadow);
        s.applied_filter_revision=s.pctx.filter_revision;
    }
    PbFeature feats[FEATURE_MAX];
    uint32_t n = features_extract_motion(&s.tracker, scan, &s.pctx.filter, &s.motion, feats, FEATURE_MAX);
    s.status.pose_valid = s.tracker.previous_pose_valid;
    s.status.deskew_enabled = s.tracker.deskew_valid;
    uint8_t payload[PB_PAYLOAD_MAX];
    uint32_t len = features_build_payload(now_us / 1000U, scan->scan_id, feats, n,
                                          payload, sizeof payload);
    if (len == 0U) return;
    if(s.pctx.stream==2U) (void)app_send_message(MSG_FEATURES, s.feature_seq++, payload, (uint16_t)len);
    /* Explicit native FOV metadata, independent of PC display angle settings. */
    uint8_t view[8]={0x2C,0x4C,0xC4,0x86,0,0,0,0}; /*19500..34500 LE; no rear slot*/
    memcpy(view+4,&scan->t1_us,4);
    if(s.pctx.stream==2U)(void)app_send_message(0x75,s.track_seq++,view,sizeof view);
    if(s.vehicle_enabled)(void)vehicle_send(0x75,s.track_seq++,view,sizeof view);
    /* Compact current-scan line capsules first; the main controller binds
     * endpoints to TRACKS by scan timestamp, scan_id AND track_id. */
    for(uint8_t b=0;b<features_segment_batches(&s.tracker);b++){
        len=features_segments_payload(&s.tracker,scan->t1_us,scan->scan_id,b,payload,sizeof payload);
        if(len){
            uint8_t seq=s.track_seq++;
            if(s.pctx.stream==2U)(void)app_send_message(MSG_SEGMENTS,seq,payload,(uint16_t)len);
            if(s.vehicle_enabled)(void)vehicle_send(MSG_SEGMENTS,seq,payload,(uint16_t)len);
        }
    }
    for (uint8_t b=0; b<features_track_batches(&s.tracker); b++) {
        len=features_tracks_payload(&s.tracker,scan->t1_us,scan->scan_id,b,payload,sizeof payload);
        if(len){
            uint8_t seq=s.track_seq++;
            if(s.pctx.stream==2U)(void)app_send_message(MSG_TRACKS,seq,payload,(uint16_t)len);
            if(s.vehicle_enabled)(void)vehicle_send(MSG_TRACKS,seq,payload,(uint16_t)len);
        }
    }
    shadow_update(&s.shadow,&s.tracker);
    len=shadow_payload(&s.shadow,scan->t1_us,payload);
    (void)app_send_message(MSG_SHADOW,s.shadow_seq++,payload,(uint16_t)len);
}

static bool app_motion(void *user, const uint8_t *p)
{
    (void)user;
    MotionPose pose;
    memcpy(&pose.t_us,p,4); memcpy(&pose.x_mm,p+4,4); memcpy(&pose.y_mm,p+8,4);
    memcpy(&pose.yaw_rad,p+12,4); memcpy(&pose.sigma_mm,p+16,2); memcpy(&pose.sigma_yaw_mrad,p+18,2);
    /* Reject timestamps outside the retained recent history or far in future. */
    int32_t age=(int32_t)(timebase_us()-pose.t_us);
    if(age < -30000 || age > 1000000) return false;
    return motion_history_add(&s.motion,&pose);
}
static uint32_t app_clock(void *user) { (void)user; return timebase_us(); }
static void app_shadow(void *user,bool anchor) {
    (void)user;
    if(anchor)shadow_request_anchor(&s.shadow);else shadow_reset(&s.shadow);
}

void app_init(void)
{
    memset(&s, 0, sizeof s);
    s.start_us = timebase_us();

    if (!board_clock_init(&s.clock)) {
        /* The whole design assumes HSE25/PLL 168 MHz; refuse to run silently. */
        Error_Handler();
    }
    if (!timebase_init()) Error_Handler();
    if (!usart1_init()) Error_Handler();
    if (!usart2_init()) Error_Handler();
    if (!vehicle_uart_init()) Error_Handler();
    pb_decoder_init(&s.vehicle_decoder);
    storage_diag_init();

    ld19_parser_init(&s.parser);
    pb_decoder_init(&s.decoder);
    scan_builder_init(&s.builder);
    bridge_init(&s.bridge, s.start_us);
    installed_features_init();
    motion_history_init(&s.motion);
    shadow_reset(&s.shadow);
    status_init(&s.status, s.start_us);
    protocol_context_init(&s.pctx, app_pb_send, &s);
    s.pctx.storage_json = app_storage_json;
    s.pctx.storage_user = &s;
    s.pctx.motion_input = app_motion;
    s.pctx.clock_us = app_clock;
    s.pctx.shadow_control = app_shadow;
    s.pctx.info.fw = APP_VERSION;
    s.pctx.info.hse_hz = s.clock.hse_hz;
    s.pctx.info.sys_hz = s.clock.sysclk_hz;
    s.pctx.info.host_baud = HOST_BAUD;
    s.pctx.info.board_profile = BOARD_PROFILE;
    s.pctx.info.caps = APP_CAPS;
    s.pctx.next_scan_id = PB_NO_SCAN_ID;
    s.next_status_us = s.start_us;
    s.prev_stream = 0U;
    usart1_service();
}

void app_poll(void)
{
    uint32_t c0 = timebase_cycles();
    uint32_t now = timebase_us();

    /* Vehicle commands are isolated from PC stream/filter/storage commands.
     * 0x68 session token resets only motion history, never PC capture state. */
    uint8_t vb[64];
    if(vehicle_uart_fault())pb_decoder_reset(&s.vehicle_decoder);
    for(unsigned guard=0;guard<8;guard++){
      uint32_t n=vehicle_uart_read(vb,sizeof vb);if(!n)break;
      PbFrame frames[4];uint32_t nf=pb_decoder_feed(&s.vehicle_decoder,vb,n,frames,4);
      for(unsigned j=0;j<nf;j++){
        PbFrame *f=&frames[j];
        if(f->cmd==0x66 && f->len==4){
          uint8_t p[12];memcpy(p,f->payload,4);uint32_t t=timebase_us();memcpy(p+4,&t,4);
          t=timebase_us();memcpy(p+8,&t,4);(void)vehicle_send(0xE6,f->seq,p,12);
        }else if(f->cmd==0x68 && f->len==5 && f->payload[4]<=1){
          static uint32_t session;uint32_t token;memcpy(&token,f->payload,4);
          if(token!=session||f->payload[4]==0){motion_history_init(&s.motion);installed_features_init();shadow_reset(&s.shadow);session=token;}
          s.vehicle_enabled=f->payload[4]!=0;
          (void)vehicle_send(0xE8,f->seq,f->payload,5);
        }else if(f->cmd==0x65 && f->len==20 && s.vehicle_enabled){(void)app_motion(NULL,f->payload);}
      }
    }
    /* 1. PC commands (small, interrupt-fed ring; framing is the outer AA55). */
    uint8_t pcbuf[64];
    for (unsigned guard = 0; guard < 8U; guard++) {
        uint32_t pn = usart1_read(pcbuf, sizeof pcbuf);
        if (pn == 0U) break;
        PbFrame frames[4];
        uint32_t fn = pb_decoder_feed(&s.decoder, pcbuf, pn, frames, 4U);
        for (uint32_t i = 0; i < fn; i++) {
            s.pctx.info.uptime_ms = HAL_GetTick();
            s.pctx.next_scan_id = (s.pctx.stream == 0U) ? PB_NO_SCAN_ID
                                                        : s.builder.next_scan_id;
            protocol_handle_frame(&s.pctx, &frames[i]);
        }
    }

    /* Capture start/discontinuity: first scan after enabling is partial. */
    if (s.pctx.stream != s.prev_stream) {
        if (s.pctx.stream != 0U) {
            s.builder.force_partial = true;
            s.bridge.pending_flags |= 0x02U;
            if(!s.vehicle_enabled){installed_features_init();shadow_reset(&s.shadow);}
        }
        s.prev_stream = s.pctx.stream;
    }

    /* 2. LD19 raw bytes -> RAW bridge + parser -> scan builder. */
    uint8_t raw[256];
    for (unsigned guard = 0; guard < 16U; guard++) {
        uint32_t rx_end_us = 0;
        bool disc = false, uerr = false;
        uint32_t rn = usart2_read(raw, sizeof raw, &rx_end_us, &disc, &uerr);
        if (rn == 0U) break;
        now = timebase_us();

        BridgeChunk chunks[6];
        uint32_t cn = bridge_feed(&s.bridge, raw, rn, rx_end_us, disc, uerr, chunks, 6U);
        for (uint32_t i = 0; i < cn; i++) {
            if (s.pctx.stream >= 1U) app_send_raw(&chunks[i]);
        }
        if (disc) {
            s.status.capture_discontinuous = true;
            ld19_parser_reset(&s.parser);
            Ld19Scan interrupted;
            (void)scan_builder_flush(&s.builder,&interrupted);
            if(!s.vehicle_enabled){installed_features_init();shadow_reset(&s.shadow);}
        }

        Ld19Packet pkts[7];
        uint32_t pn = ld19_parser_feed(&s.parser, raw, rn, pkts, 7U);
        for (uint32_t i = 0; i < pn; i++) {
            s.status.valid_ld_packets = s.parser.stats.valid;
            status_set_packet(&s.status, now);
            Ld19Scan scan;
            uint32_t packet_end_us=rx_end_us-(uint32_t)(((uint64_t)(rn-pkts[i].end_in_feed)*10000000U)/LIDAR_BAUD);
            if (scan_builder_feed(&s.builder, &pkts[i], packet_end_us, &scan))
                app_on_scan(&scan, now);
        }
        s.status.ld_crc_errors = s.parser.stats.crc_error;
        s.status.parser_resyncs = s.parser.stats.resync;
    }

    /* 3. 10 ms inactivity flush for a partially filled raw chunk. */
    BridgeChunk flush;
    if (bridge_tick(&s.bridge, now, &flush)) {
        if (s.pctx.stream >= 1U) app_send_raw(&flush);
    }

    /* 4. STATUS at 10 Hz while streaming. */
    if (s.pctx.stream >= 1U && timebase_diff_us(now, s.next_status_us) >= 100000U) {
        /* This field stores the last emission time, not a future deadline:
         * unsigned subtraction against a future time wraps and floods TX. */
        s.next_status_us = now;
        app_send_status(now);
    }

    /* 5. TX DMA housekeeping + DWT processing-time bookkeeping (main loop only). */
    usart1_service();
    uint32_t proc_us = (timebase_cycles() - c0) / 168U;
    if (proc_us > s.proc_max_us) s.proc_max_us = proc_us;
}
