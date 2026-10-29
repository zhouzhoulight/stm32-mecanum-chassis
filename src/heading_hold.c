#include "heading_hold.h"
#include "motor_test.h"
#include "chassis_config.h"
#include "wit_imu.h"
#include <string.h>


/* Isolated finite position segments. No velocity-mode or broadcast commands. */
static enum { H_IDLE, H_CHECK, H_ENABLE, H_MOVE, H_SETTLE } phase;
static bool armed;
static uint32_t armed_at, began_at, phase_at, heartbeat_at, sent_at, poll_at;
static uint32_t checksum_at_start, errors_at_start;
static uint32_t last_rx_at;
static uint16_t total, completed;
static uint8_t ack_mask, flags_mask, arrived_mask, pending_id, next_id, stable_rounds;
static uint8_t packet[8], used;
static int32_t reference;
static int16_t correction;
static uint32_t deadline;
static uint32_t angle_sample_at_segment;
static uint8_t diag_id, diag_op, diag_used, diag_expected, diag_packet[64];
static uint32_t diag_at;
static bool motion_diag, query_error_next;
static uint8_t error_query_id, pending_op, flags_value[5];
static uint8_t detail_op;

bool hold_active(void) { return phase != H_IDLE; }

int32_t hold_angle_delta(int32_t yaw, int32_t target)
{
    int64_t d = (int64_t)yaw - target;
    d = (d + 180000) % 360000;
    if (d < 0) d += 360000;
    return (int32_t)d - 180000;
}

int16_t hold_correction(int32_t delta)
{
    if (delta >= -HOLD_DEADBAND_MDEG && delta <= HOLD_DEADBAND_MDEG) return 0;
    int64_t c = (int64_t)delta * HOLD_GAIN_PULSES_PER_DEG / 1000;
    if (c > HOLD_CORRECTION_MAX) c = HOLD_CORRECTION_MAX;
    if (c < -HOLD_CORRECTION_MAX) c = -HOLD_CORRECTION_MAX;
    return (int16_t)c;
}

uint16_t hold_wheel_pulses(uint8_t id, int16_t c)
{
    if (id < 1 || id > 4 || c > HOLD_CORRECTION_MAX || c < -HOLD_CORRECTION_MAX) return 0;
    /* Left=-++-, clockwise=+-+-, in FL/FR/RL/RR order.
     * Left-front/right-front magnitude decreases for positive clockwise bias;
     * left-rear/right-rear increases. Mean lateral budget stays 160/segment. */
    bool front = id == CHASSIS_FRONT_LEFT_ID || id == CHASSIS_FRONT_RIGHT_ID;
    return (uint16_t)((int32_t)HOLD_SEGMENT_PULSES + (front ? -c : c));
}

static void stop(const char *reason)
{
    phase = H_IDLE;
    armed = false;
    pending_id = used = 0;
    query_error_next = false;
    /* Deactivate before sends, so re-entry/errors cannot issue another segment. */
    for (uint8_t id = 1; id <= 4; ++id) {
        const uint8_t s[] = {id,0xFE,0x98,0,0x6B};
        const uint8_t d[] = {id,0xF3,0xAB,0,0,0x6B};
        (void)platform_motor_send(s,sizeof s);
        (void)platform_motor_send(d,sizeof d);
    }
    platform_log("HOLD STOP: "); platform_log(reason);
    platform_log("; stop/disable sent. Observe actual vehicle.\r\n");
}

void hold_emergency_stop(void)
{
    armed = false;
    if (hold_active()) stop("UART fault");
}

void hold_init(void)
{
    phase = H_IDLE; armed = false; used = pending_id = 0;
    diag_id = diag_used = 0;
    completed = total = 0; correction = 0;
    motion_diag = query_error_next = false;
}

static void number(int32_t value)
{
    char text[12];
    unsigned pos = sizeof text - 1U;
    text[pos] = 0;
    uint32_t n = value < 0 ? (uint32_t)(-(int64_t)value) : (uint32_t)value;
    do { text[--pos] = (char)('0' + n % 10U); n /= 10U; } while (n);
    if (value < 0) text[--pos] = '-';
    platform_log(text + pos);
}

void hold_report(void)
{
    platform_log("HOLD active="); number(hold_active());
    platform_log(" armed="); number(armed);
    platform_log(" completed="); number(completed);
    platform_log(" total="); number(total);
    platform_log(" delta_mdeg="); number(hold_angle_delta(imu_data()->yaw_mdeg,reference));
    platform_log(" correction="); number(correction);
    platform_log("; segmented P=8 pulses/deg cap=16/160 RPM=5\r\n");
}

static bool send_packet(const uint8_t *p, size_t n)
{
    if (platform_motor_send(p,n)) return true;
    stop("TX failure");
    return false;
}

static bool fresh(uint32_t now)
{
    const WitImu *d = imu_data();
    return d->angle_seen && d->gyro_seen &&
        (uint32_t)(now-d->angle_at) < HOLD_IMU_FRESH_MS &&
        (uint32_t)(now-d->gyro_at) < HOLD_IMU_FRESH_MS;
}

static void begin_round(uint32_t now)
{
    flags_mask = arrived_mask = 0; next_id = 1; pending_id = 0;
    query_error_next = false;
    poll_at = now - 25U;
}

static void enable_all(uint32_t now)
{
    phase = H_ENABLE; phase_at = now; ack_mask = 0; used = 0;
    for (uint8_t id = 1; id <= 4; ++id) {
        const uint8_t p[] = {id,0xF3,0xAB,1,0,0x6B};
        if (!send_packet(p,sizeof p)) return;
    }
}

static void segment(uint32_t now)
{
    const WitImu *d = imu_data();
    correction = motion_diag ? 0 : hold_correction(hold_angle_delta(d->yaw_mdeg,reference));
    angle_sample_at_segment = d->angle_samples;
    phase = H_MOVE; phase_at = now; ack_mask = 0; stable_rounds = 0;
    pending_id = used = 0;
    platform_log("HOLD SEG n="); number(completed/HOLD_SEGMENT_PULSES+1);
    platform_log(" delta_mdeg="); number(hold_angle_delta(d->yaw_mdeg,reference));
    platform_log(" c="); number(correction);
    platform_log(" front="); number(hold_wheel_pulses(CHASSIS_FRONT_LEFT_ID,correction));
    platform_log(" rear="); number(hold_wheel_pulses(CHASSIS_REAR_LEFT_ID,correction));
    platform_log("\r\n");
    for (uint8_t id = 1; id <= 4; ++id) {
        uint16_t pulses = hold_wheel_pulses(id,correction);
        uint8_t direction = (id == CHASSIS_FRONT_LEFT_ID || id == CHASSIS_REAR_RIGHT_ID) ? 0 : 1;
        const uint8_t p[] = {id,0xFD,direction,0,5,TEST_ACCEL,0,0,(uint8_t)(pulses>>8),(uint8_t)pulses,0,0,0x6B};
        if (!send_packet(p,sizeof p)) return;
    }
}

bool hold_command(const char *line, uint32_t now)
{
    if (!strcmp(line,"stop")) {
        armed = false;
        diag_id = diag_used = 0;
        if (hold_active()) stop("operator");
        return false; /* legacy stop also revokes its independent authorization */
    }
    if (!strcmp(line,"hold_keep")) {
        if (hold_active()) heartbeat_at = now;
        return true;
    }
    if (!strcmp(line,"hold_status")) { hold_report(); return true; }
    if (hold_active()) {
        /* Status is allowed, but it does not act as a control heartbeat. */
        if (!strcmp(line,"status")) return false;
        stop("unexpected command while active");
        return true;
    }
    if (diag_id) {
        armed=false;
        platform_log("HOLD DIAG pending; wait or stop.\r\n");
        return true;
    }
    /* Fixed read-only opcodes only: no raw send or parameter-write path. */
    static const struct { const char *prefix; uint8_t op; } reads[] = {
        {"driver_config ",0x42}, {"driver_pid ",0x21}
    };
    for (unsigned i=0; i<sizeof reads/sizeof reads[0]; ++i) {
        size_t n=strlen(reads[i].prefix);
        if (strncmp(line,reads[i].prefix,n) || strlen(line)!=n+1U || line[n]<'1' || line[n]>'4') continue;
        armed=false;
        if (!test_begin_diagnostic()) return true;
        diag_id=(uint8_t)(line[n]-'0');diag_op=reads[i].op;
        diag_used=0;diag_expected=diag_op==0x21 ? 15U : 0U;diag_at=now;
        const uint8_t config[]={diag_id,0x42,0x6C,0x6B};
        const uint8_t pid[]={diag_id,0x21,0x6B};
        if (!platform_motor_send(diag_op==0x42 ? config : pid,diag_op==0x42 ? sizeof config : sizeof pid)) {
            diag_id=0;platform_log("HOLD DIAG TX failed.\r\n");
        } else platform_log("DRIVER READ sent; fixed read-only command; no enable/motion.\r\n");
        return true;
    }
    if (strlen(line)==11 && !strncmp(line,"hold_diag ",10) && line[10]>='1' && line[10]<='4') {
        armed = false;
        if (!test_begin_diagnostic()) return true;
        diag_id = (uint8_t)(line[10]-'0'); diag_op=0x37;diag_expected=8;diag_used = 0; diag_at = now;
        const uint8_t q[] = {diag_id,0x37,0x6B};
        if (!platform_motor_send(q,sizeof q)) {
            diag_id = 0; platform_log("HOLD DIAG TX failed.\r\n");
        } else platform_log("HOLD DIAG position-error query only; no enable or motion.\r\n");
        return true;
    }
    if (!strcmp(line,"arm hold")) {
        armed = false;
        if (!test_begin_diagnostic()) return true;
        if (!fresh(now)) { platform_log("HOLD REJECTED: IMU not fresh.\r\n"); return true; }
        armed = true; armed_at = now;
        platform_log("HOLD ARMED for 10s; no motion. Use left_hold_1600 or left_hold_16000.\r\n");
        return true;
    }
    if (!strcmp(line,"left_hold_1600") || !strcmp(line,"left_hold_16000") || !strcmp(line,"left_diag_160")) {
        bool allowed = armed && (uint32_t)(now-armed_at) < ARM_TIMEOUT_MS;
        armed = false;
        if (!allowed || !test_begin_diagnostic() || !fresh(now)) {
            platform_log("HOLD REJECTED: arm hold with fresh IMU first.\r\n"); return true;
        }
        motion_diag = !strcmp(line,"left_diag_160");
        total = motion_diag ? 160 : !strcmp(line,"left_hold_16000") ? 16000 : 1600;
        deadline = motion_diag ? 3000U : total == 16000 ? 80000U : 8000U;
        completed = 0; correction = 0;
        reference = imu_data()->yaw_mdeg;
        checksum_at_start = imu_data()->bad_checksum; errors_at_start = imu_data()->uart_errors;
        began_at = phase_at = heartbeat_at = now;
        phase = H_CHECK; stable_rounds = ack_mask = used = 0;
        begin_round(now);
        if (motion_diag) platform_log("HOLD DIAG START: exactly one160-pulse left segment; 5RPM; no yaw correction; flags/error/RPM/target/setpoint while active.\r\n");
        else platform_log("HOLD START: left segmented; 5RPM; 160 base pulses/segment; heartbeat required; no heading integrator.\r\n");
        return true;
    }
    /* Any unrelated command cancels pending hold authorization. */
    armed = false;
    if (!strcmp(line,"help")) platform_log(
        "Heading: arm hold -> left_hold_1600|left_hold_16000|left_diag_160; hold_keep every <=500ms; hold_status; stop.\r\n"
        "Read only: driver_config 1..4; driver_pid 1..4; hold_diag 1..4.\r\n"
        "Finite segments, P only, no integral; original left_16000 remains open-loop.\r\n");
    return false;
}

void hold_tick(uint32_t now)
{
    if (diag_id && (uint32_t)(now-diag_at)>=HOLD_REPLY_MS) {
        diag_id=diag_used=0; platform_log("HOLD DIAG timeout.\r\n");
    }
    if (armed && (uint32_t)(now-armed_at) >= ARM_TIMEOUT_MS) armed = false;
    if (!hold_active()) return;
    if ((uint32_t)(now-began_at) >= deadline) { stop("total deadline"); return; }
    if ((uint32_t)(now-heartbeat_at) >= HOLD_HEARTBEAT_MS) { stop("host heartbeat lost"); return; }
    const WitImu *d = imu_data();
    if (!fresh(now) || d->bad_checksum != checksum_at_start || d->uart_errors != errors_at_start) {
        stop("IMU stale/error"); return;
    }
    int32_t e = hold_angle_delta(d->yaw_mdeg,reference);
    if (e > HOLD_MAX_ERROR_MDEG || e < -HOLD_MAX_ERROR_MDEG) { stop("heading error limit"); return; }
    if ((uint32_t)(now-phase_at) >= HOLD_SEGMENT_MS) { stop("segment deadline"); return; }
    if (phase == H_ENABLE || phase == H_MOVE) {
        if (ack_mask != 0x1E) {
            if ((uint32_t)(now-phase_at) >= HOLD_REPLY_MS) stop("motor ACK timeout");
            return;
        }
        if (phase == H_ENABLE) {
            if ((uint32_t)(now-phase_at) >= 50U) segment(now);
            return;
        }
        phase = H_SETTLE;
        begin_round(now + 100U); /* delay first arrived query after the commands */
    }
    if (pending_id) {
        if ((uint32_t)(now-sent_at) >= HOLD_REPLY_MS) stop("motor flag timeout");
        return;
    }
    if (flags_mask == 0x1E && !query_error_next) {
        if (phase == H_CHECK) { enable_all(now); return; }
        if (arrived_mask == 0x1E) ++stable_rounds;
        else stable_rounds = 0;
        if (stable_rounds >= 2 && d->angle_samples > angle_sample_at_segment) {
            completed += HOLD_SEGMENT_PULSES;
            if (completed >= total) { stop("nominal pulse budget completed"); return; }
            segment(now); return;
        }
        begin_round(now);
    }
    /* Signed comparison supports a deliberately future first query time. */
    if ((int32_t)(now-poll_at) < 25) return;
    uint8_t id = query_error_next ? error_query_id : next_id++;
    pending_op = query_error_next ? detail_op : 0x3A;
    query_error_next = false;
    pending_id = id; sent_at = now; poll_at = now; used = 0;
    const uint8_t q[] = {id,pending_op,0x6B};
    (void)send_packet(q,sizeof q);
}

void hold_motor_rx(uint8_t byte, uint32_t now)
{
    if (diag_id) {
        if (!diag_used) {
            if (byte==diag_id) diag_packet[diag_used++]=byte;
            return;
        }
        if (diag_used==1 && byte!=diag_op && byte!=0x00) {
            diag_used=byte==diag_id ? 1U : 0U; return;
        }
        diag_packet[diag_used++]=byte;
        if (diag_used==3 && diag_op==0x42 && diag_packet[1]==0x42) {
            if (byte<5U || byte>sizeof diag_packet) {
                diag_id=diag_used=0;platform_log("DRIVER READ invalid length.\r\n");return;
            }
            diag_expected=byte;
        }
        if (diag_used==4 && diag_packet[1]==0x00) {
            if (diag_packet[2]==0xEE && byte==0x6B) {
                diag_id=diag_used=0;platform_log("HOLD DIAG rejected.\r\n");
            } else diag_used=0;
            return;
        }
        if (diag_used==diag_expected) {
            if ((diag_op==0x21 || diag_op==0x42) && diag_packet[1]==diag_op && byte==0x6B) {
                platform_log("DRIVER READ OK id=");number(diag_id);
                platform_log(" op=");number(diag_op);
                platform_log(" bytes=");number(diag_used);platform_log("; raw bytes in RX log; no motion.\r\n");
                diag_id=0;
            }
            if (diag_packet[1]==0x37 && diag_packet[2]<=1 && byte==0x6B) {
                uint32_t magnitude=((uint32_t)diag_packet[3]<<24)|((uint32_t)diag_packet[4]<<16)|
                    ((uint32_t)diag_packet[5]<<8)|diag_packet[6];
                platform_log("HOLD DIAG id=");number(diag_id);
                platform_log(" sign=");number(diag_packet[2]);
                platform_log(" error_counts=");
                /* Counts beyond int32 are rejected, never truncated to a safe-looking number. */
                if (magnitude>INT32_MAX) platform_log("OUT_OF_RANGE");
                else number((int32_t)magnitude);
                platform_log("; 65536 counts/revolution; no motion.\r\n");
                diag_id=0;
            }
            diag_used=0;
        }
        return;
    }
    if (!hold_active()) return;
    if (used && (uint32_t)(now-last_rx_at) > 50U) used = 0;
    last_rx_at = now;
    if (!used) {
        if (byte >= 1 && byte <= 4) packet[used++] = byte;
        return;
    }
    if (used == 1 && byte != 0xF3 && byte != 0xFD && byte != 0x3A && byte != 0x37 &&
        byte != 0x35 && byte != 0x33 && byte != 0x34 && byte != 0x36 && byte != 0x00) {
        used = 0; if (byte >= 1 && byte <= 4) packet[used++] = byte; return;
    }
    packet[used++] = byte;
    uint8_t expected = packet[1] == 0x35 ? 6 :
        (packet[1] == 0x37 || packet[1] == 0x33 || packet[1] == 0x34 || packet[1] == 0x36) ? 8 : 4;
    if (used < expected) return;
    used = 0;
    if (packet[expected-1] != 0x6B) return;
    uint8_t id = packet[0], op = packet[1], value = packet[2];
    if (op == 0x00) { stop("motor error reply"); return; }
    if ((phase == H_ENABLE && op == 0xF3) || (phase == H_MOVE && op == 0xFD)) {
        if (value != 0x02) { stop("motor command rejected"); return; }
        ack_mask |= (uint8_t)(1U << id);
    } else if (phase == H_SETTLE && motion_diag && id == pending_id && op==pending_op &&
               (op==0x37 || op==0x35 || op==0x33 || op==0x34 || op==0x36)) {
        pending_id = 0;
        uint32_t magnitude=op==0x35 ? ((uint32_t)packet[3]<<8)|packet[4] :
            ((uint32_t)packet[3]<<24)|((uint32_t)packet[4]<<16)|((uint32_t)packet[5]<<8)|packet[6];
        if (value>1 || magnitude>INT32_MAX) { stop("invalid position-error reply"); return; }
        platform_log(op==0x37 ? "HOLD SAMPLE ms=" : "HOLD DETAIL ms=");number((int32_t)(now-began_at));
        platform_log(" id=");number(id);
        if(op==0x37) {platform_log(" flags=");number(flags_value[id]);}
        else {platform_log(" op=");number(op);}
        platform_log(" sign=");number(value);
        platform_log(op==0x37 ? " error_counts=" : " value=");number((int32_t)magnitude);
        platform_log("\r\n");
        /* Read final target, trajectory setpoint, actual position and RPM as well as error.
         * They are observational only: the original arrived gate is unchanged. */
        if(op!=0x36) {
            detail_op=op==0x37 ? 0x35 : op==0x35 ? 0x33 : op==0x33 ? 0x34 : 0x36;
            query_error_next=true;error_query_id=id;
        }
    } else if ((phase == H_CHECK || phase == H_SETTLE) && op == 0x3A && pending_op == 0x3A && id == pending_id) {
        pending_id = 0;
        if (phase == H_CHECK) {
            platform_log("HOLD FLAG id="); number(id);
            platform_log(" raw="); number(value); platform_log("\r\n");
        }
        /* Manufacturer defines bits0..3 (enable/reached/stall/protection).
         * Real v03/hw82 reports 0x82 while idle; do not label undefined bit7
         * as stall. Retain the raw byte above; only documented fault bits stop. */
        if ((value & 0x0CU) || (phase == H_SETTLE && !(value & 0x01U))) {
            stop("motor flags fault/disabled"); return;
        }
        flags_mask |= (uint8_t)(1U << id);
        if (value & 0x02U) arrived_mask |= (uint8_t)(1U << id);
        flags_value[id] = value;
        if (motion_diag && phase == H_SETTLE) { query_error_next = true; error_query_id = id; detail_op=0x37; }
    }
}
