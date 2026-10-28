#include "yaw_trace.h"
#include <string.h>

typedef struct { char *text; size_t used, capacity; bool ok; } Writer;

static void append(Writer *w, const char *s)
{
    while (*s) {
        if (w->used + 1U >= w->capacity) { w->ok = false; return; }
        w->text[w->used++] = *s++;
    }
}

static void number(Writer *w, uint32_t value)
{
    char digits[11];
    size_t at = sizeof digits - 1U;
    digits[at] = 0;
    do { digits[--at] = (char)('0' + value % 10U); value /= 10U; } while (value);
    append(w, &digits[at]);
}

static void signed_number(Writer *w, int32_t value)
{
    if (value < 0) { append(w, "-"); number(w, (uint32_t)(-(int64_t)value)); }
    else number(w, (uint32_t)value);
}

void yaw_trace_init(YawTrace *trace) { memset(trace, 0, sizeof *trace); }

size_t yaw_trace_poll(YawTrace *trace, uint32_t now, bool moving,
                      const WitImu *imu, char *line, size_t capacity)
{
    if (capacity) line[0] = 0;
    if (!moving && !trace->active) return 0;
    bool angle_valid = imu->angle_seen && (uint32_t)(now - imu->angle_at) < IMU_FRESH_MS;
    bool gyro_valid = imu->gyro_seen && (uint32_t)(now - imu->gyro_at) < IMU_FRESH_MS;
    if (moving && !trace->active) {
        trace->active = true;
        trace->began_at = now;
        trace->reference_valid = angle_valid;
        trace->reference_mdeg = angle_valid ? imu->yaw_mdeg : 0;
    } else if (moving && (uint32_t)(now - trace->last_at) < YAW_TRACE_PERIOD_MS) {
        return 0;
    }
    trace->last_at = now;
    trace->active = moving;
    int32_t delta = 0;
    if (angle_valid && trace->reference_valid) {
        /* 仅计算最短角差，跨±180°时不出现约360°的跳变。 */
        int64_t difference = (int64_t)imu->yaw_mdeg - trace->reference_mdeg;
        difference = (difference + 180000) % 360000;
        if (difference < 0) difference += 360000;
        delta = (int32_t)difference - 180000;
    }
    uint32_t valid = (angle_valid ? 1U : 0U) | (gyro_valid ? 2U : 0U) |
                     (trace->reference_valid ? 4U : 0U);
    Writer w = {line, 0, capacity, true};
    append(&w, "YAW t="); number(&w, now);
    append(&w, " ms="); number(&w, now - trace->began_at);
    append(&w, " run="); number(&w, moving);
    append(&w, " v="); number(&w, valid);
    append(&w, " age="); number(&w, imu->angle_seen ? now - imu->angle_at : UINT32_MAX);
    append(&w, " y="); signed_number(&w, imu->yaw_mdeg);
    append(&w, " d="); signed_number(&w, delta);
    append(&w, " z="); signed_number(&w, imu->z_mdps);
    append(&w, " n="); number(&w, imu->angle_samples);
    append(&w, " b="); number(&w, imu->bad_checksum);
    append(&w, " e="); number(&w, imu->uart_errors);
    append(&w, "\r\n");
    if (!w.ok) { if (capacity) line[0] = 0; return 0; }
    line[w.used] = 0;
    return w.used;
}
