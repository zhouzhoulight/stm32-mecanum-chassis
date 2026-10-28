#include "yaw_trace.h"
#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>

static WitImu sample(uint32_t now, int32_t yaw)
{
    WitImu d = {0};
    d.angle_seen = d.gyro_seen = true;
    d.angle_at = d.gyro_at = now;
    d.angle_samples = 17;
    d.yaw_mdeg = yaw;
    return d;
}

int main(void)
{
    YawTrace t;
    char line[YAW_TRACE_LINE_CAPACITY];
    WitImu d = sample(1000, -20121), original = d;
    yaw_trace_init(&t);
    assert(!yaw_trace_poll(&t, 1000, false, &d, line, sizeof line));
    assert(yaw_trace_poll(&t, 1000, true, &d, line, sizeof line));
    assert(strstr(line, "ms=0 run=1 v=7") && strstr(line, "y=-20121 d=0"));
    assert(!memcmp(&d, &original, sizeof d));
    assert(!yaw_trace_poll(&t, 1199, true, &d, line, sizeof line));
    d.yaw_mdeg = -19121;
    assert(yaw_trace_poll(&t, 1200, true, &d, line, sizeof line));
    assert(strstr(line, "ms=200") && strstr(line, "d=1000"));
    assert(!yaw_trace_poll(&t, 1200, true, &d, line, sizeof line));
    assert(yaw_trace_poll(&t, 1201, false, &d, line, sizeof line));
    assert(strstr(line, "ms=201 run=0"));
    assert(!yaw_trace_poll(&t, 1500, false, &d, line, sizeof line));
    d = sample(2000, 42000);
    assert(yaw_trace_poll(&t, 2000, true, &d, line, sizeof line));
    assert(strstr(line, "ms=0 run=1 v=7") && strstr(line, "d=0"));
    /* 数据在恰好500ms时失效，不能将旧角度显示成有效的零偏差。 */
    assert(yaw_trace_poll(&t, 2500, true, &d, line, sizeof line));
    assert(strstr(line, "v=4 age=500"));
    d = sample(2700, 45000);
    assert(yaw_trace_poll(&t, 2700, true, &d, line, sizeof line));
    assert(strstr(line, "v=7") && strstr(line, "d=3000"));
    /* 串口错误后seen清除，即使时间很近也不可当成有效。 */
    d.angle_seen = d.gyro_seen = false;
    d.uart_errors = 1;
    assert(yaw_trace_poll(&t, 2900, true, &d, line, sizeof line));
    assert(strstr(line, "v=4 age=4294967295") && strstr(line, "e=1"));

    yaw_trace_init(&t); d = sample(0, 179000);
    assert(yaw_trace_poll(&t, 0, true, &d, line, sizeof line));
    d = sample(200, -179000);
    assert(yaw_trace_poll(&t, 200, true, &d, line, sizeof line));
    assert(strstr(line, "d=2000"));
    yaw_trace_init(&t);
    assert(yaw_trace_poll(&t, 200, true, &d, line, sizeof line));
    d = sample(400, 179000);
    assert(yaw_trace_poll(&t, 400, true, &d, line, sizeof line));
    assert(strstr(line, "d=-2000"));

    /* 缺失起点时不在运动中偷偷改参考方向；下一次动作才重新取参考。 */
    yaw_trace_init(&t); memset(&d, 0, sizeof d);
    assert(yaw_trace_poll(&t, 0, true, &d, line, sizeof line));
    assert(strstr(line, "v=0"));
    d = sample(200, 20000);
    assert(yaw_trace_poll(&t, 200, true, &d, line, sizeof line));
    assert(strstr(line, "v=3") && strstr(line, "d=0"));
    assert(yaw_trace_poll(&t, 201, false, &d, line, sizeof line));
    assert(yaw_trace_poll(&t, 202, true, &d, line, sizeof line));
    assert(strstr(line, "v=7"));

    /* 不补发过期的日志；毫秒回绕后仍按200ms节流。 */
    yaw_trace_init(&t); d = sample(UINT32_MAX - 100U, 0);
    assert(yaw_trace_poll(&t, UINT32_MAX - 100U, true, &d, line, sizeof line));
    assert(!yaw_trace_poll(&t, 98, true, &d, line, sizeof line));
    assert(yaw_trace_poll(&t, 99, true, &d, line, sizeof line));
    assert(strstr(line, "ms=200"));
    d = sample(2000, 123);
    assert(yaw_trace_poll(&t, 2000, true, &d, line, sizeof line));
    assert(!yaw_trace_poll(&t, 2000, true, &d, line, sizeof line));
    assert(!yaw_trace_poll(&t, 2199, true, &d, line, sizeof line));

    /* 最长计数和有符号极值不会截断或越界，短缓冲不泄漏半行。 */
    yaw_trace_init(&t); memset(&d, 0, sizeof d);
    d.yaw_mdeg = d.z_mdps = INT32_MIN;
    d.angle_samples = d.bad_checksum = d.uart_errors = UINT32_MAX;
    assert(yaw_trace_poll(&t, UINT32_MAX, true, &d, line, sizeof line));
    assert(strlen(line) < sizeof line && strstr(line, "y=-2147483648"));
    assert(strstr(line, "z=-2147483648") && strstr(line, "n=4294967295"));
    assert(!strcmp(line + strlen(line) - 2, "\r\n"));
    char tiny[2] = {'X', 'Q'};
    assert(!yaw_trace_poll(&t, 1, false, &d, tiny, 1));
    assert(tiny[0] == 0 && tiny[1] == 'Q');
    yaw_trace_init(&t);
    assert(!yaw_trace_poll(&t, 0, true, &d, NULL, 0));
    puts("PASS: yaw trace cadence, idle/final/restart, freshness, reference validity, wrap and bounded integer output.");
}
