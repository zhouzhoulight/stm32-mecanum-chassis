#ifndef YAW_TRACE_H
#define YAW_TRACE_H
#include "wit_imu.h"
#include <stddef.h>

#define YAW_TRACE_PERIOD_MS 200U
#define YAW_TRACE_LINE_CAPACITY 160U

/* 记录器只读取姿态和运动状态，不发送电机或传感器命令。 */
typedef struct {
    bool active, reference_valid;
    uint32_t began_at, last_at;
    int32_t reference_mdeg;
} YawTrace;

void yaw_trace_init(YawTrace *trace);
/* 返回完整记录长度；无记录或缓冲不足返回0，不输出半行。 */
size_t yaw_trace_poll(YawTrace *trace, uint32_t now, bool moving,
                      const WitImu *imu, char *line, size_t capacity);
#endif
