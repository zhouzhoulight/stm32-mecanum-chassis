#include "wit_imu.h"
#include <string.h>

static WitImu data;
static uint8_t frame[11], used;

void imu_init(void)
{
    memset(&data, 0, sizeof data);
    used = 0;
}

const WitImu *imu_data(void) { return &data; }

void imu_uart_error(void)
{
    ++data.uart_errors;
    used = 0;
    /* 丢包后不再把旧角度显示为有效，等待新的完整校验帧。 */
    data.angle_seen = data.gyro_seen = false;
}

bool imu_angle_fresh(uint32_t now)
{
    return data.angle_seen && (uint32_t)(now - data.angle_at) < IMU_FRESH_MS;
}

bool imu_gyro_fresh(uint32_t now)
{
    return data.gyro_seen && (uint32_t)(now - data.gyro_at) < IMU_FRESH_MS;
}

static int32_t scaled(const uint8_t *p, int32_t range)
{
    uint16_t raw = (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
    int32_t signed_raw = (raw & 0x8000U) ? (int32_t)raw - 65536 : (int32_t)raw;
    /* 乘法在64位中进行，避免正负满量程时32位溢出；不引入浮点输出。 */
    return (int32_t)((int64_t)signed_raw * range / 32768);
}

void imu_feed(uint8_t byte, uint32_t now)
{
    ++data.bytes;
    if (!used && byte != 0x55U) return;
    frame[used++] = byte;
    if (used < sizeof frame) return;

    uint8_t sum = 0;
    for (unsigned i = 0; i < 10; ++i) sum = (uint8_t)(sum + frame[i]);
    if (sum == frame[10] && frame[1] >= 0x50U && (frame[1] <= 0x59U || frame[1] == 0x5FU)) {
        ++data.frames;
        if (frame[1] == 0x53U) {
            data.roll_mdeg = scaled(frame + 2, 180000);
            data.pitch_mdeg = scaled(frame + 4, 180000);
            data.yaw_mdeg = scaled(frame + 6, 180000);
            data.angle_word4 = (uint16_t)(frame[8] | (uint16_t)frame[9] << 8);
            data.angle_at = now;
            data.angle_seen = true;
            ++data.angle_samples;
        } else if (frame[1] == 0x52U) {
            data.x_mdps = scaled(frame + 2, 2000000);
            data.y_mdps = scaled(frame + 4, 2000000);
            data.z_mdps = scaled(frame + 6, 2000000);
            data.gyro_at = now;
            data.gyro_seen = true;
            ++data.gyro_samples;
        } else if (frame[1] == 0x51U || frame[1] == 0x54U) {
            int16_t *axes = frame[1] == 0x51U ? data.acc_raw : data.mag_raw;
            for (unsigned i = 0; i < 3; ++i) axes[i] = (int16_t)scaled(frame + 2 + 2*i, 32768);
            if (frame[1] == 0x51U) ++data.acc_samples;
            else ++data.mag_samples;
        } else if (frame[1] == 0x5FU) {
            for (unsigned i = 0; i < 4; ++i)
                data.reg_words[i] = (uint16_t)(frame[2+2*i] | (uint16_t)frame[3+2*i] << 8);
            ++data.reg_samples;
            data.reg_at = now;
        }
        used = 0;
    } else {
        ++data.bad_checksum;
        /* 坏帧可能包含下一帧的开头；保留后缀以从丢字节/噪声中恢复。 */
        unsigned start = 1;
        while (start < sizeof frame && frame[start] != 0x55U) ++start;
        used = (uint8_t)(sizeof frame - start);
        if (used) memmove(frame, frame + start, used);
    }
}
