#include "wit_imu.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static void packet(uint8_t *p, uint8_t type, int16_t a, int16_t b, int16_t c)
{
    const int16_t axes[] = {a, b, c};
    memset(p, 0, 11); p[0] = 0x55; p[1] = type;
    for (unsigned i = 0; i < 3; ++i) {
        uint16_t u = (uint16_t)axes[i];
        p[2 + i * 2] = (uint8_t)u; p[3 + i * 2] = (uint8_t)(u >> 8);
    }
    for (unsigned i = 0; i < 10; ++i) p[10] = (uint8_t)(p[10] + p[i]);
}

static void feed(const uint8_t *p, unsigned n, uint32_t now)
{
    for (unsigned i = 0; i < n; ++i) imu_feed(p[i], now);
}

int main(void)
{
    uint8_t p[11], q[11];
    imu_init();
    assert(!imu_angle_fresh(0) && !imu_gyro_fresh(0));
    const uint8_t noise[] = {0x00, 0xff, 0x53, 0x01};
    feed(noise, sizeof noise, 10);
    packet(p, 0x53, -32768, 16384, -16384);
    feed(p, 4, 20); assert(!imu_data()->angle_seen);
    feed(p + 4, 7, 30);
    assert(imu_data()->frames == 1 && imu_data()->angle_samples == 1);
    assert(imu_data()->roll_mdeg == -180000 && imu_data()->pitch_mdeg == 90000);
    assert(imu_data()->yaw_mdeg == -90000 && imu_data()->angle_at == 30);
    assert(imu_angle_fresh(529) && !imu_angle_fresh(530));

    packet(q, 0x52, 0, 0, -32768); feed(q, 11, 40);
    assert(imu_data()->z_mdps == -2000000 && imu_data()->gyro_samples == 1);
    assert(imu_gyro_fresh(539) && !imu_gyro_fresh(540));
    packet(q, 0x52, 0, 0, 32767); feed(q, 11, 50);
    assert(imu_data()->z_mdps == 1999938);
    packet(p, 0x53, 0, 0, 32767); feed(p, 11, 50);
    assert(imu_data()->yaw_mdeg == 179994);
    packet(q, 0x51, 0, 0, 0); feed(q, 11, 600);
    assert(!imu_angle_fresh(600)); /* 加速度帧不刷新Yaw。 */

    uint32_t samples = imu_data()->angle_samples;
    p[10] ^= 0x01; feed(p, 11, 610);
    assert(imu_data()->angle_samples == samples && imu_data()->bad_checksum);
    packet(p, 0x53, 0x55, 0x5555, -0x55); feed(p, 11, 620);
    assert(imu_data()->angle_samples == samples + 1); /* 帧内0x55不是分隔符。 */
    samples = imu_data()->angle_samples;
    packet(p, 0x5f, 0, 0, 0); feed(p, 11, 630);
    assert(imu_data()->angle_samples == samples); /* 未知类型不伪造角度。 */

    imu_init();
    packet(p, 0x53, 0, 0, 16384);
    feed(p, 5, 1); feed(p + 6, 5, 1); /* 丢一个字节。 */
    feed(p, 11, 2); feed(p, 11, 3);
    assert(imu_data()->angle_seen && imu_data()->yaw_mdeg == 90000);
    imu_init(); feed(p, 6, 1); imu_feed(0x99, 1); feed(p + 6, 5, 1);
    feed(p, 11, 2); feed(p, 11, 3);
    assert(imu_data()->angle_seen && imu_data()->bad_checksum);

    imu_init(); feed(p, 11, UINT32_MAX - 100);
    assert(imu_angle_fresh(398) && !imu_angle_fresh(399));
    imu_uart_error();
    assert(imu_data()->uart_errors == 1 && !imu_angle_fresh(20));
    feed(p, 11, 30); assert(imu_angle_fresh(30));
    imu_init();
    assert(!imu_data()->bytes && !imu_data()->frames && !imu_data()->angle_samples);

    /* 包含大量伪帧头的噪声流后，必须仍能重新同步；同时由ASan检查边界。 */
    uint32_t random = 0x12345678;
    for (unsigned i = 0; i < 10000; ++i) {
        random = random * 1664525U + 1013904223U;
        imu_feed(i % 3 ? (uint8_t)(random >> 24) : 0x55, i);
    }
    feed(p, 11, 20000); feed(p, 11, 20001);
    assert(imu_angle_fresh(20001) && imu_data()->yaw_mdeg == 90000);
    puts("PASS: WIT checksum, signed scaling, freshness/wrap, split/noisy/dropped data and recovery.");
}
