#ifndef WIT_IMU_H
#define WIT_IMU_H
#include <stdbool.h>
#include <stdint.h>

#define IMU_FRESH_MS 500U
/* 本车WT901CTTL已通过有效帧实测为115200；只设主控接收速率。 */
#define IMU_DEFAULT_BAUD 115200U

/* 只有完整11字节且校验通过的对应数据帧，才能更新角度/角速度。 */
typedef struct {
    uint32_t bytes, frames, bad_checksum, uart_errors;
    uint32_t angle_samples, gyro_samples, angle_at, gyro_at;
    int32_t roll_mdeg, pitch_mdeg, yaw_mdeg, z_mdps;
    bool angle_seen, gyro_seen;
    /* Raw auxiliary data: no assumption about configured accelerometer range. */
    int32_t x_mdps, y_mdps;
    int16_t acc_raw[3], mag_raw[3];
    uint16_t angle_word4, reg_words[4];
    uint32_t acc_samples, mag_samples, reg_samples, reg_at;
} WitImu;

void imu_init(void);
void imu_feed(uint8_t byte, uint32_t now);
void imu_uart_error(void);
const WitImu *imu_data(void);
bool imu_angle_fresh(uint32_t now);
bool imu_gyro_fresh(uint32_t now);
#endif
