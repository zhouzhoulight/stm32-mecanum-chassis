#ifndef IMU_SERVICE_H
#define IMU_SERVICE_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

void imu_service_init(void);
bool imu_service_command(const char *line, uint32_t now);
void imu_service_tick(uint32_t now);
bool imu_service_busy(void);
bool imu_service_tx_enabled(void);
void imu_service_report(uint32_t now);
/* Platform callbacks never send to the motor UART. TX is off after reset. */
bool platform_imu_tx_enable(bool enable);
bool platform_imu_send(const uint8_t *bytes, size_t len);
#endif
