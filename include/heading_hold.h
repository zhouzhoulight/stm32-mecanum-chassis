#ifndef HEADING_HOLD_H
#define HEADING_HOLD_H
#include <stdbool.h>
#include <stdint.h>

#define HOLD_SEGMENT_PULSES 160U
#define HOLD_CORRECTION_MAX 16
#define HOLD_GAIN_PULSES_PER_DEG 8
#define HOLD_DEADBAND_MDEG 150
#define HOLD_MAX_ERROR_MDEG 8000
#define HOLD_IMU_FRESH_MS 300U
#define HOLD_HEARTBEAT_MS 1500U
#define HOLD_REPLY_MS 300U
#define HOLD_SEGMENT_MS 2500U

void hold_init(void);
bool hold_command(const char *line, uint32_t now);
void hold_tick(uint32_t now);
void hold_motor_rx(uint8_t byte, uint32_t now);
void hold_emergency_stop(void);
bool hold_active(void);
void hold_report(void);
/* Positive result adds clockwise motion, opposing positive/CCW yaw error. */
int32_t hold_angle_delta(int32_t yaw, int32_t reference);
int16_t hold_correction(int32_t delta);
uint16_t hold_wheel_pulses(uint8_t id, int16_t correction);
#endif
