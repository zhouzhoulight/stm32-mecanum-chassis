#ifndef MOTOR_TEST_H
#define MOTOR_TEST_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define TEST_FIRMWARE_VERSION "v2.0"

/* 这些常量是第一次架空测试的限制，不是整车运动参数。 */
#define TEST_RPM 5U
#define TEST_PULSES 160U
#define TEST_ACCEL 10U
#define ARM_TIMEOUT_MS 10000U
#define RUN_TIMEOUT_MS 3000U
#define PROBE_TIMEOUT_MS 500U

/* 六个 *_long 指令使用固定长行程；原短动作仍为 160 脉冲/3 秒。
 * 脉冲数不等于厘米数，实际移动量受细分、传动和地面影响。 */
#define LONG_TEST_PULSES 1600U
#define LONG_TEST_TIMEOUT_MS 8000U
_Static_assert(TEST_RPM <= 5U && LONG_TEST_PULSES > TEST_PULSES &&
               LONG_TEST_PULSES <= 1600U &&
               LONG_TEST_TIMEOUT_MS > RUN_TIMEOUT_MS && LONG_TEST_TIMEOUT_MS <= 8000U,
               "Long motion must remain within its fixed low-speed limits");

/* 用户明确要求的单次前进测距，独立于原来的六种长动作。
 * 80 秒仅为发停止/失能的上限，不代表实际持续运行时间或行程已完成。 */
#define CAL_FORWARD_PULSES 16000U
#define CAL_FORWARD_TIMEOUT_MS 80000U
_Static_assert(TEST_RPM <= 5U && CAL_FORWARD_PULSES == 16000U &&
               CAL_FORWARD_TIMEOUT_MS == 80000U,
               "Forward calibration must keep its fixed authorized profile");

/* 三次 16000 脉冲的实测均值为 123.8cm。
 * round(16000 * 100 / 123.8)=12924；这是待量具核对的一米试走初值。 */
#define VERIFY_FORWARD_1M_PULSES 12924U
#define VERIFY_FORWARD_1M_TIMEOUT_MS 80000U
_Static_assert(TEST_RPM <= 5U && VERIFY_FORWARD_1M_PULSES == 12924U &&
               VERIFY_FORWARD_1M_PULSES < CAL_FORWARD_PULSES &&
               VERIFY_FORWARD_1M_TIMEOUT_MS == CAL_FORWARD_TIMEOUT_MS,
               "Nominal one-metre trial must keep its fixed bounded profile");

/* 后退暂借用前进初值，只作为独立的一米试走，实际后退距离另行测量。 */
#define VERIFY_BACK_1M_PULSES 12924U
#define VERIFY_BACK_1M_TIMEOUT_MS 80000U
_Static_assert(TEST_RPM <= 5U && VERIFY_BACK_1M_PULSES == 12924U &&
               VERIFY_BACK_1M_PULSES < CAL_FORWARD_PULSES &&
               VERIFY_BACK_1M_TIMEOUT_MS == CAL_FORWARD_TIMEOUT_MS,
               "Reverse one-metre trial must keep its fixed bounded profile");

/* 横移测距独立于前进系数，只固定脉冲数，不声称等于一米。 */
#define CAL_STRAFE_PULSES 16000U
#define CAL_STRAFE_TIMEOUT_MS 80000U
_Static_assert(TEST_RPM <= 5U && CAL_STRAFE_PULSES == CAL_FORWARD_PULSES &&
               CAL_STRAFE_TIMEOUT_MS == CAL_FORWARD_TIMEOUT_MS,
               "Strafe calibration must keep its fixed low-speed profile");

/* 平台接口：主机测试用假的串口，STM32 上用真实 HAL 串口。 */
bool platform_motor_send(const uint8_t *data, size_t len);
void platform_log(const char *text);
void test_init(void);
/* 静止诊断撤销未用的授权；运行中拒绝，不自动切换硬件参数。 */
bool test_begin_diagnostic(void);
/* 纯读取，不处理tick、不消耗arm、不发送命令。 */
bool test_is_moving(void);
void test_command(const char *line, uint32_t now);
void test_tick(uint32_t now);
void test_motor_rx(uint8_t byte, uint32_t now);
void test_fault(void);
#endif
