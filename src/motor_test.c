#include "motor_test.h"
#include "chassis_config.h"
#include <string.h>

/* 只允许底盘地址 1～4，永远不使用广播地址 0 或升降电机地址 5。 */
static uint8_t selected = 1, directions[5]; /* 索引是驱动器地址；0 永不使用。 */
static bool armed;
static bool all_wheels;
static bool fault_latched;
static enum { IDLE, WAIT_ENABLE, MOVING } state;
static uint32_t arm_at, started_at;
static uint32_t active_pulses, active_timeout_ms;
static const char *active_long_command;
static bool probe_pending;
static uint8_t probe_id, probe_used, probe_reply[5];
static uint32_t probe_at;

typedef enum {
    GROUP_NONE, GROUP_FORWARD, GROUP_BACK, GROUP_LEFT, GROUP_RIGHT, GROUP_CW, GROUP_CCW,
    GROUP_FORWARD_LONG, GROUP_BACK_LONG, GROUP_LEFT_LONG, GROUP_RIGHT_LONG,
    GROUP_CW_LONG, GROUP_CCW_LONG, GROUP_FORWARD_16000, GROUP_FORWARD_1M, GROUP_BACK_1M,
    GROUP_LEFT_16000, GROUP_RIGHT_16000
} GroupMotion;

static GroupMotion parse_group_motion(const char *line)
{
    static const struct { const char *command; GroupMotion motion; } commands[] = {
        {"forward", GROUP_FORWARD}, {"back", GROUP_BACK},
        {"left", GROUP_LEFT}, {"right", GROUP_RIGHT},
        {"cw", GROUP_CW}, {"ccw", GROUP_CCW},
        {"forward_long", GROUP_FORWARD_LONG}, {"back_long", GROUP_BACK_LONG},
        {"left_long", GROUP_LEFT_LONG}, {"right_long", GROUP_RIGHT_LONG},
        {"cw_long", GROUP_CW_LONG}, {"ccw_long", GROUP_CCW_LONG},
        {"forward_16000", GROUP_FORWARD_16000}, {"forward_1m", GROUP_FORWARD_1M},
        {"back_1m", GROUP_BACK_1M},
        {"left_16000", GROUP_LEFT_16000}, {"right_16000", GROUP_RIGHT_16000}
    };
    for (size_t i = 0; i < sizeof commands / sizeof commands[0]; ++i) {
        if (!strcmp(line, commands[i].command)) return commands[i].motion;
    }
    return GROUP_NONE;
}

static const char *long_command_name(GroupMotion motion)
{
    switch (motion) {
    case GROUP_FORWARD_LONG: return "forward_long";
    case GROUP_BACK_LONG: return "back_long";
    case GROUP_LEFT_LONG: return "left_long";
    case GROUP_RIGHT_LONG: return "right_long";
    case GROUP_CW_LONG: return "cw_long";
    case GROUP_CCW_LONG: return "ccw_long";
    case GROUP_FORWARD_16000: return "forward_16000";
    case GROUP_FORWARD_1M: return "forward_1m";
    case GROUP_BACK_1M: return "back_1m";
    case GROUP_LEFT_16000: return "left_16000";
    case GROUP_RIGHT_16000: return "right_16000";
    default: return NULL;
    }
}

static void set_group_directions(GroupMotion motion)
{
    /* X 型轮组，方向按车体定义：left/right 是横移，cw/ccw 从上方看。
     * 以各轮“上沿朝车头”为正，左前/右前/左后/右后四轮的正负组合：
     * forward ++++；back ----；left -++-；right +--+；cw +-+-；ccw -+-+。
     * 只做等速、等脉冲的小幅方向测试，不据此声称走了多少厘米/转了多少度。 */
    unsigned reverse_mask = 0;
    switch (motion) {
    case GROUP_BACK: case GROUP_BACK_LONG: case GROUP_BACK_1M: reverse_mask = 0x1EU; break;
    case GROUP_LEFT: case GROUP_LEFT_LONG: case GROUP_LEFT_16000:
        reverse_mask = (1U << CHASSIS_FRONT_LEFT_ID) | (1U << CHASSIS_REAR_RIGHT_ID); break;
    case GROUP_RIGHT: case GROUP_RIGHT_LONG: case GROUP_RIGHT_16000:
        reverse_mask = (1U << CHASSIS_FRONT_RIGHT_ID) | (1U << CHASSIS_REAR_LEFT_ID); break;
    case GROUP_CW: case GROUP_CW_LONG:
        reverse_mask = (1U << CHASSIS_FRONT_RIGHT_ID) | (1U << CHASSIS_REAR_RIGHT_ID); break;
    case GROUP_CCW: case GROUP_CCW_LONG:
        reverse_mask = (1U << CHASSIS_FRONT_LEFT_ID) | (1U << CHASSIS_REAR_LEFT_ID); break;
    case GROUP_FORWARD: case GROUP_FORWARD_LONG: case GROUP_FORWARD_16000: case GROUP_FORWARD_1M:
    case GROUP_NONE: break;
    }
    for (uint8_t id = 1; id <= 4; ++id) {
        directions[id] = (uint8_t)(CHASSIS_FORWARD_DIRECTION ^ ((reverse_mask >> id) & 1U));
    }
}

static void stop_all(void)
{
    /* 不假设当前哪个驱动器在线。发送停止后再关闭使能。 */
    for (uint8_t id = 1; id <= 4; ++id) {
        const uint8_t stop[] = {id, 0xFE, 0x98, 0, 0x6B};
        const uint8_t disable[] = {id, 0xF3, 0xAB, 0, 0, 0x6B};
        (void)platform_motor_send(stop, sizeof stop);
        (void)platform_motor_send(disable, sizeof disable);
    }
    armed = false;
    all_wheels = false;
    state = IDLE;
    probe_pending = false;
    probe_used = 0;
}

void test_init(void)
{
    selected = 1;
    memset(directions, 0, sizeof directions);
    fault_latched = false;
    active_pulses = TEST_PULSES;
    active_timeout_ms = RUN_TIMEOUT_MS;
    active_long_command = NULL;
    stop_all();
    platform_log("\r\nCHASSIS F407 TEST " TEST_FIRMWARE_VERSION " | STM32F407VET6 | IDLE\r\n"
                 "Commands: help, status, probe 1..4, arm 1..4|all, jog 0|1, forward/back/left/right/cw/ccw, their _long variants, stop\r\n"
                 "Normal: 5 RPM, 160 relative pulses, 3 second stop deadline.\r\n"
                 "Six *_long commands: 5 RPM, 1600 relative pulses, 8 second stop deadline.\r\n"
                 "Calibration forward_16000: 5 RPM, 16000 relative pulses, 80 second stop deadline.\r\n"
                 "Calibration strafe left_16000/right_16000: 5 RPM, 16000 relative pulses, 80 second stop deadline.\r\n"
                 "Validation forward_1m: nominal 100 cm, 5 RPM, 12924 relative pulses, 80 second stop deadline.\r\n"
                 "Validation back_1m: nominal 100 cm, 5 RPM, 12924 relative pulses, 80 second stop deadline.\r\n"
                 "TX/RX logs are not proof of motor movement.\r\n");
}

void test_fault(void)
{
    /* 连续接收错误只处理一次，避免停止帧回显再次触发错误形成循环。
     * 后续显式 arm/probe 会开始新一轮检查，下一次错误仍会撤销它。 */
    if (fault_latched) return;
    fault_latched = true;
    stop_all();
    platform_log("FAULT: stopped/disarmed; inspect wiring, then arm again.\r\n");
}

bool test_is_moving(void) { return state == MOVING; }

bool test_begin_diagnostic(void)
{
    if (state != IDLE) {
        platform_log("BUSY: stop before IMU diagnostics.\r\n");
        return false;
    }
    armed = false;
    all_wheels = false;
    probe_pending = false;
    probe_used = 0;
    return true;
}

void test_tick(uint32_t now)
{
    if (probe_pending && (uint32_t)(now - probe_at) >= PROBE_TIMEOUT_MS) {
        probe_pending = false;
        probe_used = 0;
        platform_log("PROBE timeout: no valid version reply within 500ms. Check UART wiring/settings.\r\n");
    }
    /* unsigned 差值在系统毫秒计数溢出后仍然有效。 */
    if (armed && (uint32_t)(now - arm_at) >= ARM_TIMEOUT_MS) {
        armed = false;
        platform_log("ARM expired.\r\n");
    }
    if (state != IDLE && (uint32_t)(now - started_at) >= active_timeout_ms) {
        stop_all();
        if (active_timeout_ms == CAL_FORWARD_TIMEOUT_MS) {
            platform_log("STOP/DISABLE sent after 80 seconds (");
            platform_log(active_long_command);
            platform_log("). Observe actual motor.\r\n");
        } else if (active_long_command) {
            platform_log("STOP/DISABLE sent after 8 seconds (");
            platform_log(active_long_command);
            platform_log("). Observe actual motor.\r\n");
        } else {
            platform_log("STOP/DISABLE sent after 3 seconds. Observe actual motor.\r\n");
        }
    }
    if (state == WAIT_ENABLE && (uint32_t)(now - started_at) >= 50U) {
        /* Emm V5 相对位置、立即执行；使用当前单次动作的固定脉冲数。
         * 角度取决于驱动器细分和传动比，脉冲不等于角度或厘米。
         * 四轮测试逐地址发送，保留 5ms 帧间隔，不使用广播或同步触发。 */
        state = MOVING;
        for (uint8_t id = 1; id <= 4; ++id) {
            if (!all_wheels && id != selected) continue;
            const uint8_t move[] = {
                id, 0xFD, directions[id], TEST_RPM >> 8, TEST_RPM & 0xFF,
                TEST_ACCEL, (active_pulses >> 24) & 0xFF, (active_pulses >> 16) & 0xFF,
                (active_pulses >> 8) & 0xFF, active_pulses & 0xFF, 0, 0, 0x6B
            };
            if (!platform_motor_send(move, sizeof move)) {
                test_fault();
                return;
            }
        }
        if (active_long_command && !strcmp(active_long_command, "back_1m")) {
            platform_log("VALIDATION JOG sent: back_1m; nominal 100 cm, 5 RPM, 12924 pulses, 80-second stop deadline. Type stop to stop early.\r\n");
        } else if (active_pulses == VERIFY_FORWARD_1M_PULSES &&
            active_timeout_ms == VERIFY_FORWARD_1M_TIMEOUT_MS) {
            platform_log("VALIDATION JOG sent: forward_1m; nominal 100 cm, 5 RPM, 12924 pulses, 80-second stop deadline. Type stop to stop early.\r\n");
        } else if (active_timeout_ms == CAL_FORWARD_TIMEOUT_MS) {
            platform_log("CALIBRATION JOG sent: ");
            platform_log(active_long_command);
            platform_log("; 5 RPM, 16000 pulses, 80-second stop deadline. Type stop to stop early.\r\n");
        } else if (active_long_command) {
            platform_log("LONG JOG sent: ");
            platform_log(active_long_command);
            platform_log("; 5 RPM, 1600 pulses, 8-second stop deadline. Type stop to stop early.\r\n");
        } else {
            platform_log(all_wheels ? "FOUR-WHEEL JOG sent; observe all wheels. Type stop to stop early.\r\n"
                                    : "JOG sent; watch motor. Type stop to stop early.\r\n");
        }
    }
}

void test_motor_rx(uint8_t byte, uint32_t now)
{
    if (!probe_pending) return;
    if ((uint32_t)(now - probe_at) >= PROBE_TIMEOUT_MS) {
        test_tick(now);
        return;
    }
    /* 查询回包：[地址, 1F, 固件版本, 硬件版本, 6B]。
     * 不解析为“动作已成功”，也不触发任何后续动作。 */
    if (!probe_used) {
        if (byte == probe_id) probe_reply[probe_used++] = byte;
        return;
    }
    if (probe_used == 1 && byte != 0x1F && byte != 0x00) {
        probe_used = byte == probe_id ? 1U : 0U;
        return;
    }
    probe_reply[probe_used++] = byte;
    if (probe_used == 4 && probe_reply[1] == 0x00) {
        if (probe_reply[2] == 0xEE && byte == 0x6B) {
            probe_pending = false;
            platform_log("PROBE rejected: driver returned an error frame. Inspect RX bytes/protocol.\r\n");
        }
        probe_used = byte == probe_id ? 1U : 0U;
    } else if (probe_used == sizeof probe_reply) {
        if (probe_reply[1] == 0x1F && byte == 0x6B) {
            probe_pending = false;
            platform_log("PROBE OK: addressed version reply received. UART round trip works; see RX bytes.\r\n");
        }
        probe_used = byte == probe_id ? 1U : 0U;
    }
}

void test_command(const char *line, uint32_t now)
{
    /* 停止的优先级高于定时发送动作，不能先发送待发动作再停。 */
    if (!strcmp(line, "stop")) {
        stop_all();
        platform_log("STOP/DISABLE sent.\r\n");
        return;
    }
    if (strlen(line) == 7 && !strncmp(line, "probe ", 6) &&
        line[6] >= '1' && line[6] <= '4') {
        /* 这个分支优先于运行时 tick，不因查询而补发待执行的运动。
         * 查询撤销未使用的动作授权；运行中需先 stop。 */
        if (state != IDLE) {
            platform_log("BUSY: stop before probing.\r\n");
            return;
        }
        armed = false;
        all_wheels = false;
        fault_latched = false;
        probe_id = (uint8_t)(line[6] - '0');
        probe_used = 0;
        probe_at = now;
        probe_pending = true;
        const uint8_t query[] = {probe_id, 0x1F, 0x6B};
        if (!platform_motor_send(query, sizeof query)) {
            probe_pending = false;
            platform_log("PROBE TX failed in STM32 UART. No motion command sent.\r\n");
        } else {
            platform_log("PROBE: version query sent; no enable/motion. Waiting up to 500ms.\r\n");
        }
        return;
    }
    test_tick(now);
    GroupMotion motion = parse_group_motion(line);
    if (!strcmp(line, "help")) {
        platform_log("status; probe 3 (read only); arm 1 -> jog 0 (within 10s); stop\r\n"
                     "IMU: imu; imu baud 9600 (MCU RX only, no sensor writes).\r\n"
                     "Short: forward/back/left/right/cw/ccw (160 pulses, 3s).\r\n"
                     "Long: forward_long/back_long/left_long/right_long/cw_long/ccw_long.\r\n"
                     "Six *_long commands: 5 RPM, 1600 relative pulses, 8 second stop deadline.\r\n"
                     "Calibration forward_16000: 5 RPM, 16000 relative pulses, 80 second stop deadline.\r\n"
                     "Calibration strafe left_16000/right_16000: 5 RPM, 16000 relative pulses, 80 second stop deadline.\r\n"
                     "Validation forward_1m: nominal 100 cm, 5 RPM, 12924 relative pulses, 80 second stop deadline.\r\n"
                     "Validation back_1m: nominal 100 cm, 5 RPM, 12924 relative pulses, 80 second stop deadline.\r\n"
                     "Each group command needs arm all within 10s.\r\n"
                     "X layout; left/right = strafe; cw/ccw = rotation viewed from above.\r\n"
                     "Addresses 1..4 only. Each motion consumes the arm.\r\n");
    } else if (!strcmp(line, "arm all") ||
               (strlen(line) == 5 && !strncmp(line, "arm ", 4) &&
                line[4] >= '1' && line[4] <= '4')) {
        if (state != IDLE) {
            platform_log("BUSY: stop first.\r\n");
            return;
        }
        all_wheels = !strcmp(line, "arm all");
        if (!all_wheels) selected = (uint8_t)(line[4] - '0');
        fault_latched = false;
        probe_pending = false;
        armed = true;
        arm_at = now;
        platform_log(all_wheels ? "ALL ARMED for 10s. No motion yet. Send one group command; help lists names.\r\n"
                                : "ARMED for 10s. No motion yet. Send jog 0 or jog 1.\r\n");
    } else if (!strcmp(line, "jog 0") || !strcmp(line, "jog 1") ||
               motion != GROUP_NONE) {
        bool group = motion != GROUP_NONE;
        if (!armed || state != IDLE || group != all_wheels) {
            armed = false;
            platform_log("REJECTED: arm 1..4 for jog; arm all for group motion. One motion per arm.\r\n");
            return;
        }
        armed = false;
        if (group) set_group_directions(motion);
        else directions[selected] = (uint8_t)(line[4] - '0');
        active_long_command = long_command_name(motion);
        if (motion == GROUP_LEFT_16000 || motion == GROUP_RIGHT_16000) {
            active_pulses = CAL_STRAFE_PULSES;
            active_timeout_ms = CAL_STRAFE_TIMEOUT_MS;
        } else if (motion == GROUP_BACK_1M) {
            active_pulses = VERIFY_BACK_1M_PULSES;
            active_timeout_ms = VERIFY_BACK_1M_TIMEOUT_MS;
        } else if (motion == GROUP_FORWARD_1M) {
            active_pulses = VERIFY_FORWARD_1M_PULSES;
            active_timeout_ms = VERIFY_FORWARD_1M_TIMEOUT_MS;
        } else if (motion == GROUP_FORWARD_16000) {
            active_pulses = CAL_FORWARD_PULSES;
            active_timeout_ms = CAL_FORWARD_TIMEOUT_MS;
        } else {
            active_pulses = active_long_command ? LONG_TEST_PULSES : TEST_PULSES;
            active_timeout_ms = active_long_command ? LONG_TEST_TIMEOUT_MS : RUN_TIMEOUT_MS;
        }
        started_at = now;
        state = WAIT_ENABLE;
        for (uint8_t id = 1; id <= 4; ++id) {
            if (!all_wheels && id != selected) continue;
            const uint8_t enable[] = {id, 0xF3, 0xAB, 1, 0, 0x6B};
            if (!platform_motor_send(enable, sizeof enable)) {
                test_fault();
                return;
            }
        }
    } else {
        /* 拼错、粘贴了不完整命令，都撤销此前的动作授权。 */
        armed = false;
        platform_log("Unknown command; disarmed. Type help.\r\n");
    }
}
