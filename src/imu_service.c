#include "imu_service.h"
#include "wit_imu.h"
#include <string.h>

/* WIT official normal UART protocol, not Modbus: FF AA reg low high.
 * References and firmware compatibility limits: docs/wt901-calibration.md.
 * No arbitrary register writes, baud changes, factory reset or boot calibration. */
extern void platform_log(const char *text);
typedef enum { IDLE, READING, UNLOCKING, CALIBRATING, CAL_ERROR, DRIFT, CAL_SETTLE } State;
typedef enum { START_ACC, STOP_ACC, SAVE_PARAMS } Write;
static State state;
static Write writing;
static bool tx_enabled, cal_maybe_active, confirm_normal, zero_valid;
static bool read_cooling;
static uint8_t read_base;
static uint32_t began_at, read_count, bad_start, error_start, cool_at;
static uint32_t angle_count, gyro_count, angle_n, gyro_n;
static int32_t zero_yaw, last_yaw, yaw_change, gyro_min, gyro_max;
static int64_t gyro_sum;

static void number(int64_t value)
{
    char s[24];
    size_t n = sizeof s - 1;
    s[n] = 0;
    bool negative = value < 0;
    uint64_t u = negative ? (uint64_t)(-(value + 1)) + 1U : (uint64_t)value;
    do { s[--n] = (char)('0' + u % 10U); u /= 10U; } while (u);
    if (negative) s[--n] = '-';
    platform_log(s + n);
}

static void hex16(uint16_t value)
{
    const char digits[] = "0123456789ABCDEF";
    char s[5];
    for (unsigned i = 0; i < 4; ++i) s[i] = digits[(value >> (12U - i*4U)) & 15U];
    s[4] = 0;
    platform_log(s);
}

static int32_t delta(int32_t value, int32_t reference)
{
    int32_t d = value - reference;
    if (d > 180000) d -= 360000;
    if (d < -180000) d += 360000;
    return d;
}

static bool fresh(uint32_t now)
{
    return imu_angle_fresh(now) && imu_gyro_fresh(now);
}

static bool nearly_still(void)
{
    const WitImu *d = imu_data();
    return d->x_mdps >= -3000 && d->x_mdps <= 3000 &&
           d->y_mdps >= -3000 && d->y_mdps <= 3000 &&
           d->z_mdps >= -3000 && d->z_mdps <= 3000;
}

static bool send_reg(uint8_t reg, uint16_t value)
{
    const uint8_t p[] = {0xFF, 0xAA, reg, (uint8_t)value, (uint8_t)(value >> 8)};
    if (platform_imu_send(p, sizeof p)) return true;
    platform_log("IMU TX failed; sensor state NOT confirmed.\r\n");
    return false;
}

static void queue_write(Write action, uint32_t now)
{
    writing = action;
    if (!send_reg(0x69, 0xB588)) {
        state = cal_maybe_active ? CAL_ERROR : IDLE;
        return;
    }
    state = UNLOCKING;
    began_at = now;
}

static void read_registers(uint8_t base, bool normal_check, uint32_t now)
{
    read_base = base;
    confirm_normal = normal_check;
    read_count = imu_data()->reg_samples;
    began_at = now;
    if (send_reg(0x27, base)) state = READING;
    else state = cal_maybe_active ? CAL_ERROR : IDLE;
}

void imu_service_init(void)
{
    state = IDLE;
    tx_enabled = cal_maybe_active = confirm_normal = zero_valid = read_cooling = false;
}

bool imu_service_busy(void) { return state != IDLE || cal_maybe_active; }
bool imu_service_tx_enabled(void) { return tx_enabled; }

void imu_service_report(uint32_t now)
{
    static const char *const names[] = {"idle", "read", "unlock", "acc-cal", "cal-uncertain", "drift", "cal-settle"};
    platform_log("IMU service: state="); platform_log(names[state]);
    platform_log(tx_enabled ? " | TX=ON" : " | TX=OFF");
    platform_log(" | relative_valid="); number(zero_valid && imu_angle_fresh(now));
    platform_log(" | relative_yaw_mdeg=");
    number(zero_valid ? delta(imu_data()->yaw_mdeg, zero_yaw) : 0);
    platform_log(" | raw yaw/heading controller unchanged\r\n");
}

bool imu_service_command(const char *line, uint32_t now)
{
    bool tx_on = !strcmp(line, "imu tx on"), tx_off = !strcmp(line, "imu tx off");
    bool cal = !strcmp(line, "imu cal acc"), stop = !strcmp(line, "imu cal stop");
    bool check = !strcmp(line, "imu cal check");
    bool save = !strcmp(line, "imu save"), drift = !strcmp(line, "imu drift");
    bool zero = !strcmp(line, "imu zero"), cancel = !strcmp(line, "imu cancel");
    static const struct { const char *command; uint8_t base; } reads[] = {
        {"imu read mode", 0x24}, {"imu read output", 0x02},
        {"imu read filter", 0x1F}, {"imu read version", 0x2E}
    };
    uint8_t base = 0;
    for (unsigned i = 0; i < sizeof reads / sizeof reads[0]; ++i)
        if (!strcmp(line, reads[i].command)) base = reads[i].base;
    if (!(tx_on || tx_off || cal || stop || save || drift || zero || cancel || check || base)) return false;
    if (cancel) {
        if (cal_maybe_active || (state == UNLOCKING && writing == START_ACC))
            platform_log("IMU calibration may be active; use imu cal stop.\r\n");
        else {
            if (state == READING) { read_cooling = true; cool_at = now; }
            state = IDLE;
            platform_log("IMU diagnostic cancelled; no sensor command sent.\r\n");
        }
        return true;
    }
    if (stop && tx_enabled) {
        /* Also recovers a calibration interrupted by an MCU reset. */
        cal_maybe_active = true;
        queue_write(STOP_ACC, now);
        platform_log("IMU normal-mode request queued; waiting for CALSW readback.\r\n");
        return true;
    }
    if (check && tx_enabled && (state == IDLE || state == CAL_ERROR)) {
        cal_maybe_active = true;
        read_registers(0x01, true, now);
        return true;
    }
    if (state == CAL_ERROR && base && tx_enabled) {
        read_registers(base, false, now);
        return true;
    }
    if (imu_service_busy()) {
        platform_log("IMU busy: wait; imu cancel for diagnostics, imu cal stop for calibration.\r\n");
        return true;
    }
    if (tx_on || tx_off) {
        bool enable = tx_on;
        if (platform_imu_tx_enable(enable)) {
            tx_enabled = enable;
            platform_log(enable ? "IMU TX enabled on PD8; NO command sent.\r\n" :
                                 "IMU TX disabled; PD8 high-impedance.\r\n");
        } else platform_log("IMU UART reconfiguration failed.\r\n");
        return true;
    }
    if (zero) {
        if (!imu_angle_fresh(now)) platform_log("IMU zero rejected: fresh angle required.\r\n");
        else {
            zero_yaw = imu_data()->yaw_mdeg; zero_valid = true;
            platform_log("IMU software yaw reference set; raw yaw unchanged; no sensor write.\r\n");
        }
        return true;
    }
    if (drift) {
        if (!fresh(now)) platform_log("IMU drift rejected: fresh angle AND gyro frames required.\r\n");
        else {
            const WitImu *d = imu_data();
            state = DRIFT; began_at = now;
            angle_count = d->angle_samples; gyro_count = d->gyro_samples;
            last_yaw = d->yaw_mdeg; yaw_change = 0;
            angle_n = gyro_n = 0; gyro_sum = 0;
            gyro_min = INT32_MAX; gyro_max = INT32_MIN;
            bad_start = d->bad_checksum; error_start = d->uart_errors;
            platform_log("IMU drift: keep chassis STILL for 10s; motors locked; no sensor writes.\r\n");
        }
        return true;
    }
    if (!tx_enabled) {
        platform_log("IMU TX is off: connect PD8 -> WT901 RX, then imu tx on.\r\n");
        return true;
    }
    if (read_cooling && (uint32_t)(now - cool_at) < 1000U) {
        platform_log("IMU wait 1s after cancelled/timed-out read; late replies have no address.\r\n");
        return true;
    }
    read_cooling = false;
    if (base) {
        read_registers(base, false, now);
        platform_log("IMU register query sent; one outstanding request; not a calibration result.\r\n");
    } else if (cal) {
        const WitImu *d = imu_data();
        if (!fresh(now) || !nearly_still() || d->roll_mdeg < -5000 || d->roll_mdeg > 5000 ||
            d->pitch_mdeg < -5000 || d->pitch_mdeg > 5000) {
            platform_log("IMU cal rejected: fresh angle/gyro, still and physically LEVEL required.\r\n");
        } else {
            zero_valid = false;
            queue_write(START_ACC, now);
            platform_log("IMU acc/gyro calibration queued: keep physically LEVEL and STILL for 5s.\r\n");
        }
    } else if (save) {
        queue_write(SAVE_PARAMS, now);
    }
    return true;
}

void imu_service_tick(uint32_t now)
{
    const WitImu *d = imu_data();
    if (state == UNLOCKING && (uint32_t)(now - began_at) >= 20U) {
        /* Official normal-protocol SDK waits >=1ms after unlock; use 20ms margin. */
        if (writing == START_ACC) cal_maybe_active = true;
        bool ok = send_reg(writing == SAVE_PARAMS ? 0x00 : 0x01, writing == START_ACC ? 1 : 0);
        if (!ok) { state = cal_maybe_active ? CAL_ERROR : IDLE; return; }
        if (writing == START_ACC) { state = CALIBRATING; began_at = now; }
        else if (writing == STOP_ACC) {
            state = CAL_SETTLE; began_at = now;
        }
        else {
            state = IDLE;
            platform_log("IMU SAVE sent to sensor; no ACK in normal UART; power-cycle retention NOT verified.\r\n");
        }
    } else if (state == CALIBRATING) {
        if (imu_gyro_fresh(now) && !nearly_still()) {
            platform_log("IMU moved during calibration; ending now; result NOT validated.\r\n");
            queue_write(STOP_ACC, now);
        } else if ((uint32_t)(now - began_at) >= 5000U) queue_write(STOP_ACC, now);
    } else if (state == CAL_SETTLE && (uint32_t)(now - began_at) >= 100U) {
        read_registers(0x01, true, now);
    } else if (state == READING) {
        if ((uint32_t)(now - began_at) >= 500U) {
            state = cal_maybe_active ? CAL_ERROR : IDLE;
            read_cooling = true; cool_at = now;
            platform_log("IMU register timeout: no valid 0x5F; check PD8 -> sensor RX and firmware.\r\n");
        } else if (d->reg_samples != read_count) {
            platform_log("IMU REGS requested_base=0x"); hex16(read_base);
            for (unsigned i = 0; i < 4; ++i) { platform_log(" "); hex16(d->reg_words[i]); }
            platform_log(" (normal reply does NOT echo base)\r\n");
            if (confirm_normal) {
                if (d->reg_words[0] != 0) {
                    state = CAL_ERROR;
                    platform_log("IMU CALSW not normal; motors locked; imu cal stop to retry.\r\n");
                } else {
                    cal_maybe_active = false; state = IDLE;
                    platform_log("IMU CALSW=0 observed; run imu drift to assess result; imu save is explicit.\r\n");
                }
            } else {
                state = cal_maybe_active ? CAL_ERROR : IDLE;
                if (read_base == 0x24) {
                    platform_log(d->reg_words[0] == 0 ? "IMU mode: 9-axis (official SDK map).\r\n" :
                                 d->reg_words[0] == 1 ? "IMU mode: 6-axis (official SDK map).\r\n" :
                                                       "IMU mode: unknown firmware value.\r\n");
                }
            }
        }
    } else if (state == DRIFT) {
        if (!fresh(now) || d->bad_checksum != bad_start || d->uart_errors != error_start) {
            state = IDLE;
            platform_log("IMU drift aborted: stale data or UART/checksum error; no quality result.\r\n");
            return;
        }
        if (!nearly_still()) {
            state = IDLE;
            platform_log("IMU drift aborted: chassis moved or gyro exceeds 3deg/s.\r\n");
            return;
        }
        if (d->angle_samples != angle_count) {
            angle_count = d->angle_samples; ++angle_n;
            yaw_change += delta(d->yaw_mdeg, last_yaw); last_yaw = d->yaw_mdeg;
        }
        if (d->gyro_samples != gyro_count) {
            gyro_count = d->gyro_samples; ++gyro_n; gyro_sum += d->z_mdps;
            if (d->z_mdps < gyro_min) gyro_min = d->z_mdps;
            if (d->z_mdps > gyro_max) gyro_max = d->z_mdps;
        }
        if ((uint32_t)(now - began_at) >= 10000U) {
            state = IDLE;
            if (angle_n < 10 || gyro_n < 10) {
                platform_log("IMU drift invalid: fewer than 10 new angle/gyro observations.\r\n");
                return;
            }
            platform_log("IMU DRIFT yaw_mdeg="); number(yaw_change);
            platform_log(" | equivalent_mdeg_per_min=");
            number((int64_t)yaw_change * 60000 / (uint32_t)(now - began_at));
            platform_log(" | mean_z_mdps="); number(gyro_sum / gyro_n);
            platform_log(" | z_peak_to_peak_mdps="); number((int64_t)gyro_max - gyro_min);
            platform_log(" | observed_angle/gyro="); number(angle_n); platform_log("/"); number(gyro_n);
            platform_log("; extrapolated rate, NOT long-term accuracy; no automatic bias subtraction\r\n");
        }
    }
}
