#include "stm32f4xx_hal.h"
#include "motor_test.h"
#include "chassis_config.h"
#include "wit_imu.h"
#include "yaw_trace.h"
#include "heading_hold.h"
#include "imu_service.h"
#include <string.h>

static UART_HandleTypeDef motor_uart, console_uart, imu_uart;
static YawTrace yaw_trace;
/* 自带启动文件已完成数据初始化，不使用桌面 C 运行库启动代码。 */
void _init(void) {}

/* 中断收字节、主循环处理。发日志或发电机命令时也不会漏掉串口接收。 */
#define RX_CAPACITY 1024U
typedef struct {
    volatile uint16_t head, tail;
    volatile bool fault;
    uint8_t bytes[RX_CAPACITY];
} Ring;
static Ring console_rx, motor_rx, imu_rx;

static void rx_irq(USART_TypeDef *uart, Ring *r)
{
    uint32_t flags = uart->SR;
    if (flags & (USART_SR_RXNE | USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) {
        uint8_t byte = (uint8_t)uart->DR; /* 读 SR 后读 DR，同时清除错误标志。 */
        if (flags & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) {
            r->fault = true;
            return;
        }
        uint16_t next = (r->head + 1U) & (RX_CAPACITY - 1U);
        if (next == r->tail) r->fault = true;
        else { r->bytes[r->head] = byte; r->head = next; }
    }
}
void USART1_IRQHandler(void) { rx_irq(USART1, &console_rx); }
void UART4_IRQHandler(void) { rx_irq(UART4, &motor_rx); }
void USART3_IRQHandler(void) { rx_irq(USART3, &imu_rx); }
void SysTick_Handler(void) { HAL_IncTick(); }

static bool pop(Ring *r, uint8_t *byte)
{
    if (r->tail == r->head) return false;
    *byte = r->bytes[r->tail];
    r->tail = (r->tail + 1U) & (RX_CAPACITY - 1U);
    return true;
}

void platform_log(const char *s)
{
    /* 610 字节帮助文本在 115200/8N1 下需要约 53ms，整段发送会超过
     * 原 50ms 超时而截断末尾。按 128 字节发送，每段约 11.1ms。 */
    size_t remaining = strlen(s);
    while (remaining) {
        uint16_t chunk = remaining > 128U ? 128U : (uint16_t)remaining;
        if (HAL_UART_Transmit(&console_uart, (uint8_t *)s, chunk, 50) != HAL_OK) return;
        s += chunk;
        remaining -= chunk;
    }
}
bool platform_motor_send(const uint8_t *data, size_t len)
{
    bool ok = HAL_UART_Transmit(&motor_uart, (uint8_t *)data, (uint16_t)len, 20) == HAL_OK;
    HAL_Delay(5); /* 给驱动器处理命令和回复留出间隔；接收中断仍在运行。 */
    return ok;
}

static void clock_init(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};
    /* 用户确认 HSE=25 MHz。25/25*336/2=168 MHz，PLLQ=7 得到 48 MHz。
     * 低速 32.768 kHz 晶振在这个单电机测试里不启用。
     * 此配置按正常 3.3 V 主控供电设置电压等级和 Flash 等待周期。 */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM = 25;
    osc.PLL.PLLN = 336;
    osc.PLL.PLLP = RCC_PLLP_DIV2;
    osc.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) while (1) {}
    clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV4; /* UART4 在 APB1，42 MHz。 */
    clk.APB2CLKDivider = RCC_HCLK_DIV2; /* USART1 在 APB2，84 MHz。 */
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK) while (1) {}
}

static void uart_init(void)
{
    GPIO_InitTypeDef io = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_UART4_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();

    /* Console: USART1 PA9 TX / PA10 RX, AF7. */
    io.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    io.Mode = GPIO_MODE_AF_PP;
    io.Pull = GPIO_PULLUP;
    io.Speed = GPIO_SPEED_FREQ_HIGH;
    io.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &io);
    /* Motor: UART4 PA0 TX / PA1 RX, AF8. */
    io.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    io.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOA, &io);

    /* IMU: PD9 RX, AF7. PD8 stays high-impedance; no sensor writes. */
    io.Pin = GPIO_PIN_8;
    io.Mode = GPIO_MODE_INPUT;
    io.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOD, &io);
    io.Pin = GPIO_PIN_9;
    io.Mode = GPIO_MODE_AF_PP;
    io.Pull = GPIO_PULLUP;
    io.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &io);

    motor_uart.Instance = UART4;
    motor_uart.Init.BaudRate = 115200;
    motor_uart.Init.WordLength = UART_WORDLENGTH_8B;
    motor_uart.Init.StopBits = UART_STOPBITS_1;
    motor_uart.Init.Parity = UART_PARITY_NONE;
    motor_uart.Init.Mode = UART_MODE_TX_RX;
    motor_uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    motor_uart.Init.OverSampling = UART_OVERSAMPLING_16;
    console_uart = motor_uart;
    console_uart.Instance = USART1;
    imu_uart = motor_uart;
    imu_uart.Instance = USART3;
    imu_uart.Init.BaudRate = IMU_DEFAULT_BAUD;
    imu_uart.Init.Mode = UART_MODE_RX;
    if (HAL_UART_Init(&motor_uart) != HAL_OK || HAL_UART_Init(&console_uart) != HAL_OK ||
        HAL_UART_Init(&imu_uart) != HAL_OK) while (1) {}
    HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
    HAL_NVIC_SetPriority(UART4_IRQn, 2, 0);
    HAL_NVIC_SetPriority(USART3_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_NVIC_EnableIRQ(UART4_IRQn);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
    __HAL_UART_ENABLE_IT(&motor_uart, UART_IT_RXNE);
    __HAL_UART_ENABLE_IT(&console_uart, UART_IT_RXNE);
    __HAL_UART_ENABLE_IT(&imu_uart, UART_IT_RXNE);
}

/* Only IMU service uses these callbacks; never routed to motor_uart. */
bool platform_imu_tx_enable(bool enable)
{
    GPIO_InitTypeDef io = {0};
    HAL_NVIC_DisableIRQ(USART3_IRQn);
    __HAL_UART_DISABLE(&imu_uart);
    imu_uart.Init.Mode = enable ? UART_MODE_TX_RX : UART_MODE_RX;
    HAL_StatusTypeDef result = HAL_UART_Init(&imu_uart);
    io.Pin = GPIO_PIN_8;
    io.Mode = enable && result == HAL_OK ? GPIO_MODE_AF_PP : GPIO_MODE_INPUT;
    io.Pull = GPIO_NOPULL;
    io.Speed = GPIO_SPEED_FREQ_HIGH;
    io.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &io);
    __HAL_UART_ENABLE_IT(&imu_uart, UART_IT_RXNE);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
    return result == HAL_OK;
}

bool platform_imu_send(const uint8_t *bytes, size_t len)
{
    return imu_service_tx_enabled() && len == 5U &&
           HAL_UART_Transmit(&imu_uart, (uint8_t *)bytes, (uint16_t)len, 20) == HAL_OK;
}

static void log_number(uint32_t value, uint32_t base)
{
    /* 只转换整数，不引入 printf、堆分配或浮点格式化。 */
    char text[11];
    size_t used = sizeof text - 1U;
    text[used] = 0;
    do {
        text[--used] = "0123456789ABCDEF"[value % base];
        value /= base;
    } while (value);
    platform_log(&text[used]);
}

static void log_signed(int32_t value)
{
    if (value < 0) {
        platform_log("-");
        log_number((uint32_t)(-(int64_t)value), 10);
    } else log_number((uint32_t)value, 10);
}

static void report_imu(void)
{
    const WitImu *d = imu_data();
    uint32_t now = HAL_GetTick();
    platform_log("IMU USART3 PD8/PD9 ");
    platform_log(imu_service_tx_enabled() ? "TX_RX | baud=" : "RX_ONLY | baud="); log_number(imu_uart.Init.BaudRate, 10);
    platform_log(" | bytes="); log_number(d->bytes, 10);
    platform_log(" | frames="); log_number(d->frames, 10);
    platform_log(" | bad_checksum="); log_number(d->bad_checksum, 10);
    platform_log(" | uart_errors="); log_number(d->uart_errors, 10);
    platform_log("\r\nIMU angle: valid="); log_number(imu_angle_fresh(now), 10);
    platform_log(" | samples="); log_number(d->angle_samples, 10);
    platform_log(" | age_ms="); log_number(d->angle_seen ? now - d->angle_at : UINT32_MAX, 10);
    platform_log(" | roll_mdeg="); log_signed(d->roll_mdeg);
    platform_log(" | pitch_mdeg="); log_signed(d->pitch_mdeg);
    platform_log(" | yaw_mdeg="); log_signed(d->yaw_mdeg);
    platform_log("\r\nIMU gyro: valid="); log_number(imu_gyro_fresh(now), 10);
    platform_log(" | samples="); log_number(d->gyro_samples, 10);
    platform_log(" | age_ms="); log_number(d->gyro_seen ? now - d->gyro_at : UINT32_MAX, 10);
    platform_log(" | x/y/z_mdps="); log_signed(d->x_mdps);
    platform_log("/"); log_signed(d->y_mdps); platform_log("/"); log_signed(d->z_mdps);
    platform_log("\r\nIMU auxiliary: acc_samples="); log_number(d->acc_samples, 10);
    platform_log(" | mag_samples="); log_number(d->mag_samples, 10);
    platform_log(" | angle_word4_hex="); log_number(d->angle_word4, 16);
    platform_log(" (SDK maps to version; check sensor firmware)\r\n");
    imu_service_report(now);
}

static bool handle_imu_command(const char *line)
{
    if (strcmp(line, "imu") && strncmp(line, "imu ", 4)) return false;
    /* 不消费arm、不补发运动；诊断之前撤销尚未使用的授权。 */
    if (!test_begin_diagnostic()) return true;
    if (!strcmp(line, "imu")) { report_imu(); return true; }
    if (imu_service_command(line, HAL_GetTick())) return true;
    if (imu_service_busy()) {
        platform_log("IMU busy; baud changes unavailable until diagnostic/calibration ends.\r\n");
        return true;
    }
    static const struct { const char *command; uint32_t baud; } rates[] = {
        {"imu baud 4800", 4800}, {"imu baud 9600", 9600},
        {"imu baud 19200", 19200}, {"imu baud 38400", 38400},
        {"imu baud 57600", 57600}, {"imu baud 115200", 115200},
        {"imu baud 230400", 230400}, {"imu baud 460800", 460800},
        {"imu baud 921600", 921600}
    };
    for (unsigned i = 0; i < sizeof rates / sizeof rates[0]; ++i) {
        if (strcmp(line, rates[i].command)) continue;
        HAL_NVIC_DisableIRQ(USART3_IRQn);
        __HAL_UART_DISABLE(&imu_uart);
        imu_rx.tail = imu_rx.head = 0;
        imu_rx.fault = false;
        volatile uint32_t discard = USART3->SR;
        discard = USART3->DR;
        (void)discard;
        imu_init();
        imu_service_init();
        GPIO_InitTypeDef off = {0};
        off.Pin = GPIO_PIN_8; off.Mode = GPIO_MODE_INPUT; off.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOD, &off);
        imu_uart.Init.Mode = UART_MODE_RX;
        imu_uart.Init.BaudRate = rates[i].baud;
        HAL_StatusTypeDef ok = HAL_UART_Init(&imu_uart);
        __HAL_UART_ENABLE_IT(&imu_uart, UART_IT_RXNE);
        HAL_NVIC_EnableIRQ(USART3_IRQn);
        if (ok != HAL_OK) platform_log("IMU UART configuration failed.\r\n");
        else {
            platform_log("IMU MCU RX baud="); log_number(rates[i].baud, 10);
            platform_log("; counters reset; no sensor writes.\r\n");
        }
        return true;
    }
    platform_log("IMU service: imu zero; imu drift; imu cancel; imu tx on/off; imu cal acc; imu cal stop; imu cal check; imu save; imu read mode/output/filter/version.\r\n");
    platform_log("IMU commands: imu; imu baud 4800/9600/19200/38400/57600/115200/230400/460800/921600.\r\n");
    return true;
}

static void report_status(void)
{
    /* DEVID 0x413 identifies a family; report Flash size separately. */
    platform_log("STM32F407VET6 test | DEVID=0x");
    log_number(DBGMCU->IDCODE & 0xFFFU, 16);
    platform_log(" | Flash=");
    log_number(*(const uint16_t *)FLASHSIZE_BASE, 10);
    platform_log(" KiB\r\nHSE=25MHz | HCLK=");
    log_number(HAL_RCC_GetHCLKFreq(), 10);
    platform_log(" Hz | uptime=");
    log_number(HAL_GetTick(), 10);
    platform_log(" ms\r\nConsole: USART1 PA9/PA10 | Motor: UART4 PA0/PA1 | 115200 8N1\r\n");
    platform_log("Wheels: FL="); log_number(CHASSIS_FRONT_LEFT_ID, 10);
    platform_log(" FR="); log_number(CHASSIS_FRONT_RIGHT_ID, 10);
    platform_log(" RL="); log_number(CHASSIS_REAR_LEFT_ID, 10);
    platform_log(" RR="); log_number(CHASSIS_REAR_RIGHT_ID, 10);
    platform_log(" | all forward dir="); log_number(CHASSIS_FORWARD_DIRECTION, 10);
    platform_log("\r\n");
    platform_log("Firmware: " TEST_FIRMWARE_VERSION " | Mecanum: " CHASSIS_LAYOUT_NAME
                 " | Group: forward/back/left/right/cw/ccw\r\n"
                 "Long group: forward_long/back_long/left_long/right_long/cw_long/ccw_long\r\n");
    platform_log("Long motion: *_long | RPM="); log_number(TEST_RPM, 10);
    platform_log(" | pulses="); log_number(LONG_TEST_PULSES, 10);
    platform_log(" | deadline_ms="); log_number(LONG_TEST_TIMEOUT_MS, 10);
    platform_log("\r\n");
    platform_log("Calibration: forward_16000 | RPM="); log_number(TEST_RPM, 10);
    platform_log(" | pulses="); log_number(CAL_FORWARD_PULSES, 10);
    platform_log(" | deadline_ms="); log_number(CAL_FORWARD_TIMEOUT_MS, 10);
    platform_log("\r\n");
    platform_log("Calibration strafe: left_16000/right_16000 | RPM="); log_number(TEST_RPM, 10);
    platform_log(" | pulses="); log_number(CAL_STRAFE_PULSES, 10);
    platform_log(" | deadline_ms="); log_number(CAL_STRAFE_TIMEOUT_MS, 10);
    platform_log("\r\n");
    platform_log("Validation: forward_1m | nominal_cm=100 | RPM="); log_number(TEST_RPM, 10);
    platform_log(" | pulses="); log_number(VERIFY_FORWARD_1M_PULSES, 10);
    platform_log(" | deadline_ms="); log_number(VERIFY_FORWARD_1M_TIMEOUT_MS, 10);
    platform_log("\r\n");
    platform_log("Validation: back_1m | nominal_cm=100 | RPM="); log_number(TEST_RPM, 10);
    platform_log(" | pulses="); log_number(VERIFY_BACK_1M_PULSES, 10);
    platform_log(" | deadline_ms="); log_number(VERIFY_BACK_1M_TIMEOUT_MS, 10);
    platform_log("\r\n");
    platform_log("IMU: USART3 PD8/PD9 | ");
    platform_log(imu_service_tx_enabled() ? "TX_RX | baud=" : "RX_ONLY | baud="); log_number(imu_uart.Init.BaudRate, 10);
    platform_log(hold_active() ? " | heading control=SEGMENTED_LEFT\r\n" : " | heading control=OFF\r\n");
    platform_log("YAW trace: moving only | interval_ms=200 | heading control=OFF unless hold active\r\n");
    hold_report();
}

int main(void)
{
    HAL_Init();
    clock_init();
    uart_init();
    imu_init();
    imu_service_init();
    yaw_trace_init(&yaw_trace);
    hold_init();
    /* 上电先发停止/失能，随后等待。这里没有自动行走程序。 */
    test_init();
    report_status();
    char line[48];
    size_t used = 0;
    bool dropping = false;
    for (;;) {
        if (console_rx.fault) {
            __disable_irq();
            console_rx.tail = console_rx.head;
            console_rx.fault = false;
            __enable_irq();
            used = 0; dropping = true;
            hold_emergency_stop();
            test_fault();
        }
        if (motor_rx.fault) {
            __disable_irq();
            motor_rx.tail = motor_rx.head;
            motor_rx.fault = false;
            __enable_irq();
            /* 电机串口错误会停机，但不能吞掉独立 USART1 的诊断命令。 */
            hold_emergency_stop();
            test_fault();
        }
        uint8_t c;
        if (imu_rx.fault) {
            __disable_irq();
            imu_rx.tail = imu_rx.head;
            imu_rx.fault = false;
            __enable_irq();
            /* 扫描错误波特率时只使IMU数据失效，不触发电机收包故障。 */
            imu_uart_error();
        }
        for (unsigned i = 0; i < 128 && pop(&imu_rx, &c); ++i) imu_feed(c, HAL_GetTick());
        /* 每轮最多处理 32 个字节，给超时逻辑和电机回复留出时间。 */
        for (unsigned i = 0; i < 32 && pop(&console_rx, &c); ++i) {
            if (c == '\r' || c == '\n') {
                if (!dropping && used) {
                    line[used] = '\0';
                    if (imu_service_busy() && strcmp(line, "stop") && strcmp(line, "status") &&
                        strcmp(line, "hold_status") && strcmp(line, "help") && strcmp(line, "imu") &&
                        strncmp(line, "imu ", 4)) {
                        platform_log("IMU diagnostic/calibration active; motor commands blocked.\r\n");
                    } else if (hold_command(line, HAL_GetTick())) { /* isolated heading mode */ }
                    else if (!strcmp(line, "status")) report_status();
                    else if (handle_imu_command(line)) { /* IMU commands never enter the motor parser. */ }
                    else test_command(line, HAL_GetTick());
                }
                used = 0; dropping = false;
            } else if (!dropping) {
                if (c == 8 || c == 127) { if (used) --used; }
                else if (c >= 32 && c <= 126 && used < sizeof(line)-1) line[used++] = (char)c;
                else { used = 0; dropping = true; hold_emergency_stop(); test_fault(); }
            }
        }
        /* 先处理已收到的 stop/串口错误，再处理到期的待发运动。 */
        test_tick(HAL_GetTick());
        /* 仅显示原始回复，不把“发出命令”伪装成“电机已到位”。
         * 回复可被拆成数行；当前版本不解析驱动器确认/到位帧。 */
        char hex[3 * 16 + 10];
        size_t n = 0;
        memcpy(hex, "RX: ", 4); n = 4;
        const char digits[] = "0123456789ABCDEF";
        for (unsigned i = 0; i < 16 && pop(&motor_rx, &c); ++i) {
            test_motor_rx(c, HAL_GetTick());
            hold_motor_rx(c, HAL_GetTick());
            hex[n++] = digits[c >> 4]; hex[n++] = digits[c & 15]; hex[n++] = ' ';
        }
        if (n > 4) { hex[n++] = '\r'; hex[n++] = '\n'; hex[n] = 0; platform_log(hex); }
        /* stop/错误/tick及电机收包先处理；日志不改运动参数或执行新动作。 */
        imu_service_tick(HAL_GetTick());
        hold_tick(HAL_GetTick());
        char trace_line[YAW_TRACE_LINE_CAPACITY];
        if (yaw_trace_poll(&yaw_trace, HAL_GetTick(), test_is_moving() || hold_active(), imu_data(),
                           trace_line, sizeof trace_line)) platform_log(trace_line);
    }
}
