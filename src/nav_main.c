#include "stm32f4xx_hal.h"
#include "nav/nav_app.h"
#include "nav/nav_bus.h"
#include "nav/nav_odom.h"
#include "nav/nav_platform.h"
#include "wit_imu.h"
#include "imu_service.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
static UART_HandleTypeDef motor_uart,console_uart,imu_uart;
void _init(void){}
#define RX_CAPACITY 512U
typedef struct {volatile uint16_t head,tail;volatile bool fault;uint8_t bytes[RX_CAPACITY];uint32_t at[RX_CAPACITY];} Ring;
static Ring motor_rx,console_rx,imu_rx;
static volatile bool emergency_byte;
static uint8_t motor_tx[16];static volatile unsigned motor_n,motor_offset;
static volatile bool motor_busy;static volatile uint32_t motor_end;
#define LOG_N 8192U
static char log_bytes[LOG_N];static volatile unsigned log_head,log_tail;
static uint32_t log_drops,main_max;
static void rx_irq(USART_TypeDef *u,Ring *r)
{
 uint32_t flags=u->SR;
 if(flags&(USART_SR_RXNE|USART_SR_ORE|USART_SR_NE|USART_SR_FE|USART_SR_PE)){
  uint8_t byte=(uint8_t)u->DR;
  if(flags&(USART_SR_ORE|USART_SR_NE|USART_SR_FE|USART_SR_PE)){r->fault=true;return;}
  if(r==&console_rx&&byte=='!'){emergency_byte=true;return;}
  unsigned next=(r->head+1U)&(RX_CAPACITY-1);
  if(next==r->tail)r->fault=true;else{r->bytes[r->head]=byte;r->at[r->head]=HAL_GetTick();__DMB();r->head=(uint16_t)next;}
 }
}
void SysTick_Handler(void){HAL_IncTick();}
void USART1_IRQHandler(void)
{
 rx_irq(USART1,&console_rx);
 if((USART1->CR1&USART_CR1_TXEIE)&&(USART1->SR&USART_SR_TXE)){
  if(log_tail!=log_head){USART1->DR=(uint8_t)log_bytes[log_tail];log_tail=(log_tail+1)&(LOG_N-1);}
  else USART1->CR1&=~USART_CR1_TXEIE;
 }
}
void USART3_IRQHandler(void){rx_irq(USART3,&imu_rx);}
void UART4_IRQHandler(void)
{
 rx_irq(UART4,&motor_rx);
 if((UART4->CR1&USART_CR1_TXEIE)&&(UART4->SR&USART_SR_TXE)){
  UART4->DR=motor_tx[motor_offset++];
  if(motor_offset==motor_n){UART4->CR1&=~USART_CR1_TXEIE;UART4->CR1|=USART_CR1_TCIE;}
 }
 if((UART4->CR1&USART_CR1_TCIE)&&(UART4->SR&USART_SR_TC)){
  UART4->CR1&=~USART_CR1_TCIE;UART4->SR=~USART_SR_TC;motor_end=HAL_GetTick();__DMB();motor_busy=false;
 }
}
static bool pop(Ring *r,uint8_t *v,uint32_t *at)
{if(r->tail==r->head)return false;__DMB();*v=r->bytes[r->tail];*at=r->at[r->tail];r->tail=(r->tail+1U)&(RX_CAPACITY-1);return true;}
bool nav_hw_tx(const uint8_t *p,size_t n)
{
 if(motor_busy||!n||n>sizeof motor_tx)return false;
 if(emergency_byte && !(n>=2&&p[1]==0xFE) && !(n>=4&&p[1]==0xF3&&p[3]==0))return false;
 memcpy(motor_tx,p,n);motor_n=(unsigned)n;motor_offset=0;motor_busy=true;__DMB();UART4->CR1|=USART_CR1_TXEIE;return true;
}
bool nav_hw_tx_busy(void){return motor_busy;}
uint32_t nav_hw_tx_end(void){return motor_end;}
uint32_t nav_hw_cycles(void){return DWT->CYCCNT;}
void nav_log(const char *s)
{
 unsigned n=(unsigned)strlen(s);uint32_t irq=__get_PRIMASK();__disable_irq();
 unsigned free=(log_tail-log_head-1U)&(LOG_N-1U);
 if(n>free){log_drops++;if(!irq)__enable_irq();return;}
 for(unsigned k=0;k<n;k++){log_bytes[log_head]=s[k];log_head=(log_head+1)&(LOG_N-1);}
 USART1->CR1|=USART_CR1_TXEIE;if(!irq)__enable_irq();
}
void nav_logf(const char *fmt,...){char line[512];va_list args;va_start(args,fmt);vsnprintf(line,sizeof line,fmt,args);va_end(args);nav_log(line);}
void platform_log(const char *s){nav_log(s);}
static bool take_flag(volatile bool *flag)
{
 uint32_t irq=__get_PRIMASK();__disable_irq();bool result=*flag;*flag=false;
 if(!irq)__enable_irq();
 return result;
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


/* The independent watchdog remains active if the debugger halts the CPU.
 * No vehicle motion debugging by CPU halt. Nominal timeout ~1s at LSI=32kHz.
 * A reset re-enters boot FE+disable; lost power/physical bus remains external. */
static void watchdog_init(void)
{__HAL_DBGMCU_UNFREEZE_IWDG();IWDG->KR=0xCCCC;IWDG->KR=0x5555;IWDG->PR=4;IWDG->RLR=499;while(IWDG->SR){}IWDG->KR=0xAAAA;}
int main(void)
{
 HAL_Init();clock_init();uart_init();
 CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;DWT->CYCCNT=0;DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;
 watchdog_init();nav_app_init(HAL_GetTick());
 char line[100];unsigned len=0;bool overflow=false;uint32_t angle_samples=0;
 for(;;){
  uint32_t start=DWT->CYCCNT,now=HAL_GetTick(),at;uint8_t byte;
  if(!nav_bus_data()->tx_fault)IWDG->KR=0xAAAA;
  if(take_flag(&emergency_byte)){len=0;overflow=false;nav_app_emergency(now,"operator ! byte");}
  if(take_flag(&motor_rx.fault))nav_bus_uart_error();
  if(take_flag(&imu_rx.fault))imu_uart_error();
  if(take_flag(&console_rx.fault)){len=0;overflow=true;nav_app_emergency(now,"console RX overrun");}
  while(pop(&motor_rx,&byte,&at))nav_bus_feed(byte,at);
  while(pop(&imu_rx,&byte,&at)){
   imu_feed(byte,at);const WitImu *i=imu_data();
   if(i->angle_samples!=angle_samples){angle_samples=i->angle_samples;nav_odom_yaw(i->yaw_mdeg,i->angle_at);}
  }
  unsigned console_bytes=0,console_commands=0;
  while(console_bytes<64&&console_commands<2&&pop(&console_rx,&byte,&at)){
   console_bytes++;
   if(byte=='\r'||byte=='\n'){
    if(overflow){console_commands++;nav_log("[E] console line discarded: too long or non-ASCII/control byte\r\n");}
    if(!overflow&&len){line[len]=0;console_commands++;if(!strcmp(line,"perf"))nav_logf("[HW] main_loop_max_cycles=%lu log_drops=%lu reset_CSR=%08lX\r\n",(unsigned long)main_max,(unsigned long)log_drops,(unsigned long)RCC->CSR);nav_app_command(line,HAL_GetTick());}
    len=0;overflow=false;
   }else if(byte=='\t'||(byte>=32&&byte<127)){if(len+1<sizeof line)line[len++]=(char)byte;else overflow=true;}
   else overflow=true;
  }
  nav_app_tick(HAL_GetTick()); /* RX/idle calibration may advance time after loop entry. */
  uint32_t duration=DWT->CYCCNT-start;if(duration>main_max)main_max=duration;
 }
}
