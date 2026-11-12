/* HSE 25 MHz -> PLL(M25,N336,P2,Q7) -> 168 MHz, with register readback check. */
#include "board_clock.h"

#include "app_config.h"
#include "board_config.h"
#include "timebase.h"

static BoardClockInfo info;

void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
    timebase_dwt_init();
}

bool board_clock_init(BoardClockInfo *out)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM = 25U;
    osc.PLL.PLLN = 336U;
    osc.PLL.PLLP = RCC_PLLP_DIV2;
    osc.PLL.PLLQ = 7U;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) return false;

    clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 |
                    RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV4;
    clk.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK) return false;

    SystemCoreClockUpdate();

    uint32_t pllcfgr = RCC->PLLCFGR;
    info.hse_hz = HSE_VALUE;
    info.sysclk_hz = HAL_RCC_GetSysClockFreq();
    info.hclk_hz = HAL_RCC_GetHCLKFreq();
    info.apb1_hz = HAL_RCC_GetPCLK1Freq();
    info.apb2_hz = HAL_RCC_GetPCLK2Freq();
    info.pllm = pllcfgr & 0x3FU;
    info.plln = (pllcfgr >> 6U) & 0x1FFU;
    /* STM32F4 PLLP encoding: 00=/2, 01=/4, 10=/6, 11=/8.  The previous
     * (field+1) decode reported 1 for /2 and made the 168MHz check fail on
     * real hardware, leaving the board in Error_Handler. */
    info.pllp = 2U * (((pllcfgr >> 16U) & 0x3U) + 1U);
    info.pllq = (pllcfgr >> 24U) & 0xFU;
    info.hse_ready = (RCC->CR & RCC_CR_HSERDY) != 0U;
    info.verified = info.hse_ready &&
                    info.sysclk_hz == SYS_HZ &&
                    info.hclk_hz == SYS_HZ &&
                    info.apb1_hz == APB1_HZ &&
                    info.apb2_hz == APB2_HZ &&
                    info.pllm == 25U && info.plln == 336U &&
                    info.pllp == 2U && info.pllq == 7U &&
                    (RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL;
    if (out) *out = info;
    return info.verified;
}

const BoardClockInfo *board_clock_info(void)
{
    return &info;
}
