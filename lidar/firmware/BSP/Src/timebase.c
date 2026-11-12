/* 1 MHz 32-bit microsecond time base (TIM2) + DWT cycle counter. */
#include "timebase.h"

#include "board_config.h"

TIM_HandleTypeDef htim2;

bool timebase_init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = TIMEBASE_PSC;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xFFFFFFFFU;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) return false;
    return HAL_TIM_Base_Start(&htim2) == HAL_OK;
}

uint32_t timebase_us(void)
{
    return (uint32_t)TIM2->CNT;
}

void timebase_dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t timebase_cycles(void)
{
    return (uint32_t)DWT->CYCCNT;
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) __HAL_RCC_TIM2_CLK_ENABLE();
}
