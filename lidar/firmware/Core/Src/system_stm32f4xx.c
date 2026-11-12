/* Minimal ST system file for the standalone bench board (HSE25 -> PLL 168 MHz
 * is configured by BSP board_clock_init; SystemInit only enables FPU access and
 * keeps the vector table at its default flash location). */
#include "stm32f4xx.h"

uint32_t SystemCoreClock = 16000000U;

/* Tables referenced by the ST HAL RCC driver (normally provided by the Cube
 * system_stm32f4xx.c template). */
const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8] = {0, 0, 0, 0, 1, 2, 3, 4};

void SystemInit(void)
{
#if (__FPU_PRESENT == 1U) && (__FPU_USED == 1U)
    SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2));
#endif
    SCB->VTOR = 0x08000000U;
}

void SystemCoreClockUpdate(void)
{
    uint32_t pllcfgr = RCC->PLLCFGR;
    uint32_t pllm = pllcfgr & 0x3FU;
    uint32_t plln = (pllcfgr >> 6U) & 0x1FFU;
    uint32_t pllp = 2U * (((pllcfgr >> 16U) & 0x3U) + 1U);
    uint32_t pllsource = (pllcfgr >> 22U) & 0x1U;
    uint32_t sysclk = 0;

    switch (RCC->CFGR & RCC_CFGR_SWS) {
    case RCC_CFGR_SWS_HSI: sysclk = HSI_VALUE; break;
    case RCC_CFGR_SWS_HSE: sysclk = HSE_VALUE; break;
    case RCC_CFGR_SWS_PLL:
        /* PLLSRC=1 selects HSE; PLLP encodes /2,/4,/6,/8. */
        if (pllm != 0U) {
            uint32_t source_hz = pllsource ? HSE_VALUE : HSI_VALUE;
            sysclk = (uint32_t)(((uint64_t)source_hz * plln) / pllm / pllp);
        }
        break;
    default: sysclk = HSI_VALUE; break;
    }
    uint32_t hpre = (RCC->CFGR & RCC_CFGR_HPRE) >> 4U;
    SystemCoreClock = sysclk >> AHBPrescTable[hpre];
}
