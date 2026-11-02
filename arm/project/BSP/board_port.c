#include "board_port.h"
UART_HandleTypeDef huart1,huart4;
TIM_HandleTypeDef htim3;

void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
}
void Board_Clock_Config(void)
{
    RCC_OscInitTypeDef osc={0};
    RCC_ClkInitTypeDef clk={0};
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    osc.OscillatorType=RCC_OSCILLATORTYPE_HSE;
    osc.HSEState=RCC_HSE_ON;
    osc.PLL.PLLState=RCC_PLL_ON;
    osc.PLL.PLLSource=RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM=25;osc.PLL.PLLN=336;osc.PLL.PLLP=RCC_PLLP_DIV2;osc.PLL.PLLQ=7;
    if(HAL_RCC_OscConfig(&osc)!=HAL_OK)Error_Handler();
    clk.ClockType=RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource=RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider=RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider=RCC_HCLK_DIV4;clk.APB2CLKDivider=RCC_HCLK_DIV2;
    if(HAL_RCC_ClockConfig(&clk,FLASH_LATENCY_5)!=HAL_OK)Error_Handler();
    SystemCoreClockUpdate();
    if(HAL_RCC_GetSysClockFreq()!=BOARD_SYSCLK_HZ)Error_Handler();
}
uint32_t Board_TimerHz(void)
{
    uint32_t hz=HAL_RCC_GetPCLK1Freq();
    return (RCC->CFGR&RCC_CFGR_PPRE1)?hz*2U:hz;
}
static void uart_init(UART_HandleTypeDef *h,USART_TypeDef *instance)
{
    h->Instance=instance;
    h->Init.BaudRate=115200;h->Init.WordLength=UART_WORDLENGTH_8B;
    h->Init.StopBits=UART_STOPBITS_1;h->Init.Parity=UART_PARITY_NONE;
    h->Init.Mode=UART_MODE_TX_RX;h->Init.HwFlowCtl=UART_HWCONTROL_NONE;
    h->Init.OverSampling=UART_OVERSAMPLING_16;
    if(HAL_UART_Init(h)!=HAL_OK)Error_Handler();
}
void HAL_UART_MspInit(UART_HandleTypeDef *h)
{
    GPIO_InitTypeDef pin={0};
    IRQn_Type irq;
    GPIO_TypeDef *port;
    if(h->Instance==BOARD_DEBUG_UART){
        __HAL_RCC_GPIOA_CLK_ENABLE();__HAL_RCC_USART1_CLK_ENABLE();
        port=GPIOA;pin.Pin=GPIO_PIN_9|GPIO_PIN_10;pin.Alternate=GPIO_AF7_USART1;irq=USART1_IRQn;
        HAL_NVIC_SetPriority(irq,2,0);
    }else if(h->Instance==BOARD_MOTOR_UART){
        __HAL_RCC_GPIOC_CLK_ENABLE();__HAL_RCC_UART4_CLK_ENABLE();
        port=GPIOC;pin.Pin=GPIO_PIN_10|GPIO_PIN_11;pin.Alternate=GPIO_AF8_UART4;irq=UART4_IRQn;
        HAL_NVIC_SetPriority(irq,1,0);
    }else{Error_Handler();return;}
    pin.Mode=GPIO_MODE_AF_PP;pin.Pull=GPIO_PULLUP;pin.Speed=GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(port,&pin);HAL_NVIC_EnableIRQ(irq);
}
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *h)
{if(h->Instance==TIM3)__HAL_RCC_TIM3_CLK_ENABLE();}
void Board_Arm_Init(void)
{
    GPIO_InitTypeDef pin={0};TIM_OC_InitTypeDef oc={0};uint32_t hz;
    Board_Clock_Config();uart_init(&huart1,BOARD_MOTOR_UART);uart_init(&huart4,BOARD_DEBUG_UART);
    hz=Board_TimerHz();if(hz%1000000U)Error_Handler();
    htim3.Instance=TIM3;htim3.Init.Prescaler=hz/1000000U-1U;
    htim3.Init.CounterMode=TIM_COUNTERMODE_UP;htim3.Init.Period=19999;
    htim3.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;htim3.Init.AutoReloadPreload=TIM_AUTORELOAD_PRELOAD_DISABLE;
    if(HAL_TIM_PWM_Init(&htim3)!=HAL_OK)Error_Handler();
    oc.OCMode=TIM_OCMODE_PWM1;oc.Pulse=0;oc.OCPolarity=TIM_OCPOLARITY_HIGH;oc.OCFastMode=TIM_OCFAST_DISABLE;
    if(HAL_TIM_PWM_ConfigChannel(&htim3,&oc,TIM_CHANNEL_1)!=HAL_OK)Error_Handler();
    if(HAL_TIM_PWM_ConfigChannel(&htim3,&oc,TIM_CHANNEL_2)!=HAL_OK)Error_Handler();
    if(HAL_TIM_PWM_ConfigChannel(&htim3,&oc,TIM_CHANNEL_3)!=HAL_OK)Error_Handler();
    __HAL_TIM_DISABLE(&htim3);TIM3->CCER=0;
    TIM3->CCR1=TIM3->CCR2=TIM3->CCR3=TIM3->CCR4=0;TIM3->EGR=TIM_EGR_UG;TIM3->SR=0;
    __HAL_RCC_GPIOC_CLK_ENABLE();
    pin.Pin=GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;pin.Mode=GPIO_MODE_AF_PP;
    pin.Pull=GPIO_NOPULL;pin.Speed=GPIO_SPEED_FREQ_HIGH;pin.Alternate=GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOC,&pin);
    pin.Pin=GPIO_PIN_9;pin.Mode=GPIO_MODE_ANALOG;pin.Alternate=0;HAL_GPIO_Init(GPIOC,&pin);
    (void)huart1.Instance->SR;(void)huart1.Instance->DR;(void)huart4.Instance->SR;(void)huart4.Instance->DR;
    __HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);__HAL_UART_ENABLE_IT(&huart4,UART_IT_RXNE);
}
void Error_Handler(void)
{
    if(RCC->APB1ENR&RCC_APB1ENR_TIM3EN)TIM3->CCER=0;
    __disable_irq();while(1){}
}
