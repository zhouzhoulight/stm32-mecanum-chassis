#include "stm32f4xx_hal.h"
#include "nav/nav_arm.h"
void nav_arm_hw_init(void)
{
 extern TIM_HandleTypeDef htim3;
 __HAL_RCC_GPIOC_CLK_ENABLE();__HAL_RCC_TIM3_CLK_ENABLE();
 htim3.Instance=TIM3;htim3.Init.Prescaler=83;htim3.Init.Period=19999;
 htim3.Init.CounterMode=TIM_COUNTERMODE_UP;htim3.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
 htim3.Init.AutoReloadPreload=TIM_AUTORELOAD_PRELOAD_DISABLE;
 if(HAL_TIM_PWM_Init(&htim3)!=HAL_OK)while(1){}
 TIM_OC_InitTypeDef oc={0};oc.OCMode=TIM_OCMODE_PWM1;oc.OCPolarity=TIM_OCPOLARITY_HIGH;oc.OCFastMode=TIM_OCFAST_DISABLE;
 const uint32_t channels[4]={TIM_CHANNEL_1,TIM_CHANNEL_2,TIM_CHANNEL_3,TIM_CHANNEL_4};
 for(unsigned k=0;k<4;k++)if(HAL_TIM_PWM_ConfigChannel(&htim3,&oc,channels[k])!=HAL_OK)while(1){}
 TIM3->CCER=0;TIM3->CR1&=~TIM_CR1_CEN;TIM3->EGR=TIM_EGR_UG;TIM3->SR=0;
 GPIO_InitTypeDef io={0};io.Pin=GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;io.Mode=GPIO_MODE_AF_PP;
 io.Pull=GPIO_NOPULL;io.Speed=GPIO_SPEED_FREQ_HIGH;io.Alternate=GPIO_AF2_TIM3;HAL_GPIO_Init(GPIOC,&io);
}
