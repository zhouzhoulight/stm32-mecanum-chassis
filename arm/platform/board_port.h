#ifndef F407_R8B_BOARD_PORT_H
#define F407_R8B_BOARD_PORT_H
#include "stm32f4xx_hal.h"
#define BOARD_SYSCLK_HZ 168000000U
#define BOARD_HSE_HZ 25000000U
/* Logical names retained from R8B, independently of physical UART numbers. */
#define BOARD_MOTOR_UART UART4
#define BOARD_DEBUG_UART USART1
extern UART_HandleTypeDef huart1,huart4;
extern TIM_HandleTypeDef htim3;
void Board_Clock_Config(void);
uint32_t Board_TimerHz(void);
void Board_Arm_Init(void);
void Error_Handler(void);
#endif
