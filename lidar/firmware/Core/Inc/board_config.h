/* Fixed board resources for the standalone LD19 F407VE bench board.
 * PC13 is W25Q CS ONLY (never an LED); PB6/PB7 are I2C1 ONLY; no vehicle
 * motor/servo/IMU pins are initialised by this firmware. */
#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

#define LD19_UART              USART2
#define LD19_UART_IRQn         USART2_IRQn
#define LD19_RX_PORT           GPIOA
#define LD19_RX_PIN            GPIO_PIN_3
#define LD19_RX_AF             GPIO_AF7_USART2
#define LD19_RX_DMA_STREAM     DMA1_Stream5
#define LD19_RX_DMA_CHANNEL    DMA_CHANNEL_4
#define LD19_RX_DMA_IRQn       DMA1_Stream5_IRQn

#define HOST_UART              USART1
#define HOST_UART_IRQn         USART1_IRQn
#define HOST_TX_PORT           GPIOA
#define HOST_TX_PIN            GPIO_PIN_9
#define HOST_RX_PIN            GPIO_PIN_10
#define HOST_TX_AF             GPIO_AF7_USART1
#define HOST_TX_DMA_STREAM     DMA2_Stream7
#define HOST_TX_DMA_CHANNEL    DMA_CHANNEL_4
#define HOST_TX_DMA_IRQn       DMA2_Stream7_IRQn

#define W25Q_CS_PORT           GPIOC
#define W25Q_CS_PIN            GPIO_PIN_13
#define W25Q_SPI               SPI1
#define W25Q_SCK_PIN           GPIO_PIN_5
#define W25Q_MISO_PIN          GPIO_PIN_6
#define W25Q_MOSI_PIN          GPIO_PIN_7
#define W25Q_SPI_AF            GPIO_AF5_SPI1

#define EEPROM_I2C             I2C1
#define EEPROM_SCL_PIN         GPIO_PIN_6
#define EEPROM_SDA_PIN         GPIO_PIN_7
#define EEPROM_I2C_AF          GPIO_AF4_I2C1
#define EEPROM_ADDR7           0x50U
#define EEPROM_ADDR8           (EEPROM_ADDR7 << 1)

#define TIMEBASE_TIM           TIM2
#define TIMEBASE_PSC           83U          /* 84 MHz APB1 timer / 84 = 1 MHz */
#define TIMEBASE_US_PER_TICK   1U

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern TIM_HandleTypeDef htim2;
extern SPI_HandleTypeDef hspi1;
extern I2C_HandleTypeDef hi2c1;

void Error_Handler(void);
#endif
