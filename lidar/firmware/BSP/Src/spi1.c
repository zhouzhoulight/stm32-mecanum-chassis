/* SPI1 + PC13 software chip select for the onboard W25Q128 (read-only diag). */
#include "spi1.h"

#include <string.h>

#include "board_config.h"
#include "timebase.h"

SPI_HandleTypeDef hspi1;
static Spi1Stats stats;

static void cs_low(void) { HAL_GPIO_WritePin(W25Q_CS_PORT, W25Q_CS_PIN, GPIO_PIN_RESET); }
static void cs_high(void) { HAL_GPIO_WritePin(W25Q_CS_PORT, W25Q_CS_PIN, GPIO_PIN_SET); }

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1) {
        GPIO_InitTypeDef gpio = {0};
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_SPI1_CLK_ENABLE();
        gpio.Pin = W25Q_SCK_PIN | W25Q_MISO_PIN | W25Q_MOSI_PIN;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio.Alternate = W25Q_SPI_AF;
        HAL_GPIO_Init(GPIOA, &gpio);
    }
}

bool spi1_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    memset(&stats, 0, sizeof stats);

    /* PC13: W25Q chip select, idle HIGH before SPI is enabled. Never an LED. */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    HAL_GPIO_WritePin(W25Q_CS_PORT, W25Q_CS_PIN, GPIO_PIN_SET);
    gpio.Pin = W25Q_CS_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW; /* PC13 has board-level restrictions */
    HAL_GPIO_Init(W25Q_CS_PORT, &gpio);
    cs_high();
    stats.cs_high_at_boot = (HAL_GPIO_ReadPin(W25Q_CS_PORT, W25Q_CS_PIN) == GPIO_PIN_SET);

    hspi1.Instance = W25Q_SPI;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        stats.errors++;
        return false;
    }
    stats.init_ok = true;
    return true;
}

bool w25q_read_jedec(uint8_t id[3])
{
    uint8_t cmd = 0x9FU;
    if (!stats.init_ok || !id) return false;
    cs_low();
    HAL_StatusTypeDef s1 = HAL_SPI_Transmit(&hspi1, &cmd, 1, 10);
    HAL_StatusTypeDef s2 = (s1 == HAL_OK) ? HAL_SPI_Receive(&hspi1, id, 3, 10) : s1;
    cs_high();
    stats.jedec_reads++;
    if (s1 != HAL_OK || s2 != HAL_OK) {
        stats.errors++;
        return false;
    }
    return true;
}

const Spi1Stats *spi1_stats(void)
{
    return &stats;
}
