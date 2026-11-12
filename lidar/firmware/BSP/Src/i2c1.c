/* I2C1 (PB6/PB7) read-only probe of the onboard 24C02 EEPROM. */
#include "i2c1.h"

#include <string.h>

#include "board_config.h"

I2C_HandleTypeDef hi2c1;
static I2c1Stats stats;

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) {
        GPIO_InitTypeDef gpio = {0};
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_I2C1_CLK_ENABLE();
        gpio.Pin = EEPROM_SCL_PIN | EEPROM_SDA_PIN;
        gpio.Mode = GPIO_MODE_AF_OD;
        gpio.Pull = GPIO_PULLUP; /* user already fitted 3.3V pull-ups */
        gpio.Speed = GPIO_SPEED_FREQ_LOW;
        gpio.Alternate = EEPROM_I2C_AF;
        HAL_GPIO_Init(GPIOB, &gpio);
    }
}

bool i2c1_init(void)
{
    memset(&stats, 0, sizeof stats);
    hi2c1.Instance = EEPROM_I2C;
    hi2c1.Init.ClockSpeed = 100000U;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        stats.errors++;
        return false;
    }
    stats.init_ok = true;
    return true;
}

bool eeprom_read_byte(uint8_t mem_addr, uint8_t *value)
{
    if (!stats.init_ok || !value) return false;
    if (HAL_I2C_Master_Transmit(&hi2c1, EEPROM_ADDR8, &mem_addr, 1, 20) != HAL_OK) {
        stats.errors++;
        return false;
    }
    if (HAL_I2C_Master_Receive(&hi2c1, EEPROM_ADDR8, value, 1, 20) != HAL_OK) {
        stats.errors++;
        return false;
    }
    stats.reads++;
    return true;
}

const I2c1Stats *i2c1_stats(void)
{
    return &stats;
}
