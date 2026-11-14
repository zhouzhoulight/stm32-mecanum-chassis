#pragma once
#include <stdint.h>
#include <stdbool.h>
bool vehicle_uart_init(void);
bool vehicle_uart_send(const uint8_t *,uint32_t);
bool vehicle_uart_send_priority(const uint8_t *,uint32_t);
uint32_t vehicle_uart_read(uint8_t *,uint32_t);
void vehicle_uart_irq(void);
bool vehicle_uart_fault(void);
