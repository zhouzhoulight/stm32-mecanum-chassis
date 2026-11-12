/* USART2 (LD19, 230400 8N1): circular DMA RX ring + IDLE/HT/TC progress. */
#include "usart2.h"

#include <string.h>

#include "app_config.h"
#include "board_config.h"
#include "timebase.h"

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* Ordinary SRAM, never CCM (DMA cannot address CCM on STM32F4). */
static uint8_t rx_ring[UART2_RX_RING_BYTES] __attribute__((aligned(32)));
static uint32_t consumed_bytes;
static volatile uint32_t dma_laps;
static volatile uint32_t event_us;
static Usart2Stats stats;
static uint32_t error_seen, overrun_seen;

bool usart2_init(void)
{
    memset(&stats, 0, sizeof stats);
    consumed_bytes = 0;
    dma_laps = 0;
    event_us = 0;
    error_seen = 0;
    overrun_seen = 0;
    huart2.Instance = LD19_UART;
    huart2.Init.BaudRate = LIDAR_BAUD;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) return false;
    if (HAL_UART_Receive_DMA(&huart2, rx_ring, (uint16_t)sizeof rx_ring) != HAL_OK) return false;
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
    return true;
}

static uint32_t dma_produced(void)
{
    uint32_t key=__get_PRIMASK();
    __disable_irq();
    uint32_t laps=dma_laps;
    uint32_t remaining = __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
    /* A wrap pending in NVIC is not yet included in callback lap count. */
    if (__HAL_DMA_GET_FLAG(&hdma_usart2_rx,__HAL_DMA_GET_TC_FLAG_INDEX(&hdma_usart2_rx))) {
        laps++;
        remaining=__HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
    }
    uint32_t head = UART2_RX_RING_BYTES - remaining;
    if (head >= UART2_RX_RING_BYTES) head = 0U;
    __set_PRIMASK(key);
    return laps*UART2_RX_RING_BYTES+head;
}

uint32_t usart2_read(uint8_t *dst, uint32_t max, uint32_t *rx_end_us,
                     bool *discontinuous, bool *uart_error)
{
    uint32_t produced=dma_produced();
    uint32_t captured_us=timebase_us();
    uint32_t available = produced-consumed_bytes;
    if (discontinuous) *discontinuous = false;
    if (uart_error) *uart_error = false;

    if (available > UART2_RX_RING_BYTES) {
        /* Reader fell behind: bytes were overwritten by the DMA. Never hide it. */
        stats.overrun_bytes += available-UART2_RX_RING_BYTES;
        consumed_bytes=produced-UART2_RX_RING_BYTES;
        if (discontinuous) *discontinuous = true;
        available = UART2_RX_RING_BYTES;
    }
    uint32_t take=available>max?max:available;

    uint32_t n = 0;
    while (n < take) {
        dst[n++] = rx_ring[consumed_bytes%UART2_RX_RING_BYTES];
        consumed_bytes++;
    }
    stats.rx_bytes += n;
    if (rx_end_us) *rx_end_us = captured_us-(uint32_t)(((uint64_t)(available-take)*10000000U)/LIDAR_BAUD);
    if (stats.uart_errors != error_seen) {
        error_seen = stats.uart_errors;
        if (uart_error) *uart_error = true;
        if (discontinuous) *discontinuous = true;
    }
    if (stats.overrun_bytes != overrun_seen) {
        overrun_seen = stats.overrun_bytes;
        if (discontinuous) *discontinuous = true;
    }
    return n;
}

const Usart2Stats *usart2_stats(void)
{
    return &stats;
}

void usart2_uart_irq(void)
{
    uint32_t sr = USART2->SR;
    if (sr & USART_SR_IDLE) {
        (void)USART2->DR;
        stats.idle_events++;
        stats.last_event_us = timebase_us();
        event_us = stats.last_event_us;
    }
    if (sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) {
        (void)USART2->DR;
        stats.uart_errors++;
        stats.last_event_us = timebase_us();
    }
}

void usart2_dma_irq(void)
{
    stats.last_event_us = timebase_us();
    HAL_DMA_IRQHandler(&hdma_usart2_rx);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        dma_laps++;
        stats.dma_tc_events++;
        stats.last_event_us = timebase_us();
    }
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        stats.dma_ht_events++;
        stats.last_event_us = timebase_us();
    }
}
