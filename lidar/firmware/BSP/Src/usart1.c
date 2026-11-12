/* USART1 (PC link, 460800 8N1): DMA TX with bounded byte ring, RX IRQ ring.
 * Binary protocol only; printf text must never be mixed into this port. */
#include "usart1.h"

#include <string.h>

#include "app_config.h"
#include "board_config.h"
#include "timebase.h"

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

static uint8_t tx_ring[UART1_TX_RING_BYTES];
static uint16_t tx_head, tx_tail;
static uint8_t tx_dma_buf[256];
static volatile bool tx_busy;
static uint8_t rx_ring[UART1_RX_RING_BYTES];
static uint16_t rx_head, rx_tail;
static Usart1Stats stats;

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef gpio = {0};

    if (huart->Instance == USART1) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_USART1_CLK_ENABLE();
        __HAL_RCC_DMA2_CLK_ENABLE();
        gpio.Pin = HOST_TX_PIN | HOST_RX_PIN;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_PULLUP;
        gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = HOST_TX_AF;
        HAL_GPIO_Init(HOST_TX_PORT, &gpio);

        hdma_usart1_tx.Instance = HOST_TX_DMA_STREAM;
        hdma_usart1_tx.Init.Channel = HOST_TX_DMA_CHANNEL;
        hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart1_tx.Init.Mode = DMA_NORMAL;
        hdma_usart1_tx.Init.Priority = DMA_PRIORITY_HIGH;
        hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK) Error_Handler();
        __HAL_LINKDMA(huart, hdmatx, hdma_usart1_tx);

        HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
        HAL_NVIC_SetPriority(HOST_TX_DMA_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ(HOST_TX_DMA_IRQn);
    } else if (huart->Instance == USART2) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_USART2_CLK_ENABLE();
        __HAL_RCC_DMA1_CLK_ENABLE();
        gpio.Pin = LD19_RX_PIN;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_PULLUP;
        gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = LD19_RX_AF;
        HAL_GPIO_Init(LD19_RX_PORT, &gpio);

        hdma_usart2_rx.Instance = LD19_RX_DMA_STREAM;
        hdma_usart2_rx.Init.Channel = LD19_RX_DMA_CHANNEL;
        hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;
        hdma_usart2_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
        hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK) Error_Handler();
        __HAL_LINKDMA(huart, hdmarx, hdma_usart2_rx);

        HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(USART2_IRQn);
        HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 2, 0);
        HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
    }
}

bool usart1_init(void)
{
    memset(&stats, 0, sizeof stats);
    tx_head = tx_tail = 0;
    rx_head = rx_tail = 0;
    tx_busy = false;
    huart1.Instance = HOST_UART;
    huart1.Init.BaudRate = HOST_BAUD;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) return false;
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
    return true;
}

void usart1_service(void)
{
    if (tx_busy) return;
    uint16_t used = (uint16_t)((tx_head + UART1_TX_RING_BYTES - tx_tail) % UART1_TX_RING_BYTES);
    if (used == 0U) return;
    uint16_t n = used;
    if (n > sizeof tx_dma_buf) n = sizeof tx_dma_buf;
    uint16_t first = (uint16_t)(UART1_TX_RING_BYTES - tx_tail);
    if (first > n) first = n;
    memcpy(tx_dma_buf, &tx_ring[tx_tail], first);
    if (n > first) memcpy(&tx_dma_buf[first], &tx_ring[0], (size_t)(n - first));
    tx_tail = (uint16_t)((tx_tail + n) % UART1_TX_RING_BYTES);
    tx_busy = true;
    if (HAL_UART_Transmit_DMA(&huart1, tx_dma_buf, n) != HAL_OK) {
        tx_busy = false;
        stats.tx_dropped_bytes += n;
        return;
    }
    stats.tx_frames++;
    stats.tx_bytes += n;
}

bool usart1_send(const uint8_t *data, uint32_t len)
{
    if (!data || len == 0U) return false;
    uint16_t used = (uint16_t)((tx_head + UART1_TX_RING_BYTES - tx_tail) % UART1_TX_RING_BYTES);
    if (len > (uint32_t)(UART1_TX_RING_BYTES - 1U - used)) {
        stats.tx_dropped_bytes += len;
        return false;
    }
    for (uint32_t i = 0; i < len; i++) {
        tx_ring[tx_head] = data[i];
        tx_head = (uint16_t)((tx_head + 1U) % UART1_TX_RING_BYTES);
    }
    usart1_service();
    return true;
}

uint32_t usart1_read(uint8_t *dst, uint32_t max)
{
    uint32_t n = 0;
    while (n < max && rx_tail != rx_head) {
        dst[n++] = rx_ring[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1U) % UART1_RX_RING_BYTES);
    }
    return n;
}

const Usart1Stats *usart1_stats(void)
{
    return &stats;
}

void usart1_uart_irq(void)
{
    uint32_t sr = USART1->SR;
    if (sr & USART_SR_RXNE) {
        uint8_t b = (uint8_t)USART1->DR;
        stats.rx_bytes++;
        uint16_t next = (uint16_t)((rx_head + 1U) % UART1_RX_RING_BYTES);
        if (next == rx_tail) stats.rx_overruns++;
        else { rx_ring[rx_head] = b; rx_head = next; }
    }
    if (sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) {
        (void)USART1->DR;
        stats.uart_errors++;
    }
    /* HAL DMA completion enables UART TCIE; the final TC interrupt must
     * reach HAL so gState/tx_busy are released and the next frame can send. */
    if ((USART1->SR & USART_SR_TC) && (USART1->CR1 & USART_CR1_TCIE)) {
        HAL_UART_IRQHandler(&huart1);
    }
}

void usart1_dma_tx_irq(void)
{
    HAL_DMA_IRQHandler(&hdma_usart1_tx);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        tx_busy = false;
        usart1_service();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) stats.uart_errors++;
    if (huart->Instance == USART2) { /* counted inside usart2.c */ }
}
