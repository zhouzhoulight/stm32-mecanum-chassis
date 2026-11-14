/* Independent 230400 8N1 interboard link. IRQ rings, no DMA conflicts. */
#include "vehicle_uart.h"
#include "board_config.h"
#include "vehicle_tx_queue.h"
static uint8_t rx[1024];
static volatile unsigned rh,rt;
static VehicleTxQueue tx;
static volatile bool fault;
bool vehicle_uart_init(void){
 vehicle_tx_queue_init(&tx);rh=rt=0;fault=false;
 __HAL_RCC_GPIOB_CLK_ENABLE();__HAL_RCC_USART3_CLK_ENABLE();
 GPIO_InitTypeDef io={0};io.Pin=GPIO_PIN_10|GPIO_PIN_11;io.Mode=GPIO_MODE_AF_PP;
 io.Pull=GPIO_PULLUP;io.Speed=GPIO_SPEED_FREQ_HIGH;io.Alternate=GPIO_AF7_USART3;HAL_GPIO_Init(GPIOB,&io);
 UART_HandleTypeDef u={0};u.Instance=USART3;u.Init.BaudRate=230400;
 u.Init.WordLength=UART_WORDLENGTH_8B;u.Init.StopBits=UART_STOPBITS_1;
 u.Init.Mode=UART_MODE_TX_RX;u.Init.OverSampling=UART_OVERSAMPLING_16;
 if(HAL_UART_Init(&u)!=HAL_OK)return false;
 HAL_NVIC_SetPriority(USART3_IRQn,3,0);HAL_NVIC_EnableIRQ(USART3_IRQn);
 USART3->CR1|=USART_CR1_RXNEIE;return true;
}
bool vehicle_uart_send(const uint8_t *p,uint32_t n){
 uint32_t irq=__get_PRIMASK();__disable_irq();
 bool ok=vehicle_tx_queue_push(&tx,p,n,false);
 __DMB();if(ok)USART3->CR1|=USART_CR1_TXEIE;if(!irq)__enable_irq();return ok;
}
bool vehicle_uart_send_priority(const uint8_t *p,uint32_t n){
 uint32_t irq=__get_PRIMASK();__disable_irq();
 bool ok=vehicle_tx_queue_push(&tx,p,n,true);
 __DMB();if(ok)USART3->CR1|=USART_CR1_TXEIE;if(!irq)__enable_irq();return ok;
}
uint32_t vehicle_uart_read(uint8_t *p,uint32_t n){unsigned k=0;while(k<n&&rt!=rh){p[k++]=rx[rt];rt=(rt+1)&1023;}return k;}
bool vehicle_uart_fault(void){uint32_t irq=__get_PRIMASK();__disable_irq();bool v=fault;fault=false;if(!irq)__enable_irq();return v;}
void vehicle_uart_irq(void){
 uint32_t sr=USART3->SR;
 if(sr&(USART_SR_RXNE|USART_SR_ORE|USART_SR_FE|USART_SR_NE|USART_SR_PE)){
  uint8_t b=(uint8_t)USART3->DR;unsigned next=(rh+1)&1023;
  if((sr&(USART_SR_ORE|USART_SR_FE|USART_SR_NE|USART_SR_PE))||next==rt)fault=true;
  else {rx[rh]=b;__DMB();rh=next;}
 }
 if((USART3->CR1&USART_CR1_TXEIE)&&(sr&USART_SR_TXE)){
  uint8_t byte;if(vehicle_tx_queue_pop(&tx,&byte))USART3->DR=byte;else USART3->CR1&=~USART_CR1_TXEIE;
 }
}
