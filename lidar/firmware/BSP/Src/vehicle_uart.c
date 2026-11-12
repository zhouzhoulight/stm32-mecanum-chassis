/* Independent 230400 8N1 interboard link. IRQ rings, no DMA conflicts. */
#include "vehicle_uart.h"
#include "board_config.h"
static uint8_t rx[1024],tx[4096];
static volatile unsigned rh,rt,th,tt;
static volatile bool fault;
bool vehicle_uart_init(void){
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
 if(!p||!n||n>((tt-th-1)&4095))return false;
 for(unsigned i=0;i<n;i++){tx[th]=p[i];th=(th+1)&4095;}
 __DMB();USART3->CR1|=USART_CR1_TXEIE;return true;
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
  if(tt!=th){USART3->DR=tx[tt];tt=(tt+1)&4095;}else USART3->CR1&=~USART_CR1_TXEIE;
 }
}
