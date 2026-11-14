#include "vehicle_tx_queue.h"
#include <string.h>
void vehicle_tx_queue_init(VehicleTxQueue *q){memset(q,0,sizeof *q);}
bool vehicle_tx_queue_push(VehicleTxQueue *q,const uint8_t *p,uint32_t n,bool priority){
 if(!p||n<6||n>256||p[0]!=0xAA||p[1]!=0x55||n!=(uint32_t)p[4]+6)return false;
 uint8_t checksum=0;for(unsigned i=0;i<n;i++)checksum^=p[i];if(checksum)return false;
 unsigned mask=priority?511U:4095U,head=priority?q->ch:q->dh,tail=priority?q->ct:q->dt;
 if(n>((tail-head-1)&mask))return false;
 uint8_t *ring=priority?q->control:q->data;
 for(unsigned i=0;i<n;i++){ring[head]=p[i];head=(head+1)&mask;}
 /* Publish only after a complete envelope has been copied. The hardware
  * wrapper excludes IRQs and supplies the Cortex-M memory barrier. */
 if(priority)q->ch=(uint16_t)head;else q->dh=(uint16_t)head;
 return true;
}
bool vehicle_tx_queue_pop(VehicleTxQueue *q,uint8_t *byte){
 if(!q->remaining){
  if(q->ct!=q->ch)q->priority_active=true;
  else if(q->dt!=q->dh)q->priority_active=false;
  else return false;
  unsigned tail=q->priority_active?q->ct:q->dt,mask=q->priority_active?511U:4095U;
  const uint8_t *ring=q->priority_active?q->control:q->data;
  q->remaining=(uint16_t)(ring[(tail+4)&mask]+6);
 }
 if(q->priority_active){*byte=q->control[q->ct];q->ct=(q->ct+1)&511U;}
 else {*byte=q->data[q->dt];q->dt=(q->dt+1)&4095U;}
 q->remaining--;return true;
}
