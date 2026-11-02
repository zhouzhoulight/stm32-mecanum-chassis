/* One outstanding Emm transaction, asynchronous TX, >=5ms interpacket gap.
 * Latest speed generation coalesces; a four-wheel snapshot is never mixed.
 * Emergency packets do NOT wait for a nonresponsive motor's ACK. */
#include "nav/nav_bus.h"
#include "nav/nav_platform.h"
#include "nav/nav_odom.h"
#include "nav/nav_time.h"
#include <string.h>
#include <stdlib.h>
static NavBus b;
static const uint8_t ids[4]={4,1,3,2};
static uint8_t rx[40],used,expected,op,wheel_index;
static uint32_t sent_at,stop_at,next_poll,last_flags,batch_generation;
static uint32_t last_rx_at;
static unsigned stop_index,batch_index,enable_index,poll_index,flag_index;
static int latest[4],snapshot[4];
static bool enabling,batch,waiting;
static bool feedback_turn;
static uint8_t stop_sent,disable_sent;
static unsigned index_of(uint8_t id){unsigned k=0;while(k<4&&ids[k]!=id)k++;return k;}
static bool send(const uint8_t *p,unsigned n,uint32_t now,bool wait,unsigned index)
{
 if(b.tx_fault || nav_hw_tx_busy() || (int32_t)(now-nav_hw_tx_end())<5)return false;
 if(!nav_hw_tx(p,n))return false;
 b.tx_packets++;sent_at=now;waiting=wait;b.transaction=wait;op=p[1];wheel_index=(uint8_t)index;
 if(used)b.rx_resyncs++;
 used=0;
 return true;
}
void nav_bus_init(uint32_t now){memset(&b,0,sizeof b);used=0;waiting=batch=enabling=false;feedback_turn=true;next_poll=last_flags=now;poll_index=flag_index=0;nav_bus_emergency(now);}
const NavBus *nav_bus_data(void){return &b;}
void nav_bus_emergency(uint32_t now)
{
 if(b.stopping)return; /* Repeated ! must finish the first ALL-wheel sequence. */
 b.stopping=true;stop_index=0;stop_at=now;b.stop_ack=b.disable_ack=0;
 stop_sent=disable_sent=0;
 waiting=batch=enabling=false;b.transaction=false;b.velocity_done=b.velocity_requested;
 memset(latest,0,sizeof latest);used=0;
}
bool nav_bus_enable(void)
{
 if(b.stopping||b.tx_fault||enabling||batch||waiting||b.velocity_done!=b.velocity_requested)return false;
 for(unsigned k=0;k<4;k++)if(latest[k]!=0)return false;
 enabling=true;enable_index=0;return true;
}
uint32_t nav_bus_velocity(const int wheel[4])
{
 if(b.stopping||b.tx_fault)return 0;
 for(unsigned k=0;k<4;k++){
  if(wheel[k]<-NAV_MAX_WHEEL_RPM||wheel[k]>NAV_MAX_WHEEL_RPM)return 0;
 }
 for(unsigned k=0;k<4;k++)latest[k]=wheel[k];
 return ++b.velocity_requested;
}
void nav_bus_uart_error(void){b.errors++;used=0;nav_log("[BUS_UART_ERROR] received UART hardware error\r\n");}
static void frame(uint32_t now)
{
 unsigned k=index_of(rx[0]);if(k==4)return;
 b.rx_frames++;b.last_reply=now;
 if(rx[1]!=0x36&&rx[1]!=0x3A&&rx[2]!=2)nav_logf("[BUS_NACK] id=%u cmd=%02X status=%02X matched_pending=%u\r\n",rx[0],rx[1],rx[2],waiting&&wheel_index==k&&op==rx[1]);
 if(rx[1]==0xFE && rx[2]==2 && (stop_sent&(1U<<k)))b.stop_ack|=1U<<k;
 if(rx[1]==0xF3 && rx[2]==2 && b.stopping && (disable_sent&(1U<<k)))b.disable_ack|=1U<<k;
 bool matched=waiting&&wheel_index==k&&op==rx[1]&&(int32_t)(now-sent_at)>=0&&nav_age(now,sent_at)<150;
 if(!matched)return; /* Late/unsolicited replies do not refresh a query. */
 if(rx[1]==0x36){
  uint32_t magnitude=((uint32_t)rx[3]<<24)|((uint32_t)rx[4]<<16)|((uint32_t)rx[5]<<8)|rx[6];
  if(rx[2]>1 || magnitude>INT32_MAX){b.errors++;}
  else {int32_t value=rx[2]?-(int32_t)magnitude:(int32_t)magnitude;nav_odom_wheel(rx[0],value,now);b.position_at[k]=now;nav_logf("E,%lu,%u,%ld\r\n",(unsigned long)now,rx[0],(long)value);}
 }else if(rx[1]==0x3A){b.flags[k]=rx[2];b.flags_at[k]=now;}
 else if(rx[2]!=2)b.errors++;
 if(op==0xF6&&rx[2]==2&&batch)nav_odom_command(rx[0],snapshot[k],now);
 if(op==0xF6&&rx[2]==2&&batch)nav_logf("A,%lu,%u,%d,%lu\r\n",(unsigned long)now,rx[0],snapshot[k],(unsigned long)batch_generation);
 waiting=false;b.transaction=false;
 if(op==0xF6 && batch){feedback_turn=true;batch_index++;if(batch_index==4){batch=false;b.velocity_done=batch_generation;}}
 if(op==0xF3 && enabling){enable_index++;if(enable_index==4){enabling=false;b.enable_done++;}}
}
void nav_bus_feed(uint8_t byte,uint32_t at)
{
 if(used&&nav_age(at,last_rx_at)>15U){used=0;b.rx_resyncs++;}
 last_rx_at=at;
 if(!used){if(index_of(byte)==4)return;rx[used++]=byte;return;}
 if(used==1){if(byte!=0x36&&byte!=0x3A&&byte!=0xFE&&byte!=0xF3&&byte!=0xF6){used=0;return;}expected=byte==0x36?8:4;}
 rx[used++]=byte;
 if(used==expected){if(rx[used-1]==0x6B)frame(at);else b.errors++;used=0;}
}
/* Permit at most one urgent read between adjacent wheel writes. This keeps a
 * speed generation coherent but prevents delayed ACKs starving odometry.
 * Nominal queries remain round-robin; age priority is only a backlog policy. */
static bool urgent_read(uint32_t now)
{
 unsigned position=0,flag=0;uint32_t position_age=0,flag_age=0;
 for(unsigned k=0;k<4;k++){
  uint32_t a=b.position_at[k]?nav_age(now,b.position_at[k]):10000;
  uint32_t f=b.flags_at[k]?nav_age(now,b.flags_at[k]):10000;
  if(a>position_age){position=k;position_age=a;}
  if(f>flag_age){flag=k;flag_age=f;}
 }
 if(position_age<100 && flag_age<400)return false;
 bool flags=flag_age>3*(position_age>100000?100000:position_age);
 unsigned k=flags?flag:position;uint8_t p[3]={ids[k],flags?0x3A:0x36,0x6B};
 if(send(p,3,now,true,k)){feedback_turn=false;next_poll=now+10;if(flags)last_flags=now;return true;}
 return false;
}
void nav_bus_tick(uint32_t now)
{
 if(b.tx_fault)return;
 if(b.tx_packets&&nav_hw_tx_busy()&&nav_age(now,sent_at)>20){
  b.tx_fault=true;b.errors++;nav_log("[TX_FAULT] UART TX stuck; no false stop confirmation; watchdog reset requested\r\n");return;
 }
 if(b.stopping){
  if(stop_index<8){unsigned k=stop_index%4;uint8_t p[6]={ids[k],0xFE,0x98,0,0x6B,0};unsigned n=5;
   if(stop_index>=4){p[1]=0xF3;p[2]=0xAB;p[3]=0;p[4]=0;p[5]=0x6B;n=6;}
   if(send(p,n,now,false,k)){if(stop_index<4)stop_sent|=1U<<k;else disable_sent|=1U<<k;stop_index++;stop_at=now;}
  }else if(nav_age(now,stop_at)>=180U){b.stopping=false;nav_logf("[STOP_ACK] stop=%02X disable=%02X (0F=all ACKs only; otherwise UNCONFIRMED; physical stop needs feedback)\r\n",b.stop_ack,b.disable_ack);}
  return;
 }
 if(waiting){
  if(nav_age(now,sent_at)<150U)return;
  b.timeouts++;b.errors++;waiting=false;b.transaction=false;batch=enabling=false;used=0;
  b.velocity_done=b.velocity_requested;return; /* Caller stops before any next write. */
 }
 if(enabling){uint8_t p[6]={ids[enable_index],0xF3,0xAB,1,0,0x6B};send(p,6,now,true,enable_index);return;}
 if(!batch&&b.velocity_done!=b.velocity_requested){memcpy(snapshot,latest,sizeof snapshot);batch_generation=b.velocity_requested;batch=true;batch_index=0;}
 if(batch&&feedback_turn&&urgent_read(now))return;
 if(batch){unsigned k=batch_index;int value=snapshot[k];unsigned magnitude=(unsigned)abs(value);if(magnitude>NAV_MAX_WHEEL_RPM)magnitude=NAV_MAX_WHEEL_RPM;
  uint8_t p[8]={ids[k],0xF6,value>=0?1:0,(uint8_t)(magnitude>>8),(uint8_t)magnitude,0,0,0x6B};send(p,8,now,true,k);return;}
 if((int32_t)(now-next_poll)<0)return;
 /* Once the final zero batch completes, stale-first polling must continue.
  * At 20ms reply latency a phase change can otherwise leave the common-time
  * odometry older than 300ms while individual position samples look fresh. */
 if(urgent_read(now))return;
 /* One flag query per 100ms -> each wheel at least every 400ms. */
 bool flags=(uint32_t)(now-last_flags)>=100U;unsigned k=flags?flag_index:poll_index;
 uint8_t p[3]={ids[k],flags?0x3A:0x36,0x6B};
 if(send(p,3,now,true,k)){next_poll=now+10;if(flags){flag_index=(flag_index+1)%4;last_flags=now;}else poll_index=(poll_index+1)%4;}
}
