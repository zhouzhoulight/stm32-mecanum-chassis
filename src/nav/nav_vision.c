/* uart.docx V1.3 envelope. 05/85 is the explicit obstacle extension.
 * Console simulation transports HEX frames, not a separate map-setting path. */
#include "nav/nav_vision.h"
#include "nav/nav_map.h"
#include "nav/nav_app.h"
#include "nav/nav_platform.h"
#include <stdio.h>
#include <string.h>
static uint8_t frame[38],used,seq,round_id;
static uint32_t requested_at,last_byte;
static bool pending,hardware;
static uint8_t pending_cmd;
void nav_vision_error(void){used=0;if(pending){pending=false;if(pending_cmd==5)nav_map_invalidate();nav_log("[VISION_ERROR] UART/overflow; map invalid\r\n");}}
void nav_vision_feed(uint8_t byte,uint32_t now)
{
 if(used&&now-last_byte>100)used=0;
 last_byte=now;
 if(!used){if(byte==0xAA)frame[used++]=byte;return;}
 if(used==1&&byte!=0x55){used=byte==0xAA?1:0;return;}
 if(used==sizeof frame){nav_vision_error();return;}
 frame[used++]=byte;
 if(used<5)return;
 if(frame[4]>32){used=0;return;}
 if(used!=frame[4]+6)return;
 uint8_t check=0;for(unsigned k=0;k<used;k++)check^=frame[k];
 unsigned n=used;used=0;
 if(pending_cmd==1){
  if(check||!pending||nav_app_active()||now-requested_at>10000||frame[2]!=0x81||frame[3]!=seq||frame[4]<1||frame[4]>31){nav_log("[VISION_REJECT] QR checksum/sequence/length/state\r\n");return;}
  char text[32];for(unsigned k=0;k<frame[4];k++){unsigned char c=frame[5+k];if(c<32||c>126){nav_log("[VISION_REJECT] QR ASCII required\r\n");return;}text[k]=(char)c;}text[frame[4]]=0;
  pending=false;nav_logf("[VISION_QR] seq=%u text=%s; observation only, no chassis motion\r\n",seq,text);return;
 }
 if(check||!pending||nav_app_active()||now-requested_at>1500||frame[2]!=0x85||frame[3]!=seq||frame[4]<2||frame[5]!=round_id){nav_log("[VISION_REJECT] checksum/sequence/round/command/length/state\r\n");return;}
 bool positions=(frame[6]&0x80U)!=0;unsigned count=frame[6]&0x7FU;
 if(count>2||n!=8+(positions?4U:2U)*count){nav_log("[VISION_REJECT] obstacle length/count\r\n");return;}
 bool accepted;
 if(positions){
  uint16_t x[2],y[2];for(unsigned k=0;k<count;k++){unsigned i=7+4*k;x[k]=(uint16_t)(frame[i]|frame[i+1]<<8);y[k]=(uint16_t)(frame[i+2]|frame[i+3]<<8);}
  accepted=nav_map_set_positions(round_id,x,y,count);
 }else{
  NavObstacle o[2];for(unsigned k=0;k<count;k++){o[k].kind=frame[7+2*k];o[k].id=frame[8+2*k];}
  accepted=nav_map_set(round_id,o,count);
 }
 if(!accepted){nav_log("[VISION_REJECT] invalid/duplicate obstacle position or feature\r\n");return;}
 pending=false;nav_logf("[VISION_OK] seq=%u round=%u; map ready; NO motion\r\n",seq,round_id);
}
void nav_vision_tick(uint32_t now)
{if(pending&&now-requested_at>(pending_cmd==1?10000U:1500U)){pending=false;if(pending_cmd==5)nav_map_invalidate();nav_log("[VISION_TIMEOUT] no matching obstacle response; map invalid, no motion\r\n");}}
static int hex(char c){if(c>='0'&&c<='9')return c-'0';if(c>='A'&&c<='F')return c-'A'+10;if(c>='a'&&c<='f')return c-'a'+10;return -1;}
bool nav_vision_command(const char *line,uint32_t now)
{
 if(strncmp(line,"vision",6)|| (line[6]&&line[6]!=' '))return false;
 if(nav_app_active()){nav_log("[E] vision map commands require chassis idle\r\n");return true;}
 if(!strcmp(line,"vision console")||!strcmp(line,"vision uart")){
  if(pending){nav_log("[E] finish current vision request first\r\n");return true;}
  hardware=!strcmp(line,"vision uart");nav_logf("[VISION_TRANSPORT] %s; UART5 PC12/PD2 or console HEX bridge\r\n",hardware?"UART5":"console");return true;
 }
 if(!strcmp(line,"vision qr")){
  if(pending){nav_log("[E] vision request pending\r\n");return true;}
  seq=seq==255?1:seq+1;used=0;requested_at=now;pending=true;pending_cmd=1;
  uint8_t p[6]={0xAA,0x55,1,seq,0,0};for(unsigned k=0;k<5;k++)p[5]^=p[k];
  if(hardware&&!nav_vision_hw_send(p,sizeof p)){pending=false;nav_log("[VISION_ERROR] QR TX failed\r\n");return true;}
  nav_logf("[VISION_TX] %02X %02X %02X %02X %02X %02X\r\n",p[0],p[1],p[2],p[3],p[4],p[5]);return true;
 }
 unsigned round=0;bool request=!strncmp(line,"vision request ",15);
 if(request){const char *p=line+15;unsigned digits=0;while(*p>='0'&&*p<='9'&&digits<3){round=round*10+(unsigned)(*p++-'0');digits++;}if(*p||!digits)request=false;}
 if(request&&round>=1&&round<=255){
  pending_cmd=5;round_id=(uint8_t)round;seq=seq==255?1:seq+1;used=0;requested_at=now;pending=true;nav_map_invalidate();
  uint8_t p[7]={0xAA,0x55,0x05,seq,1,round_id,0};for(unsigned k=0;k<6;k++)p[6]^=p[k];
  if(hardware&&!nav_vision_hw_send(p,sizeof p)){pending=false;nav_log("[VISION_ERROR] TX failure\r\n");return true;}
  nav_logf("[VISION_TX] %02X %02X %02X %02X %02X %02X %02X\r\n",p[0],p[1],p[2],p[3],p[4],p[5],p[6]);return true;
 }
 if(!strncmp(line,"vision rx ",10)){
  if(hardware){nav_log("[E] HEX bridge disabled in UART5 mode\r\n");return true;}
  uint8_t bytes[38];unsigned k=0;const char *p=line+10;
  while(*p){while(*p==' ')p++;if(!*p)break;
   int a=hex(p[0]),b=p[0]?hex(p[1]):-1;
   if(a<0||b<0||k==sizeof bytes||(p[2]&&p[2]!=' ')){nav_log("[E] vision rx requires pairs of HEX digits\r\n");return true;}
   bytes[k++]=(uint8_t)((a<<4)|b);p+=2;
  }
  for(unsigned j=0;j<k;j++)nav_vision_feed(bytes[j],now);
  return true;
 }
 nav_log("[E] vision console|uart|request ROUND|rx HEX_FRAME\r\n");return true;
}
