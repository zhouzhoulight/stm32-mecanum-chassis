/* UART v1.4. Observations are logged and never initiate motion here. */
#include "nav/nav_vision.h"
#include "nav/nav_map.h"
#include "nav/nav_app.h"
#include "nav/nav_platform.h"
#include <stdio.h>
#include <string.h>
#define MAX_DATA 192U
static uint8_t frame[MAX_DATA+6U],used,seq,round_id;
static uint32_t requested_at,last_byte;
static bool pending,hardware;
static uint8_t pending_cmd;
static uint16_t token;
static uint8_t request_payload[8];
static NavVisionResult result;
static uint32_t deadline_ms;

static uint16_t u16(const uint8_t *p){return (uint16_t)(p[0]|p[1]<<8);}
static int s16(const uint8_t *p){unsigned value=u16(p);return value>=32768U?(int)value-65536:(int)value;}
static uint32_t timeout_ms(void){return deadline_ms;}
static void end_request(void){pending=false;if(pending_cmd==5)nav_map_invalidate();}
void nav_vision_error(void){used=0;if(pending){end_request();nav_log("[VISION_ERROR] UART/overflow; request cancelled\r\n");}}

static bool send_request(uint8_t cmd,const uint8_t *data,unsigned len,uint32_t now)
{
 uint8_t packet[14]={0xAA,0x55,cmd,seq==255?1:(uint8_t)(seq+1),(uint8_t)len};
 if(len)memcpy(packet+5,data,len);
 for(unsigned i=0;i<len+5;i++)packet[len+5]^=packet[i];
 if(hardware&&!nav_vision_hw_send(packet,len+6))return false;
 seq=packet[3];pending_cmd=cmd;pending=cmd!=4;used=0;requested_at=now;
 result.cmd=0;deadline_ms=cmd==1?10000U:1500U;
 if(len)memcpy(request_payload,data,len);
 char hexline[42];
 const char *digits="0123456789ABCDEF";
 for(unsigned i=0;i<len+6;i++){hexline[3*i]=digits[packet[i]>>4];hexline[3*i+1]=digits[packet[i]&15];hexline[3*i+2]=' ';}
 hexline[3*(len+6)-1]=0;
 nav_logf("[VISION_TX] %s\r\n",hexline);
 return true;
}

static bool qr_valid(const uint8_t *p,unsigned len)
{
 if(len!=15||p[3]!='+'||p[7]!='+'||p[11]!='+')return false;
 unsigned masks[4]={0};
 for(unsigned g=0;g<4;g++)for(unsigned i=0;i<3;i++){
  unsigned value=(unsigned)(p[4*g+i]-'0');
  if(value<1||value>(g%2?3U:6U)||(masks[g]&(1U<<value)))return false;
  masks[g]|=1U<<value;
 }
 return masks[0]==masks[2];
}
static void remember(const uint8_t *data,unsigned len,unsigned status,uint32_t now)
{
 if(len>sizeof result.data)return;
 result.cmd=pending_cmd;result.status=(uint8_t)status;result.length=(uint8_t)len;result.received_at=now;
 if(len)memcpy(result.data,data,len);
}

static bool observation(const uint8_t *data,unsigned len,uint32_t now)
{
 if(len<8||u16(data)!=u16(request_payload)||data[3]!=request_payload[2]||data[2]>8)return false;
 unsigned status=data[2],image=u16(data+4),age=u16(data+6);
 const uint8_t *v=data+8;
 if(status){
  if(len!=8)return false;
 }else{
  if(!image)return false;
  switch(pending_cmd){
   case 6:
    if(len<9||v[0]<1||v[0]>6||len!=9U+13U*v[0])return false;
    for(unsigned i=0;i<v[0];i++){
     const uint8_t *item=v+1+13*i;
     if(!u16(item)||item[2]<1||item[2]>6||
        (request_payload[4]&&item[2]!=request_payload[4])||item[11]>100||(item[12]&0xF0U))return false;
     for(unsigned j=0;j<i;j++)if(u16(v+1+13*j)==u16(item))return false;
    }
    break;
   case 7:
    if(len!=21||memcmp(v,request_payload+3,5)||v[11]>100||(v[12]&0xF8U)||
       (((v[12]&2U)!=0)==(u16(v+9)==32767U)))return false;
    break;
   case 8:
    if(len!=15||memcmp(v,request_payload+3,5)||v[5]>2||v[6]>100)return false;
    break;
   case 9:{
    if(len!=23||memcmp(v,request_payload+3,2)||v[2]!=3)return false;
    unsigned seen=0;
    for(unsigned i=0;i<3;i++){
     const uint8_t *item=v+3+4*i;
     if(item[0]<1||item[0]>3||(seen&(1U<<item[0]))||item[1]>6||
        item[2]>3||item[3]>100||(item[2]==1&&item[1]))return false;
     seen|=1U<<item[0];
    }
    break;
   }
   case 10:
    if(len!=20||v[10]>100||(v[11]&0xF8U)||
       (((v[11]&2U)!=0)==(u16(v+4)==32767U)))return false;
    break;
   default:return false;
  }
 }
 pending=false;
 remember(data,len,status,now);
 nav_logf("[VISION_RESULT] cmd=%02X seq=%u token=%u profile=%u status=%u image=%u age_ms=%u\r\n",
  pending_cmd,seq,u16(data),data[3],status,image,age);
 if(status)return true;
 if(pending_cmd==6)for(unsigned i=0;i<v[0];i++){
  const uint8_t *item=v+1+13*i;
  nav_logf("[VISION_RAW] track=%u color=%u d_b_mm=%d,%d v_b_mmps=%d,%d quality=%u flags=%02X\r\n",
   u16(item),item[2],s16(item+3),s16(item+5),s16(item+7),s16(item+9),item[11],item[12]);
 }
 if(pending_cmd==7)nav_logf("[VISION_ALIGN] area=%u index=%u color=%u aim=%u layer=%u d_b_mm=%d,%d yaw_ddeg=%d quality=%u flags=%02X\r\n",
  v[0],v[1],v[2],v[3],v[4],s16(v+5),s16(v+7),s16(v+9),v[11],v[12]);
 if(pending_cmd==8)nav_logf("[VISION_VERIFY] action=%u area=%u index=%u color=%u layer=%u verdict=%u quality=%u\r\n",
  v[0],v[1],v[2],v[3],v[4],v[5],v[6]);
 if(pending_cmd==9)for(unsigned i=0;i<3;i++){
  const uint8_t *item=v+3+4*i;
  nav_logf("[VISION_INVENTORY] area=%u layer=%u index=%u color=%u state=%u quality=%u\r\n",v[0],v[1],item[0],item[1],item[2],item[3]);
 }
 if(pending_cmd==10)nav_logf("[VISION_QR_LOCATE] d_px=%d,%d rot_ddeg=%d size_px=%u,%u quality=%u flags=%02X\r\n",
  s16(v),s16(v+2),s16(v+4),u16(v+6),u16(v+8),v[10],v[11]);
 return true;
}
void nav_vision_feed(uint8_t byte,uint32_t now)
{
 if(used&&now-last_byte>100)used=0;
 last_byte=now;
 if(!used){if(byte==0xAA)frame[used++]=byte;return;}
 if(used==1&&byte!=0x55){used=byte==0xAA?1:0;return;}
 if(used==sizeof frame){nav_vision_error();return;}
 frame[used++]=byte;
 if(used<5)return;
 if(frame[4]>MAX_DATA){used=0;return;}
 if(used!=frame[4]+6)return;
 uint8_t check=0;for(unsigned k=0;k<used;k++)check^=frame[k];
 unsigned n=used;used=0;
 if(check||!pending||nav_app_active()||now-requested_at>timeout_ms()||frame[3]!=seq){
  nav_log("[VISION_REJECT] checksum/sequence/state/deadline\r\n");return;
 }
 if(frame[2]==0xE0){
  uint16_t expected=pending_cmd>=6?u16(request_payload):0;
  if(frame[4]!=6||frame[5]!=pending_cmd||frame[6]<1||frame[6]>8||u16(frame+7)!=expected){
   nav_log("[VISION_REJECT] business error context\r\n");return;
  }
  end_request();
  remember(NULL,0,frame[6],now);
  nav_logf("[VISION_RESULT] cmd=%02X seq=%u token=%u status=%u detail=%u\r\n",pending_cmd,seq,expected,frame[6],u16(frame+9));return;
 }
 if(pending_cmd>=6){
  if(frame[2]!=(pending_cmd|0x80U)||!observation(frame+5,frame[4],now))nav_log("[VISION_REJECT] observation context or payload\r\n");
  return;
 }
 if(pending_cmd==1){
  if(frame[2]!=0x81||!qr_valid(frame+5,frame[4])){nav_log("[VISION_REJECT] QR task code invalid\r\n");return;}
  char text[16];memcpy(text,frame+5,15);text[15]=0;
  remember(frame+5,15,0,now);
  pending=false;nav_logf("[VISION_QR] seq=%u text=%s; observation only, no chassis motion\r\n",seq,text);return;
 }
 if(frame[2]!=0x85||frame[4]<2||frame[5]!=round_id){nav_log("[VISION_REJECT] obstacle round/command/length\r\n");return;}
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
{if(pending&&now-requested_at>timeout_ms()){end_request();nav_logf("[VISION_TIMEOUT] cmd=%02X seq=%u; no matching response\r\n",pending_cmd,seq);}}
static int hex(char c){if(c>='0'&&c<='9')return c-'0';if(c>='A'&&c<='F')return c-'A'+10;if(c>='a'&&c<='f')return c-'a'+10;return -1;}
static bool valid_observation_args(unsigned cmd,const uint8_t *a,unsigned n)
{
 if(cmd==10)return n==0;
 if(cmd==6)return n==2&&a[0]>=1&&a[0]<=2&&a[1]<=6;
 if(cmd==9)return n==2&&a[0]>=2&&a[0]<=4&&(a[1]==1||(a[0]==3&&a[1]==2));
 if(n!=5)return false;
 if(cmd==7){
  unsigned area=a[0],index=a[1],color=a[2],aim=a[3],layer=a[4];
  if(color>6)return false;
  if(aim==3)return area==3&&index>=1&&index<=3&&layer==2;
  if(layer!=1)return false;
  if(area==1)return aim==1&&index==0;
  if(index<1||index>3)return false;
  return (area==4&&(aim==1||aim==4))||((area==2||area==3)&&(aim==1||aim==2));
 }
 if(cmd==8){
  unsigned action=a[0],area=a[1],index=a[2],color=a[3],layer=a[4];
  if(color>6)return false;
  if(action==1||action==6)return area==0&&index==0&&layer==0&&(action==1?color!=0:color==0);
  if(index<1||index>3)return false;
  if(action==2)return area==4&&layer==1;
  if(action==3)return (area==2||area==3)&&layer==1;
  if(action==4)return area>=2&&area<=4&&!color&&(layer==1||(area==3&&layer==2));
  if(action==5)return area==3&&layer==2&&color!=0;
 }
 return false;
}
bool nav_vision_pending(void){return pending;}
const NavVisionResult *nav_vision_result(void){return &result;}
void nav_vision_cancel(uint32_t now)
{
 result.cmd=0;
 if(!pending)return;
 end_request();used=0;
 (void)send_request(4,NULL,0,now);
}
bool nav_vision_request(unsigned cmd,unsigned profile,const uint8_t *args,unsigned count,uint32_t timeout,uint32_t now)
{
 if(pending||nav_app_state()!=0||timeout<100||timeout>10000)return false;
 result.cmd=0;
 if(cmd==1){
  if(count||!send_request(1,NULL,0,now))return false;
 }else{
  if(cmd<6||cmd>10||!profile||profile>255||count>5||(count&&!args)||
     !valid_observation_args(cmd,args,count)||token==UINT16_MAX)return false;
  uint8_t data[8];
  token++;data[0]=(uint8_t)token;data[1]=(uint8_t)(token>>8);data[2]=(uint8_t)profile;
  if(count)memcpy(data+3,args,count);
  if(!send_request((uint8_t)cmd,data,count+3,now))return false;
 }
 deadline_ms=timeout;return true;
}
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
  if(!send_request(1,NULL,0,now))nav_log("[VISION_ERROR] QR TX failed\r\n");
  return true;
 }
 if(!strcmp(line,"vision cancel")){
  if(pending)end_request();
  used=0;
  if(!send_request(4,NULL,0,now))nav_log("[VISION_ERROR] cancel TX failed\r\n");
  return true;
 }
 if(!strncmp(line,"vision observe ",15)){
  if(pending){nav_log("[E] vision request pending\r\n");return true;}
  unsigned cmd=0,profile=0;int consumed=0;
  uint8_t data[8];unsigned n=0;
  bool ok=sscanf(line+15,"%u %u%n",&cmd,&profile,&consumed)==2&&cmd>=6&&cmd<=10&&profile>=1&&profile<=255;
  const char *p=line+15+consumed;
  while(ok&&*p){
   unsigned v=0;
   if(*p!=' '||n==5||sscanf(p," %u%n",&v,&consumed)!=1||v>255){ok=false;break;}
   data[3+n++]=(uint8_t)v;p+=consumed;
  }
  if(!ok||!valid_observation_args(cmd,data+3,n)||token==UINT16_MAX){
   nav_log("[E] vision observe CMD(6..10) PROFILE ARGS; invalid command or token exhausted\r\n");return true;
  }
  token++;data[0]=(uint8_t)token;data[1]=(uint8_t)(token>>8);data[2]=(uint8_t)profile;
  if(!send_request((uint8_t)cmd,data,n+3,now))nav_log("[VISION_ERROR] observation TX failed\r\n");
  return true;
 }
 unsigned round=0;bool request=!strncmp(line,"vision request ",15);
 if(request){const char *p=line+15;unsigned digits=0;while(*p>='0'&&*p<='9'&&digits<3){round=round*10+(unsigned)(*p++-'0');digits++;}if(*p||!digits)request=false;}
 if(request&&round>=1&&round<=255){
  if(pending){nav_log("[E] vision request pending\r\n");return true;}
  round_id=(uint8_t)round;nav_map_invalidate();
  if(!send_request(5,&round_id,1,now))nav_log("[VISION_ERROR] map TX failed\r\n");
  return true;
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
 nav_log("[E] vision console|uart|qr|observe CMD PROFILE ARGS|cancel|request ROUND|rx HEX_BYTES\r\n");return true;
}
