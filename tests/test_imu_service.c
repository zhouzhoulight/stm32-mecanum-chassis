#include "imu_service.h"
#include "wit_imu.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static uint8_t sent[50][5];
static unsigned count;
static bool fail_tx, pin_enabled;
static char logs[10000];
void platform_log(const char *s) { assert(strlen(logs)+strlen(s)<sizeof logs);strcat(logs,s); }
bool platform_imu_tx_enable(bool on) { pin_enabled=on;return true; }
bool platform_imu_send(const uint8_t *p,size_t n)
{
 assert(pin_enabled && n==5 && count<50);memcpy(sent[count++],p,5);
 if(fail_tx){fail_tx=false;return false;}return true;
}
static void frame(uint8_t type,int16_t a,int16_t b,int16_t c,uint16_t w,uint32_t now)
{
 uint8_t p[11]={0x55,type};uint16_t v[]={(uint16_t)a,(uint16_t)b,(uint16_t)c,w};
 for(unsigned i=0;i<4;++i){p[2+2*i]=(uint8_t)v[i];p[3+2*i]=(uint8_t)(v[i]>>8);}
 for(unsigned i=0;i<10;++i)p[10]=(uint8_t)(p[10]+p[i]);
 for(unsigned i=0;i<11;++i)imu_feed(p[i],now);
}
static void sample(int16_t yaw,int16_t gyro,uint32_t now)
{frame(0x53,0,0,yaw,0x1234,now);frame(0x52,0,0,gyro,0,now);}
static void reset(void)
{imu_init();imu_service_init();count=0;fail_tx=pin_enabled=false;logs[0]=0;}
static void cmd(const char *s,uint32_t now) { assert(imu_service_command(s,now)); }
static void packet(unsigned i,uint8_t reg,uint16_t value)
{
 assert(i<count);assert(sent[i][0]==0xff && sent[i][1]==0xaa && sent[i][2]==reg);
 assert(sent[i][3]==(uint8_t)value && sent[i][4]==(uint8_t)(value>>8));
}
int main(void)
{
 reset();assert(!imu_service_busy() && !imu_service_tx_enabled());
 cmd("imu read mode",0);assert(count==0 && strstr(logs,"TX is off"));
 cmd("imu zero",0);assert(strstr(logs,"fresh angle required"));
 sample(32760,0,100);cmd("imu zero",100);assert(count==0);
 sample(-32760,0,110);imu_service_report(110);assert(strstr(logs,"relative_yaw_mdeg=88"));
 imu_service_report(1000);assert(strstr(logs,"relative_valid=0"));
 cmd("imu tx on",1000);assert(pin_enabled && count==0);
 cmd("imu read mode",1001);packet(0,0x27,0x24);assert(imu_service_busy());
 frame(0x53,0,0,0,0,1010);imu_service_tick(1010);assert(imu_service_busy());
 frame(0x5f,1,0,0,0,1020);imu_service_tick(1020);
 assert(!imu_service_busy() && strstr(logs,"6-axis"));
 assert(!imu_service_command("imu factory reset",1030));
 cmd("imu read filter",1100);imu_service_tick(1600);assert(!imu_service_busy());
 unsigned before=count;frame(0x5f,0,0,0,0,1610);imu_service_tick(1610);
 cmd("imu read mode",1620);assert(count==before); /* no relabeling a late reply */
 cmd("imu read version",2600);packet(count-1,0x27,0x2e);cmd("imu cancel",2610);
 assert(!imu_service_busy());cmd("imu tx off",2620);assert(!pin_enabled);

 reset();sample(0,0,0);cmd("imu tx on",0);cmd("imu cal acc",0);
 packet(0,0x69,0xb588);assert(imu_service_busy());
 imu_service_tick(19);assert(count==1);imu_service_tick(20);packet(1,0x01,1);
 cmd("imu tx off",30);assert(pin_enabled);cmd("imu cancel",40);assert(imu_service_busy());
 imu_service_tick(5019);assert(count==2);imu_service_tick(5020);packet(2,0x69,0xb588);
 imu_service_tick(5040);packet(3,0x01,0);assert(count==4);
 imu_service_tick(5139);assert(count==4);imu_service_tick(5140);packet(4,0x27,1);
 frame(0x5f,0,0,9,6,5150);imu_service_tick(5150);assert(!imu_service_busy());
 for(unsigned i=0;i<count;++i)assert(sent[i][2]!=0x00); /* never automatic Flash save */
 cmd("imu save",5160);imu_service_tick(5180);packet(5,0x69,0xb588);packet(6,0,0);

 reset();sample(0,0,0);cmd("imu tx on",0);cmd("imu cal acc",0);
 fail_tx=true;imu_service_tick(20);assert(imu_service_busy());
 cmd("imu cal stop",100);imu_service_tick(120);imu_service_tick(220);imu_service_tick(720);
 assert(imu_service_busy() && strstr(logs,"timeout"));
 cmd("imu cal stop",800);imu_service_tick(820);imu_service_tick(920);frame(0x5f,1,0,0,0,930);
 imu_service_tick(930);assert(imu_service_busy() && strstr(logs,"CALSW not normal"));
 cmd("imu cal stop",1000);imu_service_tick(1020);imu_service_tick(1120);frame(0x5f,0,0,0,0,1130);
 imu_service_tick(1130);assert(!imu_service_busy());

 reset();sample(0,100,100);cmd("imu tx on",100);cmd("imu cal acc",100);
 assert(count==0 && !imu_service_busy() && strstr(logs,"cal rejected"));

 reset();uint32_t start=UINT32_MAX-5000U;sample(32760,0,start);cmd("imu drift",start);
 for(unsigned i=1;i<=100;++i){uint32_t t=start+i*100U;sample((int16_t)(32760U+i),1,t);imu_service_tick(t);}
 assert(!imu_service_busy() && count==0 && strstr(logs,"IMU DRIFT"));
 assert(strstr(logs,"mean_z_mdps=61"));assert(imu_data()->angle_word4==0x1234);
 frame(0x51,-32768,16384,32767,0,20000);frame(0x54,-1,2,-3,0,20000);
 assert(imu_data()->acc_raw[0]==-32768 && imu_data()->acc_raw[2]==32767);
 assert(imu_data()->mag_raw[0]==-1 && imu_data()->mag_raw[2]==-3);
 reset();sample(0,0,0);cmd("imu drift",0);imu_service_tick(500);
 assert(!imu_service_busy() && strstr(logs,"aborted"));
 reset();sample(0,0,0);cmd("imu drift",0);imu_uart_error();imu_service_tick(10);
 assert(!imu_service_busy() && count==0);
 puts("PASS: IMU UART protocol, TX opt-in, calibration stop/readback/failure, explicit save, drift wrap/staleness and no motor path.");
}
