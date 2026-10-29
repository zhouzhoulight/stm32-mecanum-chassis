#include "heading_hold.h"
#include "motor_test.h"
#include "wit_imu.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint32_t now, move_at[5], tick_at;
static WitImu data;
static unsigned sent, moves, enables, stops, disables, flag_queries;
static uint32_t pulse_sum[5];
static uint8_t last_direction[5];
static uint16_t last_pulses[5];
static uint8_t queue[30000];
static size_t head, tail;
static char logs[60000];
static bool auto_ack, auto_flag, stall, reject_move, fail_send, legacy_idle, arrived;
static uint8_t flags_high;
static bool live_diag_reply;
static uint8_t last_read[4];
static size_t last_read_len;

static void reply(uint8_t id, uint8_t op, uint8_t value)
{
    assert(head+4 < sizeof queue);
    queue[head++]=id;queue[head++]=op;queue[head++]=value;queue[head++]=0x6B;
}

bool platform_motor_send(const uint8_t *p, size_t len)
{
    assert(p[0]>=1 && p[0]<=4);
    ++sent;
    if (fail_send) { fail_send=false;return false; }
    uint8_t id=p[0];
    if (p[1]==0xFD) {
        assert(len==13 && p[3]==0 && p[4]==5 && p[5]==10);
        assert(p[6]==0 && p[7]==0 && p[10]==0 && p[11]==0 && p[12]==0x6B);
        uint16_t pulses=(uint16_t)(p[8]*256U+p[9]);
        assert(pulses>=144 && pulses<=176);
        pulse_sum[id]+=pulses;last_direction[id]=p[2];last_pulses[id]=pulses;
        ++moves;move_at[id]=now;
        if (auto_ack) reply(id,0xFD,reject_move ? 0xE2 : 0x02);
    } else if (p[1]==0xF3) {
        assert(len==6 && p[2]==0xAB && p[4]==0 && p[5]==0x6B);
        if (p[3]) { ++enables;if(auto_ack)reply(id,0xF3,0x02); }
        else ++disables;
    } else if (p[1]==0xFE) {
        assert(len==5 && p[2]==0x98 && p[3]==0 && p[4]==0x6B);++stops;
    } else if (p[1]==0x3A) {
        assert(len==3 && p[2]==0x6B);++flag_queries;
        if (auto_flag) reply(id,0x3A,(uint8_t)(flags_high | (stall ? 0x0F : moves ?
            (arrived && (uint32_t)(now-move_at[id])>=450U ? 0x03 : 0x01) : 0x02)));
    } else if (p[1]==0x37) {
        assert(len==3 && p[2]==0x6B);
        if(live_diag_reply) {
            assert(head+8<sizeof queue);
            const uint8_t q[]={id,0x37,0,0,0,0,(uint8_t)(id>=3 ? 80 : 0),0x6B};
            memcpy(queue+head,q,sizeof q);head+=sizeof q;
        }
    } else if (p[1]==0x21 || p[1]==0x42) {
        assert((p[1]==0x21 && len==3 && p[2]==0x6B) ||
               (p[1]==0x42 && len==4 && p[2]==0x6C && p[3]==0x6B));
        memcpy(last_read,p,len);last_read_len=len;
    } else if(p[1]==0x35 || p[1]==0x33 || p[1]==0x34 || p[1]==0x36) {
        assert(len==3 && p[2]==0x6B);
        if(live_diag_reply) {
            uint8_t q[]={id,p[1],0,0,0,0,160,0x6B};
            if(p[1]==0x36)q[6]=(uint8_t)(id>=3 ? 80 : 160);
            size_t count=sizeof q;
            if(p[1]==0x35){count=6;q[5]=0x6B;}
            assert(head+count<sizeof queue);memcpy(queue+head,q,count);head+=count;
        }
    } else assert(0);
    return true;
}

void platform_log(const char *s)
{
    assert(strlen(logs)+strlen(s)<sizeof logs);
    strcat(logs,s);
}
const WitImu *imu_data(void) { return &data; }
bool test_begin_diagnostic(void) { return legacy_idle; }

static void fresh(void)
{
    data.angle_seen=data.gyro_seen=true;
    data.angle_at=data.gyro_at=now;
    ++data.angle_samples;++data.gyro_samples;
}
static void reset(uint32_t at)
{
    now=at;tick_at=at;
    memset(&data,0,sizeof data);memset(move_at,0,sizeof move_at);
    memset(pulse_sum,0,sizeof pulse_sum);
    sent=moves=enables=stops=disables=flag_queries=0;head=tail=0;logs[0]=0;
    auto_ack=auto_flag=legacy_idle=arrived=true;
    stall=reject_move=fail_send=false;
    flags_high=0;
    live_diag_reply=false;
    memset(last_read,0,sizeof last_read);last_read_len=0;
    hold_init();fresh();
}
static void drain(void)
{
    while(tail<head)hold_motor_rx(queue[tail++],now);
}
static void step(bool keep, bool update)
{
    now+=10;
    if(update)fresh();
    if(keep)assert(hold_command("hold_keep",now));
    hold_tick(now);drain();
}
static void start(const char *command)
{
    assert(hold_command("arm hold",now));
    assert(moves==0 && enables==0);
    assert(hold_command(command,now));
    assert(hold_active());
}
static void until_moves(unsigned target)
{
    for(unsigned i=0;i<300 && moves<target && hold_active();++i)step(true,true);
    assert(moves>=target);
}

int main(void)
{
    assert(hold_angle_delta(-179000,179000)==2000);
    assert(hold_angle_delta(179000,-179000)==-2000);
    assert(hold_correction(150)==0 && hold_correction(-150)==0);
    assert(hold_correction(1000)==8 && hold_correction(-1000)==-8);
    assert(hold_correction(100000)==16 && hold_correction(-100000)==-16);
    assert(hold_wheel_pulses(4,8)==152 && hold_wheel_pulses(1,8)==152);
    assert(hold_wheel_pulses(3,8)==168 && hold_wheel_pulses(2,8)==168);
    assert(hold_wheel_pulses(0,0)==0 && hold_wheel_pulses(5,0)==0);
    reset(0);hold_command("left_hold_16000",now);assert(!hold_active() && sent==0);
    reset(0);hold_command("arm hold",now);now=10000;fresh();hold_command("left_hold_16000",now);assert(!hold_active());
    reset(0);legacy_idle=false;hold_command("arm hold",now);hold_command("left_hold_1600",now);assert(!hold_active() && sent==0);
    reset(0);data.angle_seen=false;hold_command("arm hold",now);hold_command("left_hold_1600",now);assert(!hold_active());
    reset(0);hold_command("arm hold",now);hold_command("help",now);hold_command("left_hold_1600",now);assert(!hold_active());

    reset(0);start("left_hold_16000");until_moves(4);
    assert(last_direction[4]==0 && last_direction[1]==1 && last_direction[3]==1 && last_direction[2]==0);
    unsigned n=moves;
    hold_command("left_hold_16000",now);assert(!hold_active() && moves==n && stops==4 && disables==4);
    reset(0);start("left_hold_16000");hold_command("stop",now);assert(!hold_active() && moves==0);
    for(unsigned i=0;i<100;++i) { step(true,true); }
    assert(moves==0);

    reset(0);start("left_hold_16000");until_moves(4);data.yaw_mdeg=1000;
    until_moves(8);assert(last_pulses[4]==152 && last_pulses[3]==168);
    hold_emergency_stop();assert(!hold_active() && stops==4 && disables==4);

    reset(0);start("left_hold_16000");auto_ack=false;
    for(unsigned i=0;i<100 && hold_active();++i)step(true,true);
    assert(!hold_active() && moves==0 && strstr(logs,"ACK timeout"));
    reset(0);auto_flag=false;start("left_hold_16000");
    for(unsigned i=0;i<100 && hold_active();++i)step(true,true);
    assert(!hold_active() && moves==0 && strstr(logs,"flag timeout"));
    reset(0);stall=true;start("left_hold_16000");step(true,true);
    assert(!hold_active() && moves==0 && stops==4);
    reset(0);flags_high=0x80;start("left_hold_16000");until_moves(4);
    assert(hold_active() && strstr(logs,"raw=130"));hold_emergency_stop();
    reset(0);reject_move=true;start("left_hold_16000");until_moves(4);
    assert(!hold_active() && strstr(logs,"rejected"));
    reset(0);start("left_hold_16000");until_moves(4);data.angle_seen=false;step(true,false);
    assert(!hold_active() && strstr(logs,"IMU stale/error"));
    reset(0);start("left_hold_16000");until_moves(4);++data.bad_checksum;step(true,true);assert(!hold_active());
    reset(0);start("left_hold_16000");until_moves(4);data.yaw_mdeg=8001;step(true,true);
    assert(!hold_active() && strstr(logs,"heading error limit"));
    reset(0);start("left_hold_16000");until_moves(4);
    for(unsigned i=0;i<160 && hold_active();++i)step(false,true);
    assert(!hold_active() && strstr(logs,"heartbeat lost"));
    reset(0);start("left_hold_16000");until_moves(4);arrived=false;
    for(unsigned i=0;i<300 && hold_active();++i)step(true,true);
    assert(!hold_active() && moves==4 && strstr(logs,"segment deadline"));
    reset(0);start("left_hold_16000");fail_send=true;step(true,true);assert(!hold_active());
    reset(0);start("left_hold_16000");until_moves(4);now+=80000;fresh();hold_command("hold_keep",now);hold_tick(now);
    assert(!hold_active() && strstr(logs,"total deadline"));

    /* Long profile completes exactly 100 finite segments; also crosses millis wrap. */
    reset(UINT32_MAX-100U);start("left_hold_16000");
    for(unsigned i=0;i<8000 && hold_active();++i)step(true,true);
    assert(!hold_active() && moves==400 && enables==4 && stops==4 && disables==4);
    for(unsigned id=1;id<=4;++id)assert(pulse_sum[id]==16000);
    assert(strstr(logs,"nominal pulse budget completed"));
    unsigned after=moves;hold_tick(now+1000);assert(moves==after);
    reset(0);assert(hold_command("hold_diag 3",now));
    assert(sent==1 && moves==0 && enables==0 && !hold_active());
    const uint8_t perr[]={3,0x37,1,0,0,0,80,0x6B};
    for(unsigned i=0;i<sizeof perr;++i)hold_motor_rx(perr[i],now);
    assert(strstr(logs,"id=3 sign=1 error_counts=80"));
    reset(0);assert(hold_command("hold_diag 4",now));now=300;hold_tick(now);
    assert(strstr(logs,"HOLD DIAG timeout") && !hold_active() && moves==0);
    reset(0);flags_high=0x80;live_diag_reply=true;arrived=false;
    start("left_diag_160");
    for(unsigned i=0;i<300 && hold_active();++i)step(true,true);
    assert(moves==4 && stops==4 && disables==4 && !hold_active());
    assert(strstr(logs,"segment deadline"));
    assert(strstr(logs,"id=3 flags=129 sign=0 error_counts=80"));
    assert(strstr(logs,"id=4 flags=129 sign=0 error_counts=80"));
    assert(strstr(logs,"id=4 op=53 sign=0 value=0"));
    assert(strstr(logs,"id=4 op=51 sign=0 value=160"));
    assert(strstr(logs,"id=4 op=52 sign=0 value=160"));
    assert(strstr(logs,"id=4 op=54 sign=0 value=80"));
    assert(strstr(logs,"id=1 op=54 sign=0 value=160"));
    for(unsigned id=1;id<=4;++id)assert(pulse_sum[id]==160);
    reset(0);live_diag_reply=true;arrived=false;start("left_diag_160");
    for(unsigned i=0;i<100 && !strstr(logs,"HOLD SAMPLE");++i)step(true,true);
    assert(strstr(logs,"HOLD SAMPLE"));live_diag_reply=false;
    for(unsigned i=0;i<4;++i)step(true,true);
    const uint8_t invalid_speed[]={1,0x35,2,0,0,0x6B};
    for(unsigned i=0;i<sizeof invalid_speed;++i)hold_motor_rx(invalid_speed[i],now);
    assert(!hold_active() && stops==4 && disables==4 && moves==4);
    reset(0);live_diag_reply=true;arrived=false;start("left_diag_160");
    for(unsigned i=0;i<100 && !strstr(logs,"HOLD SAMPLE");++i)step(true,true);
    live_diag_reply=false;
    for(unsigned i=0;i<80 && hold_active();++i)step(true,true);
    assert(!hold_active() && strstr(logs,"motor flag timeout") && moves==4);
    reset(0);live_diag_reply=true;start("left_diag_160");
    for(unsigned i=0;i<300 && hold_active();++i)step(true,true);
    assert(moves==4 && !hold_active() && strstr(logs,"nominal pulse budget completed"));
    /* Read-only parameter queries cannot enable, move, write settings or leave an arm valid. */
    reset(0);hold_command("arm hold",now);hold_command("driver_config 4",now);
    const uint8_t config_cmd[]={4,0x42,0x6C,0x6B};
    assert(last_read_len==4 && !memcmp(last_read,config_cmd,4));
    hold_command("left_hold_1600",now);assert(!hold_active() && moves==0 && enables==0);
    uint8_t config_reply[33]={4,0x42,33,21};config_reply[32]=0x6B;
    for(unsigned i=0;i<sizeof config_reply;++i)hold_motor_rx(config_reply[i],now+i);
    assert(strstr(logs,"DRIVER READ OK id=4 op=66 bytes=33"));
    hold_command("left_hold_1600",now);assert(!hold_active() && moves==0 && enables==0);
    reset(0);hold_command("driver_pid 1",now);
    const uint8_t pid_cmd[]={1,0x21,0x6B};
    assert(last_read_len==3 && !memcmp(last_read,pid_cmd,3));
    uint8_t pid_reply[15]={1,0x21};pid_reply[14]=0x6B;
    for(unsigned i=0;i<sizeof pid_reply;++i)hold_motor_rx(pid_reply[i],now+i);
    assert(strstr(logs,"DRIVER READ OK id=1 op=33 bytes=15") && !hold_active() && moves==0);
    for(unsigned length=0;length<=255;++length) {
        reset(0);hold_command("driver_config 4",now);
        uint8_t frame[256]={4,0x42,(uint8_t)length,21};
        if(length>=5 && length<=64)frame[length-1]=0x6B;
        for(unsigned i=0;i<sizeof frame;++i)hold_motor_rx(frame[i],now);
        if(length>=5 && length<=64)assert(strstr(logs,"DRIVER READ OK"));
        else assert(strstr(logs,"invalid length") && !strstr(logs,"DRIVER READ OK"));
        assert(!hold_active() && moves==0 && enables==0);
    }
    reset(0);hold_command("driver_config 4",now);
    hold_motor_rx(4,now);hold_motor_rx(0,now);hold_motor_rx(0xEE,now);hold_motor_rx(0x6B,now);
    assert(strstr(logs,"HOLD DIAG rejected"));
    reset(0);hold_command("driver_pid 1",now);
    pid_reply[0]=2;
    for(unsigned i=0;i<sizeof pid_reply;++i)hold_motor_rx(pid_reply[i],now);
    now=300;hold_tick(now);
    assert(strstr(logs,"timeout") && !strstr(logs,"DRIVER READ OK"));
    reset(0);hold_command("driver_config 4",now);hold_command("stop",now);
    for(unsigned i=0;i<sizeof config_reply;++i)hold_motor_rx(config_reply[i],now);
    assert(!strstr(logs,"DRIVER READ OK"));
    reset(0);hold_command("driver_pid 0",now);hold_command("driver_config 5",now);
    assert(sent==0 && !hold_active());
    reset(0);legacy_idle=false;hold_command("driver_config 4",now);assert(sent==0);
    reset(0);start("left_hold_1600");until_moves(4);unsigned before=sent;
    hold_command("driver_config 4",now);
    assert(!hold_active() && last_read_len==0 && sent==before+8);
    puts("PASS: heading sign/wrap/limits; finite 100-segment budget; ACK/flags/IMU/heartbeat/TX/timeout/stop guards; no auto-repeat.");
}
