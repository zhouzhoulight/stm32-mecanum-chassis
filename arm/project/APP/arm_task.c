/* F407VET6 bench. UART4 is exclusively Emm5; USART1 is exclusively console.
 * No automatic move/homing/flash writes. Explicit measure hold clears driver
 * coordinates WHILE DISABLED, verifies them, then restores holding torque.
 */
#include "headerfile.h"
#include "arm_task.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "profile_store.h"

#define VERSION "GC1_ARM_F407VE_USART1_20261102_R9E"
#define SERVO_COUNT 4

#ifndef STUDIO_LIFT_RPM
#define STUDIO_ROUTE_RPM 200
#define STUDIO_ROUTE_ACCEL 600
#define STUDIO_ROUTE_YAW 1000
#define STUDIO_LIFT_RPM 60
#define STUDIO_LIFT_ACCEL 80
#define STUDIO_JAW_RATE 900
#define STUDIO_TURN_RATE 700
#define STUDIO_TRAY_RATE 700
#define STUDIO_POLL_MS 100
#define STUDIO_DWELL_MS 80
#define STUDIO_STABLE_MS 200
#define STUDIO_SPREAD 1
#define STUDIO_WORK_RPM 30
#define STUDIO_IMU_DELAY 55
#define STUDIO_ALGORITHM 0
#define STUDIO_FUSED 0
#define STUDIO_FEATURES 3
#define STUDIO_ARM_RADIUS 0
#define STUDIO_ARM_X 0
#define STUDIO_ARM_Y 0
#define STUDIO_TURN_SETTLE 200
#endif
#ifndef STUDIO_SERVO_SMOOTH
#define STUDIO_SERVO_SMOOTH 0
#endif

#define ADDR 5U
#define MASK 255U
typedef struct { volatile unsigned short head,tail; volatile unsigned char err; unsigned char b[256]; } Ring;
static Ring motor_rx, debug_rx;
static unsigned short servo[SERVO_COUNT],goal[SERVO_COUNT];
static unsigned char servo_on[SERVO_COUNT];
static const unsigned short smin[SERVO_COUNT]={1380,870,550,500},smax[SERVO_COUNT]={2260,2500,2350,2500};
static unsigned short jaw_low=1560,jaw_high=2260;
static const uint32_t channels[SERVO_COUNT]={TIM_CHANNEL_1,TIM_CHANNEL_2,TIM_CHANNEL_3,TIM_CHANNEL_4};
static unsigned long servo_tick;
static unsigned short servo_rate[SERVO_COUNT]={250,20,100,100},turn_low,turn_high;
static uint32_t servo_credit[SERVO_COUNT],servo_done_tick[SERVO_COUNT];
static unsigned char servo_smooth=STUDIO_SERVO_SMOOTH,servo_profile_active[SERVO_COUNT];
#ifndef STUDIO_COMPACT
#define STUDIO_COMPACT 1
#endif
static unsigned char compact_work=STUDIO_COMPACT;
static unsigned short servo_profile_goal[SERVO_COUNT],servo_profile_start[SERVO_COUNT];
static unsigned long servo_profile_tick[SERVO_COUNT],servo_profile_duration[SERVO_COUNT];
static unsigned short tune_turn_settle=STUDIO_TURN_SETTLE;
static unsigned char turn_physically_settled(unsigned long now);
static unsigned char packet[40],packet_len,pending,flags,flags_valid,pos_valid,config_valid;
static unsigned char pending_quiet,tx_quiet;
static unsigned long packet_tick,pending_tick,flag_tick,pos_tick;
static uint32_t pulses_per_rev=3200;
static unsigned short lift_rpm=5,lift_accel=10,arrival_window_ddeg;
static unsigned short tune_lift=STUDIO_LIFT_RPM,tune_accel=STUDIO_LIFT_ACCEL,tune_jaw=STUDIO_JAW_RATE,tune_turn=STUDIO_TURN_RATE,tune_tray=STUDIO_TRAY_RATE,tune_poll=STUDIO_POLL_MS,tune_dwell=STUDIO_DWELL_MS,tune_stable=STUDIO_STABLE_MS,tune_spread=STUDIO_SPREAD;
static uint64_t arrival_window_scaled;
static unsigned short app_window_pulses;
static int64_t stable_min_raw,stable_max_raw;
static unsigned long stable_tick;
static int64_t idle_min_raw,idle_max_raw;
static unsigned long idle_stable_tick,lift_skip_count;
static unsigned char idle_stable_count;
static int64_t position_raw,driver_target_raw,driver_error_raw;
static unsigned char driver_target_valid,driver_error_valid,target_confirmed;
static long position,origin,low_limit=-100,high_limit=100,target;
static unsigned char referenced,moving,arrival_count,motor_access,fault;
static char last_stop_reason[64]="none";
static unsigned long stop_count,last_stop_tick;
static long stop_position,stop_target,stop_origin;
static int64_t stop_position_raw,stop_driver_target_raw;
static unsigned char stop_flags,stop_target_verified,stop_arrival_count;
static unsigned long move_tick,move_timeout,poll_tick;
static long move_delta_pulses; /* v2.7.13: pulses commanded by the active FD */
static unsigned char poll_flags;
static unsigned char trace_enabled,trace_index,current_valid,pid_valid;
static unsigned short phase_current_ma;
static uint32_t position_kp,position_ki,position_kd;
static unsigned char probe_active,probe_index;
static unsigned long probe_tick;
static const unsigned char queries[7]={0x1F,0x24,0x42,0x33,0x37,0x3A,0x36};
static char line[96];
static unsigned char line_len,line_bad;
/* open, close, pick rotation, place rotation, low, high clearance, drop */
static const char *const names[7]={"open","close","pick","place","low","high","drop"};
static long taught[7];
static unsigned char taught_mask,task_active,task_step,task_issued;
static unsigned long task_tick;
/* User physical calibration 2026-10-27, raw-before-hold values:
 * pick +7863, clearance -41849, drop +2607; 3200 pulses/rev.
 * Round RAW DIFFERENCES, not the difference between rounded positions. */
#define GRAB_PICK 2427L
#define GRAB_CLEARANCE 0L
#define GRAB_DROP 2171L
/* Contact heights are independently mapped into the proven work datum.
 * Old records have no mapping and cannot silently activate old grab heights.
 * Common loaded-safe clearance stays0; rough/stack heights are unchanged. */
static long grab_height[2]={GRAB_PICK,GRAB_DROP};
static unsigned char grab_height_mask,grab_height_capture;
static const unsigned short grab_tray[3]={550,1450,2350};
static unsigned char grab_ready,grab_active,grab_step,grab_issued,grab_settling,grab_slot=2;
static unsigned long grab_settle_tick;
/* grab owns preparation internally: query -> datum/hold -> clearance -> PWM.
 * First grab after boot explicitly declares the physical start is clearance.
 * After a stop/fault, reference must be recovered; never silently re-zero low. */
static unsigned char grab_preparing,grab_prepare_slot,grab_recovered_hold;
static unsigned long grab_prepare_tick,grab_prepare_wait_tick;
static unsigned short grab_prepare_delay;
static unsigned long grab_hold_release_tick;
static unsigned char grab_hold_release_done;
/* tray pickup, rough placement, placement on existing block, safe swing floor */
static const char *const point_names[4]={"tray","rough","stack","swing"};
static long point_height[4];
/* The task clearance is a physical pose; coordinate rebasing must not move it. */
static long clearance_height;
static long camera_height[2];
static unsigned char camera_mask;
static const char *const camera_names[2]={"view","qr"};
static unsigned char datum_capture;
static long datum_capture_local;
static unsigned char point_mask,point_capture,motion_mode=1,requested_job;
static unsigned char work_active,work_kind,work_step,work_issued,work_settling;
static unsigned long work_settle_tick;
static long cal_saved_origin=-2411;
static unsigned char cal_stored;
/* Volatile: upgrade handoff may request/observe a save through SWD. */
static volatile unsigned char cal_save_pending,cal_save_result;
static unsigned char measure_keep_ref;
static long measure_keep_origin,measure_keep_low,measure_keep_high;
static long measure_hold_origin;
static unsigned char measure_frame_known;
/* Only the enable/hold capture may follow a small firmware-selected target.
 * Actual FD move-target checking is unchanged. */
#define HOLD_CAPTURE_MAX 100L
#define HOLD_TRACK_WINDOW 20L
typedef struct {uint32_t schema;int32_t saved_origin;int32_t heights[4];uint32_t mask,mode;} CalRecord;
typedef struct {CalRecord base;int32_t grab_heights[2];uint32_t grab_mask;} CalRecordV10;
typedef struct {CalRecordV10 prior;int32_t clearance;} CalRecordV11;
typedef struct {CalRecordV11 prior;int32_t camera[2];uint32_t camera_mask;} CalRecordV12;
enum { MEAS_OFF,MEAS_DISABLING,MEAS_FREE,MEAS_STILL,MEAS_ZERO_ACK,
       MEAS_ZERO_VERIFY,MEAS_ENABLING,MEAS_ERROR };
static unsigned char measure_state,measure_ack,measure_capture,measure_mask;
static unsigned char measure_verify,measure_samples,measure_rebased;
static unsigned long measure_tick,measure_poll_tick,measure_still_tick;
static int64_t measure_min_raw,measure_max_raw,measure_shift_raw,measure_marks[3];
static const char *const measure_names[3]={"pick","clearance","drop"};

static void say(const char *fmt,...)
{
    char text[192]; int n; va_list args;
    va_start(args,fmt); n=vsnprintf(text,sizeof(text),fmt,args); va_end(args);
    if(n<0)return;
    if(n>=(int)sizeof(text))n=sizeof(text)-1;
#ifdef ARM_NAV_BUILD
    nav_log(text);
#else
    HAL_UART_Transmit(&huart4,(uint8_t*)text,(uint16_t)n,25);
#endif
}
static void irq_read(USART_TypeDef *uart,Ring *r)
{
    uint32_t sr=uart->SR;
    if(sr&(USART_SR_RXNE|USART_SR_ORE|USART_SR_FE|USART_SR_NE)) {
        unsigned char b=(unsigned char)uart->DR;
        unsigned short next=(r->head+1U)&MASK;
        if((sr&(USART_SR_ORE|USART_SR_FE|USART_SR_NE))||next==r->tail)r->err=1;
        else if(sr&USART_SR_RXNE){r->b[r->head]=b;r->head=next;}
    }
}
void ArmTask_MotorIRQ(void){irq_read(huart1.Instance,&motor_rx);}
void ArmTask_DebugIRQ(void){irq_read(huart4.Instance,&debug_rx);}
static int tx(const unsigned char *b,unsigned short n)
{
    char hex[80]; unsigned int j;
#ifdef ARM_NAV_BUILD
    HAL_StatusTypeDef result=nav_bus_lift_submit(b,n)?HAL_OK:HAL_BUSY;
#else
    HAL_StatusTypeDef result=HAL_UART_Transmit(&huart1,(uint8_t*)b,n,20);
#endif
    for(j=0;j<n;j++)sprintf(hex+j*3,"%02X ",b[j]); hex[n*3]=0;
    if(!tx_quiet||result!=HAL_OK)say("[TX1] %s status=%u\r\n",hex,(unsigned int)result);
    return result==HAL_OK;
}
static int servo_busy(void)
{
    int i;for(i=0;i<SERVO_COUNT;i++)if(servo_on[i]&&servo[i]!=goal[i])return 1;
    return 0;
}
static void halt(const char *why)
{
    static const unsigned char stop[]={ADDR,0xFE,0x98,0,0x6B};
    /* Freeze evidence BEFORE FE may relatch the driver's holding target. */
    stop_position=position;stop_target=target;stop_origin=origin;
    stop_position_raw=position_raw;stop_driver_target_raw=driver_target_raw;
    stop_flags=flags;stop_target_verified=target_confirmed;stop_arrival_count=arrival_count;
    datum_capture=0;
    strncpy(last_stop_reason,why,sizeof(last_stop_reason)-1);last_stop_reason[sizeof(last_stop_reason)-1]=0;
    stop_count++;last_stop_tick=HAL_GetTick();
    if((motor_access||moving)&&!measure_state)tx(stop,sizeof(stop));
    if(measure_state){measure_state=MEAS_ERROR;measure_capture=0;}
    int i;for(i=0;i<SERVO_COUNT;i++)goal[i]=servo[i];
    task_active=0;task_issued=0;grab_active=grab_ready=grab_preparing=work_active=0;moving=0;pending=0;probe_active=0;point_capture=grab_height_capture=0;
    referenced=0;taught_mask=0;pos_valid=0;flags_valid=0;fault=1;target_confirmed=0;idle_stable_count=0;
    say("[STOP] %s; servos hold current PWM; lift reference/teach invalid\r\n",why);
}
static int query(unsigned char code,unsigned char quiet)
{
    unsigned char b[4]={ADDR,code,0x6B,0x6B};
    int ok;
    if(pending)return 0;
    if(code==0x42)b[2]=0x6C;
    packet_len=0;pending=code;pending_quiet=quiet;pending_tick=HAL_GetTick();
    tx_quiet=quiet;ok=tx(b,code==0x42?4:3);tx_quiet=0;
    if(!ok){pending=0;halt("UART1 TX failed");return 0;}
    return 1;
}
static int fresh(void)
{
    unsigned long now=HAL_GetTick();
    return config_valid&&pos_valid&&flags_valid&&now-pos_tick<1000UL&&now-flag_tick<1000UL;
}
static int lift_ready(void){return !measure_state&&referenced&&fresh()&&(flags&1U)&&!(flags&12U)&&!fault;}
static void motion_apply(void)
{
    lift_rpm=motion_mode==1?tune_lift:(motion_mode==2?20:30);
    lift_accel=motion_mode==1?tune_accel:20;app_window_pulses=20;
    servo_rate[0]=motion_mode==1?tune_jaw:(motion_mode==2?200:400);
    servo_rate[1]=motion_mode==1?tune_turn:(motion_mode==2?200:300);
    servo_rate[2]=motion_mode==1?tune_tray:(motion_mode==2?200:300);
}
static long profile_high(void)
{
    long high=clearance_height>0?clearance_height:0;int i;
    for(i=0;i<2;i++)if((grab_height_mask&(1U<<i))&&grab_height[i]>high)high=grab_height[i];
    for(i=0;i<3;i++)if((point_mask&(1U<<i))&&point_height[i]>high)high=point_height[i];
    for(i=0;i<2;i++)if((camera_mask&(1U<<i))&&camera_height[i]>high)high=camera_height[i];
    return high+30>6000?6000:high+30;
}
static long profile_low(void)
{
    long low=clearance_height<0?clearance_height:0;int i;
    for(i=0;i<2;i++)if((grab_height_mask&(1U<<i))&&grab_height[i]<low)low=grab_height[i];
    for(i=0;i<3;i++)if((point_mask&(1U<<i))&&point_height[i]<low)low=point_height[i];
    for(i=0;i<2;i++)if((camera_mask&(1U<<i))&&camera_height[i]<low)low=camera_height[i];
    low-=100;return low < -6000 ? -6000 : low;
}
static void cal_load(void)
{
    CalRecord c;CalRecordV10 n;CalRecordV11 latest;CalRecordV12 v12;long clear=0;int i,extended=0;
    if(Board_CalRead(&v12,sizeof(v12))&&v12.prior.prior.base.schema==12){
        if(v12.camera_mask>3)return;
        for(i=0;i<2;i++)if(v12.camera[i]<-5900||v12.camera[i]>5900)return;
        camera_mask=(unsigned char)v12.camera_mask;
        for(i=0;i<2;i++)camera_height[i]=v12.camera[i];
        latest=v12.prior;n=latest.prior;c=n.base;clear=latest.clearance;extended=1;
    }else if(Board_CalRead(&latest,sizeof(latest))&&latest.prior.base.schema==11){
        n=latest.prior;c=n.base;clear=latest.clearance;extended=1;
    }else if(Board_CalRead(&n,sizeof(n))&&n.base.schema==10){
        c=n.base;extended=1;
    }else if(!Board_CalRead(&c,sizeof(c))||c.schema!=9)return;
    if(clear < -5900||clear > 5900)return;
    if(extended&&(n.grab_mask>3||n.grab_heights[0]<-5900||n.grab_heights[0]>5900||n.grab_heights[1]<-5900||n.grab_heights[1]>5900))return;
    if(c.mask>15||c.mode>2)return;
    for(i=0;i<4;i++)if(c.heights[i]<-6000||c.heights[i]>6000)return;
    if((c.mask&8)&&c.heights[3]<clear)return;
    for(i=0;i<4;i++)point_height[i]=c.heights[i];
    point_mask=(unsigned char)c.mask;motion_mode=(unsigned char)c.mode;
    grab_height[0]=extended?n.grab_heights[0]:GRAB_PICK;
    grab_height[1]=extended?n.grab_heights[1]:GRAB_DROP;
    grab_height_mask=extended?(unsigned char)n.grab_mask:0;
    clearance_height=clear;cal_saved_origin=c.saved_origin;cal_stored=1;
}
static int cal_save_now(void)
{
    CalRecordV10 c;CalRecordV11 record;CalRecordV12 v12;int i,ok;
    memset(&c,0,sizeof(c));c.base.schema=12;
    c.base.saved_origin=referenced?origin:
        ((measure_state==MEAS_FREE&&measure_keep_ref)?measure_keep_origin:cal_saved_origin);
    for(i=0;i<4;i++)c.base.heights[i]=(int32_t)point_height[i];c.base.mask=point_mask;c.base.mode=motion_mode;
    c.grab_heights[0]=(int32_t)grab_height[0];c.grab_heights[1]=(int32_t)grab_height[1];c.grab_mask=grab_height_mask;
    record.prior=c;record.clearance=(int32_t)clearance_height;
    v12.prior=record;v12.camera_mask=camera_mask;for(i=0;i<2;i++)v12.camera[i]=(int32_t)camera_height[i];
    ok=Board_CalWrite(&v12,sizeof(v12));
    if(ok){cal_saved_origin=c.base.saved_origin;cal_stored=1;say("[SAVED] work/grab heights/mode/datum verified in Flash; driver reset still requires physical reference\r\n");}
    else say("[E] calibration Flash save failed\r\n");return ok;
}
static void cal_save_poll(void)
{
    if(!cal_save_pending||moving||task_active||grab_active||grab_preparing||work_active||measure_state||fault||
       pending||probe_active||servo_busy()||!referenced)return;
    cal_save_pending=0;cal_save_result=cal_save_now()?1:2;
}
static void points_print(void)
{
    int i;
    say("[DATUM] active=%u origin=%ld saved_origin=%ld local=%ld; saved counter requires SAME Emm session\r\n",referenced,origin,cal_saved_origin,
        referenced?position-origin:(measure_frame_known?position-measure_hold_origin:(measure_keep_ref?position-measure_keep_origin:0)));
    for(i=0;i<4;i++)say("[POINT] %s=%ld valid=%u local pulse\r\n",point_names[i],point_height[i],!!(point_mask&(1U<<i)));
    for(i=0;i<2;i++)say("[CAMERA_HEIGHT] %s=%ld valid=%u; not used by chassis route\r\n",camera_names[i],camera_height[i],!!(camera_mask&(1U<<i)));
    say("[PERFORMANCE] mode=%s rpm=%u accel=%u rate=%u,%u,%u; swing=%ld; flash_saved=%u\r\n",
        motion_mode==1?"fast":(motion_mode==2?"slow":"normal"),lift_rpm,lift_accel,servo_rate[0],servo_rate[1],servo_rate[2],
        point_mask&8?point_height[3]:0,cal_stored);
    say("[GRAB_HEIGHTS] pick=%ld drop=%ld valid=%u%u; clearance=%ld; overlap=%s\r\n",grab_height[0],grab_height[1],!!(grab_height_mask&1),!!(grab_height_mask&2),clearance_height,
        (point_mask&8)&&point_height[3]>clearance_height?"on":"off");
}
static void grab_height_position(void)
{
    unsigned char kind=grab_height_capture;long local,pick,drop;
    if(!kind)return;grab_height_capture=0;
    if(fault||(!referenced&&!(measure_state==MEAS_FREE&&measure_keep_ref))){say("[E] grab alignment: no retained datum\r\n");return;}
    local=position-(referenced?origin:measure_keep_origin);pick=grab_height[0];drop=grab_height[1];
    if(kind==1){pick=local;drop=local-(GRAB_PICK-GRAB_DROP);}
    else if(kind==2){drop=local;pick=local+(GRAB_PICK-GRAB_DROP);}
    else if(kind==3)pick=local;else drop=local;
    if(pick<clearance_height||drop<clearance_height||pick>5900||drop>5900||pick<-5900||drop<-5900){say("[E] grab contacts must be below clearance and within coordinate bounds; no data changed\r\n");return;}
    grab_height[0]=pick;grab_height[1]=drop;
    if(kind<=2)grab_height_mask=3;else grab_height_mask|=(kind==3?1:2);
    cal_stored=0;grab_ready=0;if(referenced)high_limit=profile_high();
    say("[GRAB_ALIGNED] pick=%ld drop=%ld valid=%u%u; current origin=%ld; work heights NOT changed; save to persist\r\n",
        pick,drop,!!(grab_height_mask&1),!!(grab_height_mask&2),referenced?origin:measure_keep_origin);
}
static void point_position(void)
{
    int i;long local;
    if(!point_capture)return;i=point_capture-1;point_capture=0;
    if(fault||(!referenced&&!(measure_state==MEAS_FREE&&measure_keep_ref))){say("[E] point: no retained datum\r\n");return;}
    local=position-(referenced?origin:measure_keep_origin);
    if(i==7||i==8){
        if(local<-5900||local>5900){say("[E] camera height outside bounds\r\n");return;}
        i-=7;camera_height[i]=local;camera_mask|=(1U<<i);cal_stored=0;
        if(referenced){low_limit=profile_low();high_limit=profile_high();}
        say("[CAMERA_HEIGHT_SAVED] %s=%ld; save to persist; no automatic move\r\n",camera_names[i],local);return;
    }
    if(i==6){
        if(local<-5900||local>5900){say("[E] clearance outside coordinate bounds\r\n");return;}
        for(i=0;i<2;i++)if((grab_height_mask&(1U<<i))&&grab_height[i]<local){say("[E] clearance must be above contacts\r\n");return;}
        for(i=0;i<3;i++)if((point_mask&(1U<<i))&&point_height[i]<local){say("[E] clearance must be above contacts\r\n");return;}
        clearance_height=point_height[3]=local;point_mask|=8;cal_stored=0;grab_ready=0;
        if(referenced){low_limit=profile_low();high_limit=profile_high();}
        say("[CLEARANCE_SAVED] clearance=%ld; descent during turn OFF; save to persist\r\n",local);return;
    }
    if(local<-5900||local>5900||(i==3&&local<clearance_height)){say("[E] point height outside local bounds\r\n");return;}
    point_height[i]=local;point_mask|=(1U<<i);cal_stored=0;
    if(referenced)high_limit=profile_high();
    say("[POINT_SAVED] %s=%ld local pulse; datum unchanged; save to persist\r\n",point_names[i],local);
}
static void datum_position(void)
{
    unsigned char mode=datum_capture;long old_origin,shift=0,new_clear,new_points[4],new_grab[2],new_camera[2];int i;
    if(!mode)return;datum_capture=0;
    if(fault||moving||task_active||grab_active||grab_preparing||work_active||servo_busy()||!fresh()||
       (flags&12U)||idle_stable_count<3||HAL_GetTick()-idle_stable_tick<200UL){
        say("[E] datum capture needs healthy stable idle position; no data changed\r\n");return;
    }
    if(mode==1){
        if(!referenced&&!(measure_state==MEAS_FREE&&measure_keep_ref)){
            say("[E] prior frame unknown; use datum at KNOWN_POSE or datum new fresh\r\n");return;
        }
        old_origin=referenced?origin:measure_keep_origin;shift=position-old_origin;
        new_clear=clearance_height-shift;
        if(new_clear<-5900||new_clear>5900)goto range;
        for(i=0;i<4;i++){
            new_points[i]=(point_mask&(1U<<i))?point_height[i]-shift:0;
            if(new_points[i]<-5900||new_points[i]>5900)goto range;
        }
        for(i=0;i<2;i++){
            new_grab[i]=(grab_height_mask&(1U<<i))?grab_height[i]-shift:0;
            if(new_grab[i]<-5900||new_grab[i]>5900)goto range;
        }
        for(i=0;i<2;i++){
            new_camera[i]=(camera_mask&(1U<<i))?camera_height[i]-shift:0;
            if(new_camera[i]<-5900||new_camera[i]>5900)goto range;
        }
        /* Commit only after every translated coordinate passed its bounds. */
        clearance_height=new_clear;for(i=0;i<2;i++)camera_height[i]=new_camera[i];
        for(i=0;i<4;i++)point_height[i]=new_points[i];
        for(i=0;i<2;i++)grab_height[i]=new_grab[i];
    }else if(mode==2){
        clearance_height=0;point_mask=grab_height_mask=camera_mask=0;memset(camera_height,0,sizeof camera_height);
        memset(point_height,0,sizeof(point_height));memset(grab_height,0,sizeof(grab_height));
    }
    origin=position-(mode==3?datum_capture_local:0);
    low_limit=profile_low();high_limit=profile_high();
    if(low_limit>-100)low_limit=-100;if(high_limit<100)high_limit=100;
    if(measure_state==MEAS_FREE){
        referenced=0;measure_keep_ref=1;measure_keep_origin=origin;
        measure_keep_low=low_limit;measure_keep_high=high_limit;
    }else referenced=1;
    measure_frame_known=0;target_confirmed=0;taught_mask=0;grab_ready=0;cal_stored=0;
    say("[DATUM_SET] mode=%s origin=%ld local=%ld clearance=%ld; no driver zero/enable/move; save to persist\r\n",
        mode==1?"REBASE_KEEP":(mode==2?"FRESH_RETEACH":"KNOWN_POSE"),origin,position-origin,clearance_height);
    if(mode==1)say("[DATUM_SHIFT] subtract%ld from saved local heights; physical targets preserved\r\n",shift);
    if(mode==2)say("[HEIGHTS_CLEARED] teach grabpick/grabdrop/tray/rough/stack; current safe zero is clearance\r\n");
    return;
range:say("[E] rebased heights exceed coordinate bounds; no data changed\r\n");
}
static long raw_pulses(int64_t raw)
{
    int64_t value=raw*(int64_t)pulses_per_rev;
    return (long)(value<0?-((-value+32768)/65536):(value+32768)/65536);
}
static int measure_disabled(void)
{
    return flags_valid&&HAL_GetTick()-flag_tick<1000UL&&!(flags&1U);
}
static void measure_fail(const char *why)
{
    measure_state=MEAS_ERROR;measure_capture=0;point_capture=grab_height_capture=datum_capture=0;referenced=0;taught_mask=0;grab_active=grab_ready=grab_preparing=work_active=0;
    target_confirmed=0;fault=1;
    say("[MEASURE_ERROR] %s; no automatic torque change; support lift; motion locked\r\n",why);
}
static void measure_list(void)
{
    int i;
    say("[MARKS] RAM only; frame=%s; reboot loses marks; original session raw retained\r\n",measure_rebased?"LOCAL_AFTER_HOLD":"DRIVER_BEFORE_HOLD");
    for(i=0;i<3;i++)if(measure_mask&(1U<<i)){
        int64_t raw=measure_marks[i];
        say("[MARK] %s session_raw=%c%lu value=%ld pulse\r\n",measure_names[i],raw<0?'-':'+',
            (unsigned long)(raw<0?-raw:raw),raw_pulses(raw-(measure_rebased?measure_shift_raw:0))-(measure_rebased?origin:0));
    }
}
static void measure_position(void)
{
    int64_t raw=position_raw,mag=raw<0?-raw:raw;
    uint64_t milli=(uint64_t)mag*360000U/65536U;
    say("[ENC] raw=%c%lu angle=%c%lu.%03lu deg; driver=%ld pulse\r\n",raw<0?'-':'+',(unsigned long)mag,
        raw<0?'-':'+',(unsigned long)(milli/1000U),(unsigned long)(milli%1000U),position);
    if(measure_capture){
        unsigned char i=(unsigned char)(measure_capture-1);measure_capture=0;
        if(measure_state!=MEAS_FREE||!config_valid||!measure_disabled()){
            say("[E] mark discarded: confirmed disabled feedback required\r\n");return;
        }
        measure_marks[i]=position_raw;measure_mask|=(1U<<i);
        say("[MARK_SAVED] %s driver=%ld pulse; captured from this reply; RAM only\r\n",measure_names[i],position);
    }
}
static void measure_poll(unsigned long now)
{
    unsigned char q=0;
    if(!measure_state||measure_state==MEAS_FREE||measure_state==MEAS_ERROR)return;
    if(now-measure_tick>=(measure_state==MEAS_ENABLING?5000UL:4000UL)){
        measure_fail(measure_state==MEAS_ENABLING?"hold target/position did not settle":"transition timeout");return;
    }
    if(pending)return;
    if(measure_state==MEAS_DISABLING){
        if(measure_ack&&measure_disabled()){
            measure_state=MEAS_FREE;fault=0;
            say("[MEASURE_READY] enabled=0 verified; support lift, move by hand; r / measure mark NAME\r\n");return;
        }
        q=0x3A;
    }else if(measure_state==MEAS_STILL){
        if(!measure_disabled()){q=0x3A;}
        else if(measure_samples>=3&&now-measure_still_tick>=400UL){
            static const unsigned char clear[]={ADDR,0x0A,0x6D,0x6B};
            if(measure_keep_ref&&(position-measure_keep_origin < -6000||position-measure_keep_origin > 6000)){
                measure_fail("manual position exceeds retained datum range");return;
            }
            measure_shift_raw=position_raw;measure_rebased=0;measure_ack=0;measure_verify=0;measure_frame_known=0;
            measure_state=MEAS_ZERO_ACK;measure_tick=now;
            pos_valid=driver_target_valid=driver_error_valid=0;
            say("[MEASURE] clearing driver coordinates while DISABLED; keep lift supported and still\r\n");
            if(!tx(clear,sizeof(clear)))measure_fail("driver zero TX failed");return;
        }else q=(measure_verify++&1U)?0x3A:0x36;
    }else if(measure_state==MEAS_ZERO_ACK){
        if(!measure_ack)return;
        measure_state=MEAS_ZERO_VERIFY;measure_verify=0;measure_tick=now;
    }
    if(measure_state==MEAS_ZERO_VERIFY){
        static const unsigned char check[]={0x36,0x33,0x37,0x3A};
        if(measure_verify==15){
            static const unsigned char en[]={ADDR,0xF3,0xAB,1,0,0x6B};
            int64_t window=(65536U+pulses_per_rev-1U)/pulses_per_rev;
            if(!fresh()||!measure_disabled()||(flags&12U)||!driver_target_valid||!driver_error_valid||driver_target_raw!=0||
               position_raw < -window||position_raw > window||driver_error_raw < -window||driver_error_raw > window){
                measure_fail("zero/target/error/disabled verification failed");return;
            }
            measure_rebased=1;measure_state=MEAS_ENABLING;measure_ack=0;measure_tick=now;
            measure_hold_origin=measure_keep_ref?measure_keep_origin-raw_pulses(measure_shift_raw):0;
            measure_frame_known=1;origin=measure_hold_origin;
            measure_verify=0;
            flags_valid=pos_valid=driver_target_valid=driver_error_valid=0;idle_stable_count=0;
            if(!tx(en,sizeof(en)))measure_fail("enable TX failed");return;
        }
        {int j;for(j=0;j<4;j++)if(!(measure_verify&(1U<<j))){q=check[j];break;}}
    }else if(measure_state==MEAS_ENABLING){
        static const unsigned char check_hold[]={0x3A,0x36,0x33,0x37};
        if(flags_valid&&(flags&12U)){measure_fail("driver stall/protection during hold");return;}
        if(measure_ack&&fresh()&&(flags&1U)&&driver_target_valid&&driver_error_valid){
            int64_t excursion=(int64_t)HOLD_CAPTURE_MAX*65536;
            int64_t window=(int64_t)HOLD_TRACK_WINDOW*65536;
            int64_t p=position_raw*(int64_t)pulses_per_rev;
            int64_t t=driver_target_raw*(int64_t)pulses_per_rev;
            int64_t e=p-t,reported=driver_error_raw*(int64_t)pulses_per_rev;
            if(p < -excursion||p > excursion||t < -excursion||t > excursion){measure_fail("hold capture shifted over100 pulses");return;}
            if(e < -window||e > window||reported < -window||reported > window||
               idle_stable_count<3||now-idle_stable_tick<200UL)goto hold_query;
            measure_state=MEAS_OFF;origin=measure_hold_origin;
            referenced=1;fault=0;grab_ready=0;
            low_limit=measure_keep_ref?measure_keep_low:-100;high_limit=measure_keep_ref?measure_keep_high:100;
            if(measure_keep_ref){
                long here=position-origin;if(here<low_limit)low_limit=here-20;if(here>high_limit)high_limit=here+20;
                if(point_mask&&profile_high()>high_limit)high_limit=profile_high();
                if(low_limit<-6000)low_limit=-6000;if(high_limit>6000)high_limit=6000;
            }
            target_confirmed=0;
            idle_stable_count=0;
            say("[MEASURE_HOLD] enabled=1 verified; datum_%s origin=%ld local=%ld; no travel commanded\r\n",
                measure_keep_ref?"PRESERVED":"NEW",origin,position-origin);measure_keep_ref=0;
            measure_list();return;
        }
hold_query:q=check_hold[measure_verify%4U];
    }
    if(q&&!pending&&now-measure_poll_tick>=100UL){
        measure_poll_tick=now;if(measure_state==MEAS_ENABLING)measure_verify++;query(q,0);
    }
}
static int within_arrival_window(void)
{
    /* Only a freshly read, independently command-checked driver target may
     * center the arrival window. Mixing the integer command and quantized
     * encoder target rejected the real 891 target / 945 position trace. */
    int64_t error=(position_raw-driver_target_raw)*(int64_t)pulses_per_rev;
    if(!target_confirmed)return 0;
    int64_t window=app_window_pulses?(int64_t)app_window_pulses*65536:(int64_t)arrival_window_scaled;
    return error>=-window&&error<=window;
}
static void arrival_sample(void)
{
    int64_t spread;
    if(!target_confirmed||!within_arrival_window()){arrival_count=0;return;}
    if(!app_window_pulses){if(arrival_count<255)arrival_count++;return;}
    if(!arrival_count){
        stable_min_raw=stable_max_raw=position_raw;stable_tick=HAL_GetTick();arrival_count=1;return;
    }
    if(position_raw<stable_min_raw)stable_min_raw=position_raw;
    if(position_raw>stable_max_raw)stable_max_raw=position_raw;
    spread=(stable_max_raw-stable_min_raw)*(int64_t)pulses_per_rev;
    if(spread>(int64_t)65536*tune_spread){
        stable_min_raw=stable_max_raw=position_raw;stable_tick=HAL_GetTick();arrival_count=1;
    }else if(arrival_count<255)arrival_count++;
}
static int move_to(long local)
{
    unsigned char b[13]={ADDR,0xFD,0,0,0,10,0,0,0,0,1,0,0x6B};
    long delta,destination; uint32_t count,distance; uint64_t timeout;
    if(!lift_ready()||moving||pending||probe_active||local<low_limit||local>high_limit)return 0;
    destination=origin+local;delta=destination-position;
    /* v2.7.13 (field bug, 2026-11-05 station=4 raw2): a commanded delta of only a
     * few pulses must not be sent as an FD absolute-position command. The driver
     * then answers 0x33 with the PREVIOUS target (e.g. commanded -2306, read back
     * -2300.7) and the strict check below halts. When the measured encoder is
     * already inside the destination application window, the pose is physically
     * complete, so report ALREADY_STABLE without requiring the 3-sample/200 ms
     * idle-stability counter. Larger deltas keep the original path. */
    if(app_window_pulses&&labs(delta)<=(long)app_window_pulses&&pos_valid){
        int64_t e=position_raw*(int64_t)pulses_per_rev-(int64_t)destination*65536;
        int64_t w=(int64_t)app_window_pulses*65536;
        if(e>=-w&&e<=w){
            target=destination;target_confirmed=driver_target_valid=0;arrival_count=0;lift_skip_count++;
            say("[LIFT_SKIP_SMALL] local=%ld target_local=%ld driver_abs=%ld delta=%ld window=%u; measured position inside app window, NO FD sent\r\n",
                local,local,destination,delta,(unsigned)app_window_pulses);
            return 1;
        }
    }
    /* APP20 already accepts a stable measured pose in this window. In the
     * captured trace, repeating the completed target returned near actual
     * position instead of the nominal pulse target. Avoid redundant commands
     * when fresh measurements already prove pose completion.
     * Every genuinely issued FD still has the original tight target check. */
    if(app_window_pulses&&idle_stable_count>=3&&HAL_GetTick()-idle_stable_tick>=200UL){
        int64_t e=position_raw*(int64_t)pulses_per_rev-(int64_t)destination*65536;
        int64_t w=(int64_t)app_window_pulses*65536;
        if(e>=-w&&e<=w){
            target=destination;target_confirmed=driver_target_valid=0;arrival_count=0;lift_skip_count++;
            say("[LIFT_DONE] local=%ld target_local=%ld acceptance=ALREADY_STABLE; NO FD sent, no driver-target claim\r\n",position-origin,local);
            return 1;
        }
    }
    /* Absolute DRIVER target equals operator LOCAL origin + requested local.
     * A relative driver command can use a different previous-target baseline. */
    count=(uint32_t)(destination<0?-destination:destination);b[2]=destination<0?1:0;
    b[3]=(unsigned char)(lift_rpm>>8);b[4]=(unsigned char)lift_rpm;
    b[5]=(unsigned char)lift_accel;
    b[6]=(unsigned char)(count>>24);b[7]=(unsigned char)(count>>16);
    b[8]=(unsigned char)(count>>8);b[9]=(unsigned char)count;
    if(!tx(b,sizeof(b))){halt("position TX failed");return 0;}
    move_delta_pulses=delta;
    target=destination; moving=1;motor_access=1;arrival_count=0;target_confirmed=0;
    move_tick=HAL_GetTick();poll_tick=move_tick;poll_flags=0;trace_index=0;idle_stable_count=0;
    distance=(uint32_t)(delta<0?-delta:delta);
    timeout=3000U+((uint64_t)distance*60000U)/(lift_rpm*pulses_per_rev);
    move_timeout=(unsigned long)(timeout>30000U?30000U:timeout);
    say("[MOVE] local=%ld driver_abs=%ld delta=%ld RPM=%u timeout=%lu ms; target readback required\r\n",local,target,delta,lift_rpm,move_timeout);
    return 1;
}
static void lift_poll(unsigned long now)
{
    if(!moving)return;
    if(now-move_tick>=move_timeout){halt("lift motion timeout");return;}
    if(target_confirmed&&arrival_count>=3&&fresh()&&(flags&1)&&!(flags&12)&&flag_tick-move_tick<0x80000000UL&&
       (app_window_pulses?(now-stable_tick>=(unsigned long)tune_stable):!!(flags&2))){
        moving=0;say("[LIFT_DONE] local=%ld target_local=%ld acceptance=%s driver_arrived=%u; driver target verified\r\n",
            position-origin,target-origin,app_window_pulses?"APP_STABLE":"STRICT",!!(flags&2));
    }
}
static void servo_poll(unsigned long now)
{
    unsigned long elapsed=now-servo_tick;int i;
    if(elapsed<20UL)return;
    servo_tick=now;if(elapsed>100UL)elapsed=100UL;
    for(i=0;i<SERVO_COUNT;i++){
        if(servo_on[i]&&servo[i]!=goal[i]){
            unsigned short diff=servo[i]<goal[i]?goal[i]-servo[i]:servo[i]-goal[i];
            unsigned long step;
            if(servo_smooth==1||(servo_smooth>=2&&(i==1||(servo_smooth==3&&i==2)))){
                if(!servo_profile_active[i]||servo_profile_goal[i]!=goal[i]){
                    servo_profile_active[i]=1;servo_profile_goal[i]=goal[i];servo_profile_start[i]=servo[i];servo_profile_tick[i]=now;
                    servo_profile_duration[i]=(unsigned long)(1875.0f*diff/servo_rate[i]+.999f);if(servo_profile_duration[i]<20)servo_profile_duration[i]=20;
                }
                float q=(float)(now-servo_profile_tick[i])/servo_profile_duration[i];if(q>1)q=1;
                float h=q*q*q*(10+q*(-15+6*q));
                servo[i]=(unsigned short)(servo_profile_start[i]+((float)goal[i]-servo_profile_start[i])*h+.5f);
            }else{
            servo_credit[i]+=(uint32_t)servo_rate[i]*elapsed;
            step=servo_credit[i]/1000U;servo_credit[i]%=1000U;
            if(step>diff)step=diff;
            if(!step)continue;
            if(servo[i]<goal[i])servo[i]+=(unsigned short)step;else servo[i]-=(unsigned short)step;
            }
            __HAL_TIM_SET_COMPARE(&htim3,channels[i],servo[i]);
            if(servo[i]==goal[i]){servo_done_tick[i]=now;say("[SERVO_DONE] %d %u us; actual angle not sensed\r\n",i+1,servo[i]);}
        }else {servo_credit[i]=0;servo_profile_active[i]=0;}
    }
}
static void accept_packet(void)
{
    unsigned char code=packet[1]; char hex[121]; unsigned int j;
    unsigned char quiet=pending==code&&pending_quiet;
    for(j=0;j<packet_len;j++)sprintf(hex+j*3,"%02X ",packet[j]);hex[packet_len*3]=0;
    if(!quiet)say("[RX1] %s\r\n",hex);
    if(packet[packet_len-1]!=0x6B){say("[E] invalid frame tail\r\n");return;}
    if(code==0x00){pending=0;halt("driver rejected command");return;}
    if((code==0xFD||code==0xF3||code==0x0A)&&packet_len==4){
        if(packet[2]==0xE2||packet[2]==0xEE)halt("driver command rejected/condition unmet");
        else {
            if(packet[2]==2&&((code==0xF3&&(measure_state==MEAS_DISABLING||measure_state==MEAS_ENABLING))||
               (code==0x0A&&measure_state==MEAS_ZERO_ACK)))measure_ack=1;
            say("[ACK] cmd=%02X state=%02X; ACK is not arrival\r\n",code,packet[2]);
        }
        return;
    }
    if(pending!=code){say("[RX1] unsolicited frame; ignored for freshness\r\n");return;}
    pending=0;
    if(code==0x42&&packet_len==33&&packet[2]==33){
        uint32_t steps=packet[4]==0x19?200U:(packet[4]==0x32?400U:0U);
        if(!steps){config_valid=0;say("[E] unsupported motor type\r\n");return;}
        pulses_per_rev=steps*(packet[9]?packet[9]:256U);config_valid=1;
        arrival_window_ddeg=((unsigned short)packet[30]<<8)|packet[31];
        arrival_window_scaled=((uint64_t)arrival_window_ddeg*65536U*pulses_per_rev+3599U)/3600U+pulses_per_rev;
        say("[CONF] pulse/rev=%lu serial_mode=%u addr=%u checksum=%u\r\n",
            (unsigned long)pulses_per_rev,packet[6],packet[20],packet[21]);
        if(packet[6]!=2||packet[20]!=ADDR||packet[21]!=0){config_valid=0;say("[E] incompatible driver configuration\r\n");}
        if(!arrival_window_ddeg||arrival_window_ddeg>5||arrival_window_scaled>8ULL*65536U){
            config_valid=0;say("[E] driver arrival window outside bounded bench policy\r\n");
        }
        say("[WINDOW] ddeg=%u tol_pulses_ceiling=%lu; raw encoder comparison\r\n",arrival_window_ddeg,(unsigned long)((arrival_window_scaled+65535U)/65536U));
    }else if((code==0x33||code==0x36||code==0x37)&&packet_len==8&&packet[2]<=1){
        uint32_t raw=((uint32_t)packet[3]<<24)|((uint32_t)packet[4]<<16)|((uint32_t)packet[5]<<8)|packet[6];
        uint64_t mag=((uint64_t)raw*pulses_per_rev+32768U)>>16;
        if(mag>1000000000U){halt("position numeric range exceeded");return;}
        int64_t signed_raw=packet[2]?-(int64_t)raw:(int64_t)raw;
        long pulses=packet[2]?-(long)mag:(long)mag;
        if(code==0x33){
            driver_target_raw=signed_raw;driver_target_valid=1;
            if(measure_state==MEAS_ZERO_VERIFY)measure_verify|=2;
            if(moving){
                int64_t error=driver_target_raw*(int64_t)pulses_per_rev-(int64_t)target*65536;
                /* Measured command41 -> target850 counts =41.504 pulses.
                 * Half an input pulse + one encoder count covers representation
                 * quantization; the final arrival window remains independent. */
                int64_t quantization=32768+(int64_t)pulses_per_rev;
                if(error<-quantization||error>quantization){
                    /* v2.7.13: a SMALL commanded delta (<= app window) can make the
                     * driver answer 0x33 with its previous target while the lift is
                     * physically already at the requested pose. Accept only in that
                     * case, and only when the MEASURED encoder is inside the
                     * commanded target's application window. A real large move keeps
                     * the original strict mismatch halt below (fault analysis F1). */
                    bool small_accept=false;
                    if(app_window_pulses&&labs(move_delta_pulses)<=(long)app_window_pulses&&pos_valid&&HAL_GetTick()-pos_tick<1000UL){
                        int64_t e=position_raw*(int64_t)pulses_per_rev-(int64_t)target*65536;
                        int64_t w=(int64_t)app_window_pulses*65536;
                        small_accept=(e>=-w&&e<=w);
                    }
                    if(small_accept){
                        target_confirmed=1;
                        say("[LIFT_SMALL_ACCEPT] commanded=%ld read_raw=%c%lu pos_local=%ld delta=%ld window=%u; measured position inside target app window, no halt\r\n",
                            target,signed_raw<0?'-':'+',(unsigned long)(signed_raw<0?-signed_raw:signed_raw),
                            position-origin,move_delta_pulses,(unsigned)app_window_pulses);
                    }else{
                        say("[TARGET_MISMATCH] commanded=%ld read_raw=%c%lu; no valid target confirmation\r\n",target,
                            signed_raw<0?'-':'+',(unsigned long)(signed_raw<0?-signed_raw:signed_raw));
                        halt("driver target differs from commanded absolute target");return;
                    }
                }
                target_confirmed=1;
            }
            if(!quiet)say("[TARGET] raw=%c%lu pulse=%ld matches_active=%u\r\n",packet[2]?'-':'+',(unsigned long)raw,pulses,target_confirmed);
            return;
        }
        if(code==0x37){
            driver_error_raw=signed_raw;driver_error_valid=1;
            if(measure_state==MEAS_ZERO_VERIFY)measure_verify|=4;
            if(!quiet)say("[PERR] raw=%c%lu pulse=%ld\r\n",packet[2]?'-':'+',(unsigned long)raw,pulses);
            return;
        }
        position_raw=signed_raw;position=pulses;pos_valid=1;pos_tick=HAL_GetTick();
        if(!idle_stable_count){idle_min_raw=idle_max_raw=position_raw;idle_stable_tick=HAL_GetTick();}
        if(position_raw<idle_min_raw)idle_min_raw=position_raw;
        if(position_raw>idle_max_raw)idle_max_raw=position_raw;
        if((idle_max_raw-idle_min_raw)*(int64_t)pulses_per_rev>65536){
            idle_min_raw=idle_max_raw=position_raw;idle_stable_tick=HAL_GetTick();idle_stable_count=0;
        }
        if(idle_stable_count<255)idle_stable_count++;
        if(measure_state==MEAS_ZERO_VERIFY)measure_verify|=1;
        if(!quiet)say("[POS] driver=%ld local=%ld pulse; referenced=%u\r\n",position,referenced?position-origin:
            (measure_frame_known?position-measure_hold_origin:(measure_keep_ref?position-measure_keep_origin:0)),referenced);
        if(!quiet||measure_capture)measure_position();
        datum_position();point_position();
        grab_height_position();
        if(measure_state==MEAS_STILL){
            if(!measure_samples){measure_min_raw=measure_max_raw=position_raw;measure_still_tick=HAL_GetTick();}
            if(position_raw<measure_min_raw)measure_min_raw=position_raw;
            if(position_raw>measure_max_raw)measure_max_raw=position_raw;
            if((measure_max_raw-measure_min_raw)*(int64_t)pulses_per_rev>65536){
                measure_min_raw=measure_max_raw=position_raw;measure_samples=0;measure_still_tick=HAL_GetTick();
            }
            if(measure_samples<255)measure_samples++;
        }
        if(moving){
            arrival_sample();
            if(referenced&&(position-origin<low_limit-2||position-origin>high_limit+2))halt("measured lift outside limits");
        }
    }else if(code==0x3A&&packet_len==4){
        unsigned char old_flags=flags;
        flags=packet[2];flags_valid=1;flag_tick=HAL_GetTick();
        if(measure_state==MEAS_ZERO_VERIFY)measure_verify|=8;
        if(!quiet||flags!=old_flags)say("[FLAGS] raw=%02X enabled=%u arrived=%u stall=%u protection=%u\r\n",
            flags,!!(flags&1),!!(flags&2),!!(flags&4),!!(flags&8));
        if((moving||task_active||grab_active||work_active)&&(!(flags&1)||(flags&12)))halt("driver disabled/stall/protection");
        if((measure_state==MEAS_FREE||measure_state==MEAS_STILL||measure_state==MEAS_ZERO_VERIFY)&&(flags&1))
            measure_fail("unexpected enable during manual measurement");
    }else if(code==0x21&&packet_len==15){
        position_kp=((uint32_t)packet[2]<<24)|((uint32_t)packet[3]<<16)|((uint32_t)packet[4]<<8)|packet[5];
        position_ki=((uint32_t)packet[6]<<24)|((uint32_t)packet[7]<<16)|((uint32_t)packet[8]<<8)|packet[9];
        position_kd=((uint32_t)packet[10]<<24)|((uint32_t)packet[11]<<16)|((uint32_t)packet[12]<<8)|packet[13];
        pid_valid=1;
        say("[PID] kp=%lu ki=%lu kd=%lu; READ ONLY\r\n",(unsigned long)position_kp,(unsigned long)position_ki,(unsigned long)position_kd);
    }else if(code==0x27&&packet_len==5){
        phase_current_ma=((unsigned short)packet[2]<<8)|packet[3];current_valid=1;
        say("[CURRENT] phase=%u mA; READ ONLY, not configured current limit\r\n",phase_current_ma);
    }else if(code==0x24&&packet_len==5){say("[VBUS] %u mV\r\n",((unsigned int)packet[2]<<8)|packet[3]);}
    else if(code==0x1F&&packet_len==5){say("[VER] fw=%u hardware=%u\r\n",packet[2],packet[3]);}
    else {if(code==0x36)measure_capture=point_capture=grab_height_capture=datum_capture=0;say("[E] unexpected query payload\r\n");}
}
static void motor_byte(unsigned char b)
{
    unsigned int expected;unsigned long now=HAL_GetTick();
    if(packet_len&&now-packet_tick>40UL)packet_len=0;
    if(!packet_len&&b!=ADDR)return;
    packet_tick=now;packet[packet_len++]=b;
    if(packet_len<2)return;
    switch(packet[1]){
    case 0x1F:case 0x24:case 0x27:expected=5;break;
    case 0x21:expected=15;break;
    case 0x33:case 0x36:case 0x37:expected=8;break;
    case 0x42:expected=33;break;
    case 0x00:case 0x3A:case 0xFD:case 0xF3:case 0xFE:case 0x0A:expected=4;break;
    default:packet_len=0;return;
    }
    if(packet_len==expected){accept_packet();packet_len=0;}
}
static void status(void)
{
    say("[STATUS] " VERSION " clock=%lu pwm=50Hz channels=%u%u%u%u servo=%u,%u target=%u,%u\r\n",
        HAL_RCC_GetSysClockFreq(),servo_on[0],servo_on[1],servo_on[2],servo_on[3],servo[0],servo[1],goal[0],goal[1]);
    say("[TRAY] enabled=%u servo=%u target=%u rate=%u us/s range=550..2350; PC8, no position sensor\r\n",
        servo_on[2],servo[2],goal[2],servo_rate[2]);
    say("[CAMERA_SERVO] enabled=%u pwm=%u target=%u rate=%u; PC9, UNCALIBRATED endpoints\r\n",servo_on[3],servo[3],goal[3],servo_rate[3]);
    say("[JAW] enabled=%u working=%u..%u allowed=%u..%u us; PC6\r\n",servo_on[0],jaw_low,jaw_high,smin[0],smax[0]);
    say("[LIFT5] conf=%u pos_valid=%u flag_valid=%u enabled=%u ref=%u local=%ld limits=%ld..%ld moving=%u fault=%u pending=%02X\r\n",
        config_valid,pos_valid,flags_valid,!!(flags&1),referenced,referenced?position-origin:0,low_limit,high_limit,moving,fault,pending);
    say("[TASK] active=%u step=%u taught=%02X; NO gripper contact sensor\r\n",task_active,task_step,taught_mask);
    say("[GRAB] ready=%u preparing=%u active=%u step=%u slot=%u; pick=%ld clearance=%ld drop=%ld valid=%u%u; jaw=1560/1850 turn=1000/2130\r\n",
        grab_ready,grab_preparing,grab_active,grab_step,grab_slot,grab_height[0],clearance_height,grab_height[1],!!(grab_height_mask&1),!!(grab_height_mask&2));
    say("[WORK] active=%u kind=%s step=%u points=%02X; mode=%s\r\n",work_active,work_kind==4?"unrough":work_kind==1?"rough":"stack",work_step,point_mask,motion_mode==1?"fast":(motion_mode==2?"slow":"normal"));
    say("[MODEL] mode=ABS rpm=%u window_ddeg=%u target_ok=%u target_read=%u perr_read=%u perr_pulses=%ld turn_bounds=%u..%u rate=%u,%u us/s\r\n",
        lift_rpm,arrival_window_ddeg,target_confirmed,driver_target_valid,driver_error_valid,
        (long)(driver_error_raw*(int64_t)pulses_per_rev/65536),turn_low,turn_high,servo_rate[0],servo_rate[1]);
    say("[ARRIVAL] mode=%s app_pulses=%u stable_count=%u\r\n",app_window_pulses?"APP_STABLE":"STRICT",app_window_pulses,arrival_count);
    say("[DIAG] trace=%u current_valid=%u phase_mA=%u pid_valid=%u\r\n",trace_enabled,current_valid,phase_current_ma,pid_valid);
    say("[STOP_INFO] count=%lu tick_ms=%lu reason=%s; retained until reset\r\n",stop_count,last_stop_tick,last_stop_reason);
    if(stop_count)say("[STOP_SNAPSHOT] actual=%ld raw=%ld command=%ld origin=%ld target_raw=%ld flags=%02X verified=%u stable=%u; BEFORE stop\r\n",
        stop_position,(long)stop_position_raw,stop_target,stop_origin,(long)stop_driver_target_raw,stop_flags,stop_target_verified,stop_arrival_count);
    say("[NOOP] stable_samples=%u skipped=%lu; actual FD target verification unchanged\r\n",idle_stable_count,lift_skip_count);
    say("[MEASURE_STATUS] state=%u (0=normal,1=disabling,2=manual,3..6=hold transition,7=error) marks=%02X rebased=%u; RAM only\r\n",
        measure_state,measure_mask,measure_rebased);
}
static unsigned char next_feedback_query(void)
{
    unsigned char code;
    static const unsigned char trace_queries[4]={0x36,0x3A,0x37,0x27};
    if(moving&&!target_confirmed)return 0x33;
    if(moving&&trace_enabled){code=trace_queries[trace_index];trace_index=(trace_index+1U)%4U;return code;}
    code=poll_flags?0x3A:0x36;poll_flags=!poll_flags;return code;
}
static int number(const char *s,long *v)
{
    char *end;long n;if(!s||!*s)return 0;n=strtol(s,&end,10);
    if(*end||n<-30000L||n>30000L)return 0;*v=n;return 1;
}
static void command(char *text)
{
    char *v[5],*p;int argc=0,i;long a,b;
    p=strtok(text," \t");while(p&&argc<5){v[argc++]=p;p=strtok(NULL," \t");}
    if(!argc)return;if(p)goto syntax;
    if(!strcmp(v[0],"stop")||!strcmp(v[0],"!")){halt("operator");return;}
    if(!strcmp(v[0],"p")){say("[PONG] " VERSION "\r\n");return;}
    if(!strcmp(v[0],"status")){status();return;}
    if(task_active||grab_active||grab_preparing||work_active||moving){say("[E] task/lift active; only p/status/stop\r\n");return;}
    if(datum_capture){say("[E] datum capture pending; wait for DATUM_SET\r\n");return;}
    if(!strcmp(v[0],"?")||!strcmp(v[0],"help")){
        say("[HELP] R9E: servo4 camera PC9 (seed known pose); point view/qr; height view/qr (marked only); unrough [slot] rough->tray reverse\r\n");
        say("[HELP] p status probe r flags tpos perr pid current voltage enable stop | seed ID CURRENT_US | servo ID US | j ID DELTA | off ID\r\n");
        say("[HELP] measure on | measure mark pick/clearance/drop | measure list | measure hold | disable (=measure on); support lift before torque release\r\n");
        say("[HELP] grab [1/2/3] (default2): auto query/reference/PWM/cycle; FIRST start must be physical datum0, empty jaw; lift to stored clearance before turn\r\n");
        say("[HELP] cal (prepare only) | rough [slot] | stack [slot] | point tray/rough/stack/swing | swing N (tighten only) | points | save | datum [saved] | motion fast/normal/slow\r\n");
        say("[HELP] grab align pick/drop (capture one correct pose, shift BOTH old contacts) | point grabpick/grabdrop (independent capture)\r\n");
        say("[HELP] datum new (keep heights, change zero) | datum new fresh (clear heights) | datum at zero/clearance/tray/rough/stack/grabpick/grabdrop | point clearance\r\n");
        say("[HELP] zero limits MIN MAX jog DELTA lift LOCAL | teach open/close/pick/place/low/high/drop | task\r\n");
        say("[HELP] speed RPM(1..5000) | accel 1..255 | rate ID US_PER_SEC(5..5000) | jawlimits MIN MAX | turnlimits MIN MAX | arrival strict/app PULSES | trace on/off\r\n");
        say("[HELP] servo IDs:1 jaw PC6,2 turn PC7(870..2500us),3 tray PC8(550..2350us),4 camera PC9(500..2500us); seed output is immediate, ramp commands wait for other servos\r\n");return;
    }
    if(!strcmp(v[0],"points")){points_print();return;}
    if(!strcmp(v[0],"grab")&&argc==3&&!strcmp(v[1],"align")){
        if(!strcmp(v[2],"pick"))i=1;else if(!strcmp(v[2],"drop"))i=2;else goto syntax;
        if(probe_active||servo_busy()||fault||!config_valid||
           (!referenced&&!(measure_state==MEAS_FREE&&measure_keep_ref)))goto blocked;
        grab_height_capture=(unsigned char)i;
        if(!pending)query(0x36,0);else say("[GRAB_ALIGN_WAIT] capturing next fresh position\r\n");return;
    }
    if(!strcmp(v[0],"recover")){
        if(argc!=2||(strcmp(v[1],"clearance")&&strcmp(v[1],"zero")))goto syntax;
        if(pending||probe_active||servo_busy()||measure_state||!fresh()||!config_valid||!(flags&1U)||(flags&12U)||idle_stable_count<3||HAL_GetTick()-idle_stable_tick<200UL)goto blocked;
        origin=position-(!strcmp(v[1],"zero")?0:clearance_height);referenced=1;fault=0;target_confirmed=0;grab_ready=0;taught_mask=0;low_limit=profile_low();high_limit=profile_high();
        say("[REF_RECOVERED] operator confirms PHYSICAL %s; heights retained; no enable/zero/travel\r\n",v[1]);return;
    }
    if(!strcmp(v[0],"datum")){
        if(argc==1){points_print();return;}
        if((argc==2&&!strcmp(v[1],"new"))||(argc==3&&!strcmp(v[1],"new")&&!strcmp(v[2],"fresh"))||
           (argc==3&&!strcmp(v[1],"at"))){
            if(fault||pending||probe_active||servo_busy()||!fresh()||(flags&12U)||
               (measure_state!=MEAS_OFF&&measure_state!=MEAS_FREE)||
               (measure_state==MEAS_FREE?(flags&1U):!(flags&1U)))goto blocked;
            if(!strcmp(v[1],"new")){
                if(argc==2&&!referenced&&!(measure_state==MEAS_FREE&&measure_keep_ref)){
                    say("[E] prior frame unknown; use datum at KNOWN_POSE or datum new fresh\r\n");return;
                }
                datum_capture=argc==3?2:1;
            }else{
                if(!strcmp(v[2],"zero"))datum_capture_local=0;
                else if(!strcmp(v[2],"clearance"))datum_capture_local=clearance_height;
                else if(!strcmp(v[2],"grabpick")||!strcmp(v[2],"grabdrop")){
                    i=!strcmp(v[2],"grabpick")?0:1;if(!(grab_height_mask&(1U<<i)))goto blocked;
                    datum_capture_local=grab_height[i];
                }else{
                    for(i=0;i<3;i++)if(!strcmp(v[2],point_names[i]))break;
                    if(i==3)goto syntax;if(!(point_mask&(1U<<i)))goto blocked;
                    datum_capture_local=point_height[i];
                }
                datum_capture=3;
            }
            say("[DATUM_CAPTURE] reading current stationary pose; zero/at command is the operator's physical-pose declaration\r\n");
            query(0x36,0);return;
        }
        if(argc!=2||strcmp(v[1],"saved"))goto syntax;
        if(measure_state||!fresh()||!(flags&1)||fault||pending||servo_busy())goto blocked;
        if(position-cal_saved_origin < profile_low()||position-cal_saved_origin > profile_high()){
            say("[E] saved datum incompatible with current driver counter\r\n");return;
        }
        origin=cal_saved_origin;referenced=1;grab_ready=0;low_limit=profile_low();high_limit=profile_high();
        say("[DATUM_USED] origin=%ld; operator confirms SAME Emm counter session; no move\r\n",origin);return;
    }
    if(!strcmp(v[0],"height")){
        if(argc!=2)goto syntax;
        for(i=0;i<2;i++)if(!strcmp(v[1],camera_names[i]))break;
        if(i==2)goto syntax;
        if(!(camera_mask&(1U<<i))){say("[E] camera height unmarked; point view/qr then save; route unaffected\r\n");return;}
        if(!lift_ready()||pending||probe_active||servo_busy())goto blocked;
        if(!move_to(camera_height[i]))goto blocked;return;
    }
    if(!strcmp(v[0],"point")){
        if(argc!=2)goto syntax;
        if(!strcmp(v[1],"view"))i=7;else if(!strcmp(v[1],"qr"))i=8;else if(!strcmp(v[1],"grabpick"))i=4;else if(!strcmp(v[1],"grabdrop"))i=5;else if(!strcmp(v[1],"clearance"))i=6;
        else {for(i=0;i<4;i++)if(!strcmp(v[1],point_names[i]))break;if(i==4)goto syntax;}
        if(probe_active||servo_busy()||fault||!config_valid||
           (!referenced&&!(measure_state==MEAS_FREE&&measure_keep_ref)))goto blocked;
        if(i==4||i==5)grab_height_capture=(unsigned char)(i-1);else point_capture=(unsigned char)(i+1);
        if(!pending)query(0x36,0);else say("[POINT_WAIT] next fresh position reply will capture requested point\r\n");return;
    }
    if(!strcmp(v[0],"swing")){
        if(argc!=2)goto syntax;
        if(!strcmp(v[1],"off"))a=clearance_height;else if(!number(v[1],&a))goto syntax;
        if(moving||servo_busy()||fault||measure_state||!(point_mask&8)||a<clearance_height||a>point_height[3]){
            say("[E] swing only tightens an existing safe bound while idle; capture point swing to widen\r\n");return;
        }
        point_height[3]=a;cal_stored=0;say("[SWING] safe-turn lower bound=%ld local; datum/contact heights unchanged; save to persist\r\n",a);return;
    }
    if(!strcmp(v[0],"save")){
        if(argc!=1||pending||probe_active||servo_busy()||fault||
           (measure_state&&!(measure_state==MEAS_FREE&&measure_keep_ref)))goto blocked;
        if(!referenced&&measure_state==MEAS_OFF)
            say("[SAVE_CONFIG] heights only; previous counter datum retained as history; physical reference remains UNKNOWN\r\n");
        cal_save_now();return;
    }
    if(!strcmp(v[0],"tuning")){
        long c=0;
        if(servo_busy()||probe_active||fault||measure_state)goto blocked;
        if(argc==4&&!strcmp(v[1],"lift")&&number(v[2],&a)&&number(v[3],&b)&&a>=20&&a<=900&&b>=20&&b<=600){tune_lift=a;tune_accel=b;}
        else if(argc==5&&!strcmp(v[1],"servo")&&number(v[2],&a)&&number(v[3],&b)&&number(v[4],&c)&&a>=200&&a<=8000&&b>=200&&b<=8000&&c>=200&&c<=8000){tune_jaw=a;tune_turn=b;tune_tray=c;}
        else if(argc==4&&!strcmp(v[1],"timing")&&number(v[2],&a)&&number(v[3],&b)&&a>=30&&a<=100&&b>=40&&b<=200){tune_poll=a;tune_dwell=b;}
        else if(argc==3&&!strcmp(v[1],"settle")&&number(v[2],&a)&&a>=100&&a<=400){tune_turn_settle=a;}
        else if(argc==3&&!strcmp(v[1],"shape")&&number(v[2],&a)&&a>=0&&a<=3){servo_smooth=(unsigned char)a;memset(servo_profile_active,0,sizeof servo_profile_active);}
        else if(argc==3&&!strcmp(v[1],"compact")&&number(v[2],&a)&&a>=0&&a<=1){compact_work=(unsigned char)a;}
        else if(argc==4&&!strcmp(v[1],"arrival")&&number(v[2],&a)&&number(v[3],&b)&&a>=100&&a<=300&&b>=1&&b<=10){tune_stable=a;tune_spread=b;}
        else goto syntax;
        motion_apply();say("[TUNING] lift=%u accel=%u servo=%u,%u,%u poll=%u dwell=%u; RAM only\r\n",tune_lift,tune_accel,tune_jaw,tune_turn,tune_tray,tune_poll,tune_dwell);return;
    }
    if(!strcmp(v[0],"motion")){
        if(argc!=2||servo_busy()||probe_active)goto blocked;
        if(!strcmp(v[1],"fast"))motion_mode=1;else if(!strcmp(v[1],"normal"))motion_mode=0;
        else if(!strcmp(v[1],"slow"))motion_mode=2;else goto syntax;
        motion_apply();cal_stored=0;points_print();return;
    }
    if((!strcmp(v[0],"grab")&&!(argc==2&&!strcmp(v[1],"setup")))||!strcmp(v[0],"rough")||!strcmp(v[0],"unrough")||!strcmp(v[0],"stack")||!strcmp(v[0],"cal")){
        a=2;if(argc==2){if(!number(v[1],&a))goto syntax;}else if(argc!=1)goto syntax;
        if(a<1||a>3)goto syntax;
        b=!strcmp(v[0],"unrough")?4:!strcmp(v[0],"rough")?1:(!strcmp(v[0],"stack")?2:(!strcmp(v[0],"cal")?3:0));
        if(b==0&&grab_height_mask!=3){
            say("[E] grab contacts use an old datum; align one correct pose with grab align pick/drop, then save; rough/stack still available\r\n");return;
        }
        if(b!=3&&b&&(!(point_mask&1U)||!(point_mask&(1U<<(b==4?1:b))))){
            say("[E] %s height missing; teach point tray and point %s first\r\n",v[0],v[0]);return;
        }
        if(fault){say("[E] grab: stopped/faulted (%s); restore clearance with measure on/hold before restarting\r\n",last_stop_reason);return;}
        if(measure_state!=MEAS_OFF&&measure_state!=MEAS_FREE){say("[E] grab: measurement transition/error state=%u; wait or recover hold\r\n",measure_state);return;}
        requested_job=(unsigned char)b;grab_preparing=1;grab_prepare_slot=(unsigned char)a;grab_recovered_hold=0;grab_hold_release_done=0;grab_prepare_tick=HAL_GetTick();
        say("[GRAB_PREP] slot=%u; automatic driver queries, datum and PWM; FIRST start declares PHYSICAL datum0; clearance=%ld\r\n",grab_prepare_slot,clearance_height);return;
    }
    if(!strcmp(v[0],"measure")||!strcmp(v[0],"disable")||!strcmp(v[0],"enable")){
        int start=argc==1&&!strcmp(v[0],"disable");
        int hold=argc==1&&!strcmp(v[0],"enable");
        if(!strcmp(v[0],"measure")&&argc==2){start=!strcmp(v[1],"on");hold=!strcmp(v[1],"hold");}
        if(!strcmp(v[0],"measure")&&argc==2&&!strcmp(v[1],"list")){measure_list();return;}
        if(start){
            static const unsigned char dis[]={ADDR,0xF3,0xAB,0,0,0x6B};
            if(measure_state==MEAS_FREE){say("[MEASURE_READY] already disabled; existing marks retained\r\n");return;}
            if(pending||probe_active||servo_busy()||!config_valid||
               (measure_state!=MEAS_OFF&&measure_state!=MEAS_ERROR))goto blocked;
            measure_keep_ref=referenced||measure_frame_known;measure_keep_origin=origin;measure_keep_low=low_limit;measure_keep_high=high_limit;
            measure_frame_known=0;
            referenced=0;taught_mask=0;grab_ready=0;target_confirmed=0;measure_capture=measure_mask=measure_rebased=0;
            measure_state=MEAS_DISABLING;measure_ack=0;measure_tick=HAL_GetTick();measure_poll_tick=measure_tick;
            flags_valid=0;motor_access=1;
            if(!tx(dis,sizeof(dis)))measure_fail("disable TX failed");
            else say("[MEASURE] disable requested; WAIT for MEASURE_READY before moving by hand\r\n");return;
        }
        if(hold){
            if(measure_state==MEAS_OFF&&argc==1&&!strcmp(v[0],"enable")&&fresh()&&(flags&1U)){
                say("[ENABLE] already enabled; no driver write; use zero to establish software reference\r\n");return;
            }
            if(measure_state==MEAS_OFF&&fresh()&&measure_disabled()){
                measure_keep_ref=referenced;measure_keep_origin=origin;measure_keep_low=low_limit;measure_keep_high=high_limit;
                measure_state=MEAS_FREE;measure_mask=measure_rebased=0;
            }
            if(measure_state!=MEAS_FREE||pending||probe_active||servo_busy()||!fresh()||!measure_disabled()||(flags&12U)||fault)goto blocked;
            measure_state=MEAS_STILL;measure_samples=measure_verify=0;measure_tick=HAL_GetTick();measure_poll_tick=measure_tick;
            referenced=0;taught_mask=0;
            say("[MEASURE] hold requested; support lift and keep still; driver zero/target verification precedes enable\r\n");return;
        }
        if(!strcmp(v[0],"measure")&&argc==3&&!strcmp(v[1],"mark")){
            if(measure_state!=MEAS_FREE||pending||probe_active||servo_busy()||!config_valid||!measure_disabled()||fault)goto blocked;
            for(i=0;i<3;i++)if(!strcmp(v[2],measure_names[i]))break;if(i==3)goto syntax;
            measure_capture=(unsigned char)(i+1);query(0x36,0);return;
        }
        goto syntax;
    }
    if(measure_state&&strcmp(v[0],"r")&&strcmp(v[0],"flags")&&strcmp(v[0],"tpos")&&strcmp(v[0],"perr")&&
       strcmp(v[0],"current")&&strcmp(v[0],"voltage")&&strcmp(v[0],"pid")&&
       !(measure_state==MEAS_FREE&&(!strcmp(v[0],"seed")||!strcmp(v[0],"arm")||!strcmp(v[0],"servo")||
         !strcmp(v[0],"j")||!strcmp(v[0],"off")||!strcmp(v[0],"rate")||!strcmp(v[0],"jawlimits")||!strcmp(v[0],"turnlimits")))){
        say("[E] manual measurement locks electric lift/task; use measure hold before electric lift motion\r\n");return;
    }
    if(measure_state&&measure_state!=MEAS_FREE&&measure_state!=MEAS_ERROR)goto blocked;
    if(!strcmp(v[0],"grab")){
        if(argc==2&&!strcmp(v[1],"setup")){
            long here=position-origin;
            if(!lift_ready()||pending||probe_active||servo_busy()||!servo_on[0]||!servo_on[1]||!servo_on[2]||
               here < clearance_height-20||here > clearance_height+20||servo[0]<1560||servo[0]>2260||servo[1]<1000||servo[1]>2130)goto grab_blocked;
            if(pulses_per_rev!=3200){say("[E] grab calibration requires3200 input pulses/rev\r\n");return;}
            jaw_low=1560;jaw_high=2260;turn_low=1000;turn_high=2130;
            low_limit=profile_low();high_limit=profile_high();motion_apply();
            taught_mask=0;grab_ready=1;
            say("[GRAB_SETUP] current datum retained; pick=2427 drop=2171; calibrated profile applied; no move\r\n");return;
        }
        a=2;if(argc==2){if(!number(v[1],&a))goto syntax;}else if(argc!=1)goto syntax;
        if(a<1||a>3)goto syntax;
        if(!grab_ready||!lift_ready()||pending||probe_active||servo_busy()||
           !servo_on[0]||!servo_on[1]||!servo_on[2]||pulses_per_rev!=3200||
           low_limit>clearance_height||high_limit<grab_height[0]||high_limit<grab_height[1]||jaw_low>1560||jaw_high<1850||turn_low>1000||turn_high<2130)goto grab_blocked;
        grab_slot=(unsigned char)a;grab_step=grab_issued=grab_settling=0;grab_active=1;
        say("[GRAB_START] slot=%u tray=%u; one pick/place cycle; empty jaw and object alignment required\r\n",grab_slot,grab_tray[grab_slot-1]);return;
grab_blocked:
        say("[E] legacy setup: ref=%u fresh=%u fault=%u pending=%02X pwm=%u%u%u busy=%u local=%ld; simply use grab for automatic preparation\r\n",
            referenced,!!fresh(),fault,pending,servo_on[0],servo_on[1],servo_on[2],!!servo_busy(),position-origin);return;
    }
    if(!strcmp(v[0],"arrival")){
        if(pending||probe_active||servo_busy())goto blocked;
        if(argc==2&&!strcmp(v[1],"strict"))app_window_pulses=0;
        else if(argc==3&&!strcmp(v[1],"app")&&number(v[2],&a)&&a>=1&&a<=20)app_window_pulses=(unsigned short)a;
        else goto syntax;
        arrival_count=0;taught_mask=0;
        say("[ARRIVAL] mode=%s app_pulses=%u; no move; teach cleared; application tolerance must be measured\r\n",app_window_pulses?"APP_STABLE":"STRICT",app_window_pulses);return;
    }
    if(!strcmp(v[0],"trace")){
        if(argc!=2)goto syntax;
        if(pending||probe_active||servo_busy())goto blocked;
        if(!strcmp(v[1],"on"))trace_enabled=1;
        else if(!strcmp(v[1],"off"))trace_enabled=0;
        else goto syntax;
        trace_index=0;say("[TRACE] %u; no move, no driver parameter writes; motion polls pos/flags/error/current\r\n",trace_enabled);return;
    }
    if(!strcmp(v[0],"probe")){
        if(argc!=1||pending||probe_active||servo_busy())goto blocked;
        pos_valid=0;flags_valid=0;config_valid=0;referenced=0;taught_mask=0;grab_ready=0;driver_target_valid=driver_error_valid=target_confirmed=0;
    probe_index=0;probe_active=1;probe_tick=HAL_GetTick()-300UL;
        say("[PROBE] seven READ queries including target/error, no motion\r\n");return;
    }
    if(!strcmp(v[0],"r")||!strcmp(v[0],"flags")||!strcmp(v[0],"tpos")||!strcmp(v[0],"perr")||!strcmp(v[0],"pid")||!strcmp(v[0],"current")||!strcmp(v[0],"voltage")){
        unsigned char code;
        if(argc!=1||pending||probe_active)goto blocked;
        if(!strcmp(v[0],"r")){pos_valid=0;code=0x36;}
        else if(!strcmp(v[0],"flags")){flags_valid=0;code=0x3A;}
        else if(!strcmp(v[0],"tpos")){driver_target_valid=0;code=0x33;}
        else if(!strcmp(v[0],"pid")){pid_valid=0;code=0x21;}
        else if(!strcmp(v[0],"current")){current_valid=0;code=0x27;}
        else if(!strcmp(v[0],"voltage")){code=0x24;}
        else{driver_error_valid=0;code=0x37;}
        query(code,0);return;
    }
    if(!strcmp(v[0],"speed")){
        if(argc!=2||!number(v[1],&a))goto syntax;
        if(a<1||a>5000||servo_busy()||probe_active)goto blocked;
        lift_rpm=(unsigned short)a;say("[SPEED] lift=%u RPM; no move\r\n",lift_rpm);return;
    }
    if(!strcmp(v[0],"accel")){
        if(argc!=2||!number(v[1],&a))goto syntax;
        if(a<1||a>255||servo_busy()||probe_active)goto blocked;
        lift_accel=(unsigned short)a;say("[ACCEL] lift=%u; no move\r\n",lift_accel);return;
    }
    if(!strcmp(v[0],"rate")){
        if(argc!=3||!number(v[1],&a)||!number(v[2],&b)||a<1||a>SERVO_COUNT)goto syntax;
        if(b<5||b>5000||servo_busy()||probe_active)goto blocked;
        servo_rate[a-1]=(unsigned short)b;servo_credit[a-1]=0;
        say("[RATE] servo%ld=%ld us/s; no move\r\n",a,b);return;
    }
    if(!strcmp(v[0],"jawlimits")){
        if(argc!=3||!number(v[1],&a)||!number(v[2],&b))goto syntax;
        if(servo_busy()||probe_active||a<smin[0]||b>smax[0]||a>=b)goto blocked;
        if(servo_on[0]&&(servo[0]<a||servo[0]>b))goto blocked;
        jaw_low=(unsigned short)a;jaw_high=(unsigned short)b;taught_mask=0;
        say("[JAW_LIMITS] %u..%u us; no move; teach cleared; resets to1560..2260 on boot\r\n",jaw_low,jaw_high);return;
    }
    if(!strcmp(v[0],"turnlimits")){
        if(argc!=3||!number(v[1],&a)||!number(v[2],&b))goto syntax;
        if(!servo_on[1]||servo_busy()||probe_active||a<smin[1]||b>smax[1]||a>=b||servo[1]<a||servo[1]>b)goto blocked;
        turn_low=(unsigned short)a;turn_high=(unsigned short)b;taught_mask=0;
        say("[TURN_LIMITS] %u..%u us; operator verifies measured working range\r\n",turn_low,turn_high);return;
    }
    if(!strcmp(v[0],"arm")||!strcmp(v[0],"seed")||!strcmp(v[0],"servo")||!strcmp(v[0],"j")){
        if(argc!=3||!number(v[1],&a)||!number(v[2],&b)||a<1||a>SERVO_COUNT)goto syntax;
        if(servo_busy()||probe_active)goto blocked;i=(int)a-1;
        if(!strcmp(v[0],"j")){
            if(b<-20||b>20)goto blocked;b+=servo[i];
        }
        if(b<smin[i]||b>smax[i]){say("[E] servo%ld range=%u..%u\r\n",a,smin[i],smax[i]);return;}
        if(i==0&&(b<jaw_low||b>jaw_high)){say("[E] servo1 working=%u..%u us; jawlimits MIN MAX\r\n",jaw_low,jaw_high);return;}
        if(!strcmp(v[0],"arm")||!strcmp(v[0],"seed")){
            if(i>0&&!strcmp(v[0],"arm")){say("[E] turn/tray requires seed ID CURRENT_US matched to actual pose; no default angle\r\n");return;}
            if(servo_on[i])goto blocked;
            servo[i]=goal[i]=(unsigned short)b;__HAL_TIM_SET_COMPARE(&htim3,channels[i],b);
            if(HAL_TIM_PWM_Start(&htim3,channels[i])!=HAL_OK){halt("PWM start failed");return;}
            servo_on[i]=1;servo_credit[i]=0;
            if(i==1){turn_low=(unsigned short)(b>smin[i]+60?b-60:smin[i]);turn_high=(unsigned short)(b<smax[i]-60?b+60:smax[i]);}
            say("[SEEDED] servo%ld=%ld us; KNOWN current pose required; initial output is immediate\r\n",a,b);
        }else{
            if(!servo_on[i])goto blocked;
            if(i==1&&(b<turn_low||b>turn_high))goto blocked;
            goal[i]=(unsigned short)b;servo_credit[i]=0;
            say("[RAMP] servo%ld target=%ld us, rate=%u us/s\r\n",a,b,servo_rate[i]);
        }return;
    }
    if(!strcmp(v[0],"off")){
        if(argc!=2||!number(v[1],&a)||a<1||a>SERVO_COUNT)goto syntax;i=(int)a-1;
        HAL_TIM_PWM_Stop(&htim3,channels[i]);servo_on[i]=0;servo[i]=goal[i]=0;taught_mask=0;
        if(i==1)turn_low=turn_high=0;
        say("[OFF] servo%ld PWM off; torque may be lost\r\n",a);return;
    }
    if(!strcmp(v[0],"zero")){
        if(argc!=1||!fresh()||probe_active||servo_busy()||!(flags&1)||(flags&12))goto blocked;
        if(point_mask){point_mask=0;cal_stored=0;say("[POINTS_CLEARED] explicit zero changes datum; reteach new processing heights\r\n");}
        if(grab_height_mask){grab_height_mask=0;cal_stored=0;say("[GRAB_HEIGHTS_CLEARED] explicit zero changes datum\r\n");}
        origin=position;clearance_height=0;referenced=1;fault=0;taught_mask=0;grab_ready=0;low_limit=-100;high_limit=100;target_confirmed=0;idle_stable_count=0;measure_frame_known=0;
        say("[REF] current position is LOCAL zero; not mechanical home; limits=-100..100\r\n");return;
    }
    if(!strcmp(v[0],"limits")){
        if(argc!=3||!number(v[1],&a)||!number(v[2],&b))goto syntax;
        if(!lift_ready()||pending||probe_active||a>=b||a<-6000||b>6000||position-origin<a||position-origin>b)goto blocked;
        low_limit=a;high_limit=b;taught_mask=0;say("[LIMITS] %ld..%ld local pulses; teach cleared\r\n",a,b);return;
    }
    if(!strcmp(v[0],"jog")||!strcmp(v[0],"lift")){
        if(argc!=2||!number(v[1],&a))goto syntax;
        if(servo_busy())goto blocked;
        if(!strcmp(v[0],"jog")){if(!a||a<-100||a>100)goto blocked;a+=position-origin;}
        if(!move_to(a))goto blocked;return;
    }
    if(!strcmp(v[0],"teach")){
        if(argc!=2||!lift_ready()||pending||probe_active||servo_busy()||!servo_on[0]||!servo_on[1])goto blocked;
        for(i=0;i<7;i++)if(!strcmp(v[1],names[i]))break;if(i==7)goto syntax;
        taught[i]=i<2?servo[0]:(i<4?servo[1]:position-origin);taught_mask|=(1U<<i);
        say("[TEACH] %s=%ld (operator verified; servo/contact not sensed)\r\n",names[i],taught[i]);return;
    }
    if(!strcmp(v[0],"task")){
        if(argc!=1||taught_mask!=0x7F||!lift_ready()||pending||probe_active||servo_busy()||!servo_on[0]||!servo_on[1])goto blocked;
        for(i=4;i<7;i++)if(taught[i]<low_limit||taught[i]>high_limit)goto blocked;
        if(taught[2]<turn_low||taught[2]>turn_high||taught[3]<turn_low||taught[3]>turn_high)goto blocked;
        if(taught[0]==taught[1]||taught[4]==taught[5]||taught[6]==taught[5])goto blocked;
        task_active=1;task_step=0;task_issued=0;
        say("[TASK] start taught pick-and-place; grasp result unverified without sensor\r\n");return;
    }
syntax:say("[E] command/arguments invalid; help\r\n");return;
blocked:say("[E] blocked: idle/fresh feedback/reference/range/teach required; status\r\n");
    say("[BLOCK] cmd=%s ref=%u fault=%u fresh=%u pending=%02X servo_busy=%u probe=%u\r\n",
        v[0],referenced,fault,!!fresh(),pending,!!servo_busy(),probe_active);
}
static void debug_byte(unsigned char b)
{
    if(b=='!'){line_len=0;line_bad=0;halt("operator !");return;}
    if(b=='\r'||b=='\n'){
        if(line_bad){say("[E] line too long; discarded\r\n");halt("console line overflow");}
        else if(line_len){line[line_len]=0;command(line);}
        line_len=0;line_bad=0;
    }else if(b>=32&&b<=126){if(line_len<sizeof(line)-1&&!line_bad)line[line_len++]=(char)b;else line_bad=1;}
}
static void task_poll(unsigned long now)
{
    /* Servo steps are PWM-command completion; lift steps require sensor convergence. */
    static const unsigned char axis[11]={0,2,1,2,0,2,1,2,0,2,1};
    static const unsigned char value[11]={0,5,2,4,1,5,3,6,0,5,2};
    unsigned char a;
    if(!task_active)return;
    if(!task_issued){
        if(!lift_ready()){halt("task feedback stale/unhealthy");return;}
        if(pending)return;
        a=axis[task_step];
        say("[TASK] step=%u axis=%u value=%ld\r\n",task_step,a,taught[value[task_step]]);
        if(a==2){if(!move_to(taught[value[task_step]])){halt("task lift blocked");return;}}
        else goal[a]=(unsigned short)taught[value[task_step]];
        task_tick=HAL_GetTick();task_issued=1;
    }else if(!moving&&!servo_busy()&&now-task_tick>=400UL){
        task_step++;task_issued=0;
        if(task_step>=11){task_active=0;say("[TASK] sequence finished; ACTUAL grasp/release must be observed\r\n");}
    }
}
static unsigned char turn_physically_settled(unsigned long now)
{return !servo_on[1]||!servo_done_tick[1]||now-servo_done_tick[1]>=tune_turn_settle;}
static void grab_poll(unsigned long now)
{
    /* First raise before any turn/tray change. All PWM ramps and lift sensor
     * arrivals complete before dwell begins; no fixed-delay lift estimates. */
    static const unsigned char axis[11]={3,2,0,1,3,0,3,1,3,0,3};
    static const long values[11]={0,0,1850,2130,GRAB_PICK,1560,0,1000,GRAB_DROP,1850,0};
    static const unsigned short dwell[11]={100,200,150,150,100,350,100,150,100,350,100};
    unsigned char a;long value;
    if(!grab_active)return;
    if(!lift_ready()){halt("grab feedback stale/unhealthy");return;}
    if(!servo_on[0]||!servo_on[1]||!servo_on[2]){halt("grab servo PWM disabled");return;}
    if(!grab_issued){
        if(pending)return;
        a=axis[grab_step];value=a==2?grab_tray[grab_slot-1]:values[grab_step];
        if(grab_step==4)value=grab_height[0];else if(grab_step==8)value=grab_height[1];else if(a==3)value=clearance_height;else if(a==3)value=clearance_height;
        if((a==1||a==2)&&(position-origin < clearance_height-20||position-origin > clearance_height+20)){
            halt("grab rotation requires clearance height");return;
        }
        say("[GRAB_STEP] %u/11 axis=%u value=%ld\r\n",grab_step+1,a,value);
        if(a==3){if(!move_to(value)){halt("grab lift blocked");return;}}
        else {
            goal[a]=(unsigned short)value;
            if(motion_mode==1&&grab_step==1){
                goal[0]=1850;goal[1]=2130;
                say("[PARALLEL] tick=%lu jaw1850+tray%u+turn2130 issued together at clearance\r\n",now,grab_tray[grab_slot-1]);
            }
            if(motion_mode==1&&grab_step==7&&(point_mask&8)&&point_height[3]>clearance_height){
                long approach=point_height[3]<grab_height[1]?point_height[3]:grab_height[1];
                if(!move_to(approach)){halt("grab safe overlap lift blocked");return;}
                say("[PARALLEL] turn inside + descend only within taught swing band to%ld\r\n",approach);
            }
        }
        grab_issued=1;grab_settling=0;
    }else if(!moving&&!servo_busy()&&turn_physically_settled(now)){
        if(!grab_settling){grab_settling=1;grab_settle_tick=now;}
        unsigned short wait=motion_mode==1?((grab_step==5||grab_step==9)?200:tune_dwell):dwell[grab_step];
        if(now-grab_settle_tick>=wait){
            if(motion_mode==1&&grab_step==1)grab_step=4;else grab_step++;
            grab_issued=grab_settling=0;
            if(grab_step>=11){grab_active=0;say("[GRAB_DONE] slot=%u; lift at clearance, turn inside, jaw open; verify actual object placement\r\n",grab_slot);}
        }
    }
}
static void grab_begin(void)
{
    grab_slot=grab_prepare_slot;grab_preparing=0;
    grab_step=grab_issued=grab_settling=0;grab_ready=1;
    if(requested_job==3){
        say("[CAL_READY] datum retained; PWM enabled; use jog/servo/point or measure on; no pick/place started\r\n");return;
    }
    if(requested_job){
        work_kind=requested_job;work_active=1;work_step=work_issued=work_settling=0;
        if(compact_work&&motion_mode==1&&lift_ready()&&!moving&&!pending&&labs(position-origin-clearance_height)<=20&&
           servo_on[0]&&servo_on[1]&&servo_on[2]&&servo[0]==1850&&goal[0]==1850&&
           servo[1]==(work_kind==4?2130:1000)&&goal[1]==servo[1]&&servo[2]==grab_tray[grab_slot-1]&&goal[2]==servo[2]&&turn_physically_settled(HAL_GetTick())){
            work_step=2;say("[WORK_COMPACT] verified clearance/open/source turn/tray; skip two redundant preparation steps\r\n");
        }
        say("[WORK_START] %s slot=%u tray_pick=%ld destination=%ld; base positions chassis-aligned\r\n",
            work_kind==4?"unrough":work_kind==1?"rough":"stack",grab_slot,point_height[0],point_height[work_kind==4?1:work_kind]);return;
    }
    grab_active=1;
    say("[GRAB_START] slot=%u tray=%u; one pick/place cycle\r\n",grab_slot,grab_tray[grab_slot-1]);
}
static void grab_prepare_poll(unsigned long now)
{
    unsigned int i;
    if(!grab_preparing)return;
    if(fault){grab_preparing=0;say("[E] grab preparation cancelled by fault\r\n");return;}
    if(now-grab_prepare_tick>=20000UL){halt("grab preparation timeout");return;}
    if(pending||probe_active)return;
    switch(grab_preparing){
    case 1:
        if(servo_busy())return;
        config_valid=0;grab_preparing=2;query(0x42,0);return;
    case 2:
        if(!config_valid){halt("grab driver config invalid; see CONF/error above");return;}
        if(pulses_per_rev!=3200){halt("grab calibration needs3200 pulses/rev");return;}
        flags_valid=0;grab_preparing=3;query(0x3A,0);return;
    case 3:
        if(!flags_valid||(flags&12U)){halt("grab driver flags invalid/stall/protection");return;}
        pos_valid=0;grab_preparing=4;query(0x36,0);return;
    case 4:
        if(!fresh()){halt("grab position/flags missing or stale");return;}
        if(!(flags&1U)){
            /* Explicit grab permits the same supported, zero-verified hold
             * recovery as measure hold. No unverified direct enable. */
            if(!(measure_state==MEAS_FREE&&measure_keep_ref)){
                measure_keep_ref=referenced;measure_keep_origin=origin;measure_keep_low=low_limit;measure_keep_high=high_limit;
            }
            measure_state=MEAS_STILL;measure_samples=measure_verify=0;
            measure_tick=now;measure_poll_tick=now;referenced=0;taught_mask=0;
            grab_recovered_hold=1;grab_preparing=5;
            say("[GRAB_PREP] motor disabled; keep lift supported/still until HOLD_READY\r\n");return;
        }
        if(!referenced){
            origin=(position-cal_saved_origin>=-20&&position-cal_saved_origin<=20)?cal_saved_origin:position;
            referenced=1;taught_mask=0;
            say("[GRAB_DATUM] current physical datum recorded as LOCAL0, driver=%ld; operator fixed start pose assumed; clearance=%ld\r\n",origin,clearance_height);
        }
        grab_preparing=6;return;
    case 5:
        if(measure_state!=MEAS_OFF)return;
        grab_hold_release_tick=now;
        say("[GRAB_HOLD_READY] holding restored; withdraw support before automatic motion\r\n");
        grab_preparing=6;return;
    case 6:
        if(grab_recovered_hold&&!grab_hold_release_done){
            if(now-grab_hold_release_tick<3000UL)return;grab_hold_release_done=1;
        }
        if(!lift_ready()){halt("grab lift not ready after preparation");return;}
        if(position-origin < -6000||position-origin > profile_high()){halt("grab start outside calibrated travel");return;}
        if(grab_ready&&(low_limit>clearance_height||high_limit<grab_height[0]||high_limit<grab_height[1])){halt("grab operator limits exclude calibrated heights");return;}
        jaw_low=1560;jaw_high=2260;turn_low=1000;turn_high=2130;
        low_limit=profile_low();if(position-origin<low_limit)low_limit=position-origin-30;
        if(low_limit<-6000)low_limit=-6000;high_limit=profile_high();
        if(!grab_ready){
            motion_apply();
        }
        if(position-origin < clearance_height-20||position-origin > clearance_height+20){
            grab_preparing=7;
            if(!move_to(clearance_height))halt("grab initial clearance lift failed");return;
        }
        grab_preparing=7;return;
    case 7:{
        unsigned char newly_enabled=0;
        if(moving)return;
        if(!lift_ready()||position-origin < clearance_height-20||position-origin > clearance_height+20){halt("grab failed to reach clearance before PWM");return;}
        for(i=0;i<3;i++)if(!servo_on[i]){
            unsigned short first=i==0?1850:(i==1?(requested_job==0||requested_job==4?2130:1000):grab_tray[grab_prepare_slot-1]);
            servo[i]=goal[i]=first;servo_credit[i]=0;__HAL_TIM_SET_COMPARE(&htim3,channels[i],first);
            if(HAL_TIM_PWM_Start(&htim3,channels[i])!=HAL_OK){halt("grab automatic PWM start failed");return;}
            servo_on[i]=1;say("[GRAB_PWM] servo%u enabled at%u us; clearance verified\r\n",i+1,first);
            newly_enabled=1;
        }
        if(newly_enabled&&requested_job==0)say("[PARALLEL_PREP] tick=%lu initial jaw1850+turn2130+tray%u; no earlier tray-only phase\r\n",now,grab_tray[grab_prepare_slot-1]);
        grab_prepare_delay=newly_enabled?1000:0;
        grab_ready=1;grab_prepare_wait_tick=now;grab_preparing=8;
        say("[GRAB_PREP] high-level targets initialized together; cycle begins after settle\r\n");return;
    }
    case 8:
        if(!lift_ready()){halt("grab feedback lost before cycle");return;}
        if(turn_physically_settled(now)&&now-grab_prepare_wait_tick>=grab_prepare_delay&&idle_stable_count>=3&&now-idle_stable_tick>=(compact_work&&motion_mode==1?(unsigned long)tune_stable:200UL))grab_begin();
        return;
    default:halt("grab preparation state invalid");return;
    }
}
static void work_poll(unsigned long now)
{
    static const unsigned char axis[9]={3,2,3,0,3,1,3,0,3};
    unsigned char a;long value;unsigned short wait;
    if(!work_active)return;
    if(!lift_ready()||!servo_on[0]||!servo_on[1]||!servo_on[2]){halt("work feedback/PWM not ready");return;}
    if(!work_issued){
        if(pending)return;
        a=axis[work_step];value=clearance_height;
        if(work_step==1)value=grab_tray[grab_slot-1];
        else if(work_step==2)value=point_height[work_kind==4?1:0];
        else if(work_step==3)value=1560;
        else if(work_step==5)value=work_kind==4?1000:2130;
        else if(work_step==6)value=point_height[work_kind==4?0:work_kind];
        else if(work_step==7)value=1850;
        if((a==1||a==2)&&(position-origin < clearance_height-20||position-origin > clearance_height+20)){halt("work rotation requires clearance");return;}
        say("[WORK_STEP] %s %u/9 axis=%u value=%ld\r\n",work_kind==4?"unrough":work_kind==1?"rough":"stack",work_step+1,a,value);
        if(a==3){if(!move_to(value)){halt("work lift blocked");return;}}
        else {
            goal[a]=(unsigned short)value;
            if(work_step==1){goal[0]=1850;goal[1]=work_kind==4?2130:1000;}
            if(work_step==5&&motion_mode==1&&(point_mask&8)&&point_height[3]>clearance_height){
                long destination=point_height[work_kind==4?0:work_kind];
                long approach=point_height[3]<destination?point_height[3]:destination;
                if(!move_to(approach)){halt("work overlap lift blocked");return;}
                say("[PARALLEL] loaded turn outside + descend within taught swing band to%ld\r\n",approach);
            }
        }
        work_issued=1;work_settling=0;
    }else if(!moving&&!servo_busy()&&turn_physically_settled(now)){
        if(!work_settling){work_settling=1;work_settle_tick=now;}
        wait=(work_step==3||work_step==7)?(motion_mode==1?200:350):(motion_mode==1?tune_dwell:150);
        if(now-work_settle_tick>=wait){
            work_step++;work_issued=work_settling=0;
            if(work_step>=9){work_active=0;say("[WORK_DONE] %s slot=%u; clearance, jaw open; unrough ends INSIDE; observe release/stack\r\n",work_kind==4?"unrough":work_kind==1?"rough":"stack",grab_slot);}
        }
    }
}
void ArmTask_Init(void)
{
    cal_load();
#ifdef ARM_NAV_BUILD
    motion_mode=1;motion_apply();
#endif
#ifdef ARM_NAV_BUILD
    say("[ARM_INIT] " VERSION " shared UART4 PA0/PA1 address5; ALL FOUR PWM OFF; no automatic lift query/motion\r\n");
#else
    say("\r\n[BOOT] " VERSION " USART1 115200 8N1; UART4 Emm5; ALL PWM OFF; NO AUTO MOVE\r\n");
#endif
    status();
#ifndef ARM_NAV_BUILD
    probe_index=0;probe_active=1;probe_tick=HAL_GetTick()-300UL;
#endif
#ifndef ARM_NAV_BUILD
    say("[BOOT] read-only Emm discovery; no enable/zero/move\r\n");
#endif
}
void ArmTask_Tick(void)
{
        unsigned long now;
        if(motor_rx.err||debug_rx.err){
            motor_rx.err=debug_rx.err=0;motor_rx.tail=motor_rx.head;debug_rx.tail=debug_rx.head;
            packet_len=line_len=line_bad=0;halt("UART error/overflow");
        }
        while(debug_rx.tail!=debug_rx.head){unsigned char b=debug_rx.b[debug_rx.tail];debug_rx.tail=(debug_rx.tail+1U)&MASK;debug_byte(b);}
        while(motor_rx.tail!=motor_rx.head){unsigned char b=motor_rx.b[motor_rx.tail];motor_rx.tail=(motor_rx.tail+1U)&MASK;motor_byte(b);}
        now=HAL_GetTick();
        servo_poll(now);
        now=HAL_GetTick();
        if(pending&&now-pending_tick>=250UL){
            say("[E] query %02X timeout, NO valid feedback\r\n",pending);
            if(pending==0x36)pos_valid=0;if(pending==0x3A)flags_valid=0;if(pending==0x42)config_valid=0;
            if(pending==0x33)driver_target_valid=target_confirmed=0;if(pending==0x37)driver_error_valid=0;
            if(pending==0x21)pid_valid=0;if(pending==0x27)current_valid=0;
            measure_capture=point_capture=grab_height_capture=datum_capture=0;
            pending=0;packet_len=0;if(moving||task_active||grab_active||grab_preparing||work_active)halt("feedback timeout");
            if(measure_state&&measure_state!=MEAS_FREE&&measure_state!=MEAS_ERROR)measure_fail("feedback timeout during transition");
        }
        now=HAL_GetTick();
        if(probe_active&&!pending&&now-probe_tick>=300UL){
            if(probe_index>=sizeof(queries)){probe_active=0;say("[PROBE] done; conf=%u flags=%u pos=%u target=%u error=%u\r\n",config_valid,flags_valid,pos_valid,driver_target_valid,driver_error_valid);}
            else{probe_tick=now;query(queries[probe_index++],0);}
        }
        lift_poll(now);
        measure_poll(now);
        cal_save_poll();
        if((!grab_preparing||grab_preparing>=7||(grab_preparing==6&&grab_recovered_hold))&&(!measure_state||measure_state==MEAS_FREE)&&(moving||task_active||grab_active||work_active||config_valid)&&!probe_active&&!pending&&now-poll_tick>=(moving||task_active||grab_active||grab_preparing||work_active?(unsigned long)tune_poll:100UL)){
            poll_tick=now;
            query(next_feedback_query(),!(moving||task_active));
        }
        task_poll(HAL_GetTick());
        grab_poll(HAL_GetTick());
        grab_prepare_poll(HAL_GetTick());
        work_poll(HAL_GetTick());
}
int ArmTask_Run(void)
{
    HAL_Init();Board_Arm_Init();ArmTask_Init();
    for(;;)ArmTask_Tick();
}
