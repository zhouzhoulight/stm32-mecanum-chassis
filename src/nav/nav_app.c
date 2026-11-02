#include "nav/nav_app.h"
#include "nav/nav_control.h"
#include "nav/nav_odom.h"
#include "nav/nav_bus.h"
#include "nav/nav_platform.h"
#include "nav/nav_time.h"
#include "nav/nav_route.h"
#include "wit_imu.h"
#include "imu_service.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
/* 0 idle;1 zero ACK / disabled stationary;2 enable ACK;3 zero ACK + stationary;
 * 4 driving;5 zero-speed settling;6 fault. No automatic multi-leg motion. */
static unsigned phase,algorithm,ceiling;
static bool turning,blending,routing;
static NavRoute route;
static float blend_start,blend_delta,blend_duration,blend_time,blend_rate;
static float turn_target,turn_last_error,turn_wheel;
static unsigned turn_stable;
static bool fused,xy,preparation_disabled;
static NavControl control;
static float goal[2],b0_ratio,heading_reference;
static unsigned requested_rpm;
static uint32_t phase_at,run_at,last_control,last_log,zero_generation,enable_generation;
static uint32_t bus_errors,imu_errors,imu_checksums,deadline,slip_since;
static uint32_t control_max,odom_max,bus_max,loop_max,zero_ack_at;
static const char *reason="none";
static const char *names[4]={"cascade","adrc","lqr","mpc"};
/* Bound every numeric token BEFORE scanf's integer conversions. scanf does
 * not report overflow reliably. Also normalize pasted spaces/tabs. */
static bool normalize_command(const char *input,char output[100])
{
 unsigned used=0;
 while(*input){
  while(*input==' '||*input=='\t')input++;
  if(!*input)break;
  if(used){if(used>=99)return false;output[used++]=' ';}
  const char *first=input;bool numeric=*input=='+'||*input=='-'||(*input>='0'&&*input<='9');
  uint32_t value=0,limit=*input=='-'?2147483648U:2147483647U;unsigned digits=0;
  while(*input&&*input!=' '&&*input!='\t'){
   unsigned char ch=(unsigned char)*input;
   if(ch<32||ch>126||used>=99)return false;
   if(numeric){
    if(input==first&&(ch=='+'||ch=='-')){}
    else{if(ch<'0'||ch>'9'||value>(limit-(ch-'0'))/10)return false;value=value*10+(ch-'0');digits++;}
   }
   output[used++]=*input++;
  }
  if(numeric&&!digits)return false;
 }
 output[used]=0;return used!=0;
}
static int mm(float x){return (int)roundf(x*1000);}
static float wrapped(float x){while(x>3.14159265f)x-=6.28318531f;while(x< -3.14159265f)x+=6.28318531f;return x;}
static const NavPose *pose(void){const NavOdom *o=nav_odom_data();return fused?&o->fused:&o->base;}
unsigned nav_app_state(void){return phase;}
bool nav_app_active(void){return phase>0&&phase<6;}
void nav_app_emergency(uint32_t now,const char *why)
{
 if(phase==6&&nav_bus_data()->stopping)return;
 routing=false;phase=6;reason=why;nav_bus_emergency(now);nav_logf("[FAULT] %s; FE+disable queued for ALL four; no automatic resume\r\n",why);
}
static bool feedback(uint32_t now,bool enabled)
{
 const NavBus *b=nav_bus_data();const WitImu *i=imu_data();
 if(!i->angle_seen || nav_age(now,i->angle_at)>300 || !i->gyro_seen || nav_age(now,i->gyro_at)>300 || nav_odom_data()->fault)return false;
 for(unsigned k=0;k<4;k++)if(!b->position_at[k]||nav_age(now,b->position_at[k])>300 || !b->flags_at[k] || nav_age(now,b->flags_at[k])>800 || (b->flags[k]&12) || (enabled && !(b->flags[k]&1)))return false;
 return true;
}
static void report(uint32_t now)
{
 const NavOdom *o=nav_odom_data();const NavBus *b=nav_bus_data();const WitImu *i=imu_data();
 nav_logf("[NAV] %s state=%u reason=%s algorithm=%s control=%s mode=%s ceiling=%uRPM\r\n",NAV_VERSION,phase,reason,names[algorithm],xy?"xy":"hold",fused?"fused":"base",ceiling);
 nav_logf("[ODOM] base_mm=%d,%d fused_mm=%d,%d yaw_mdeg=%d valid=%u fault=%lu epoch=%lu age=%lums IMU_age=%lums\r\n",mm(o->base.x_m),mm(o->base.y_m),mm(o->fused.x_m),mm(o->fused.y_m),(int)(nav_odom_heading_now()*57295.7795f),o->base.valid,(unsigned long)o->fault,(unsigned long)o->epoch,(unsigned long)(now-o->base.at),(unsigned long)(i->angle_seen?now-i->angle_at:UINT32_MAX));
 nav_logf("[BUS] errors=%lu timeouts=%lu flags=%02X,%02X,%02X,%02X stop_ack=%02X disable_ack=%02X packets=%lu\r\n",(unsigned long)b->errors,(unsigned long)b->timeouts,b->flags[0],b->flags[1],b->flags[2],b->flags[3],b->stop_ack,b->disable_ack,(unsigned long)b->tx_packets);
 nav_logf("[CONFIG] ppm=%u,%u L_mm=%d latency_ms=%u,%u b0_ratio_permille=%d; RAM only\r\n",o->ppm_x,o->ppm_y,mm(o->lever_m),o->imu_delay_ms,o->encoder_delay_ms,mm(b0_ratio));
 nav_logf("[PERF] DWT max_cycles controller=%lu odom=%lu bus=%lu app_tick=%lu (168cycles/us); qp_fallback=%u qp_iterations=%u; pose uncertainty is NOT a calibrated confidence bound\r\n",(unsigned long)control_max,(unsigned long)odom_max,(unsigned long)bus_max,(unsigned long)loop_max,control.qp_fallbacks,control.qp_steps);
 nav_logf("[TRANSPORT] tx_fault=%u rx_resyncs=%lu gyro_age=%lums\r\n",b->tx_fault,(unsigned long)b->rx_resyncs,(unsigned long)(i->gyro_seen?nav_age(now,i->gyro_at):UINT32_MAX));
 nav_logf("[MPC] proven_infeasible_skips=%u (included in qp_fallback; same PI fallback)\r\n",control.qp_skips);
 nav_logf("[RESPONSE] estimate_ms=%u accepted_samples=%u; provisional effective actuator model, NOT a slip/ground-position measurement\r\n",(unsigned)(o->response_s*1000),o->response_samples);
 nav_logf("[RAW] FL4=%ld FR1=%ld RL3=%ld RR2=%ld reject=%02X ambiguous=%02X R_ratio=%d\r\n",(long)o->raw[0],(long)o->raw[1],(long)o->raw[2],(long)o->raw[3],o->rejected_mask,o->ambiguous_mask,(int)o->max_R);
}
void nav_app_init(uint32_t now)
{
 routing=false;memset(&route,0,sizeof route);blending=false;blend_time=0;turning=false;turn_target=turn_last_error=turn_wheel=0;turn_stable=0;phase=0;algorithm=0;ceiling=10;fused=false;xy=false;preparation_disabled=false;b0_ratio=1;reason="none";
 heading_reference=0;control_max=odom_max=bus_max=loop_max=0;last_log=now;zero_ack_at=0;
 nav_odom_init();imu_init();imu_service_init();nav_bus_init(now);
 nav_log("\r\n[BOOT] " NAV_VERSION " HSE25MHz F407VE168MHz; CHASSIS ONLY; NO PWM; boot FE+disable, no auto motion\r\n");
 nav_log("help | status | origin | algorithm cascade/adrc/lqr/mpc | run forward/back/left/right mm rpm | !\r\n");
 nav_log("CSV,t_ms,state,algorithm,mode,control,bx_mm,by_mm,fx_mm,fy_mm,yaw_mdeg,remaining_mm,cross_mm,wFL,wFR,wRL,wRR,pose_age,imu_age,reject,ambiguous,control_cycles,qp_fallback\r\n");
}
static void begin(float x,float y,unsigned rpm,uint32_t now,unsigned kind,float angle)
{
 bool rotation=kind==1;bool route_request=kind==3;
 const NavOdom *o=nav_odom_data();const NavBus *b=nav_bus_data();const WitImu *i=imu_data();
 float length=rotation?fabsf(wrapped(angle-nav_odom_heading_now()))*o->lever_m:hypotf(x-pose()->x_m,y-pose()->y_m);
 if(route_request)length=route.length;
 unsigned minimum=rpm>=100?1800:rpm>=75?800:rpm>=50?1200:rpm>=30?800:rpm>=20?400:50;
 if(phase||b->stopping||imu_service_busy()){nav_log("[E] idle required; stop/clear first; calibration must finish\r\n");return;}
 if(rpm<5||rpm>ceiling||rpm>(route_request?200U:rotation?10U:100U)||length<(rotation?.005f:minimum*.001f)||length>(route_request?25.0f:2.4f)){nav_logf("[E] RPM 5..%u; distance %u..2400mm for requested RPM\r\n",ceiling,minimum);return;}
 if(!feedback(now,false)||!pose()->valid||nav_age(now,pose()->at)>300){nav_log("[E] feedback/frame not ready (angle AND gyro required); inspect status; origin only at a known stationary marker\r\n");return;}
 if(!rotation&&fabsf(wrapped(nav_odom_heading_now()-heading_reference))>.0349066f){nav_log("[E] heading >2deg from origin; align vehicle and origin explicitly\r\n");return;}
 /* Real Emm firmware rejects F6 (even zero) while disabled. Never pretend
  * that rejection was an ACK. Require a coherent enable state; all-disabled
  * startup first verifies stationary feedback, then enables and sends F6zero.
  * All-enabled restart retains its initial zero-ACK gate. */
 unsigned enabled=0;for(unsigned k=0;k<4;k++)if(b->flags[k]&1)enabled++;
 if(enabled!=0&&enabled!=4){nav_log("[E] mixed driver enable states; stop then clear before run\r\n");return;}
 routing=route_request;blending=kind==2;turning=rotation;turn_target=angle;turn_stable=0;turn_wheel=0;turn_last_error=wrapped(angle-nav_odom_heading_now());
 preparation_disabled=enabled==0;
 goal[0]=x;goal[1]=y;requested_rpm=rpm;run_at=phase_at=now;bus_errors=b->errors;imu_errors=i->uart_errors;imu_checksums=i->bad_checksum;
 control_max=odom_max=bus_max=loop_max=0;zero_ack_at=0;
 deadline=(uint32_t)(length/(3200.0f/o->ppm_x/60*rpm)*2000)+15000;if(deadline>180000)deadline=180000;
 int zero[4]={0};zero_generation=preparation_disabled?0:nav_bus_velocity(zero);phase=1;reason="none";slip_since=0;
 nav_logf("[PREP] target_mm=%d,%d rpm_cap=%u algorithm=%s; disabled: stationary -> enable ACK -> F6zero ACK -> stationary; enabled: zero first; no driver reset\r\n",mm(x),mm(y),rpm,names[algorithm]);
}
void nav_app_command(const char *input,uint32_t now)
{
 char normalized[100];
 if(!normalize_command(input,normalized)){nav_log("[E] invalid/oversized command or integer overflow\r\n");return;}
 const char *line=normalized;
 char cmd[20],arg[20],extra;int a,b,angle,n=0;unsigned rpm;
 if(!strcmp(line,"!")||!strcmp(line,"stop")){nav_app_emergency(now,"operator stop");return;}
 if(!strcmp(line,"help")||!strcmp(line,"?")){
  nav_log("status | odom | perf | origin | clear | stop / !\r\nalgorithm cascade|adrc|lqr|mpc ; mode base|fused ; control hold|xy\r\npose X_MM Y_MM YAW_MDEG RPM (wide corridors only) ; turn YAW_MDEG 5|10 (absolute map yaw; no origin reset) ; limit 10|20|30|50|75|100|150|200 ; route plan 25..100 (NO move) ; route start (central, task stops only) ; run forward|back|left|right MM RPM ; goto X_MM Y_MM RPM\r\nodom scale FORWARD_PPM LATERAL_PPM ; odom delay IMU_MS ENCODER_MS ; odom lever MM\r\nadrc b0 PERMILLE (500..2000; default1000); all config RAM only\r\nimu / imu drift / imu tx on / imu cal acc / imu cal stop / imu cal check / imu save (idle only)\r\n");return;
 }
 if(!strcmp(line,"status")||!strcmp(line,"odom")||!strcmp(line,"perf")){
  report(now);nav_logf("[ROUTE] ready=%u active=%u samples=%u percent=%u predicted_ms=%lu elapsed_ms=%lu station=%s error_mm=%d yaw_error_mdeg=%d reason=%s; prediction is NOT measured duration\r\n",route.ready,routing,route.n,route.percent,(unsigned long)(route.duration*1000),(unsigned long)(route.elapsed*1000),nav_route_station(route.station),mm(route.error_m),(int)(route.yaw_error*57295.7795f),route.fault?route.fault:"none");return;
 }
 if(!strcmp(line,"imu")){const WitImu *i=imu_data();nav_logf("[IMU] yaw_mdeg=%ld gyro_mdps=%ld samples=%lu bad_checksum=%lu uart_errors=%lu\r\n",(long)i->yaw_mdeg,(long)i->z_mdps,(unsigned long)i->angle_samples,(unsigned long)i->bad_checksum,(unsigned long)i->uart_errors);imu_service_report(now);return;}
 if(!strcmp(line,"origin")){
  if(nav_app_active()||nav_bus_data()->stopping||imu_service_busy()||!imu_angle_fresh(now)){nav_log("[E] origin requires idle and fresh IMU; operator must align stationary ground marker\r\n");return;}
  nav_odom_rebind(true);heading_reference=0;nav_log("[ORIGIN] explicit physical marker declaration; XY/yaw=0; driver counters untouched; wait for fresh samples\r\n");return;
 }
 if(!strcmp(line,"clear")){
  if(nav_bus_data()->stopping||nav_app_active()){nav_log("[E] stop must finish first\r\n");return;}
  if(nav_odom_data()->fault){nav_log("[E] odometry frame lost; physically align then origin; do not silently resume old coordinates\r\n");return;}
  phase=0;reason="none";nav_log("[CLEAR] idle, no movement or enable\r\n");return;
 }
 if(phase||nav_bus_data()->stopping){nav_log("[E] configuration/run requires idle; stop then clear\r\n");return;}
 if(!strncmp(line,"imu ",4)){if(!imu_service_command(line,now))nav_log("[E] unknown IMU command\r\n");return;}
 if(sscanf(line,"%19s %19s %c",cmd,arg,&extra)==2){
  if(!strcmp(cmd,"algorithm")){for(unsigned k=0;k<4;k++)if(!strcmp(arg,names[k])){algorithm=k;nav_logf("[ALGORITHM] %s; no move; model Ts=100ms tau=160ms provisional\r\n",arg);return;}}
  if(!strcmp(cmd,"mode")&&(!strcmp(arg,"base")||!strcmp(arg,"fused"))){fused=!strcmp(arg,"fused");nav_logf("[MODE] %s; both estimators remain logged\r\n",arg);return;}
  if(!strcmp(cmd,"control")&&(!strcmp(arg,"hold")||!strcmp(arg,"xy"))){xy=!strcmp(arg,"xy");nav_logf("[CONTROL] %s\r\n",arg);return;}
 }
 if(sscanf(line,"limit %d %n",&a,&n)==1&&n>0&&!line[n]&&(a==10||a==20||a==30||a==50||a==75||a==100||a==150||a==200)){ceiling=(unsigned)a;nav_logf("[LIMIT] actual wheel cap ceiling=%u RPM; no move\r\n",ceiling);return;}
 n=0;if(sscanf(line,"route plan %d %n",&a,&n)==1&&n>0&&!line[n]&&a>=25&&a<=100){
  const NavOdom *q=nav_odom_data();
  uint32_t cycles=nav_hw_cycles();
  bool ok=nav_route_plan(&route,(unsigned)a,q->ppm_x,q->ppm_y,q->lever_m);
  cycles=nav_hw_cycles()-cycles;nav_logf("[ROUTE_PLAN_PERF] cycles=%lu; 168cycles/us on F407; no hardware measurement on a desktop build\r\n",(unsigned long)cycles);
  nav_logf("[ROUTE_PLAN] valid=%u samples=%u length_mm=%d model_ms=%lu cap=%uRPM yaw_max_mrad_s=%u; %s; NO motion\r\n",ok,route.n,mm(route.length),(unsigned long)(route.duration*1000),(unsigned)a*2,(unsigned)a*10,route.fault);return;
 }
 if(!strcmp(line,"route start")){
  if(!route.ready||route.index||route.elapsed||algorithm!=0||!xy||ceiling<route.percent*2||hypotf(pose()->x_m,pose()->y_m)>.020f||fabsf(nav_odom_heading_now())>.0174533f){nav_log("[E] route requires new valid plan, cascade+xy, sufficient limit, known zone1 WEST origin within20mm/1deg\r\n");return;}
  const NavOdom *q=nav_odom_data();
  if(fabsf(q->lever_m-route.lever)>.00001f||fabsf(3200.0f/q->ppm_x/60-route.rpm_mps)>.00000001f||fabsf((float)q->ppm_y/q->ppm_x-route.scale_y)>.000001f){nav_log("[E] odometry config changed; route plan again\r\n");return;}
  begin(0,0,route.percent*2,now,3,0);return;
 }
 n=0;if(sscanf(line,"adrc b0 %d %n",&a,&n)==1&&n>0&&!line[n]&&a>=500&&a<=2000){b0_ratio=a*.001f;nav_log("[ADRC] b0 ratio set; no move\r\n");return;}
 const NavOdom *o=nav_odom_data();unsigned x=o->ppm_x,y=o->ppm_y,im=o->imu_delay_ms,en=o->encoder_delay_ms;float lever=o->lever_m;bool config=false;
 n=0;if(sscanf(line,"odom scale %d %d %n",&a,&b,&n)==2&&n>0&&!line[n]){x=(unsigned)a;y=(unsigned)b;config=true;}
 n=0;if(sscanf(line,"odom delay %d %d %n",&a,&b,&n)==2&&n>0&&!line[n]){im=(unsigned)a;en=(unsigned)b;config=true;}
 n=0;if(sscanf(line,"odom lever %d %n",&a,&n)==1&&n>0&&!line[n]){lever=a*.001f;config=true;}
 if(config){nav_log(nav_odom_config(x,y,im,en,lever)?"[CONFIG] changed; map preserved, velocity baselines rebound; wait feedback\r\n":"[E] config range or odometry frame fault; align physical origin after frame loss\r\n");return;}
 n=0;if(sscanf(line,"run %19s %d %u %n",arg,&a,&rpm,&n)==3&&n>0&&!line[n]&&a>0){
  float dx=0,dy=0;if(!strcmp(arg,"forward"))dx=a*.001f;else if(!strcmp(arg,"back"))dx=-a*.001f;else if(!strcmp(arg,"left"))dy=a*.001f;else if(!strcmp(arg,"right"))dy=-a*.001f;else{nav_log("[E] direction forward/back/left/right\r\n");return;}
  begin(pose()->x_m+dx,pose()->y_m+dy,rpm,now,false,0);return;
 }
 n=0;if(sscanf(line,"pose %d %d %d %u %n",&a,&b,&angle,&rpm,&n)==4&&n>0&&!line[n]&&angle>=-180000&&angle<=180000){
  if(!xy||algorithm!=0){nav_log("[E] pose requires cascade and control xy\r\n");return;}
  begin(a*.001f,b*.001f,rpm,now,2,angle*.00001745329252f);return;
 }
 n=0;if(sscanf(line,"turn %d %u %n",&a,&rpm,&n)==2&&n>0&&!line[n]&&a>=-180000&&a<=180000){
  begin(pose()->x_m,pose()->y_m,rpm,now,true,a*.00001745329252f);return;
 }
 n=0;if(sscanf(line,"goto %d %d %u %n",&a,&b,&rpm,&n)==3&&n>0&&!line[n]){begin(a*.001f,b*.001f,rpm,now,false,0);return;}
 nav_log("[E] command/arguments invalid; help\r\n");
}
void nav_app_tick(uint32_t now)
{
 uint32_t start=nav_hw_cycles(),t=start;nav_odom_tick(now);uint32_t elapsed=nav_hw_cycles()-t;if(elapsed>odom_max)odom_max=elapsed;
 imu_service_tick(now);
 const NavBus *b=nav_bus_data();const NavOdom *o=nav_odom_data();const WitImu *i=imu_data();
 if(nav_app_active()){
  if(b->errors!=bus_errors){nav_app_emergency(now,"motor transaction/ACK/UART failure");goto end;}
  if(i->uart_errors!=imu_errors || i->bad_checksum-imu_checksums>=3){nav_app_emergency(now,"new IMU stream errors");goto end;}
  if(!i->angle_seen || nav_age(now,i->angle_at)>300 || !i->gyro_seen || nav_age(now,i->gyro_at)>300 || nav_age(now,run_at)>deadline){nav_app_emergency(now,"angle/gyro stale or run deadline");goto end;}
  if(phase<4 && nav_age(now,phase_at)>5000){nav_app_emergency(now,"preparation timeout");goto end;}
 }
 if(phase==1&&(preparation_disabled||b->velocity_done==zero_generation)){
  if(!zero_ack_at)zero_ack_at=now;
  bool all_new=true;for(unsigned k=0;k<4;k++)if((int32_t)(b->position_at[k]-zero_ack_at)<=0)all_new=false;
  if(all_new&&nav_age(now,zero_ack_at)>=250&&o->quiet>=4&&feedback(now,false)){
   if(preparation_disabled){
    enable_generation=b->enable_done;
    if(nav_bus_enable()){nav_odom_rebind(false);phase=2;phase_at=now;}
   }else{
    /* No F3, no history/filter reset, no discarded displacement. */
    phase=3;phase_at=now;
   }
  }
 }else if(phase==2&&b->enable_done!=enable_generation){
  nav_odom_rebind(false);zero_generation=nav_bus_velocity((int[4]){0,0,0,0});phase=3;phase_at=now;
 }
 else if(phase==3){
  bool all_new=true;for(unsigned k=0;k<4;k++)if((int32_t)(b->position_at[k]-phase_at)<=0 || (int32_t)(b->flags_at[k]-phase_at)<=0)all_new=false;
  if(b->velocity_done==zero_generation&&all_new&&nav_age(now,phase_at)>=600&&feedback(now,true)&&pose()->valid&&o->quiet>=4){
   nav_control_begin(&control,pose(),goal[0],goal[1],requested_rpm,xy);
   nav_control_config(&control,algorithm,o->ppm_x,o->ppm_y,o->lever_m,b0_ratio);
   if(routing)deadline=(uint32_t)(route.duration*2000+30000);
   if(blending){
    blend_start=nav_odom_heading_now();blend_delta=wrapped(turn_target-blend_start);blend_time=0;blend_rate=1;
    /* Quintic yaw reference: zero initial/final yaw rate and acceleration.
     * max derivative is 1.875; cap reference at .16 rad/s.
     * Reserve translation time instead of demanding impossible wheel sums. */
    blend_duration=fmaxf(3.0f,fmaxf(1.875f*fabsf(blend_delta)/(requested_rpm>50?.20f:.16f),control.length/(requested_rpm*control.rpm_mps*.65f)));
    control.translation_limit=fminf(requested_rpm*control.rpm_mps,control.length/blend_duration);
    float needed=blend_duration*2000+20000;if(needed>deadline)deadline=(uint32_t)fminf(180000,needed);
   }
   phase=4;phase_at=last_control=now;nav_log("[RUN] continuous F6; stationary post-enable baselines rebound; map preserved\r\n");
  }
 }
 if(phase==4||phase==5){
  if(!feedback(now,true)||!pose()->valid||nav_age(now,pose()->at)>300){nav_app_emergency(now,"feedback/frame stale or driver stall/disabled/protection");goto end;}
  if(!routing&&!turning&&fabsf(wrapped(nav_odom_heading_now()-heading_reference))>.0523599f){nav_app_emergency(now,"heading deviation >3deg");goto end;}
  if(o->rejected_mask||o->ambiguous_mask){if(!slip_since)slip_since=now;else if(now-slip_since>500){nav_app_emergency(now,"persistent wheel inconsistency; inspect traction/geometry");goto end;}}else slip_since=0;
 }
 if(phase==4 && now-last_control>=NAV_TICK_MS){
  float dt=(now-last_control)*.001f;if(dt>.15f){nav_app_emergency(now,"controller period >150ms");goto end;}last_control=now;
  float gyro=i->z_mdps*.00001745329252f; /* Freshness required above; never silently substitute zero. */
  if(blending){
   /* Pause yaw clock if tracking lags, retaining the existing 3deg guard. */
   float lag=wrapped(heading_reference-nav_odom_heading_now())*(blend_delta>=0?1.0f:-1.0f);
   /* Freeze only when the vehicle LAGS. Freezing an AHEAD reference
    * makes its error grow while the chassis is still rotating. */
   float desired_rate=fmaxf(0,fminf(1.2f,1-lag/.03f));
   blend_rate+=fmaxf(-1.5f*dt,fminf(1.5f*dt,desired_rate-blend_rate));
   blend_time=fminf(blend_duration,blend_time+dt*blend_rate);
   float q=blend_time/blend_duration,q2=q*q,q3=q2*q;
   float h=q3*(10+q*(-15+6*q));
   heading_reference=blend_start+blend_delta*h;
   control.yaw_ff=blend_time<blend_duration?blend_delta*(30*q2*(1-q)*(1-q))*blend_rate/blend_duration:0;
   nav_logf("H,%lu,%d,%d,%d,%d\r\n",(unsigned long)now,(int)(heading_reference*57295.7795f),(int)(nav_odom_heading_now()*57295.7795f),(int)(control.yaw_ff*57295.7795f),(int)(lag*57295.7795f));
  }
  int wheel[4];t=nav_hw_cycles();int result;
  if(routing){
   NavPose predicted;
   if(!nav_odom_predict(fused,now,gyro,&predicted)){nav_app_emergency(now,"route prediction horizon/feedback invalid");goto end;}
   route.response_s=o->response_s;
   unsigned station=route.station;result=nav_route_step(&route,&predicted,gyro,dt,wheel);
   for(unsigned k=0;k<4;k++)control.wheel[k]=route.wheel[k];
   control.remaining=route.length-route.p[route.index].s;control.cross_error=route.error_m;
   if(station!=route.station)nav_logf("[STATION] %u %s elapsed_ms=%lu; stopped; no origin reset\r\n",route.station,nav_route_station(route.station),(unsigned long)(route.elapsed*1000));
   if(result<0){nav_app_emergency(now,route.fault);goto end;}
   if(result==1){goal[0]=route.p[route.n-1].x;goal[1]=route.p[route.n-1].y;heading_reference=route.p[route.n-1].h;routing=false;}
  }else if(turning){
   float error=wrapped(turn_target-nav_odom_heading_now());
   float ae=fabsf(error),speed=fabsf(gyro),predicted=error-gyro*.35f;
   if(ae>fabsf(turn_last_error)+.035f){nav_app_emergency(now,"turn error increased >2deg");goto end;}
   if(hypotf(pose()->x_m-goal[0],pose()->y_m-goal[1])>.04f){nav_app_emergency(now,"turn translation >40mm");goto end;}
   bool close=ae<=.006981317f;
   float command=0;
   if(!close && error*predicted>0){
    float w=fminf((float)requested_rpm,fmaxf(1.0f,ae*o->lever_m/(3200.0f/o->ppm_x/60)*1.2f));
    command=error>0?w:-w;
   }
   float delta=command-turn_wheel,bound=20*dt;
   turn_wheel+=fmaxf(-bound,fminf(bound,delta));
   int v=(int)roundf(turn_wheel);wheel[0]=-v;wheel[1]=v;wheel[2]=-v;wheel[3]=v;
   if(close&&speed<.0174533f&&v==0)turn_stable++;else turn_stable=0;
   control.remaining=error;control.cross_error=hypotf(pose()->x_m-goal[0],pose()->y_m-goal[1]);
   for(unsigned k=0;k<4;k++)control.wheel[k]=(float)wheel[k];
   result=turn_stable>=3?1:0;
  }else result=nav_control_step(&control,pose(),wrapped(nav_odom_heading_now()-heading_reference),gyro,dt,wheel);
  elapsed=nav_hw_cycles()-t;if(elapsed>control_max)control_max=elapsed;
  nav_logf("C,%lu,%u,%d,%lu,%u,%u\r\n",(unsigned long)now,(unsigned)roundf(dt*1000),result,(unsigned long)elapsed,control.qp_steps,control.qp_fallbacks);
  if(elapsed>1680000U){nav_app_emergency(now,"controller >10ms computation budget");goto end;}
  if(result<0){nav_app_emergency(now,"controller invalid state or >15mm overshoot");goto end;}
  if(result==1){int zero[4]={0};zero_generation=nav_bus_velocity(zero);phase=5;phase_at=now;nav_log("[SETTLE] zero speed; no FE restart tail\r\n");}
  else nav_bus_velocity(wheel);
 }
 if(phase==5&&b->velocity_done==zero_generation&&nav_age(now,phase_at)>=600){
  float dx=goal[0]-pose()->x_m,dy=goal[1]-pose()->y_m;
  if(hypotf(pose()->vx_mps,pose()->vy_mps)<.003f){
   if((!turning&&xy&&hypotf(dx,dy)>.012f)||(turning&&hypotf(dx,dy)>.04f)||fabsf(wrapped(nav_odom_heading_now()-(turning?turn_target:heading_reference)))>.0174533f||i->z_mdps>1000||i->z_mdps< -1000){nav_app_emergency(now,"settled endpoint or heading/rate out of tolerance");goto end;}
   if(turning){heading_reference=turn_target;turning=false;nav_logf("[TURN_DONE] target_mdeg=%d; XY/origin preserved\r\n",(int)(turn_target*57295.7795f));}
   if(blending){heading_reference=turn_target;blending=false;nav_log("[POSE_DONE] heading and XY settled; world frame preserved\r\n");}
   phase=0;nav_logf("[DONE] estimated_error_mm=%d,%d yaw_mdeg=%d elapsed_ms=%lu; measure GROUND error independently\r\n",mm(dx),mm(dy),(int)(wrapped(nav_odom_heading_now()-heading_reference)*57295.7795f),(unsigned long)(now-run_at));
  }else if(now-phase_at>3000)nav_app_emergency(now,"zero-command settling timeout");
 }
 if(now-last_log>=200){last_log=now;
  nav_logf("D,%lu,%u,%u,%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%lu,%lu,%u,%u,%lu,%u\r\n",(unsigned long)now,phase,algorithm,fused,xy,mm(o->base.x_m),mm(o->base.y_m),mm(o->fused.x_m),mm(o->fused.y_m),(int)(nav_odom_heading_now()*57295.7795f),mm(control.remaining),mm(control.cross_error),(int)roundf(control.wheel[0]),(int)roundf(control.wheel[1]),(int)roundf(control.wheel[2]),(int)roundf(control.wheel[3]),(unsigned long)(now-o->base.at),(unsigned long)(i->angle_seen?now-i->angle_at:UINT32_MAX),o->rejected_mask,o->ambiguous_mask,(unsigned long)control_max,control.qp_fallbacks);
 }
end:
 /* Evaluate received faults BEFORE servicing a queued nonzero F6. Timeouts
  * discovered by the bus return without another write, then enter stop here. */
 t=nav_hw_cycles();nav_bus_tick(now);elapsed=nav_hw_cycles()-t;if(elapsed>bus_max)bus_max=elapsed;
 if(nav_app_active()&&b->errors!=bus_errors){nav_app_emergency(now,"motor transaction/ACK/UART failure");nav_bus_tick(now);}
 elapsed=nav_hw_cycles()-start;if(elapsed>loop_max)loop_max=elapsed;
}
