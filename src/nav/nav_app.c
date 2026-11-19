#include "nav/studio_profile.h"
#include "nav/nav_app.h"
#include "nav/nav_control.h"
#include "nav/nav_odom.h"
#include "nav/nav_lidar.h"
#include "nav/nav_lidar_map.h"
#include "nav/nav_bus.h"
#include "nav/nav_platform.h"
#include "nav/nav_time.h"
#include "nav/nav_route.h"
#include "nav/nav_map.h"
#include "nav/nav_arm.h"
#include "nav/nav_arm2.h"
#include "nav/nav_vision.h"
#include "nav/nav_mission.h"
#include "wit_imu.h"
#include "imu_service.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* 0 idle;1 zero ACK / disabled stationary;2 enable ACK;3 zero ACK + stationary;
 * 4 driving;5 zero-speed settling;6 fault. No automatic multi-leg motion. */
static unsigned phase,algorithm,ceiling;
static bool avoid_enabled,replan_busy,route_from_lidar,replan_plan_ready,survey_route;
static bool qr_catalog_transition;
static bool qr_anchor_requested;
static bool qr_anchor_accumulating;
static unsigned detours;
static uint32_t replan_at,latest_tick,replan_finished,replan_cycles,lidar_start_since;
bool nav_app_replanning(void){return replan_busy&&phase==7;}

static bool turning,blending,routing,preflight_only;
static NavRoute route;
static bool calibrating,segmented,work_translation;
static unsigned station_stage;
static uint32_t station_at,wait_at;
static uint32_t route_wait_ms;
bool nav_app_station_wait(void){return routing&&route.waiting;}
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
/* v2.7.34: consistency flags are statistical slip/model evidence, not proof of
 * a hardware fault. Default behaviour logs and slows the route reference;
 * explicit strict diagnostic mode retains the old stop policy. Actual driver
 * stall/protection, feedback loss, collision and operator stop are independent. */
static bool wheel_guard_strict;
static uint8_t wheel_rej_bits;
static uint32_t wheel_rej_since,wheel_amb_since,wheel_log_at;
static uint32_t traction_at;
static uint32_t traction_update_at;
static float traction_scale=1;
float nav_app_traction_scale(void){return traction_scale;}
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
bool nav_app_indexing(void){return work_translation&&phase>0&&phase<6;}
bool nav_app_segmented(void){return segmented;}
bool nav_app_official_plan_ready(void)
{return route.ready&&route.n>1&&route_from_lidar&&avoid_enabled&&!survey_route&&route.p[route.n-1].stop==9;}
void nav_app_pose(float out[3]){out[0]=pose()->x_m;out[1]=pose()->y_m;out[2]=nav_odom_heading_now();}
unsigned nav_app_work_station(void){return phase==0&&segmented&&route.waiting&&!route.finished?route.station:0;}
bool nav_app_station_target(float out[3])
{if(!nav_app_work_station())return false;out[0]=route.p[route.index].x;out[1]=route.p[route.index].y;out[2]=route.p[route.index].h;return true;}
bool nav_app_work_clear(float x,float y)
{
 float center[3];if(!nav_app_station_target(center)||route.station==9||hypotf(x-center[0],y-center[1])>.18f)return false;
 float sx=pose()->x_m,sy=pose()->y_m,yaw=nav_odom_heading_now();
 unsigned n=(unsigned)ceilf(hypotf(x-sx,y-sy)/.005f)+1;
 for(unsigned k=0;k<=n;k++)if(!nav_route_footprint(sx+(x-sx)*k/n,sy+(y-sy)*k/n,yaw,.015f))return false;
 return true;
}
bool nav_app_upcoming(float out[3],unsigned *station,float *remaining)
{
 if(!routing||!nav_app_active()||route.finished)return false;
 for(unsigned k=route.index+1;k<route.n;k++)if(route.p[k].stop){out[0]=route.p[k].x;out[1]=route.p[k].y;out[2]=route.p[k].h;*station=route.p[k].stop;*remaining=hypotf(pose()->x_m-out[0],pose()->y_m-out[1]);return true;}
 return false;
}
bool nav_app_arm_approach_clear(float radius,float ox,float oy)
{
 if(!routing||!nav_app_active()||radius<.15f)return false;
 float h=nav_odom_heading_now();if(!nav_map_arm_safe(pose()->x_m,pose()->y_m,h,radius,ox,oy))return false;
 for(unsigned k=route.index;k<route.n;k++){
  if(fabsf(wrapped(route.p[k].h-h))>.02f||!nav_map_arm_safe(route.p[k].x,route.p[k].y,route.p[k].h,radius,ox,oy))return false;
  if(k>route.index&&route.p[k].stop)return route.p[k].stop==1||route.p[k].stop==4;
 }
 return false;
}
bool nav_app_active(void){return phase>0&&(phase<6||phase==7);}
void nav_app_emergency(uint32_t now,const char *why)
{
 if(phase==6&&nav_bus_data()->stopping)return;
 const NavBus *snap=nav_bus_data();
 nav_logf("[FAULT_SNAPSHOT] flags=%02X,%02X,%02X,%02X pos_age=%lu,%lu,%lu,%lu flag_age=%lu,%lu,%lu,%lu odom_fault=%lu\r\n",snap->flags[0],snap->flags[1],snap->flags[2],snap->flags[3],(unsigned long)(now-snap->position_at[0]),(unsigned long)(now-snap->position_at[1]),(unsigned long)(now-snap->position_at[2]),(unsigned long)(now-snap->position_at[3]),(unsigned long)(now-snap->flags_at[0]),(unsigned long)(now-snap->flags_at[1]),(unsigned long)(now-snap->flags_at[2]),(unsigned long)(now-snap->flags_at[3]),(unsigned long)nav_odom_data()->fault);
 replan_busy=false;routing=false;station_stage=0;phase=6;reason=why;nav_bus_emergency(now);nav_arm_stop();nav_logf("[FAULT] %s; FE+disable queued for ALL four; no automatic resume\r\n",why);
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
 nav_logf("[LIFT_BUS] errors=%lu timeouts=%lu FE5_ack=%u; four-wheel ACK mask excludes lift\r\n",(unsigned long)b->lift_errors,(unsigned long)b->lift_timeouts,b->lift_stop_ack);
 nav_logf("[CONFIG] ppm=%u,%u L_mm=%d latency_ms=%u,%u b0_ratio_permille=%d; RAM only\r\n",o->ppm_x,o->ppm_y,mm(o->lever_m),o->imu_delay_ms,o->encoder_delay_ms,mm(b0_ratio));
 nav_logf("[PERF] DWT max_cycles controller=%lu odom=%lu bus=%lu app_tick=%lu (168cycles/us); qp_fallback=%u qp_iterations=%u; pose uncertainty is NOT a calibrated confidence bound\r\n",(unsigned long)control_max,(unsigned long)odom_max,(unsigned long)bus_max,(unsigned long)loop_max,control.qp_fallbacks,control.qp_steps);
 nav_logf("[TRANSPORT] tx_fault=%u rx_resyncs=%lu gyro_age=%lums\r\n",b->tx_fault,(unsigned long)b->rx_resyncs,(unsigned long)(i->gyro_seen?nav_age(now,i->gyro_at):UINT32_MAX));
 nav_logf("[MPC] proven_infeasible_skips=%u (included in qp_fallback; same PI fallback)\r\n",control.qp_skips);
 nav_logf("[RESPONSE] estimate_ms=%u accepted_samples=%u; provisional effective actuator model, NOT a slip/ground-position measurement\r\n",(unsigned)(o->response_s*1000),o->response_samples);
 nav_logf("[RAW] FL4=%ld FR1=%ld RL3=%ld RR2=%ld reject=%02X ambiguous=%02X R_ratio=%d\r\n",(long)o->raw[0],(long)o->raw[1],(long)o->raw[2],(long)o->raw[3],o->rejected_mask,o->ambiguous_mask,(int)o->max_R);
 nav_logf("[ODOM_MODEL] angular_disagreement=%u samples=%u wheel_null_mmps=%d; angular model/delay disagreement is not identified wheel slip\r\n",o->angular_model,o->angular_model_samples,mm(o->null_residual_mps));
 nav_logf("[TRACTION] mode=%s reference_permille=%u; statistical wheel rejection is not a driver stall\r\n",wheel_guard_strict?"strict":"continue",(unsigned)roundf(traction_scale*1000));
}
void nav_app_init(uint32_t now)
{
 avoid_enabled=replan_busy=route_from_lidar=replan_plan_ready=survey_route=false;detours=0;latest_tick=now;
 qr_catalog_transition=false;
 qr_anchor_requested=false;qr_anchor_accumulating=false;
 routing=preflight_only=false;memset(&route,0,sizeof route);calibrating=segmented=work_translation=false;station_stage=0;route_wait_ms=0;wait_at=now;blending=false;blend_time=0;turning=false;turn_target=turn_last_error=turn_wheel=0;turn_stable=0;phase=0;algorithm=STUDIO_ALGORITHM;ceiling=STUDIO_ROUTE_RPM;fused=STUDIO_FUSED;xy=false;preparation_disabled=false;b0_ratio=1;reason="none";
 heading_reference=0;control_max=odom_max=bus_max=loop_max=0;last_log=now;zero_ack_at=0;
 wheel_guard_strict=false;wheel_rej_bits=0;wheel_rej_since=wheel_amb_since=wheel_log_at=0;
 traction_at=0;traction_scale=1;traction_update_at=now;
 nav_odom_init();imu_init();imu_service_init();nav_bus_init(now);
 nav_log("\r\n[BOOT] " NAV_VERSION " HSE25MHz F407VE168MHz; SHARED Emm1..5; PC6..PC9 PWM OFF; boot FE all +disable WHEELS only; no auto motion\r\n");
 nav_log("help | status | origin | algorithm cascade/adrc/lqr/mpc | run forward/back/left/right mm rpm | !\r\n");
 nav_log("CSV,t_ms,state,algorithm,mode,control,bx_mm,by_mm,fx_mm,fy_mm,yaw_mdeg,remaining_mm,cross_mm,wFL,wFR,wRL,wRR,pose_age,imu_age,reject,ambiguous,control_cycles,qp_fallback\r\n");
}
static void begin(float x,float y,unsigned rpm,uint32_t now,unsigned kind,float angle)
{
 bool rotation=kind==1;bool route_request=kind==3;if(!route_request)preflight_only=false;bool work_request=kind==4;
 const NavOdom *o=nav_odom_data();const NavBus *b=nav_bus_data();const WitImu *i=imu_data();
 float length=rotation?fabsf(wrapped(angle-nav_odom_heading_now()))*o->lever_m:hypotf(x-pose()->x_m,y-pose()->y_m);
 if(route_request)length=route.length;
 /* A sampled route already owns its acceleration and speed limits. The
  * single-leg bench minimum must not reject a shorter QR-only survey. */
 unsigned minimum=route_request?5:work_request?8:(rpm>=100?1800:rpm>=75?800:rpm>=50?1200:rpm>=30?800:rpm>=20?400:50);
 if(nav_arm_busy()){nav_log("[E] finish arm/lift activity before driving\r\n");return;}
 if(((route_request&&segmented)||work_request)&&!nav_arm_transport_ready()&&!(work_request&&nav_mission_work_pose())){nav_log("[E] transport requires referenced lift at clearance and turn1000; no travel\r\n");return;}
 nav_arm_suspend_reads();
 if(route_request&&calibrating&&!nav_arm_calibration_ready()){nav_log("[E] calibrating requires known servo2 at1000 and turnlimits1000..2130; set physical pose/limits before route\r\n");return;}
 if(phase||b->stopping||imu_service_busy()){nav_log("[E] idle required; stop/clear first; calibration must finish\r\n");return;}
 if((work_request&&(rpm>75||length>.40f))||rpm<5||rpm>ceiling||rpm>(route_request?400U:rotation?50U:100U)||length<(rotation?.005f:minimum*.001f)||length>(route_request?50.0f:2.4f)){nav_logf("[E] RPM 5..%u; route metric<=50m; single-leg distance %u..2400mm for requested RPM\r\n",ceiling,minimum);return;}
 if(!feedback(now,false)||!pose()->valid||nav_age(now,pose()->at)>300){nav_log("[E] feedback/frame not ready (angle AND gyro required); inspect status; origin only at a known stationary marker\r\n");return;}
 if(!rotation&&fabsf(wrapped(nav_odom_heading_now()-heading_reference))>.0349066f){nav_log("[E] heading >2deg from origin; align vehicle and origin explicitly\r\n");return;}
 /* Real Emm firmware rejects F6 (even zero) while disabled. Never pretend
  * that rejection was an ACK. Require a coherent enable state; all-disabled
  * startup first verifies stationary feedback, then enables and sends F6zero.
  * All-enabled restart retains its initial zero-ACK gate. */
 unsigned enabled=0;for(unsigned k=0;k<4;k++)if(b->flags[k]&1)enabled++;
 if(enabled!=0&&enabled!=4){nav_log("[E] mixed driver enable states; stop then clear before run\r\n");return;}
 lidar_start_since=0;work_translation=work_request;routing=route_request;blending=kind==2;turning=rotation;turn_target=angle;turn_stable=0;turn_wheel=0;turn_last_error=wrapped(angle-nav_odom_heading_now());
 preparation_disabled=enabled==0;
 goal[0]=x;goal[1]=y;requested_rpm=rpm;run_at=phase_at=now;route_wait_ms=0;bus_errors=b->errors;imu_errors=i->uart_errors;imu_checksums=i->bad_checksum;
 control_max=odom_max=bus_max=loop_max=0;zero_ack_at=0;
 deadline=(uint32_t)(length/(3200.0f/o->ppm_x/60*rpm)*2000)+15000;if(deadline>180000)deadline=180000;
 if(work_request&&enabled==4&&nav_bus_zero_acked()&&o->quiet>=1&&hypotf(pose()->vx_mps,pose()->vy_mps)<.003f&&i->z_mdps<500&&i->z_mdps> -500){
  nav_control_begin(&control,pose(),goal[0],goal[1],requested_rpm,xy);
  nav_control_config(&control,algorithm,o->ppm_x,o->ppm_y,o->lever_m,b0_ratio);control.work_move=true;control.work_mode=nav_mission_indexing_mode();control.work_accel=nav_mission_index_accel()*.001f;control.work_jerk=nav_mission_index_jerk()*.001f;
  phase=4;phase_at=last_control=now;nav_log("[RUN_REUSE] work move: zero ACK+fresh stationary feedback retained; no F3 or repeated baseline wait\r\n");return;
 }
 int zero[4]={0};zero_generation=preparation_disabled?0:nav_bus_velocity(zero);phase=1;reason="none";slip_since=0;wheel_rej_bits=0;wheel_rej_since=0;wheel_amb_since=0;
 nav_logf("[PREP] target_mm=%d,%d rpm_cap=%u algorithm=%s; disabled: stationary -> enable ACK -> F6zero ACK -> stationary; enabled: zero first; no driver reset\r\n",mm(x),mm(y),rpm,names[algorithm]);
}
void nav_app_command(const char *input,uint32_t now)
{
 if(!strncmp(input,"vision",6)){
  if(nav_mission_owns_vision())nav_log("[VISION_BUSY] official mission owns camera requests\r\n");
  else if(!nav_vision_command(input,now))nav_log("[E] unknown vision command\r\n");
  return;
 }
 char normalized[100];
 if(!normalize_command(input,normalized)){nav_log("[E] invalid/oversized command or integer overflow\r\n");return;}
 input=normalized;
 if(!strcmp(input,"route next")){
  if(!routing||!route.waiting||station_stage!=3){nav_log("[E] next requires CAL_WAIT after outward servo settled\r\n");return;}
  if(!nav_arm_turn(1000)){nav_log("[E] turn retraction blocked; inspect arm\r\n");return;}
  station_stage=4;station_at=now;nav_log("[CAL_RETRACT] staying parked until servo1000 settles\r\n");return;
 }
 if(!strcmp(input,"route pause")){
  if(!routing||segmented||calibrating){nav_log("[E] route pause needs standalone active route\r\n");return;}
  route.manual=true;nav_log("[ROUTE_PAUSE_REQUEST] stop at next planned work station; no origin change\r\n");return;
 }
 if(!strcmp(input,"route resume")){
  if(!routing||!route.waiting||segmented||calibrating){nav_log("[E] no standalone route breakpoint\r\n");return;}
  route.manual=false;route.waiting=false;nav_log("[ROUTE_RESUMED] same trajectory/frame\r\n");return;
 }
 if(nav_mission_command(input,now))return;
 if(nav_arm2_command(input))return;
 if(nav_arm_command(input))return;

 const char *line=normalized;
 char cmd[20],arg[20],extra;int a,b,angle,n=0;unsigned rpm;
 if(!strcmp(line,"!")||!strcmp(line,"stop")||!strcmp(line,"route cancel")){nav_app_emergency(now,"operator stop");return;}
 if(!strcmp(line,"help")||!strcmp(line,"?")){
  nav_log("nav1j: map | footprint FRONT REAR LEFT RIGHT (mm) | station offset raw/rough/left DX DY (world local mm)\r\nvision console|uart|request ROUND|rx HEX_FRAME ; route calibrate on|off ; route next ; route cancel\r\narm status ; seed/servo/rate ID VALUE (IDs1..4 PC6..PC9) ; point view|qr ; height view|qr ; unrough [1|2|3]\r\n");
  nav_log("status | odom | perf | origin | clear | stop / !\r\nalgorithm cascade|adrc|lqr|mpc ; mode base|fused ; control hold|xy\r\npose X_MM Y_MM YAW_MDEG RPM (wide corridors only) ; turn YAW_MDEG 5|10|20|30|50 (absolute map yaw; no origin reset) ; limit 10|20|30|50|75|100|150|200 ; route plan 25..100 (NO move) ; route start (central, task stops only) ; run forward|back|left|right MM RPM ; goto X_MM Y_MM RPM\r\nodom scale FORWARD_PPM LATERAL_PPM ; odom delay IMU_MS ENCODER_MS ; odom lever MM\r\nadrc b0 PERMILLE (500..2000; default1000); all config RAM only\r\nimu / imu drift / imu tx on / imu cal acc / imu cal stop / imu cal check / imu save (idle only)\r\n");return;
 }
 if(!strcmp(line,"status")||!strcmp(line,"odom")||!strcmp(line,"perf")){
  nav_logf("[CAL_STATE] enabled=%u waiting=%u stage=%u wait_ms=%lu; next only atstage3\r\n",calibrating,route.waiting,station_stage,(unsigned long)route_wait_ms);
  report(now);nav_logf("[DETOUR] enabled=%u stopped=%u planning=%u attempts=%u; max6, no world reset\r\n",avoid_enabled,phase==7,replan_busy,detours);
 nav_logf("[ROUTE] ready=%u active=%u samples=%u percent=%u predicted_ms=%lu elapsed_ms=%lu station=%s error_mm=%d yaw_error_mdeg=%d reason=%s; prediction is NOT measured duration\r\n",route.ready,routing,route.n,route.percent,(unsigned long)(route.duration*1000),(unsigned long)(route.elapsed*1000),nav_route_station(route.station),mm(route.error_m),(int)(route.yaw_error*57295.7795f),route.fault?route.fault:"none");return;
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
 /* Permit explicit IMU test/read cleanup after a completed emergency stop,
  * WITHOUT clearing the vehicle fault, moving wheels or re-zeroing a map. */
 if(phase==6&&!nav_bus_data()->stopping&&nav_bus_data()->stop_ack==15&&nav_bus_data()->disable_ack==15&&
    (!strncmp(line,"imu test ",9)||!strncmp(line,"imu read ",9)||!strncmp(line,"imu tx ",7)||!strcmp(line,"imu samples"))){
  if(!imu_service_command(line,now))nav_log("[E] unknown stopped IMU command\r\n");
  return;
 }
 if(!strcmp(line,"lidar status")||!strcmp(line,"lidar tracks")||!strcmp(line,"lidar memory status")||!strcmp(line,"lidar memory list")||!strcmp(line,"lidar plan")||!strcmp(line,"lidar plan live")||!strcmp(line,"lidar plan memory")||!strcmp(line,"lidar hazard")||!strcmp(line,"lidar hazard live")||!strcmp(line,"lidar hazard near")||!strcmp(line,"lidar hazard memory")){(void)nav_lidar_command(line,false,now);return;}
 if(phase||nav_bus_data()->stopping||nav_arm_busy()){nav_log("[E] configuration/run requires chassis and arm idle; stop then clear\r\n");return;}
 if(!strncmp(line,"imu ",4)){if(!imu_service_command(line,now))nav_log("[E] unknown IMU command\r\n");return;}
 if(sscanf(line,"%19s %19s %c",cmd,arg,&extra)==2){
  if(!strcmp(cmd,"algorithm")){for(unsigned k=0;k<4;k++)if(!strcmp(arg,names[k])){algorithm=k;nav_logf("[ALGORITHM] %s; no move; model Ts=100ms tau=160ms provisional\r\n",arg);return;}}
  if(!strcmp(cmd,"mode")&&(!strcmp(arg,"base")||!strcmp(arg,"fused"))){fused=!strcmp(arg,"fused");nav_logf("[MODE] %s; both estimators remain logged\r\n",arg);return;}
  if(!strcmp(cmd,"control")&&(!strcmp(arg,"hold")||!strcmp(arg,"xy"))){xy=!strcmp(arg,"xy");nav_logf("[CONTROL] %s\r\n",arg);return;}
 }
 if(!strcmp(line,"route smooth legacy")||!strcmp(line,"route smooth trapezoid")||!strcmp(line,"route smooth s")){
  unsigned profile=!strcmp(line,"route smooth s")?2:!strcmp(line,"route smooth trapezoid")?1:0;
  nav_route_smooth(profile);nav_logf("[ROUTE_SMOOTH] profile=%u; plan again; RAM only; no motion\r\n",profile);return;
 }
 if(!strcmp(line,"route lowyaw on")||!strcmp(line,"route lowyaw off")){
  nav_route_lowyaw(!strcmp(line,"route lowyaw on"));nav_log("[ROUTE_LOWYAW] accepted; plan again; no motion\r\n");return;
 }
 if(sscanf(line,"route yawgain %d %n",&a,&n)==1&&n>0&&!line[n]){if(a<0||!nav_route_yaw_gain((unsigned)a))nav_log("[E] route yawgain2000..8000\r\n");else nav_logf("[ROUTE_YAWGAIN] %d\r\n",a);return;}
 if(sscanf(line,"limit %d %n",&a,&n)==1&&n>0&&!line[n]&&(a>=5&&a<=400)){ceiling=(unsigned)a;nav_logf("[LIMIT] actual wheel cap ceiling=%u RPM; no move\r\n",ceiling);return;}
 bool sensed_request=!strncmp(line,"route sensorplan ",17),survey_request=!strncmp(line,"route surveyplan ",17);
 n=0;if(sscanf(line,survey_request?"route surveyplan %d %n":sensed_request?"route sensorplan %d %n":"route plan %d %n",&a,&n)==1&&n>0&&!line[n]&&a>=25&&a<=100){
  bool sensed=survey_request||sensed_request||avoid_enabled;survey_route=false;
  if(sensed&&!nav_lidar_avoid_ready(now)){route.ready=false;nav_log("[E] sensorplan needs fresh configured lidar with obstacle+memory enabled; NO motion\r\n");return;}
  const NavOdom *q=nav_odom_data();
  uint32_t cycles=nav_hw_cycles();
  nav_map_plan_start(pose()->x_m,pose()->y_m,nav_odom_heading_now());
   bool qr_first=sensed&&nav_map_qr_enabled()&&!calibrating;
   bool ok=survey_request||qr_first?nav_route_plan_qr(&route,(unsigned)a,q->ppm_x,q->ppm_y,q->lever_m):sensed?nav_route_plan_sensed(&route,(unsigned)a,q->ppm_x,q->ppm_y,q->lever_m):nav_route_plan(&route,(unsigned)a,q->ppm_x,q->ppm_y,q->lever_m);
  survey_route=survey_request&&ok;
  route_from_lidar=sensed&&ok;
  cycles=nav_hw_cycles()-cycles;nav_logf("[ROUTE_PLAN_PERF] cycles=%lu; 168cycles/us on F407; no hardware measurement on a desktop build\r\n",(unsigned long)cycles);
  nav_logf("[ROUTE_PLAN] valid=%u samples=%u length_mm=%d model_ms=%lu cap=%uRPM yaw_max_mrad_s=%u; %s; NO motion\r\n",ok,route.n,mm(route.length),(unsigned long)(route.duration*1000),route.max_rpm,(unsigned)(route.yaw_limit*a*10),route.fault);return;
 }
 if(!strcmp(line,"route segmented on")||!strcmp(line,"route segmented off")){segmented=!strcmp(line,"route segmented on");nav_logf("[SEGMENTED] %u; task stops enter idle; explicit continue, no automatic arm action\r\n",segmented);return;}
 if(!strcmp(line,"route calibrate on")||!strcmp(line,"route calibrate off")){
  calibrating=!strcmp(line,"route calibrate on");nav_logf("[CALIBRATION_MODE] %u; six work stops wait for route next; firmware never senses actual servo pose\r\n",calibrating);return;
 }
 n=0;unsigned cap=0,accel=0,yawcap=0;
 if(sscanf(line,"route dynamics %u %u %u %n",&cap,&accel,&yawcap,&n)==3&&n&&!line[n]){
  if(nav_route_dynamics(cap,accel,yawcap))nav_logf("[ROUTE_DYNAMICS] cap=%uRPM accel=%umm/s2 yaw=%umrad/s; plan again, RAM only\r\n",cap,accel,yawcap);
  else nav_log("[E] route dynamics cap50..400 accel200..1600 yaw250..1800\r\n");
  route.ready=false;return;
 }
 if(!strcmp(line,"wheel guard strict")||!strcmp(line,"wheel guard distributed")){
  wheel_guard_strict=!strcmp(line,"wheel guard strict");
  wheel_rej_bits=0;wheel_rej_since=0;wheel_amb_since=0;wheel_log_at=0;slip_since=0;
  traction_at=0;traction_scale=1;traction_update_at=now;
  nav_logf("[WHEEL_GUARD] mode=%s; RAM only; %s\r\n",wheel_guard_strict?"strict":"distributed",
   wheel_guard_strict?"v2.7.13 semantics: any rejected/ambiguous mask >500ms stops":
   "statistical rejection logs +65% reference clock, recover after1s clear; hardware/feedback guards unchanged");return;
 }
 if(!strcmp(line,"map lidar")){
  if(!nav_lidar_avoid_ready(now)){nav_log("[E] map lidar needs fresh lidar with obstacle+memory enabled\r\n");return;}
  nav_map_set(1,0,0);nav_log("[MAP_LIDAR] fixed yellow/fixtures retained; no manually supplied random obstacle locations; live+memory geometry used\r\n");return;
 }
 if(!strcmp(line,"route detourcheck")){
  if(!route.ready||!pose()->valid||nav_odom_data()->quiet<3){nav_log("[E] detourcheck needs new ready plan and stationary measured pose\r\n");return;}
  const NavOdom *q=nav_odom_data();uint32_t cycles=nav_hw_cycles();
  bool ok=nav_route_replan(&route,pose(),q->ppm_x,q->ppm_y,q->lever_m);cycles=nav_hw_cycles()-cycles;
  nav_logf("[DETOUR_CHECK] valid=%u cycles=%lu samples=%u reason=%s; NO motion; plan invalidated after timing\r\n",ok,(unsigned long)cycles,route.n,route.fault);
  route.ready=false;return;
 }
 if(!strcmp(line,"route avoid on")||!strcmp(line,"route avoid off")){
  if(!strcmp(line,"route avoid on")&&!nav_lidar_avoid_ready(now)){nav_log("[E] avoid needs fresh configured lidar, obstacle on, memory on and map capacity\r\n");return;}
  avoid_enabled=!strcmp(line,"route avoid on");nav_logf("[ROUTE_AVOID] enabled=%u; stop/feedback/new path checks before resume; no move\r\n",avoid_enabled);return;
 }
 if(nav_lidar_command(line,!nav_app_active(),now))return;
 if(nav_map_command(line))return;
 if(!strcmp(line,"route continue")){
  if((route_from_lidar||avoid_enabled)&&(!avoid_enabled||!nav_lidar_avoid_ready(now)||!nav_arm_transport_ready())){nav_log("[E] sensed continue requires fresh lidar and transport clearance; NO travel\r\n");return;}
  if(!segmented||calibrating||!nav_app_work_station()||!route.ready||route.map_generation!=nav_map_generation()||hypotf(pose()->x_m-route.p[route.index].x,pose()->y_m-route.p[route.index].y)>.02f||fabsf(wrapped(nav_odom_heading_now()-route.p[route.index].h))>.0174533f){nav_log("[E] continue requires ready parked task center within20mm/1deg\r\n");return;}
  begin(0,0,route.max_rpm,now,3,0);
  if(phase==1){
   route.waiting=false;
   if(route.station==9&&route_from_lidar&&!survey_route&&!nav_lidar_map_frozen()){
    qr_catalog_transition=true;
    qr_anchor_requested=false;
   }
  }
  return;
 }
 if(!strcmp(line,"route start")||!strcmp(line,"route preflight")){
  preflight_only=!strcmp(line,"route preflight");
  bool start_laser_ready=nav_lidar_avoid_ready(now);
  bool start_arm_ready=nav_arm_transport_ready();
  if((route_from_lidar||avoid_enabled)&&(!avoid_enabled||!start_laser_ready||!start_arm_ready)){
   nav_logf("[START_GATE] route_from_lidar=%u avoid=%u lidar_ready=%u lidar_fresh=%u arm_ready=%u phase=%u route_ready=%u scan_age=%u px=%d py=%d\r\n",
    route_from_lidar,avoid_enabled,start_laser_ready,nav_lidar_fresh(now),start_arm_ready,
    phase,route.ready,(unsigned)(now-nav_lidar_last_scan_ms()),(int)roundf(pose()->x_m*1000),(int)roundf(pose()->y_m*1000));
   nav_log("[E] sensed route needs avoid on, fresh lidar and arm referenced at transport clearance; NO travel\r\n");return;
  }
  if(algorithm==3){nav_log("[E] route MPC not released: moving-reference integration test exceeded tracking envelope; use line tests only\r\n");return;}
  if(route.map_generation!=nav_map_generation()){nav_log("[E] map/footprint changed; plan again\r\n");return;}
  if(segmented&&calibrating){nav_log("[E] select segmented OR manual calibration mode\r\n");return;}
  route.manual=calibrating||segmented;station_stage=0;wait_at=now;detours=0;qr_catalog_transition=false;qr_anchor_requested=false;qr_anchor_accumulating=false;nav_lidar_qr_context(false);nav_lidar_qr_anchor_reset();
  if(!route.ready||route.index||route.elapsed||algorithm>3||!xy||ceiling<route.max_rpm||hypotf(pose()->x_m-route.p[0].x,pose()->y_m-route.p[0].y)>.020f||fabsf(wrapped(nav_odom_heading_now()-route.p[0].h))>.0174533f){nav_log("[E] route requires new valid plan, selected controller+xy, sufficient limit, known zone1 WEST origin within20mm/1deg\r\n");return;}
  if(!nav_route_controller(&route,algorithm,b0_ratio)){nav_log("[E] route controller configuration invalid\r\n");return;}
  nav_logf("[ROUTE_CONTROLLER] %s; common trajectory/feedforward/allocator and station settler; moving feedback selected\r\n",names[algorithm]);
  const NavOdom *q=nav_odom_data();
  if(fabsf(q->lever_m-route.lever)>.00001f||fabsf(3200.0f/q->ppm_x/60-route.rpm_mps)>.00000001f||fabsf((float)q->ppm_y/q->ppm_x-route.scale_y)>.000001f){nav_log("[E] odometry config changed; route plan again\r\n");return;}
  begin(0,0,route.max_rpm,now,3,0);return;
 }
 n=0;if(sscanf(line,"frame restore %d %d %d %n",&a,&b,&angle,&n)==3&&n&&!line[n]){
  /* Division rounds the exact millimetre boundary2100 to the same float as
   * the2.10m range limit. Multiplication by an inexact .001f can exceed it. */
  float frame_x=a/1000.0f,frame_y=b/1000.0f;
  if(!imu_angle_fresh(now)||!nav_route_footprint(frame_x,frame_y,angle*.00001745329252f,.005f)||!nav_odom_restore_pose(frame_x,frame_y,angle*.00001745329252f)){nav_log("[E] frame restore requires stationary saved zone1 pose and fresh IMU\r\n");return;}
  heading_reference=angle*.00001745329252f;nav_log("[FRAME_RESTORED] explicit saved zone1 coordinates; same physical map, no motor write\r\n");return;
 }
 n=0;if(sscanf(line,"work goto %d %d %u %n",&a,&b,&rpm,&n)==3&&n&&!line[n]){
  float target[3];
  if(!nav_app_station_target(target)||route.station==9||hypotf(a*.001f-target[0],b*.001f-target[1])>.18f){nav_log("[E] work move must remain within180mm of a parked task center\r\n");return;}
  float x=pose()->x_m,y=pose()->y_m;unsigned steps=(unsigned)ceilf(hypotf(a*.001f-x,b*.001f-y)/.005f)+1;
  float yaw=nav_odom_heading_now();
  for(unsigned j=0;j<=steps;j++){
   float sx=x+(a*.001f-x)*j/steps,sy=y+(b*.001f-y)*j/steps;
   const char *block=nav_map_footprint_blocker(sx,sy,yaw,.015f);
   if(!block)continue;
   nav_logf("[WORK_PATH_BLOCKED] station=%u start_mm=%d,%d target_mm=%d,%d sample_mm=%d,%d sample=%u/%u margin_mm=15 source=%s\r\n",route.station,mm(x),mm(y),a,b,mm(sx),mm(sy),j,steps,block);
   if(!strcmp(block,"lidar"))nav_lidar_plan_blockers(sx,sy,yaw,.015f,true);
   nav_log("[E] work line footprint intersects map\r\n");return;
  }
  begin(a*.001f,b*.001f,rpm,now,4,0);return;
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
/* v2.7.14 wheel-consistency decision (also called by the native regression).
 * distributed (default): stop only when the SAME wheel bit stays rejected in
 * every consecutive sample for >500 ms. A rotating mask means distributed
 * slip/model residual -> reset the window. ambiguous_mask never stops alone;
 * it is counted and rate-limit logged after 2 s. strict: v2.7.13 semantics
 * (any rejected OR ambiguous mask sustained >500 ms stops). */
bool nav_app_wheel_guard(uint32_t now,uint8_t rejected,uint8_t ambiguous,unsigned pairs,float max_ratio,const char **why)
{
 if(why)*why="persistent wheel inconsistency; inspect traction/geometry";
 if(wheel_guard_strict){
  if(rejected||ambiguous){if(!slip_since)slip_since=now;else if(now-slip_since>500U)return true;}
  else slip_since=0;
  return false;
 }
 if(rejected||ambiguous){
  traction_at=now;
  /* Do not slam the common speed reference between 1.00 and 0.65: the
   * resulting one-second surge/hesitation is visible as a lurch. Ramp the
   * existing route-speed scale at <=0.8/s; safety decisions and individual
   * wheel limits are unchanged. Ambiguous-only evidence gets a gentler cap. */
  float target=rejected?.70f:.88f;
  uint32_t dt_ms=traction_update_at?now-traction_update_at:20U;
  if(dt_ms>100U)dt_ms=100U;
  float step=.0008f*(float)dt_ms;
  if(traction_scale>target)traction_scale=fmaxf(target,traction_scale-step);
  traction_update_at=now;
 }else{
  uint32_t dt_ms=traction_update_at?now-traction_update_at:20U;
  if(dt_ms>100U)dt_ms=100U;
  float step=.0008f*(float)dt_ms;
  if(traction_scale<1)traction_scale=fminf(1,traction_scale+step);
  traction_update_at=now;
  if((uint32_t)(now-traction_at)>=1000U)traction_scale=1;
 }
 if(rejected){
  uint8_t bits=wheel_rej_bits?(uint8_t)(wheel_rej_bits&rejected):rejected;
  if(!bits){ /* mask rotated to a disjoint subset: new window, no emergency */
   wheel_rej_bits=rejected;wheel_rej_since=now;
  }else{
   wheel_rej_bits=bits;
   if(!wheel_rej_since)wheel_rej_since=now;
   /* A roller losing traction is not an Emm stall/communications fault.
    * Continue using the current map and reduced reference speed. */
  }
 }else{wheel_rej_bits=0;wheel_rej_since=0;}
 if(ambiguous){
  if(!wheel_amb_since)wheel_amb_since=now;
  else if(now-wheel_amb_since>=2000U&&now-wheel_log_at>=1000U){
   wheel_log_at=now;
   nav_logf("[WHEEL_AMBIGUOUS] amb=%02X pairs=%u; ambiguous slip sustained >2s, no emergency\r\n",ambiguous,pairs);
  }
 }else wheel_amb_since=0;
 if((rejected||ambiguous)&&now-wheel_log_at>=1000U){
  wheel_log_at=now;
  nav_logf("[WHEEL_SLIP] rej=%02X amb=%02X pairs=%u maxR=%ld distributed; no emergency; reference=%upermille\r\n",
   rejected,ambiguous,pairs,(long)lroundf(max_ratio),(unsigned)roundf(traction_scale*1000));
 }
 return false;
}
/* QR observation accumulation is enabled only near the QR stop, while the
 * 4x90 observation turn is running. It must stop before phase7 so the frozen
 * transition sees the accumulated set, not a fresh empty scan. */
static bool qr_dock_near(void)
{
 if(segmented&&route.waiting&&route.station==9&&route_from_lidar&&!nav_lidar_map_frozen()&&phase==0)return true;
 if(!nav_map_qr_enabled()||!route_from_lidar||!routing||phase==7)return false;
 for(unsigned k=route.index;k<route.n;k++)if(route.p[k].stop==9U){
  return hypotf(pose()->x_m-route.p[k].x,pose()->y_m-route.p[k].y)<1.00f;
 }
 return false;
}
static void qr_anchor_service(void)
{
 bool near=qr_dock_near();
 nav_lidar_qr_context(near);
 if(near&&!qr_anchor_accumulating){
  nav_lidar_qr_anchor_begin();qr_anchor_accumulating=true;
  nav_log("[CATALOG_ANCHOR_ACCUM] begin; collect qualified returns across QR observation headings\r\n");
 }else if(!near&&qr_anchor_accumulating){
  nav_lidar_qr_anchor_end();qr_anchor_accumulating=false;
  nav_logf("[CATALOG_ANCHOR_ACCUM] end; unique_anchors=%u\r\n",nav_lidar_qr_anchor_count());
 }
}
/* Only discard a translation reference already passed on this local straight.
 * Bound the search to 150 mm, 40 mm cross-track and 10 degrees; never cross an
 * unserved station or a turn. Current-body clearance is checked independently.
 * Do not modify the controller's time/index or the navigation coordinate frame. */
static unsigned forward_hazard_sample(const NavPose *p)
{
 unsigned first=route.index;
 for(unsigned k=route.index;k+1<route.n;k++){
  const NavRoutePoint *a=route.p+k,*b=a+1;
  if(b->s-route.p[route.index].s>.15f||(a->stop&&route.station!=a->stop))break;
  float dx=b->x-a->x,dy=b->y-a->y,len2=dx*dx+dy*dy;
  if(len2<.00000001f)break;
  float ah=atan2f(sinf(p->yaw_rad-a->h),cosf(p->yaw_rad-a->h));
  float bh=atan2f(sinf(b->h-a->h),cosf(b->h-a->h));
  if(fabsf(ah)>.17453293f||fabsf(bh)>.017453293f)break;
  float t=((p->x_m-a->x)*dx+(p->y_m-a->y)*dy)/len2;
  if(t<=0)break;
  float q=fminf(t,1.0f),ex=p->x_m-a->x-q*dx,ey=p->y_m-a->y-q*dy;
  if(ex*ex+ey*ey>.04f*.04f)break;
  first=k+1;
  if(t<1.0f||b->stop)break;
 }
 return first;
}
void nav_app_tick(uint32_t now)
{
 latest_tick=now;
 uint32_t start=nav_hw_cycles(),t=start;nav_odom_tick(now);uint32_t elapsed=nav_hw_cycles()-t;if(elapsed>odom_max)odom_max=elapsed;
 nav_lidar_departure(routing&&!route.station&&route.index<route.n&&route.p[route.index].s<.30f);
 qr_anchor_service();
 imu_service_tick(now);nav_vision_tick(now);nav_lidar_tick(now,fused,(imu_data()->gyro_seen&&now-imu_data()->gyro_at<300)?imu_data()->z_mdps*.00001745329252f:NAN);

 if(routing&&route.waiting){route_wait_ms+=now-wait_at;}wait_at=now;
 const NavBus *b=nav_bus_data();const NavOdom *o=nav_odom_data();const WitImu *i=imu_data();
 if(phase!=7&&nav_app_active()){
  /* Preparation sends only zero speed. Wait for post-enable aligned scans
   * here instead of declaring a data fault before the histories can rebuild.
   * Once phase4 begins, the unchanged500ms stale guard is immediate. */
  bool waiting_laser=avoid_enabled&&routing&&phase<4&&!nav_lidar_fresh(now);
  bool hazard=!waiting_laser&&nav_lidar_hazard(now,pose());unsigned hazard_path=route.n;
  if(!hazard&&avoid_enabled&&routing&&phase==4&&!route.waiting){
   float speed=hypotf(pose()->vx_mps,pose()->vy_mps),look=.20f+.30f*speed+speed*speed;
   unsigned first=forward_hazard_sample(pose());
   for(unsigned k=first;k<route.n&&route.p[k].s-route.p[route.index].s<=look;k++)
    if(!nav_lidar_clear(route.p[k].x,route.p[k].y,route.p[k].h,.03f)){hazard=true;hazard_path=k;break;}
  }
  if(hazard){
   nav_logf("[HAZARD_CAUSE] class=%s sample=%u catalog_frozen=%u\r\n",hazard_path<route.n?"remaining_path":nav_lidar_hazard_cause(),hazard_path,nav_lidar_map_frozen());
   if(hazard_path<route.n)nav_lidar_plan_blockers(route.p[hazard_path].x,route.p[hazard_path].y,route.p[hazard_path].h,.03f,true);
   else if(!strcmp(nav_lidar_hazard_cause(),"classified_body_overlap"))
    nav_lidar_plan_blockers(pose()->x_m,pose()->y_m,pose()->yaw_rad,.03f,true);
   nav_lidar_capture_hazard(now,pose());
   if(survey_route&&routing){nav_app_emergency(now,"QR survey hazard/stale; stop without automatic replan");goto end;}
   if(nav_lidar_map_frozen()){nav_app_emergency(now,"live hazard after QR catalog freeze; no automatic replan");goto end;}
   bool laser_ready=nav_lidar_avoid_ready(now),body_clear=nav_lidar_clear(pose()->x_m,pose()->y_m,pose()->yaw_rad,.005f),wheels_ready=feedback(now,true),arm_held=nav_arm_transport_held();
   nav_logf("[AVOID_GATE] laser=%u body_clear=%u wheels=%u arm_held=%u arm_fresh=%u attempts=%u; cached hold permits stop/refresh only\r\n",laser_ready,body_clear,wheels_ready,arm_held,nav_arm_transport_ready(),detours);
   if(!laser_ready)(void)nav_lidar_command("lidar status",false,now);
   if(!avoid_enabled||!routing||phase!=4||route.waiting||!laser_ready||!body_clear||!wheels_ready||!arm_held||detours>=6){
    nav_app_emergency(now,"lidar hazard/stale data or detour precondition failed");goto end;
   }
   zero_generation=nav_bus_velocity((int[4]){0,0,0,0});phase=7;replan_at=now;replan_plan_ready=false;
   nav_logf("[DETOUR_STOP] completed_station=%u; zero all wheels; preserve frame/task order\r\n",route.station);
  }
 }
 if(nav_app_active()){
  if(b->errors!=bus_errors){nav_app_emergency(now,"motor transaction/ACK/UART failure");goto end;}
  if(i->uart_errors!=imu_errors || i->bad_checksum-imu_checksums>=3){nav_app_emergency(now,"new IMU stream errors");goto end;}
  if(!i->angle_seen || nav_age(now,i->angle_at)>300 || !i->gyro_seen || nav_age(now,i->gyro_at)>300 || nav_age(now,run_at)-route_wait_ms>deadline){nav_app_emergency(now,"angle/gyro stale or run deadline");goto end;}
  if(phase<4 && nav_age(now,phase_at)>5000){nav_app_emergency(now,"preparation timeout");goto end;}
 }
 if(phase==7&&!replan_busy){
  bool lidar_ready=nav_lidar_avoid_ready(now);
  bool feedback_ready=feedback(now,true);
  bool arm_held=nav_arm_transport_held();
  bool post_plan_lidar_grace=replan_plan_ready&&!lidar_ready&&(uint32_t)(now-replan_finished)<300U;
  if((!lidar_ready&&!post_plan_lidar_grace)||!feedback_ready||!arm_held){
   nav_logf("[DETOUR_GATE] lidar=%u grace=%u feedback=%u arm_held=%u scan_age=%u; fault only after real loss\r\n",
    lidar_ready,post_plan_lidar_grace,feedback_ready,arm_held,(unsigned)(now-nav_lidar_last_scan_ms()));
   nav_app_emergency(now,"detour feedback/transport/lidar lost");goto end;
  }
  if((!replan_plan_ready&&now-replan_at>5000)||(replan_plan_ready&&now-replan_finished>1500)){
   nav_app_emergency(now,"detour stationary/arm refresh timeout");goto end;
  }
  if(b->velocity_done==zero_generation&&o->quiet>=3&&abs(i->z_mdps)<1000&&hypotf(pose()->vx_mps,pose()->vy_mps)<.003f&&nav_arm_transport_ready()){
   if(!replan_plan_ready){
    if(qr_catalog_transition){
     if(!segmented&&now-replan_at<1000)goto end;
     if(!qr_anchor_requested){
      qr_anchor_requested=true;
      nav_logf("[CATALOG_ANCHOR] accumulated=%u during QR approach (points+lines); freeze next\r\n",
       nav_lidar_qr_anchor_count());
     }
     nav_lidar_qr_anchor_end();qr_anchor_accumulating=false;
     nav_lidar_map_freeze(true);
     nav_logf("[CATALOG_FROZEN] at=qr confirmed=%u anchors=%u; planning on fixed scene\r\n",
      nav_lidar_map_count(),nav_lidar_qr_anchor_count());
    }
    replan_busy=true;uint32_t cycles=nav_hw_cycles();
    bool ok=nav_route_replan(&route,pose(),o->ppm_x,o->ppm_y,o->lever_m);
    if(ok)ok=nav_route_controller(&route,algorithm,b0_ratio);
    replan_cycles=nav_hw_cycles()-cycles;now=latest_tick;
    bool still_waiting=phase==7;replan_busy=false;
    if(!still_waiting)goto end;
    if(!ok||!feedback(now,true)||!nav_arm_transport_held()){
     nav_app_emergency(now,ok?"detour post-plan feedback invalid":route.fault);goto end;
    }
    replan_plan_ready=true;replan_finished=now;
    goto end; /* Finish any in-flight read and recheck FRESH arm feedback. */
   }
   bool catalog=qr_catalog_transition;
   if(catalog)nav_logf("[CATALOG_FUSION] at=qr enabled=%u anchors=%u; applied only after planning\r\n",
     nav_lidar_qr_fusion_enable(),nav_lidar_qr_anchor_count());
   qr_catalog_transition=false;if(!catalog)detours++;route_wait_ms+=now-replan_at;phase=4;last_control=phase_at=now;replan_plan_ready=false;
   deadline=nav_age(now,run_at)+(uint32_t)(route.duration*2000)+30000;
   nav_logf("[%s] attempt=%u samples=%u completed_station=%u plan_cycles=%lu; fresh transport verified; same origin; no F3/reset\r\n",catalog?"CATALOG_RESUME":"DETOUR_RESUME",detours,route.n,route.station,(unsigned long)replan_cycles);
  }
  goto end;
 }
 if(phase==7)goto end; /* recursive service while planner is active */
 if(phase==1&&(preparation_disabled||b->velocity_done==zero_generation)){
  if(!zero_ack_at)zero_ack_at=now;
  bool all_new=true;for(unsigned k=0;k<4;k++)if((int32_t)(b->position_at[k]-zero_ack_at)<=0)all_new=false;
  if(all_new&&nav_bus_lift_idle()&&nav_age(now,zero_ack_at)>=100&&o->quiet>=1&&feedback(now,false)){
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
  bool all_new=true;for(unsigned k=0;k<4;k++)if((int32_t)(b->position_at[k]-phase_at)<=0 || ((preparation_disabled||!work_translation)&&(int32_t)(b->flags_at[k]-phase_at)<=0))all_new=false;
  if(b->velocity_done==zero_generation&&all_new&&nav_age(now,phase_at)>=(work_translation&&!preparation_disabled?30U:120U)&&feedback(now,true)&&pose()->valid&&o->quiet>=1){
   if(routing&&avoid_enabled){
    if(nav_arm_faulted()){nav_app_emergency(now,"startup transport fault");goto end;}
    if(!nav_lidar_start_ready(now,phase_at)){lidar_start_since=0;goto end;}
    if(!lidar_start_since){lidar_start_since=now;nav_log("[LIDAR_PREP] post-enable scan aligned; verify150ms stable while wheels zero\r\n");}
    if(now-lidar_start_since<150||!nav_arm_transport_ready())goto end;
   }
   if(preflight_only){
    preflight_only=false;routing=false;phase=0;nav_bus_emergency(now);
    nav_log("[PREFLIGHT_DONE] zero-only preparation passed; wheels disable queued; NO nonzero velocity or trajectory executed\r\n");goto end;
   }
   if(qr_catalog_transition){
    phase=7;replan_at=now;replan_plan_ready=false;
    nav_log("[CATALOG_QR_CONTINUE] QR dwell complete; freeze scene and plan remaining stations\r\n");
    goto end;
   }
   nav_control_begin(&control,pose(),goal[0],goal[1],requested_rpm,xy);
   nav_control_config(&control,algorithm,o->ppm_x,o->ppm_y,o->lever_m,b0_ratio);control.work_move=work_translation;control.work_mode=nav_mission_indexing_mode();control.work_accel=nav_mission_index_accel()*.001f;control.work_jerk=nav_mission_index_jerk()*.001f;
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
 if((phase==4||phase==5)&&work_translation&&!nav_route_footprint(pose()->x_m,pose()->y_m,pose()->yaw_rad,.005f)){nav_app_emergency(now,"work move body intersects map");goto end;}
 if(phase==4||phase==5){
  if(!feedback(now,true)||!pose()->valid||nav_age(now,pose()->at)>300){nav_app_emergency(now,"feedback/frame stale or driver stall/disabled/protection");goto end;}
  /* v2.7.12: hard collision protection now runs on the MEASURED pose (5 mm
   * margin) for routes as well. The v2.7.11 stop came from the predicted-pose
   * guard in nav_route.c; that guard is now transient-tolerant while this
   * measured check keeps the real vehicle footprint protected at all times. */
  if(routing&&!nav_route_footprint(pose()->x_m,pose()->y_m,pose()->yaw_rad,.005f)){nav_app_emergency(now,"measured body intersects map");goto end;}
  if(!routing&&!turning&&fabsf(wrapped(nav_odom_heading_now()-heading_reference))>.0523599f){nav_app_emergency(now,"heading deviation >3deg");goto end;}
  {const char *wheel_why;
   if(nav_app_wheel_guard(now,o->rejected_mask,o->ambiguous_mask,(unsigned)o->pair_count,o->max_R,&wheel_why)){nav_app_emergency(now,wheel_why);goto end;}
  }
 }
 if(phase==4 && now-last_control>=NAV_TICK_MS){
  float dt=(now-last_control)*.001f;if(dt>.15f){nav_app_emergency(now,"controller period >150ms");goto end;}last_control=now;
  float gyro=i->z_mdps*.00001745329252f; /* Freshness required above; never silently substitute zero. */
  if(!nav_odom_gyro_valid(gyro,now)){nav_app_emergency(now,"sustained implausible yaw rate");goto end;}
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
   if(!nav_odom_predict(fused,now,gyro,&predicted)){
    nav_logf("[PREDICT_REJECT] task=route fused=%u valid=%u age_ms=%lu gyro_mdps=%ld\r\n",fused?1U:0U,(unsigned)pose()->valid,(unsigned long)(now-pose()->at),(long)i->z_mdps);
    nav_app_emergency(now,"route prediction horizon/feedback invalid");goto end;
   }
   route.response_s=o->response_s;
   route.traction_scale=traction_scale;
   route.predict_age_ms=nav_age(now,pose()->at); /* v2.7.12 guard diagnostic */
   unsigned station=route.station;result=nav_route_step(&route,&predicted,gyro,dt,wheel);
   control.qp_fallbacks=route.tracker.qp_fallbacks;control.qp_steps=route.tracker.qp_steps;control.qp_skips=route.tracker.qp_skips;
   for(unsigned k=0;k<4;k++)control.wheel[k]=route.wheel[k];
   control.remaining=route.length-route.p[route.index].s;control.cross_error=route.error_m;
   if(station!=route.station&&route.waiting&&!calibrating&&!segmented)nav_logf("[ROUTE_PAUSED] station=%u; record ground error then route resume\r\n",route.station);
   if(station!=route.station&&route.waiting&&calibrating){station_stage=1;station_at=now;}
   if(station!=route.station){
    nav_logf("[STATION] %u %s elapsed_ms=%lu; stopped; no origin reset\r\n",route.station,nav_route_station(route.station),(unsigned long)(route.elapsed*1000));
    nav_lidar_fusion_report(route.station);
   }
   if(station!=route.station&&route.station==9&&route_from_lidar&&!survey_route&&!segmented&&!calibrating&&!nav_lidar_map_frozen()){
    zero_generation=nav_bus_velocity((int[4]){0});phase=7;replan_at=now;replan_plan_ready=false;qr_catalog_transition=true;qr_anchor_requested=false;
   if(qr_anchor_accumulating){nav_lidar_qr_anchor_end();qr_anchor_accumulating=false;}
    nav_log("[CATALOG_QR_WAIT] settled QR; collect1000ms then freeze confirmed obstacles and plan remaining work; no origin reset\r\n");goto end;
   }
   if(route.waiting&&calibrating){
    if(station_stage==1&&b->velocity_done==b->velocity_requested&&o->quiet>=3){
     if(!nav_arm_turn(2130)){nav_app_emergency(now,"station outward turn blocked");goto end;}
     station_stage=2;station_at=now;
    }else if(station_stage==2&&nav_arm_turn_done(2130)&&now-station_at>=2300){
     station_stage=3;nav_logf("[CAL_WAIT] %s; chassis zeroRPM, servo2=2130 PWM; measure then route next; NO auto departure\r\n",nav_route_station(route.station));
    }else if(station_stage==4&&nav_arm_turn_done(1000)&&now-station_at>=2300){
     station_stage=0;route.waiting=false;nav_log("[CAL_CONTINUE] servo2=1000 settled; same world origin\r\n");
    }
   }
   if(result<0){nav_app_emergency(now,route.fault);goto end;}
   if(segmented&&route.waiting&&station!=route.station)result=1;
   if(result==1){unsigned end=segmented&&route.waiting?route.index:route.n-1;goal[0]=route.p[end].x;goal[1]=route.p[end].y;heading_reference=route.p[end].h;routing=false;}
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
   float delta=command-turn_wheel,bound=(requested_rpm>10?120:20)*dt;
   turn_wheel+=fmaxf(-bound,fminf(bound,delta));
   int v=(int)roundf(turn_wheel);wheel[0]=-v;wheel[1]=v;wheel[2]=-v;wheel[3]=v;
   if(close&&speed<.0174533f&&v==0)turn_stable++;else turn_stable=0;
   control.remaining=error;control.cross_error=hypotf(pose()->x_m-goal[0],pose()->y_m-goal[1]);
   for(unsigned k=0;k<4;k++)control.wheel[k]=(float)wheel[k];
   result=turn_stable>=3?1:0;
  }else if(work_translation){
   NavPose predicted;
   if(!nav_odom_predict(fused,now,gyro,&predicted)){
    nav_logf("[PREDICT_REJECT] task=work fused=%u valid=%u age_ms=%lu gyro_mdps=%ld\r\n",fused?1U:0U,(unsigned)pose()->valid,(unsigned long)(now-pose()->at),(long)i->z_mdps);
    nav_app_emergency(now,"work prediction feedback invalid");goto end;
   }
   result=nav_control_step(&control,&predicted,wrapped(nav_odom_heading_now()-heading_reference),gyro,dt,wheel);
  }else result=nav_control_step(&control,pose(),wrapped(nav_odom_heading_now()-heading_reference),gyro,dt,wheel);
  elapsed=nav_hw_cycles()-t;if(elapsed>control_max)control_max=elapsed;
  nav_logf("C,%lu,%u,%d,%lu,%u,%u\r\n",(unsigned long)now,(unsigned)roundf(dt*1000),result,(unsigned long)elapsed,control.qp_steps,control.qp_fallbacks);
  if(elapsed>1680000U){nav_app_emergency(now,"controller >10ms computation budget");goto end;}
  if(result<0){nav_app_emergency(now,"controller invalid state or >15mm overshoot");goto end;}
  if(result==1){int zero[4]={0};zero_generation=nav_bus_velocity(zero);phase=5;phase_at=now;nav_log("[SETTLE] zero speed; no FE restart tail\r\n");}
  else nav_bus_velocity(wheel);
 }
 if(phase==5&&b->velocity_done==zero_generation&&nav_age(now,phase_at)>=(work_translation?50U:120U)){
  float dx=goal[0]-pose()->x_m,dy=goal[1]-pose()->y_m;
  if(hypotf(pose()->vx_mps,pose()->vy_mps)<.003f){
   if((!turning&&xy&&hypotf(dx,dy)>.012f)||(turning&&hypotf(dx,dy)>.04f)||fabsf(wrapped(nav_odom_heading_now()-(turning?turn_target:heading_reference)))>.0174533f||i->z_mdps>1000||i->z_mdps< -1000){nav_app_emergency(now,"settled endpoint or heading/rate out of tolerance");goto end;}
   if(turning){heading_reference=turn_target;turning=false;nav_logf("[TURN_DONE] target_mdeg=%d; XY/origin preserved\r\n",(int)(turn_target*57295.7795f));}
   if(blending){heading_reference=turn_target;blending=false;nav_log("[POSE_DONE] heading and XY settled; world frame preserved\r\n");}
   phase=0;if(segmented&&route.waiting)nav_logf("[MISSION_STOP] %u %s; chassis idle, no auto departure\r\n",route.station,nav_route_station(route.station));nav_logf("[DONE] estimated_error_mm=%d,%d yaw_mdeg=%d elapsed_ms=%lu; measure GROUND error independently\r\n",mm(dx),mm(dy),(int)(wrapped(nav_odom_heading_now()-heading_reference)*57295.7795f),(unsigned long)(now-run_at));
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
