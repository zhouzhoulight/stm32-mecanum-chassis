#include "nav/studio_profile.h"
/* Two-batch preliminary-rule rehearsal. The STM32 owns the whole sequence.
 * Vision/colour are explicitly simulated; transfer counts are commanded jobs,
 * NOT measured object-presence or competition-score claims. */
#include "nav/nav_mission.h"
#include "nav/nav_app.h"
#include "nav/nav_arm.h"
#include "nav/nav_map.h"
#include "nav/nav_platform.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

enum {TRAVEL=1,FEED,ALIGN,MOVE_WAIT,JOB_START,JOB_WAIT,CENTER,RESUME,QR_WAIT};
static bool active,issuing,flat_confirmed,pause_requested,paused;
static uint32_t pause_at;
static unsigned state,station,slot,batch,kind,retrieval;
static unsigned loaded,coarse,stored,stacked,transfers,completed_stops;
static uint32_t started,entered,delay_until,last_pick_started;
static unsigned work_rpm=STUDIO_WORK_RPM;
static unsigned indexing_mode=STUDIO_INDEX_MODE;
static unsigned index_accel=STUDIO_INDEX_ACCEL,index_jerk=STUDIO_INDEX_JERK;
static float center[3],target[2];
static unsigned coordination=STUDIO_FEATURES;
static float arm_radius=STUDIO_ARM_RADIUS*.001f,arm_ox=STUDIO_ARM_X*.001f,arm_oy=STUDIO_ARM_Y*.001f;
static bool coordinated_transfer,early_extended;
static uint32_t prefetch_at;
unsigned nav_coordination_features(void){return coordination;}
unsigned nav_mission_indexing_mode(void){return indexing_mode;}
unsigned nav_mission_index_accel(void){return index_accel;}
unsigned nav_mission_index_jerk(void){return index_jerk;}
bool nav_mission_work_pose(void){return active&&coordinated_transfer&&nav_arm_envelope_ready();}
static bool arm_line_clear(const float p[3],const float q[2]){
 if(arm_radius<.15f)return false;
 unsigned n=(unsigned)ceilf(hypotf(q[0]-p[0],q[1]-p[1])/.01f);if(!n)n=1;
 for(unsigned k=0;k<=n;k++)if(!nav_map_arm_safe(p[0]+(q[0]-p[0])*k/n,p[1]+(q[1]-p[1])*k/n,p[2],arm_radius,arm_ox,arm_oy))return false;
 return true;
}
static const char *failure;
void nav_mission_init(void)
{active=issuing=flat_confirmed=pause_requested=paused=false;pause_at=0;state=station=slot=batch=kind=retrieval=0;loaded=coarse=stored=stacked=transfers=completed_stops=0;failure="none";}
static void issue(const char *text,uint32_t now)
{issuing=true;nav_app_command(text,now);issuing=false;}
static void phase(unsigned value,uint32_t now){state=value;entered=now;}
static void fail(uint32_t now,const char *why)
{
 if(!active)return;
 active=false;failure=why;
 if(nav_app_state()!=6)nav_app_emergency(now,why);
 nav_logf("[MISSION_ABORT] reason=%s phase=%u station=%u slot=%u wall_ms=%lu; no retry\r\n",why,state,station,slot,(unsigned long)(now-started));
}
static void report(uint32_t now)
{
 nav_logf("[MISSION] active=%u phase=%u station=%u batch=%u slot=%u retrieval=%u loaded=%02X coarse=%02X stored=%02X stacked=%02X transfers=%u wall_ms=%lu reason=%s; commanded transfers, no contact sensor\r\n",active,state,station,batch,slot,retrieval,loaded,coarse,stored,stacked,transfers,(unsigned long)(active?now-started:0),failure);
}
bool nav_mission_command(const char *line,uint32_t now)
{
 if(issuing)return false;
 if(!strcmp(line,"mission pause")){
  if(!active){nav_log("[E] no active mission\r\n");return true;}
  pause_requested=true;nav_log("[MISSION_PAUSE_REQUEST] next safe transport/work boundary\r\n");return true;
 }
 if(!strcmp(line,"mission resume")){
  if(!active||!paused){nav_log("[E] mission is not paused\r\n");return true;}
  paused=false;entered+=now-pause_at;nav_log("[MISSION_RESUMED] world and inventory preserved\r\n");return true;
 }
 if(!strcmp(line,"mission status")){report(now);return true;}
 if(!strcmp(line,"mission stop")){if(active)fail(now,"operator mission stop");else nav_app_emergency(now,"operator mission stop");return true;}
 if(active){
  if(!strcmp(line,"status")||!strcmp(line,"odom")||!strcmp(line,"perf")||!strcmp(line,"imu")||!strcmp(line,"stop")||!strcmp(line,"!")||!strcmp(line,"route cancel"))return false;
  nav_log("[E] mission owns motion; read status or stop first\r\n");return true;
 }
 if(!strcmp(line,"mission flat rough")){
  if(nav_app_state()||nav_arm_busy()){nav_log("[E] idle required for flat-height confirmation\r\n");return true;}
  flat_confirmed=true;nav_log("[MISSION_FLAT] operator confirms equal work planes; batch1 uses rough height, batch2 stack\r\n");return true;
 }
 unsigned mask=0;int radius=0,ox=0,oy=0,used=0;
 if(sscanf(line,"coordination %u %d %d %d %n",&mask,&radius,&ox,&oy,&used)==4&&used&&!line[used]){
  if(nav_app_state()||nav_arm_busy()||mask>15||radius<0||radius>500||ox< -150||ox>150||oy< -150||oy>150||((mask&8)&&radius<150)){nav_log("[E] coordination idle mask0..15 radius0..500(>=150 for motion), offsets+-150\r\n");return true;}
  coordination=mask;arm_radius=radius*.001f;arm_ox=ox*.001f;arm_oy=oy*.001f;nav_logf("[COORDINATION] mask=%u radius=%d offset=%d,%d; RAM only\r\n",mask,radius,ox,oy);return true;
 }
 unsigned setting=0;int consumed=0;
 if(sscanf(line,"mission indexing %u %n",&setting,&consumed)==1&&consumed&&!line[consumed]){
  if(nav_app_state()||nav_arm_busy()||setting>1){nav_log("[E] indexing0 legacy,1 smooth; FIXED place123/recover123; idle required\r\n");return true;}
  indexing_mode=setting;nav_logf("[MISSION_INDEXING] mode=%u; no move\r\n",indexing_mode);return true;
 }
 consumed=0;
 unsigned jerk=0;
 if(sscanf(line,"mission indexdynamics %u %u %n",&setting,&jerk,&consumed)==2&&consumed&&!line[consumed]){
  if(nav_app_state()||nav_arm_busy()||setting<200||setting>1200||jerk<500||jerk>8000){nav_log("[E] index accel200..1200mm/s2 jerk500..8000mm/s3; idle required\r\n");return true;}
  index_accel=setting;index_jerk=jerk;nav_logf("[INDEX_DYNAMICS] accel=%u jerk=%u; no move\r\n",index_accel,index_jerk);return true;
 }
 consumed=0;
 if(sscanf(line,"mission speed %u %n",&setting,&consumed)==1&&consumed&&!line[consumed]){
  if(nav_app_state()||setting<10||setting>75){nav_log("[E] mission speed10..75 idle required\r\n");return true;}
  work_rpm=setting;nav_logf("[MISSION_SPEED] %u RPM; RAM only\r\n",work_rpm);return true;
 }
 if(!strcmp(line,"mission start")){
  if(nav_app_state()||!nav_app_segmented()||!nav_map_qr_enabled()||!nav_map_valid()||!flat_confirmed||!nav_arm_transport_ready()){
   nav_log("[E] mission needs idle, valid obstacle map/plan, segmented+QR on, flat rough confirmation, referenced clearance and turn1000\r\n");return true;
  }
  loaded=coarse=stored=stacked=transfers=completed_stops=0;station=slot=batch=kind=retrieval=0;failure="none";
  issue("route start",now);
  if(!nav_app_active()){nav_log("[E] route did not start; mission remains idle\r\n");return true;}
  active=true;coordinated_transfer=early_extended=false;prefetch_at=0;pause_requested=paused=false;started=now;last_pick_started=now-5000U;phase(TRAVEL,now);
  nav_log("[MISSION_START] two batches, fixed slot order1/2/3; QR5s simulated; raw start-to-start interval>=5s; no human confirmation pauses\r\n");return true;
 }
 if(!strncmp(line,"mission",7)){nav_log("[E] mission status|flat rough|start|stop\r\n");return true;}
 return false;
}
static bool at_target(void)
{float p[3];nav_app_pose(p);return hypotf(p[0]-target[0],p[1]-target[1])<.009f;}
static unsigned desired_turn(void){return state==CENTER?1000:kind==1&&retrieval?2130:1000;}
static void request_line(uint32_t now,unsigned next)
{
 if(!nav_arm_clearance_ready())return;
 float p[3];nav_app_pose(p);coordinated_transfer=(coordination&8U)&&arm_line_clear(p,target);
 if(!coordinated_transfer&&!nav_arm_transport_ready()){
  if(!nav_arm_busy()){nav_arm_turn(1000);}
  return;
 }
 if(at_target()){
  if(coordination&1U||coordinated_transfer)nav_arm_prefetch(slot?slot:1,coordinated_transfer?desired_turn():0,true);
  coordinated_transfer=false;phase(next,now);return;
 }
 char command[64];snprintf(command,sizeof command,"work goto %d %d %u",(int)lroundf(target[0]*1000),(int)lroundf(target[1]*1000),work_rpm);
 issue(command,now);
 if(!nav_app_active()){fail(now,"work line not accepted");return;}
 if(coordination&1U||coordinated_transfer)nav_arm_prefetch(slot?slot:1,coordinated_transfer?desired_turn():0,true);
 phase(MOVE_WAIT,now);
}
void nav_mission_tick(uint32_t now)
{
 if(!active)return;
 if(nav_app_state()==7)return; /* paused transport: no prefetch/extension during detour */
 if(nav_app_state()==6||nav_arm_faulted()){fail(now,"chassis or arm fault");return;}
 if(paused)return;
 if(pause_requested&&nav_app_state()==0&&nav_arm_transport_ready()){
  paused=true;pause_requested=false;pause_at=now;nav_logf("[MISSION_PAUSED] phase=%u station=%u slot=%u; joints at transport clearance\r\n",state,station,slot);return;
 }
 if(now-started>1200000U){fail(now,"mission20min deadline");return;}
 unsigned timeout=state==TRAVEL?180000:state==JOB_WAIT?45000:20000;
 if(now-entered>timeout){fail(now,"mission phase deadline");return;}
 if(state==TRAVEL){
  if(nav_app_active()){
   if(now-prefetch_at>=100){
    prefetch_at=now;float next[3],remaining;unsigned dest;
    if(nav_app_upcoming(next,&dest,&remaining)&&dest>=1&&dest<=6){
     if(coordination&1U)nav_arm_prefetch(1,0,false);
     if((coordination&8U)&&(dest==1||dest==4)&&remaining<.45f&&nav_app_arm_approach_clear(arm_radius,arm_ox,arm_oy)){
      if(nav_arm_prefetch(1,2130,true))early_extended=true;
     }
    }
   }
   if(early_extended){float p[3];nav_app_pose(p);if(!nav_map_arm_safe(p[0],p[1],p[2],arm_radius,arm_ox,arm_oy)){fail(now,"extended arm actual envelope changed");return;}}
   return;
  }
  early_extended=false;
  station=nav_app_work_station();
  if(!station){
   if(completed_stops==7&&loaded==0&&coarse==0&&stored==7&&stacked==7&&transfers==24){
    active=false;nav_logf("[MISSION_DONE] wall_ms=%lu transfers=%u; six raw picks, six coarse placements, six coarse recoveries, three flat+three stack; verify actual objects\r\n",(unsigned long)(now-started),transfers);nav_app_emergency(now,"mission complete park");return;
   }
   fail(now,"unexpected route completion or inventory");return;
  }
  completed_stops++;
  if(!nav_app_station_target(center)){fail(now,"task center unavailable");return;}
  if(station==9){delay_until=now+5000;phase(QR_WAIT,now);nav_log("[MISSION_QR] simulated5s, no physical camera height/servo operation\r\n");return;}
  batch=station<=3?1:2;kind=(station-1)%3;slot=1;retrieval=0;
  if((kind==0&&loaded)||(kind!=0&&loaded!=7)||(kind==1&&coarse)||(kind==2&&((batch==1&&stored)||(batch==2&&stored!=7)))){fail(now,"task inventory precondition");return;}
  if(kind==0){delay_until=last_pick_started+5000U;phase(FEED,now);}else phase(ALIGN,now);
  return;
 }
 if(state==QR_WAIT||state==FEED){
  if(state==FEED&&(coordination&4U)&&nav_app_state()==0)nav_arm_prefetch(slot,2130,true);
  if((int32_t)(now-delay_until)<0)return;
  phase(state==QR_WAIT?RESUME:JOB_START,now);return;
 }
 if(state==ALIGN||state==CENTER){
  if(!nav_arm_clearance_ready())return;
  if(state==CENTER){slot=0;target[0]=center[0];target[1]=center[1];request_line(now,RESUME);}
  else{float forward=(2-(int)slot)*.150f;target[0]=center[0]+cosf(center[2])*forward;target[1]=center[1]+sinf(center[2])*forward;request_line(now,JOB_START);}
  return;
 }
 if(state==MOVE_WAIT){
  if(coordinated_transfer){float p[3];nav_app_pose(p);if(!nav_map_arm_safe(p[0],p[1],p[2],arm_radius,arm_ox,arm_oy)){fail(now,"work arm envelope changed");return;}}
  if(nav_app_active())return;
  if(!at_target()){fail(now,"work line endpoint");return;}
  if(!nav_arm_idle_ready())return;
  coordinated_transfer=false;phase(slot?JOB_START:RESUME,now);return;
 }
 if(state==JOB_START){
  if(!nav_arm_idle_ready())return;
  unsigned bit=1U<<(slot-1);
  if((kind==0&&(loaded&bit))||(kind==1&&(!retrieval?(!(loaded&bit)||(coarse&bit)):(!(coarse&bit)||(loaded&bit))))||(kind==2&&(!(loaded&bit)||(batch==1?(stored&bit):(!(stored&bit)||(stacked&bit)))))){
   fail(now,"transfer inventory precondition");return;
  }
  const char *job=kind==0?"grab":kind==1?(retrieval?"unrough":"rough"):(batch==1?"rough":"stack");
  if(!nav_arm_job_start(job,slot)){fail(now,"arm job not accepted");return;}
  if(kind==0)last_pick_started=now;
  nav_logf("[MISSION_JOB] station=%u batch=%u %s slot=%u\r\n",station,batch,job,slot);phase(JOB_WAIT,now);return;
 }
 if(state==JOB_WAIT){
  if(nav_arm_busy()||!nav_arm_clearance_ready())return;
  unsigned bit=1U<<(slot-1);transfers++;
  if(kind==0)loaded|=bit;
  else if(kind==1){if(retrieval){coarse&=~bit;loaded|=bit;}else{loaded&=~bit;coarse|=bit;}}
  else{loaded&=~bit;if(batch==1)stored|=bit;else stacked|=bit;}
  slot++;
  if(slot>=1&&slot<=3){if(kind==0){delay_until=last_pick_started+5000U;phase(FEED,now);}else phase(ALIGN,now);return;}
  if(kind==1&&!retrieval){if(coarse!=7||loaded){fail(now,"coarse must contain all3 before recovery");return;}retrieval=1;slot=1;phase(ALIGN,now);return;}
  phase(CENTER,now);return;
 }
 if(state==RESUME){
  if(!nav_arm_transport_ready())return;
  if((station==1||station==2||station==4||station==5)&&loaded!=7){fail(now,"travel requires three objects on tray");return;}
  if((station==3||station==6)&&loaded){fail(now,"storage departure requires empty tray");return;}
  issue("route continue",now);
  if(!nav_app_active()){fail(now,"route continue rejected");return;}
  phase(TRAVEL,now);return;
 }
}
