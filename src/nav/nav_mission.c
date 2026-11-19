#include "nav/studio_profile.h"
/* Two-batch preliminary-rule rehearsal. The STM32 owns the whole sequence.
 * Vision/colour are explicitly simulated; transfer counts are commanded jobs,
 * NOT measured object-presence or competition-score claims. */
#include "nav/nav_mission.h"
#include "nav/nav_app.h"
#include "nav/nav_arm.h"
#include "nav/nav_map.h"
#include "nav/nav_platform.h"
#include "nav/nav_vision.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

enum {TRAVEL=1,FEED,ALIGN,MOVE_WAIT,JOB_START,JOB_WAIT,CENTER,RESUME,QR_WAIT,
      OBSERVE_PREP,OBSERVE_WAIT,OBSERVE_RETURN,ADJUST,OBSERVE_RETRY};
enum {SCAN_QR, FIND_RAW, ALIGN_FIELD, VERIFY_JOB};
enum {VIEW_QR, VIEW_RAW, VIEW_ROUGH, VIEW_STORAGE, VIEW_TRAY};
typedef struct {unsigned profile,turn,pwm;} CameraView;
static CameraView views[5];
static bool official,park_only,camera_started,corrected,observation_done;
static unsigned home,obs_job,obs_cmd,obs_attempt,obs_view,move_next;
static uint32_t obs_start,camera_settle_at;
static unsigned verified_picks,verified_places,unverified,skipped,quarantine,rough_unavailable;
static unsigned colors[2][3],locations[2][3];
static float measured_delta[2];
static unsigned obs_next;
static uint32_t raw_capture_at;
static int raw_velocity[2],raw_error[2];
static bool raw_moving,raw_reacquired;
static unsigned verdict;
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
unsigned nav_mission_home(bool *only_park)
{
 *only_park=active&&official&&park_only;
 return active&&official?home:0;
}
void nav_mission_select_park(void){if(active&&official)park_only=true;}
bool nav_mission_owns_vision(void){return active&&official;}
static bool arm_line_clear(const float p[3],const float q[2]){
 if(arm_radius<.15f)return false;
 unsigned n=(unsigned)ceilf(hypotf(q[0]-p[0],q[1]-p[1])/.01f);if(!n)n=1;
 for(unsigned k=0;k<=n;k++)if(!nav_map_arm_safe(p[0]+(q[0]-p[0])*k/n,p[1]+(q[1]-p[1])*k/n,p[2],arm_radius,arm_ox,arm_oy))return false;
 return true;
}
static const char *failure;
void nav_mission_init(void)
{active=issuing=flat_confirmed=pause_requested=paused=official=park_only=false;pause_at=0;state=station=slot=batch=kind=retrieval=0;loaded=coarse=stored=stacked=transfers=completed_stops=0;failure="none";memset(views,0,sizeof views);}
static void issue(const char *text,uint32_t now)
{issuing=true;nav_app_command(text,now);issuing=false;}
static void phase(unsigned value,uint32_t now){state=value;entered=now;}
static void fail(uint32_t now,const char *why)
{
 if(!active)return;
 active=false;failure=why;
 if(nav_app_state()!=6)nav_app_emergency(now,why);
 if(official)nav_vision_cancel(now);
 nav_logf("[MISSION_ABORT] reason=%s phase=%u station=%u slot=%u wall_ms=%lu; no retry\r\n",why,state,station,slot,(unsigned long)(now-started));
}
static void report(uint32_t now)
{
 nav_logf("[MISSION] active=%u phase=%u station=%u batch=%u slot=%u retrieval=%u loaded=%02X coarse=%02X stored=%02X stacked=%02X transfers=%u wall_ms=%lu reason=%s; commanded transfers, no contact sensor\r\n",active,state,station,batch,slot,retrieval,loaded,coarse,stored,stacked,transfers,(unsigned long)(active?now-started:0),failure);
 if(official)nav_logf("[OFFICIAL_SCORE] verified_pick=%u verified_place=%u unverified=%u skipped=%u quarantined=%02X\r\n",verified_picks,verified_places,unverified,skipped,quarantine);
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
 char view_name[12];unsigned profile,turn,pwm;
 if(sscanf(line,"mission view %11s %u %u %u %n",view_name,&profile,&turn,&pwm,&consumed)==4&&consumed&&!line[consumed]){
  const char *const names[]={"qr","raw","rough","storage","tray"};
  unsigned k=0;while(k<5&&strcmp(names[k],view_name))k++;
  if(nav_app_state()||nav_arm_busy()||k==5||profile>255||
     (profile&&((turn!=1000&&turn!=2130)||pwm<500||pwm>2500))){
   nav_log("[E] mission view qr|raw|rough|storage|tray PROFILE TURN_PWM CAMERA_PWM; idle\r\n");return true;
  }
  views[k]=(CameraView){profile,turn,pwm};
  nav_logf("[MISSION_VIEW] type=%s profile=%u turn=%u camera=%u; RAM only\r\n",view_name,profile,turn,pwm);return true;
 }
 consumed=0;
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
 bool race=!strcmp(line,"mission start official");
 if(race||!strcmp(line,"mission start")){
  if(nav_app_state()||!nav_app_segmented()||!nav_map_qr_enabled()||!nav_map_valid()||!flat_confirmed||!nav_arm_transport_ready()){
   nav_log("[E] mission needs idle, valid obstacle map/plan, segmented+QR on, flat rough confirmation, referenced clearance and turn1000\r\n");return true;
  }
  if(race&&(!nav_app_official_plan_ready()||nav_vision_pending())){
   nav_log("[E] official needs QR-first lidar plan and no pending vision request\r\n");return true;
  }
  float start_pose[3];nav_app_pose(start_pose);
  unsigned chosen_home=start_pose[1]>1.05f?2:1;
  if(race&&(hypotf(start_pose[0],start_pose[1]-(chosen_home==2?2.1f:0))>.020f||
            fabsf(atan2f(sinf(start_pose[2]),cosf(start_pose[2])))>.01746f)){
   nav_log("[E] official start requires physical zone1 or zone2 center and WEST heading\r\n");return true;
  }
  loaded=coarse=stored=stacked=transfers=completed_stops=0;station=slot=batch=kind=retrieval=0;failure="none";
  if(race)(void)nav_vision_command("vision uart",now);
  issue("route start",now);
  if(!nav_app_active()){nav_log("[E] route did not start; mission remains idle\r\n");return true;}
  official=race;home=chosen_home;park_only=false;
  verified_picks=verified_places=unverified=skipped=quarantine=rough_unavailable=0;
  memset(colors,0,sizeof colors);
  active=true;coordinated_transfer=early_extended=false;prefetch_at=0;pause_requested=paused=false;started=now;last_pick_started=now-5000U;phase(TRAVEL,now);
  if(official){nav_logf("[MISSION_START] official home=%u; visual task, bounded retries, partial-credit enabled\r\n",home);return true;}
  nav_log("[MISSION_START] two batches, fixed slot order1/2/3; QR5s simulated; raw start-to-start interval>=5s; no human confirmation pauses\r\n");return true;
 }
 if(!strncmp(line,"mission",7)){nav_log("[E] mission status|flat rough|start [official]|view TYPE PROFILE TURN_PWM CAM_PWM|stop\r\n");return true;}
 return false;
}
static bool at_target(void)
{float p[3];nav_app_pose(p);return hypotf(p[0]-target[0],p[1]-target[1])<.009f;}
static unsigned desired_turn(void){return state==CENTER?1000:kind==1&&retrieval?2130:1000;}
static void request_line(uint32_t now,unsigned next,unsigned rpm)
{
 if(!nav_arm_clearance_ready())return;
 float p[3];nav_app_pose(p);coordinated_transfer=!official&&(coordination&8U)&&arm_line_clear(p,target);
 if(!coordinated_transfer&&!nav_arm_transport_ready()){
  if(!nav_arm_busy()){nav_arm_turn(1000);}
  return;
 }
 if(at_target()){
  if(coordination&1U||coordinated_transfer)nav_arm_prefetch(slot?slot:1,coordinated_transfer?desired_turn():0,true);
  coordinated_transfer=false;phase(next,now);return;
 }
 char command[64];snprintf(command,sizeof command,"work goto %d %d %u",(int)lroundf(target[0]*1000),(int)lroundf(target[1]*1000),rpm);
 issue(command,now);
 if(!nav_app_active()){fail(now,"work line not accepted");return;}
 if(coordination&1U||coordinated_transfer)nav_arm_prefetch(slot?slot:1,coordinated_transfer?desired_turn():0,true);
 move_next=next;phase(MOVE_WAIT,now);
}
static unsigned field_index(void)
{
 if(!official||kind==0)return slot;
 if(kind==1||batch==1)return locations[batch-1][slot-1];
 unsigned color=colors[1][slot-1];
 for(unsigned k=0;k<3;k++)if(colors[0][k]==color)return locations[0][k];
 return 0;
}
static unsigned get_u16(const uint8_t *p){return p[0]|((unsigned)p[1]<<8);}
static int get_s16(const uint8_t *p){unsigned v=get_u16(p);return v>=32768?(int)v-65536:(int)v;}
static unsigned observation_age(const NavVisionResult *r,uint32_t now)
{return r->length>=8?get_u16(r->data+6)+(now-r->received_at):65535U;}
static void begin_observation(unsigned job,uint32_t now)
{
 obs_job=job;obs_attempt=0;obs_start=now;obs_cmd=0;
 camera_started=false;camera_settle_at=0;
 obs_view=job==SCAN_QR?VIEW_QR:job==FIND_RAW?VIEW_RAW:
   job==VERIFY_JOB&&(kind==0||retrieval)?VIEW_TRAY:kind==1?VIEW_ROUGH:VIEW_STORAGE;
 phase(OBSERVE_PREP,now);
}
static void next_official_slot(uint32_t now)
{
 corrected=observation_done=raw_moving=raw_reacquired=false;
 if(++slot<=3){phase(kind==0?JOB_START:ALIGN,now);return;}
 if(kind==1&&!retrieval){retrieval=1;slot=1;phase(ALIGN,now);return;}
 phase(CENTER,now);
}
static void skip_official_slot(uint32_t now,const char *why)
{
 unsigned index=field_index(),bit=1U<<(slot-1);
 if(kind==1&&retrieval&&(coarse&bit)){
  rough_unavailable|=1U<<(index-1);coarse&=~bit;
 }
 if(((kind==1&&!retrieval)||kind==2)&&(loaded&bit)){
  quarantine|=bit;loaded&=~bit;
 }
 /* A part still on the tray must keep occupying its slot across batches. */
 skipped++;
 nav_logf("[MISSION_SKIP] station=%u batch=%u tray=%u field=%u retrieval=%u reason=%s\r\n",station,batch,slot,index,retrieval,why);
 next_official_slot(now);
}
static void commit_official_job(uint32_t now)
{
 unsigned index=field_index(),tray_bit=1U<<(slot-1),field_bit=1U<<(index-1);
 bool pick=kind==0||(kind==1&&retrieval);
 transfers++;
 if(verdict==2){
  quarantine|=tray_bit;loaded&=~tray_bit;coarse&=~tray_bit;
  if(kind==1)rough_unavailable|=field_bit;
  skipped++;
  nav_logf("[MISSION_VERIFY_FAIL] station=%u tray=%u field=%u; possible occupancy retained\r\n",station,slot,index);
 }else{
  if(verdict==1){if(pick)verified_picks++;else verified_places++;}
  else unverified++;
  if(kind==0)loaded|=tray_bit;
  else if(kind==1){
   if(retrieval){coarse&=~tray_bit;loaded|=tray_bit;}
   else{loaded&=~tray_bit;coarse|=tray_bit;}
  }else{
   loaded&=~tray_bit;
   if(batch==1)stored|=field_bit;else stacked|=field_bit;
  }
 }
 nav_logf("[MISSION_TRANSFER] batch=%u station=%u tray=%u field=%u verdict=%u\r\n",batch,station,slot,index,verdict);
 next_official_slot(now);
}
static bool send_observation(uint32_t now)
{
 unsigned cmd=0,count=0,timeout=650,area=kind==1?2:3;
 uint8_t args[5]={0};
 unsigned index=slot?field_index():0,color=slot?colors[batch-1][slot-1]:0;
 if(obs_job==SCAN_QR){
  cmd=obs_attempt==1&&views[VIEW_QR].profile&&camera_started?10:1;
  timeout=cmd==1?2200:650;
 }else if(obs_job==FIND_RAW){
  cmd=6;count=2;args[0]=(uint8_t)batch;args[1]=(uint8_t)color;
 }else if(obs_job==ALIGN_FIELD){
  unsigned aim=kind==1?(retrieval?1:2):batch==1?2:3;
  cmd=7;count=5;args[0]=(uint8_t)area;args[1]=(uint8_t)index;
  args[2]=(uint8_t)(aim==2?0:color);args[3]=(uint8_t)aim;args[4]=(uint8_t)(aim==3?2:1);
 }else{
  bool pick=kind==0||retrieval;
  unsigned action=pick?2:kind==2&&batch==2?5:3;
  cmd=8;count=5;args[0]=(uint8_t)action;args[1]=(uint8_t)(pick?4:area);
  args[2]=(uint8_t)(pick?slot:index);args[3]=(uint8_t)color;args[4]=(uint8_t)(action==5?2:1);
 }
 if(!nav_vision_request(cmd,views[obs_view].profile,args,count,timeout,now))return false;
 obs_cmd=cmd;obs_attempt++;phase(OBSERVE_WAIT,now);return true;
}
static bool raw_candidate(const NavVisionResult *r,uint32_t now)
{
 unsigned age=observation_age(r,now);
 if(r->length<22||age>300)return false;
 const uint8_t *best=NULL;
 for(unsigned i=0;i<r->data[8];i++){
  const uint8_t *item=r->data+9+13*i;
  float dx=(float)get_s16(item+3),dy=(float)get_s16(item+5);
  unsigned flags=item[12];
  if(!(flags&1U)||item[11]<25||hypotf(dx,dy)>25)continue;
  if((flags&8U)&&((flags&7U)!=7U||hypotf((float)get_s16(item+7),(float)get_s16(item+9))>180))continue;
  if(!best||item[11]>best[11])best=item;
 }
 if(!best)return false;
 raw_moving=(best[12]&8U)!=0;
 raw_capture_at=now-age;
 raw_error[0]=get_s16(best+3);raw_error[1]=get_s16(best+5);
 raw_velocity[0]=get_s16(best+7);raw_velocity[1]=get_s16(best+9);
 measured_delta[0]=(float)raw_error[0];measured_delta[1]=(float)raw_error[1];
 return true;
}
static void finish_observation(const NavVisionResult *r,uint32_t now)
{
 bool ok=r&&r->cmd==obs_cmd&&r->status==0;
 unsigned next=JOB_START;
 if(obs_job==SCAN_QR){
  if(ok&&r->cmd==1&&r->length==15){
   for(unsigned g=0;g<2;g++)for(unsigned k=0;k<3;k++){
    colors[g][k]=(unsigned)(r->data[g*8+k]-'0');
    locations[g][k]=(unsigned)(r->data[g*8+4+k]-'0');
   }
   nav_logf("[MISSION_TASK] code=%.*s\r\n",15,(const char *)r->data);
  }else if(obs_attempt<3&&obs_cmd&&send_observation(now))return;
  else{
   park_only=true;
   nav_log("[MISSION_PARTIAL] QR unavailable after retries; return to starting zone\r\n");
  }
  next=RESUME;
 }else if(obs_job==FIND_RAW){
  if(ok&&raw_candidate(r,now)){
   next=!raw_moving&&!corrected&&hypotf(measured_delta[0],measured_delta[1])>=8?ADJUST:JOB_START;
   if(!raw_moving&&corrected&&hypotf(measured_delta[0],measured_delta[1])>18)next=0;
  }else if(now-obs_start<10500&&obs_attempt<32&&obs_cmd){
   delay_until=now+350;phase(OBSERVE_RETRY,now);return;
  }else next=0;
 }else if(obs_job==ALIGN_FIELD){
  if(ok&&r->length==21&&observation_age(r,now)<=400&&r->data[19]>=25&&
     (r->data[20]&5U)==1U){
   measured_delta[0]=(float)get_s16(r->data+13);measured_delta[1]=(float)get_s16(r->data+15);
   float size=hypotf(measured_delta[0],measured_delta[1]);
   if(size>=8&&!corrected&&size<=25)next=ADJUST;
   else if(size>30||((corrected||retrieval||(kind==2&&batch==2))&&size>20))next=0;
  }else if(r&&r->cmd==7&&r->status==1&&(retrieval||(kind==2&&batch==2))){
   if(obs_attempt<2&&send_observation(now))return;
   next=0; /* Explicitly absent source or stack support. */
  }else nav_log("[MISSION_FALLBACK] visual alignment unavailable; taught field pose\r\n");
 }else{
  verdict=ok&&r->length==15&&observation_age(r,now)<=650&&r->data[14]>=25?r->data[13]:0;
  next=JOB_WAIT;
 }
 obs_next=next;phase(OBSERVE_RETURN,now);
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
     if(!official&&(coordination&8U)&&(dest==1||dest==4)&&remaining<.45f&&nav_app_arm_approach_clear(arm_radius,arm_ox,arm_oy)){
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
   if(official&&((park_only&&completed_stops==1)||completed_stops==7)){
    active=false;
    nav_logf("[MISSION_DONE] mode=official outcome=%s home=%u wall_ms=%lu transfers=%u verified_pick=%u verified_place=%u unverified=%u skipped=%u\r\n",
      !park_only&&transfers==24&&!skipped&&!unverified?"verified":"partial",home,
      (unsigned long)(now-started),transfers,verified_picks,verified_places,unverified,skipped);
    nav_app_emergency(now,"mission complete park");return;
   }
   if(completed_stops==7&&loaded==0&&coarse==0&&stored==7&&stacked==7&&transfers==24){
    active=false;nav_logf("[MISSION_DONE] wall_ms=%lu transfers=%u; six raw picks, six coarse placements, six coarse recoveries, three flat+three stack; verify actual objects\r\n",(unsigned long)(now-started),transfers);nav_app_emergency(now,"mission complete park");return;
   }
   fail(now,"unexpected route completion or inventory");return;
  }
  if(!nav_app_station_target(center)){fail(now,"task center unavailable");return;}
  if(station==7){phase(RESUME,now);return;} /* Optional zone2 transit, no transfer. */
  completed_stops++;
  if(station==9){
   if(official){begin_observation(SCAN_QR,now);return;}
   delay_until=now+5000;phase(QR_WAIT,now);nav_log("[MISSION_QR] simulated5s, no physical camera height/servo operation\r\n");return;
  }
  batch=station<=3?1:2;kind=(station-1)%3;slot=1;retrieval=0;
  if(official){corrected=observation_done=raw_moving=raw_reacquired=false;phase(kind==0?JOB_START:ALIGN,now);return;}
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
  if(state==CENTER){slot=0;target[0]=center[0];target[1]=center[1];request_line(now,RESUME,work_rpm);}
  else{
   unsigned index=field_index(),bit=1U<<(slot-1),field_bit=1U<<(index-1);
   if(official){
    bool needed=kind==1?(retrieval?(coarse&bit)!=0:(loaded&bit)&&!(rough_unavailable&field_bit)):
      (loaded&bit)&&(batch==1||(stored&field_bit));
    if(!needed){skip_official_slot(now,"no available material or work position");return;}
   }
   float forward=(2-(int)index)*.150f;
   target[0]=center[0]+cosf(center[2])*forward;target[1]=center[1]+sinf(center[2])*forward;
   if(official&&!nav_app_work_clear(target[0],target[1])){skip_official_slot(now,"field position blocked");return;}
   request_line(now,JOB_START,work_rpm);
  }
  return;
 }
 if(state==MOVE_WAIT){
  if(coordinated_transfer){float p[3];nav_app_pose(p);if(!nav_map_arm_safe(p[0],p[1],p[2],arm_radius,arm_ox,arm_oy)){fail(now,"work arm envelope changed");return;}}
  if(nav_app_active())return;
  if(!at_target()){fail(now,"work line endpoint");return;}
  if(!nav_arm_idle_ready())return;
  coordinated_transfer=false;phase(move_next,now);return;
 }
 if(state==OBSERVE_PREP){
  const CameraView *view=views+obs_view;bool qr=obs_job==SCAN_QR;
  if(!view->profile||!nav_arm_observation_available(qr)){
   if(qr&&send_observation(now))return;
   finish_observation(NULL,now);return;
  }
  if(!camera_started){
   if(!nav_arm_clearance_ready())return;
   if(!nav_arm_turn_done(view->turn)){
    (void)nav_arm_turn(view->turn);return;
   }
   if(!nav_arm_observation_begin(qr,view->pwm)){finish_observation(NULL,now);return;}
   camera_started=true;
  }
  if(!nav_arm_observation_ready())return;
  if(!camera_settle_at){camera_settle_at=now;return;}
  if(now-camera_settle_at<150)return;
  if(!send_observation(now))finish_observation(NULL,now);
  return;
 }
 if(state==OBSERVE_WAIT){
  if(nav_vision_pending())return;
  finish_observation(nav_vision_result(),now);return;
 }
 if(state==OBSERVE_RETRY){
  if((int32_t)(now-delay_until)<0)return;
  if(!send_observation(now)){obs_cmd=0;finish_observation(NULL,now);}
  return;
 }
 if(state==OBSERVE_RETURN){
  if(camera_started){
   if(!nav_arm_observation_return())return;
   camera_started=false;
  }
  if(!nav_arm_clearance_ready())return;
  if(!obs_next){skip_official_slot(now,"visual target missing or correction too large");return;}
  if(obs_job==FIND_RAW||obs_job==ALIGN_FIELD)observation_done=obs_next==JOB_START;
  phase(obs_next,now);return;
 }
 if(state==ADJUST){
  float p[3];nav_app_pose(p);
  float x=measured_delta[0]*.001f,y=measured_delta[1]*.001f;
  target[0]=p[0]+cosf(p[2])*x-sinf(p[2])*y;target[1]=p[1]+sinf(p[2])*x+cosf(p[2])*y;
  if(!nav_app_work_clear(target[0],target[1])){
   if(kind==0||retrieval||(kind==2&&batch==2)){skip_official_slot(now,"correction would leave clear work envelope");return;}
   nav_log("[MISSION_FALLBACK] correction blocked; taught field pose\r\n");
   observation_done=true;phase(JOB_START,now);return;
  }
  corrected=true;
  request_line(now,JOB_START,10);return;
 }
 if(state==JOB_START){
  if(!nav_arm_idle_ready())return;
  unsigned bit=1U<<(slot-1);
  if(official){
   if(kind==0&&((loaded|quarantine)&bit)){skip_official_slot(now,"tray slot unavailable");return;}
   if(!observation_done){begin_observation(kind==0?FIND_RAW:ALIGN_FIELD,now);return;}
   if(kind==0&&raw_moving){
    float lead=.80f+(now-raw_capture_at)*.001f;
    float dx=raw_error[0]+lead*raw_velocity[0],dy=raw_error[1]+lead*raw_velocity[1];
    if(hypotf(dx,dy)>22){
     if(!raw_reacquired){raw_reacquired=true;begin_observation(FIND_RAW,now);return;}
     skip_official_slot(now,"moving target left grasp window");return;
    }
   }
  }else if((kind==0&&(loaded&bit))||(kind==1&&(!retrieval?(!(loaded&bit)||(coarse&bit)):(!(coarse&bit)||(loaded&bit))))||(kind==2&&(!(loaded&bit)||(batch==1?(stored&bit):(!(stored&bit)||(stacked&bit)))))){
   fail(now,"transfer inventory precondition");return;
  }
  const char *job=kind==0?"grab":kind==1?(retrieval?"unrough":"rough"):(batch==1?"rough":"stack");
  if(!nav_arm_job_start(job,slot)){
   if(official&&!nav_arm_faulted()){skip_official_slot(now,"arm job prerequisites unavailable");return;}
   fail(now,"arm job not accepted");return;
  }
  if(kind==0)last_pick_started=now;
  verdict=3; /* Official job still needs its optional observation. */
  nav_logf("[MISSION_JOB] station=%u batch=%u %s slot=%u\r\n",station,batch,job,slot);phase(JOB_WAIT,now);return;
 }
 if(state==JOB_WAIT){
  if(nav_arm_busy()||!nav_arm_clearance_ready())return;
  if(official){
   if(verdict==3)begin_observation(VERIFY_JOB,now);
   else commit_official_job(now);
   return;
  }
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
  if(!nav_arm_transport_ready()){
   if(official&&nav_arm_clearance_ready())(void)nav_arm_turn(1000);
   return;
  }
  if(!official&&(station==1||station==2||station==4||station==5)&&loaded!=7){fail(now,"travel requires three objects on tray");return;}
  if(!official&&(station==3||station==6)&&loaded){fail(now,"storage departure requires empty tray");return;}
  issue("route continue",now);
  if(!nav_app_active()){fail(now,"route continue rejected");return;}
  phase(TRAVEL,now);return;
 }
}
