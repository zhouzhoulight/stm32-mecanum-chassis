#include "nav/studio_profile.h"
/* Reuse the tested R9 arm state machine without its clock/main/console UART.
 * Only nav_bus serializes bytes to the FIVE address bus. */
#include "nav/nav_arm.h"
#include "nav/nav_bus.h"
#include "nav/nav_app.h"
#include "nav/nav_mission.h"
#include "nav/nav_platform.h"
#define ARM_NAV_BUILD 1
#include "../../arm/project/APP/arm_task.c"
#include "../../arm/platform/profile_store.c"

TIM_HandleTypeDef htim3;
static bool arm_lift_permitted;
static bool observation_pose;
static unsigned short observation_previous_pwm;
bool nav_arm_bus_allowed(void){return arm_lift_permitted;}
void nav_arm_init(void){nav_arm_hw_init();ArmTask_Init();}
void nav_arm_tick(bool permit_lift)
{
 arm_lift_permitted=permit_lift;
 if(permit_lift)ArmTask_Tick();else servo_poll(HAL_GetTick());
}
void nav_arm_reply(const uint8_t *p,unsigned n)
{for(unsigned k=0;k<n;k++)motor_byte(p[k]);}
void nav_arm_sent(uint32_t at){if(pending)pending_tick=at;}
void nav_arm_suspend_reads(void)
{
 nav_bus_lift_cancel_read();probe_active=0;pending=0;packet_len=0;
}
void nav_arm_stop(void)
{
 observation_pose=false;
 /* Bus emergency owns FE5; halt() must not enqueue a second UART writer. */
 unsigned char access=motor_access;motor_access=0;moving=0;
 halt("shared chassis/arm stop");motor_access=access;
}
bool nav_arm_busy(void)
{return servo_busy()||moving||task_active||grab_active||grab_preparing||work_active||measure_state||datum_capture||point_capture||grab_height_capture;}
bool nav_arm_turn(unsigned pwm)
{
 if(nav_arm_busy()||servo_busy()||!servo_on[1]||pwm<1000||pwm>2130||pwm<turn_low||pwm>turn_high)return false;
 servo_rate[1]=tune_turn; /* Known transfer endpoints; measurement automation rate. */
 goal[1]=(unsigned short)pwm;servo_credit[1]=0;
 nav_logf("[ARM_TURN] target=%u PC7; PWM completion only, no physical sensor\r\n",pwm);return true;
}
bool nav_arm_turn_done(unsigned pwm)
{return servo_on[1]&&servo[1]==pwm&&goal[1]==pwm&&!servo_busy()&&!nav_arm_busy()&&turn_physically_settled(HAL_GetTick());}
bool nav_arm_calibration_ready(void)
{return nav_arm_turn_done(1000)&&turn_low<=1000&&turn_high>=2130;}
bool nav_arm_transport_held(void)
{return !nav_arm_busy()&&!measure_state&&referenced&&config_valid&&pos_valid&&flags_valid&&(flags&1U)&&!(flags&12U)&&!fault&&!probe_active&&labs(position-origin-clearance_height)<=20&&nav_arm_turn_done(1000);}
bool nav_arm_transport_ready(void)
{return !nav_arm_busy()&&lift_ready()&&!pending&&!probe_active&&labs(position-origin-clearance_height)<=20&&nav_arm_turn_done(1000);}
bool nav_arm_idle_ready(void){return !nav_arm_busy()&&lift_ready()&&!pending&&!probe_active;}
bool nav_arm_clearance_ready(void){return nav_arm_idle_ready()&&labs(position-origin-clearance_height)<=20;}
bool nav_arm_observation_available(bool qr)
{return servo_on[3]&&(camera_mask&(qr?2U:1U));}
bool nav_arm_observation_begin(bool qr,unsigned camera_pwm)
{
 if(observation_pose||nav_app_active()||!nav_arm_clearance_ready()||
    !nav_arm_observation_available(qr)||camera_pwm<500||camera_pwm>2500)return false;
 if(!move_to(camera_height[qr?1:0]))return false;
 observation_previous_pwm=servo[3];
 goal[3]=(unsigned short)camera_pwm;
 servo_credit[3]=0;
 observation_pose=true;return true;
}
bool nav_arm_observation_ready(void)
{return observation_pose&&nav_arm_idle_ready();}
bool nav_arm_observation_return(void)
{
 if(!observation_pose)return true;
 if(!nav_arm_idle_ready()||!move_to(clearance_height))return false;
 goal[3]=observation_previous_pwm;servo_credit[3]=0;
 observation_pose=false;return true;
}
bool nav_arm_faulted(void){return fault||measure_state;}
bool nav_arm_envelope_ready(void)
{return referenced&&config_valid&&pos_valid&&flags_valid&&!fault&&(flags&1)&&!(flags&12)&&!moving&&!task_active&&!grab_active&&!grab_preparing&&!work_active&&!measure_state&&labs(position-origin-clearance_height)<=20&&servo_on[0]&&servo_on[1]&&servo_on[2];}
bool nav_arm_prefetch(unsigned slot,unsigned turn,bool open_jaw)
{
 if(slot<1||slot>3||!nav_arm_envelope_ready())return false;
 if(turn&&turn!=1000&&turn!=2130)return false;
 if(turn&&(turn<turn_low||turn>turn_high))return false;
 unsigned changed=0;
 if(goal[2]!=grab_tray[slot-1]){goal[2]=grab_tray[slot-1];servo_rate[2]=tune_tray;servo_credit[2]=0;changed|=1;}
 if(turn&&goal[1]!=turn){goal[1]=turn;servo_rate[1]=tune_turn;servo_credit[1]=0;changed|=2;}
 if(open_jaw&&goal[0]!=1850){goal[0]=1850;servo_rate[0]=tune_jaw;servo_credit[0]=0;changed|=4;}
 if(changed)nav_logf("[PREFETCH] slot=%u turn=%u open=%u mask=%u; PWM only, lift HOLD unchanged\r\n",slot,turn,open_jaw,changed);
 return true;
}
bool nav_arm_job_start(const char *job,unsigned slot)
{
 if(nav_app_active()||!nav_arm_idle_ready()||slot<1||slot>3||(!strcmp(job,"grab")&&grab_height_mask!=3))return false;
 if(strcmp(job,"grab")&&strcmp(job,"rough")&&strcmp(job,"unrough")&&strcmp(job,"stack"))return false;
 bool cached=(nav_coordination_features()&2U)&&nav_arm_envelope_ready()&&fresh()&&!pending&&!probe_active&&pulses_per_rev==3200;
 char request[32];snprintf(request,sizeof request,"%s %u",job,slot);command(request);
 if(cached&&grab_preparing==1){grab_preparing=6;nav_log("[PREP_CACHED] fresh enabled flags/pose, known3200, same reference; clearance/arrival guards retained\r\n");}
 return grab_preparing||grab_active||work_active;
}
bool nav_arm_command(const char *input)
{
 const char *q=input;bool prefixed=!strncmp(q,"arm ",4);if(prefixed)q+=4;
 const char *words[]={"recover ","tuning ","seed ","servo ","j ","rate ","off ","turnlimits ","jawlimits ","point ","points","probe","r","flags","tpos","perr","current","voltage","measure ","datum ","save","grab","rough","stack","unrough","height ","limits ","lift ","jog ","speed ","accel ","arrival ","motion ","cal","swing ","trace ","zero","enable","disable"};
 bool recognized=prefixed;
 for(unsigned k=0;k<sizeof words/sizeof words[0];k++){
  size_t n=strlen(words[k]);if(!strncmp(q,words[k],n)&&(words[k][n-1]==' '||!q[n]||q[n]==' '))recognized=true;
 }
 if(!recognized)return false;
 if(nav_app_active()){
  nav_log("[E] arm commands require chassis idle; station uses route next/route cancel; no lift while driving\r\n");return true;
 }
 if(!arm_lift_permitted){nav_log("[E] bus stopping; wait for STOP_ACK\r\n");return true;}
 if(strlen(q)>=sizeof line){nav_log("[E] arm command too long\r\n");return true;}
 char copy[96];strcpy(copy,q);command(copy);return true;
}
