#include "nav/nav_arm2_emm.h"
#include <math.h>

bool nav_arm2_pulses(const Arm2Axis *a,float q,int32_t *out)
{
 if(!a||!out||!isfinite(q)||!isfinite(a->machine_zero)||
    !isfinite(a->travel_per_output_rev)||a->travel_per_output_rev<=0||
    !isfinite(a->motor_rev_per_output_rev)||a->motor_rev_per_output_rev<=0||
    !a->pulses_per_motor_rev||(a->direction!=1&&a->direction!=-1))return false;
 float pulses=(q-a->machine_zero)*(float)a->direction*a->motor_rev_per_output_rev*
               (float)a->pulses_per_motor_rev/a->travel_per_output_rev+(float)a->driver_zero;
 if(!isfinite(pulses)||fabsf(pulses)>10000000)return false;
 *out=(int32_t)lroundf(pulses);return true;
}
bool nav_arm2_emm_move(uint8_t id,Arm2EmmTarget t,bool sync,uint8_t f[13])
{
 if(!f||(id!=5&&id!=6)||!t.rpm||t.rpm>3000||t.position< -10000000||t.position>10000000)return false;
 bool positive=t.position>=0;uint32_t mag=(uint32_t)(positive?t.position:-t.position);
 f[0]=id;f[1]=0xFD;f[2]=positive?0:1; /* Legacy Emm position direction differs from wheel speed mapping. */
 f[3]=(uint8_t)(t.rpm>>8);f[4]=(uint8_t)t.rpm;f[5]=t.accel;
 f[6]=(uint8_t)(mag>>24);f[7]=(uint8_t)(mag>>16);f[8]=(uint8_t)(mag>>8);f[9]=(uint8_t)mag;
 f[10]=1;f[11]=sync;f[12]=0x6B;
 return true;
}
bool nav_arm2_emm_begin(Arm2EmmBatch *b,Arm2EmmTarget lift,Arm2EmmTarget rack)
{
 uint8_t frame[13];
 if(!b||b->stage!=ARM2_EMM_IDLE||!nav_arm2_emm_move(5,lift,true,frame)||
    !nav_arm2_emm_move(6,rack,true,frame))return false;
 b->target[0]=lift;b->target[1]=rack;b->stage=ARM2_EMM_LIFT_ACK;return true;
}
size_t nav_arm2_emm_next(const Arm2EmmBatch *b,uint8_t f[13])
{
 if(!b||!f)return 0;
 if(b->stage==ARM2_EMM_READY){f[0]=0;f[1]=0xFF;f[2]=0x66;f[3]=0x6B;return 4;}
 if(b->stage==ARM2_EMM_LIFT_ACK||b->stage==ARM2_EMM_RACK_ACK){
  unsigned k=b->stage==ARM2_EMM_RACK_ACK;
  return nav_arm2_emm_move((uint8_t)(5+k),b->target[k],true,f)?13:0;
 }
 return 0;
}
bool nav_arm2_emm_ack(Arm2EmmBatch *b,const uint8_t f[4])
{
 if(!b||!f||(b->stage!=ARM2_EMM_LIFT_ACK&&b->stage!=ARM2_EMM_RACK_ACK))return false;
 unsigned expected=b->stage==ARM2_EMM_LIFT_ACK?5:6;
 if(f[0]!=expected||f[1]!=0xFD||f[3]!=0x6B)return false;
 if(f[2]!=2){b->stage=ARM2_EMM_BLOCKED;return false;}
 b->stage=expected==5?ARM2_EMM_RACK_ACK:ARM2_EMM_READY;return true;
}
void nav_arm2_emm_abort(Arm2EmmBatch *b){if(b&&b->stage!=ARM2_EMM_IDLE)b->stage=ARM2_EMM_BLOCKED;}
bool nav_arm2_emm_trigger_sent(Arm2EmmBatch *b)
{
 if(!b||b->stage!=ARM2_EMM_READY)return false;
 b->stage=ARM2_EMM_TRIGGERED;return true;
}
bool nav_arm2_emm_clear(Arm2EmmBatch *b,bool cleared)
{
 if(!b||!cleared)return false;
 b->stage=ARM2_EMM_IDLE;return true;
}
