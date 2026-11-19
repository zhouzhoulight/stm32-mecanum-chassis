#include "nav/nav_arm2.h"
#include "nav/nav_bus.h"
#include "nav/nav_app.h"
#include "nav/nav_arm.h"
#include "nav/nav_platform.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

bool nav_arm2_command(const char *line)
{
 if(strncmp(line,"arm2",4)||(line[4]&&line[4]!=' '))return false;
 if(!strcmp(line,"arm2 version")||!strcmp(line,"arm2 flags")||!strcmp(line,"arm2 position")){
  unsigned opcode=!strcmp(line,"arm2 version")?0x1F:!strcmp(line,"arm2 flags")?0x3A:0x36;
  if(nav_app_state()||nav_arm_busy()||!nav_bus_rack_read((uint8_t)opcode))
   nav_log("[E] arm2 read requires idle chassis/arm and free shared bus\r\n");
  else nav_log("[ARM2_QUERY] queued ID6 read; no motion\r\n");
  return true;
 }
 int x,y,z,used=0;
 if(sscanf(line,"arm2 solve %d %d %d %n",&x,&y,&z,&used)==3&&used&&!line[used]){
  Arm2Solution s={0};
  bool ok=nav_arm2_solve(&nav_arm2_preview,(Arm2Point){(float)x,(float)y,(float)z},0,false,3,&s);
  nav_logf("[ARM2_PREVIEW] feasible=%u yaw_mdeg=%d reach_mm=%d z_mm=%d miss_mm=%d; provisional geometry; no motion\r\n",
   ok,(int)lroundf(s.pose.yaw*57295.7795f),(int)lroundf(s.pose.reach),(int)lroundf(s.pose.z),(int)lroundf(s.error_mm));
  return true;
 }
 int tx,ty,tz;used=0;
 if(sscanf(line,"arm2 plan %d %d %d %d %d %d %n",&x,&y,&z,&tx,&ty,&tz,&used)==6&&used&&!line[used]){
  if(nav_app_state()){nav_log("[E] arm2 preview planning requires idle chassis\r\n");return true;}
  const Arm2Geometry *g=&nav_arm2_preview;
  Arm2Solution start={0},goal={0};
  Arm2Plan plan={0};
  bool solved=nav_arm2_solve(g,(Arm2Point){(float)x,(float)y,(float)z},0,false,3,&start)&&
              nav_arm2_solve(g,(Arm2Point){(float)tx,(float)ty,(float)tz},start.pose.yaw,false,3,&goal)&&
              nav_arm2_transfer(g,start.pose,goal.pose,NULL,0,true,&plan);
  nav_logf("[ARM2_PLAN] feasible=%u stages=%u duration_ms=%u; empty preview scene; no motion\r\n",
   solved,plan.count,(unsigned)lroundf(plan.seconds*1000));
  for(unsigned k=0;k<plan.count;k++){
   const Arm2Stage *s=plan.stage+k;
   nav_logf("[ARM2_STAGE] axes=%u yaw_mdeg=%d reach_mm=%d z_mm=%d duration_ms=%u\r\n",
    s->axes,(int)lroundf(s->target.yaw*57295.7795f),(int)lroundf(s->target.reach),
    (int)lroundf(s->target.z),(unsigned)lroundf(s->seconds*1000));
  }
  return true;
 }
 nav_log("[ARM2_HELP] version|flags|position; solve X Y Z; plan X0 Y0 Z0 X1 Y1 Z1 (body mm, preview)\r\n");
 return true;
}
