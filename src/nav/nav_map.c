/* Nine junctions / twelve corridor links. ROI reports are conservative
 * occupancy regions, NOT measured pole centers. Live lidar layer is checked separately in N frame. */
#include "nav/nav_map.h"
#include "nav/nav_lidar.h"
#include "nav/studio_profile.h"
#include "nav/nav_platform.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#define PI 3.14159265358979323846f
static const unsigned edge[12][2]={{0,1},{1,2},{0,3},{1,4},{2,5},{3,4},{4,5},{3,6},{4,7},{5,8},{6,7},{7,8}};
static const float coord[3]={.275f,1.2f,2.125f};
static NavObstacle obstacle[2];
static unsigned count,round_id,generation;
static bool valid,qr_checkpoint;
static bool direct_home=STUDIO_DIRECT_HOME;
static float planning_speed=.7f,planning_yaw=1;
void nav_map_plan_profile(unsigned percent,unsigned ppm){planning_speed=fmaxf(.05f,200.f*.88f*percent*.01f*3200.f/ppm/60);planning_yaw=fmaxf(.25f,percent*.01f);}
bool nav_map_qr_enabled(void){return qr_checkpoint;}
static float offsets[3][2]={{.020f,.044f},{-.028f,-.018f},{-.060f,.014f}};
static float start_pose[3];
void nav_map_plan_start(float x,float y,float h){start_pose[0]=x;start_pose[1]=y;start_pose[2]=h;}
static float raw_min_x=1.10f,raw_max_x=1.30f,raw_y=2.40f,raw_radius=.15f;
static float body[4]={.15f,.15f,.15f,.15f}; /* front,rear,left,right */
/* Figure 1 and Table 2, lower-left map coordinates. Half extents in metres. */
static const float work_regions[2][4]={{1.20f,.075f,.290f,.075f},{.075f,1.20f,.075f,.290f}};
bool nav_map_valid(void){return valid;}
void nav_map_invalidate(void){valid=false;generation++;}
unsigned nav_map_generation(void){return generation;}
bool nav_map_has_obstacles(void){return count!=0;}
bool nav_map_set(unsigned round,const NavObstacle *p,unsigned n)
{
 if(round<1||round>255||n>2||(!p&&n))return false;
 for(unsigned k=0;k<n;k++){
  if(p[k].kind<1||p[k].kind>3||p[k].id<1||p[k].id>(p[k].kind==1?12U:p[k].kind==2?9U:36U))return false;
  for(unsigned j=0;j<k;j++)if(p[j].kind==p[k].kind&&p[j].id==p[k].id)return false;
 }
 count=n;round_id=round;for(unsigned k=0;k<n;k++)obstacle[k]=p[k];valid=true;generation++;nav_map_report();return true;
}
/* Raw camera/map observations, mm from lower-left. Classify on F407, never
 * accept a PC-computed route. Conservative regions retain the swept-body
 * checks used by the existing junction graph planner. Validate atomically. */
bool nav_map_set_positions(unsigned round,const uint16_t *xs,const uint16_t *ys,unsigned n)
{
 if(n>2||(!xs&&n)||(!ys&&n)||round<1||round>255)return false;
 NavObstacle mapped[2]={{0,0},{0,0}};unsigned unique=0;
 for(unsigned k=0;k<n;k++){
  if(xs[k]>2400||ys[k]>2400)return false;
  for(unsigned j=0;j<k;j++)if(xs[k]==xs[j]&&ys[k]==ys[j])return false;
  float x=xs[k]*.001f,y=ys[k]*.001f,best=100;unsigned chosen=0;
  for(unsigned e=0;e<12;e++){
   unsigned u=edge[e][0],v=edge[e][1];float ax=coord[u%3],ay=coord[2-u/3],dx=coord[v%3]-ax,dy=coord[2-v/3]-ay;
   float t=fmaxf(0,fminf(1,((x-ax)*dx+(y-ay)*dy)/(dx*dx+dy*dy)));
   float distance=hypotf(x-ax-t*dx,y-ay-t*dy);
   if(distance<best){best=distance;chosen=e;}
  }
  if(best>.300f)return false;
  unsigned u=edge[chosen][0],v=edge[chosen][1];
  float f=u/3==v/3?(x-(u%3==0?.550f:1.400f))/.150f:((u/3==0?1.850f:1.000f)-y)/.150f;
  int section=(int)floorf(f+1e-5f);if(section<0)section=0;if(section>2)section=2;
  /* Round-5: always use the LOCALIZED 150 mm section ROI (kind3) instead of
 * the whole-channel ROI (kind1) for middle-section pole reports. blocked()
 * still forbids driving through that channel, so collision safety is
 * unchanged; only the conservative wall ROI no longer overhangs the
 * neighbouring junction, which restores 90-deg rotation feasibility there
 * (QR heading stays 90 deg). */
 NavObstacle o={3U,chosen*3+(unsigned)section+1};
  bool duplicate=false;for(unsigned j=0;j<unique;j++)if(mapped[j].kind==o.kind&&mapped[j].id==o.id)duplicate=true;
  if(!duplicate)mapped[unique++]=o;
 }
 if(!nav_map_set(round,mapped,unique))return false;
 for(unsigned k=0;k<n;k++)nav_logf("[OBSTACLE_POS] index=%u x_mm=%u y_mm=%u; classified and planned on MCU\r\n",k+1,xs[k],ys[k]);
 return true;
}
static bool blocked(unsigned e)
{
 for(unsigned k=0;k<count;k++){
  if(obstacle[k].kind==1&&obstacle[k].id==e+1)return true;
  if(obstacle[k].kind==3&&(obstacle[k].id-1)/3==e)return true;
  if(obstacle[k].kind==2&&(obstacle[k].id==edge[e][0]+1||obstacle[k].id==edge[e][1]+1))return true;
 }return false;
}
/* SAT for chassis OBB vs map-aligned rectangle; expanded by margin. */
static bool intersects(float x,float y,float h,float margin,float rx,float ry,float hx,float hy)
{
 float c=cosf(h),s=sinf(h),ac=fabsf(c),as=fabsf(s);
 float ox=(body[0]-body[1])*.5f,oy=(body[2]-body[3])*.5f;
 x-=c*ox-s*oy;y-=s*ox+c*oy;
 float a=(body[0]+body[1])*.5f+margin,b=(body[2]+body[3])*.5f+margin;
 float dx=x-rx,dy=y-ry;
 if(fabsf(dx)>=hx+a*ac+b*as||fabsf(dy)>=hy+a*as+b*ac)return false;
 return fabsf(c*dx+s*dy)<a+hx*ac+hy*as&&fabsf(-s*dx+c*dy)<b+hx*as+hy*ac;
}
static float point_box_d2(float x,float y,float a,float b)
{float dx=fmaxf(0,fabsf(x)-a),dy=fmaxf(0,fabsf(y)-b);return dx*dx+dy*dy;}
static float segment_box_d2(float px,float py,float qx,float qy,float a,float b)
{
 float d[2]={qx-px,qy-py},p[2]={px,py},extent[2]={a,b},enter=0,leave=1;
 bool hit=true;
 for(unsigned k=0;k<2;k++){
  if(fabsf(d[k])<1e-8f){if(fabsf(p[k])>extent[k])hit=false;}
  else{
   float lo=(-extent[k]-p[k])/d[k],hi=(extent[k]-p[k])/d[k];
   if(lo>hi){float t=lo;lo=hi;hi=t;}enter=fmaxf(enter,lo);leave=fminf(leave,hi);
  }
 }
 if(hit&&enter<=leave)return 0;
 float best=fminf(point_box_d2(px,py,a,b),point_box_d2(qx,qy,a,b));
 float norm=d[0]*d[0]+d[1]*d[1];
 for(unsigned i=0;i<2;i++)for(unsigned j=0;j<2;j++){
  float x=(i?1:-1)*a,y=(j?1:-1)*b;
  float t=norm>1e-12f?fmaxf(0,fminf(1,((x-px)*d[0]+(y-py)*d[1])/norm)):0;
  float dx=x-px-t*d[0],dy=y-py-t*d[1];best=fminf(best,dx*dx+dy*dy);
 }
 return best;
}
static bool touches_raw_fixture(float x,float y,float h,float margin)
{
 float c=cosf(h),s=sinf(h),ox=(body[0]-body[1])*.5f,oy=(body[2]-body[3])*.5f;
 x-=c*ox-s*oy;y-=s*ox+c*oy;
 float dx=raw_min_x-x,dy=raw_y-y,ex=raw_max_x-x;
 float px=c*dx+s*dy,py=-s*dx+c*dy,qx=c*ex+s*dy,qy=-s*ex+c*dy;
 float distance=segment_box_d2(px,py,qx,qy,(body[0]+body[1])*.5f+margin,(body[2]+body[3])*.5f+margin);
 return distance<=raw_radius*raw_radius+1e-8f;
}
const char *nav_map_footprint_blocker(float lx,float ly,float h,float margin)
{
 if(!isfinite(lx)||!isfinite(ly)||!isfinite(h)||!isfinite(margin)||margin<0)return "invalid_pose";
 if(!nav_lidar_plan_clear(lx,ly,h,nav_lidar_plan_token()?fmaxf(margin,.03f):margin))return "lidar";
 float x=2.25f-lx,y=2.25f-ly,c=cosf(h),s=sinf(h);
 float ox=(body[0]-body[1])*.5f,oy=(body[2]-body[3])*.5f;
 float cx=x-c*ox+s*oy,cy=y-s*ox-c*oy;
 float a=(body[0]+body[1])*.5f,b=(body[2]+body[3])*.5f;
 float hx=a*fabsf(c)+b*fabsf(s),hy=a*fabsf(s)+b*fabsf(c);
 if(cx-hx<-.0001f||cx+hx>2.4001f||cy-hy<-.0001f||cy+hy>2.4001f)return "field_outline";
 if(touches_raw_fixture(x,y,h,margin))return "raw_turntable";
 for(unsigned k=0;k<2;k++){
  const float *region=work_regions[k];
  if(intersects(x,y,h,margin,region[0],region[1],region[2],region[3]))
   return k?"storage_region":"rough_region";
 }
 for(unsigned i=0;i<2;i++)for(unsigned j=0;j<2;j++)
  if(intersects(x,y,h,margin,.775f+.85f*i,.775f+.85f*j,.225f,.225f))return "yellow_region";
 for(unsigned k=0;k<count;k++){
  unsigned id=obstacle[k].kind==3?(obstacle[k].id-1)/3:obstacle[k].id-1;
  if(obstacle[k].kind==2){
   float qx=coord[id%3],qy=coord[2-id/3];
   float wx=id%3==1?.20f:.275f,wy=id/3==1?.20f:.275f;
   if(intersects(x,y,h,margin,qx,qy,wx+.025f,wy+.025f))return "declared_obstacle";
  }else{
   unsigned u=edge[id][0],v=edge[id][1];float ux=coord[u%3],uy=coord[2-u/3];
   bool horizontal=u/3==v/3;float wx=horizontal?.225f:(u%3==1?.20f:.275f),wy=horizontal?(u/3==1?.20f:.275f):.225f;
   /* Physical road ROIs follow yellow bounds550..1000 /1400..1850,
    * not endpoint midpoints (outer and inner roads have unequal widths). */
   float rx=horizontal?(u%3==0?.775f:1.625f):ux;
   float ry=horizontal?uy:(u/3==0?1.625f:.775f);
   if(obstacle[k].kind==3){
    unsigned section=(obstacle[k].id-1)%3;
    /* An end-section report includes its neighbouring junction: "near G"
     * must not be silently narrowed to a guessed pole centre in the road. */
    if(horizontal){
     float lo=rx-.225f+section*.15f,hi=lo+.15f;
     if(section==0)lo=coord[u%3]-(u%3==1?.20f:.275f);
     if(section==2)hi=coord[v%3]+(v%3==1?.20f:.275f);
     rx=(lo+hi)*.5f;wx=(hi-lo)*.5f;
    }else{
     float hi=ry+.225f-section*.15f,lo=hi-.15f;
     if(section==0)hi=coord[2-u/3]+(u/3==1?.20f:.275f);
     if(section==2)lo=coord[2-v/3]-(v/3==1?.20f:.275f);
     ry=(lo+hi)*.5f;wy=(hi-lo)*.5f;
    }
   }
   if(intersects(x,y,h,margin,rx,ry,wx+.025f,wy+.025f))return "declared_obstacle";
  }
 }return NULL;
}
bool nav_map_footprint(float x,float y,float h,float margin)
{return nav_map_footprint_blocker(x,y,h,margin)==NULL;}

bool nav_map_work_span(const NavMapNode *dock)
{
 float c=cosf(dock->h),s=sinf(dock->h);
 for(int offset=-150;offset<=150;offset+=5){
  float d=offset*.001f,x=dock->x+c*d,y=dock->y+s*d;
  const char *block=nav_map_footprint_blocker(x,y,dock->h,.020f);
  if(!block)continue;
  nav_logf("[WORK_SPAN_BLOCKED] station=%u offset_mm=%d local_mm=%d,%d source=%s\r\n",
    dock->stop,offset,(int)lroundf(x*1000),(int)lroundf(y*1000),block);
  if(!strcmp(block,"lidar"))nav_lidar_plan_blockers(x,y,dock->h,.03f,false);
  return false;
 }
 return true;
}
static bool add(NavMapNode *p,unsigned *n,float x,float y,float h,unsigned stop)
{
 if(*n&&hypotf(p[*n-1].x-x,p[*n-1].y-y)<.00001f&&fabsf(p[*n-1].h-h)<.00001f){if(stop)p[*n-1].stop=stop;return true;}
 if(*n==NAV_MAP_NODES)return false;
 p[(*n)++]=(NavMapNode){x,y,h,stop};return true;
}
static int work_index(unsigned n){return n==1?0:n==7?1:n==3?2:-1;}
/* The 150 mm work strip plus the chassis side and 35 mm of clearance.
 * Preserve the taught offset along the row. Clamp only the approach normal. */
static float work_normal_limit(void){return 2.25f-(.150f+body[3]+.035f);}
static float jx(unsigned n){int k=work_index(n);float x=2.25f-coord[n%3]+(k>=0?offsets[k][0]:0);return n==3?fminf(x,work_normal_limit()):x;}
static float jy(unsigned n){int k=work_index(n);float y=2.25f-coord[2-n/3]+(k>=0?offsets[k][1]:(n==0||n==2?.050f:0));return n==7?fminf(y,work_normal_limit()):y;}
/* Travel junctions and taught work docks are different objects. In particular,
 * the raw dock has19mm platter clearance and cannot be a through-road turn.
 * Upper travel usesN.y=200mm; rotations use a verified275mm clearance bay.
 * Shift work-side junctions inboard so the graph can reach the docks. */
static float graph_x(unsigned n){float x=2.25f-coord[n%3];return n==3?fminf(x,1.915f):x;}
static float graph_y(unsigned n){float y=2.25f-coord[2-n/3];return n==7?fminf(y,1.915f):y+(n==1?.075f:(n==0||n==2?.050f:0));}
static bool blended_edge(const NavMapNode *a,const NavMapNode *b,float h0,float h1)
{
 float l=hypotf(a->x-b->x,a->y-b->y),d=atan2f(sinf(h1-h0),cosf(h1-h0));
 if(l<.45f||fabsf(d)>PI*.501f)return false;
 unsigned n=(unsigned)ceilf(l/.005f);
 for(unsigned j=0;j<=n;j++){
  if(!(j&15U)&&!nav_hw_plan_service())return false;
  float u=(float)j/n,v=u*u*u*(10+u*(-15+6*u));
  if(!nav_map_footprint(a->x+(b->x-a->x)*u,a->y+(b->y-a->y)*u,h0+d*v,.030f))return false;
 }
 return true;
}
static void blend_turns(NavMapNode *p,unsigned *n)
{
 for(unsigned k=0;k+2<*n;k++){
  if(hypotf(p[k].x-p[k+1].x,p[k].y-p[k+1].y)>.0001f||p[k+1].stop||fabsf(p[k].h-p[k+1].h)<.1f)continue;
  bool blend=false;
  if(k&&!p[k].stop&&fabsf(p[k-1].h-p[k].h)<.001f&&blended_edge(&p[k-1],&p[k],p[k].h,p[k+1].h)){
   p[k].h=p[k+1].h;blend=true;
  }else if(fabsf(p[k+2].h-p[k+1].h)<.001f&&blended_edge(&p[k],&p[k+2],p[k].h,p[k+1].h))blend=true;
  if(blend){memmove(p+k+1,p+k+2,(*n-k-2)*sizeof *p);(*n)--;if(k)k--;}
 }
}
static bool fillet(NavMapNode *p,unsigned *n)
{
 static NavMapNode q[NAV_MAP_NODES];unsigned used=0;
 for(unsigned k=0;k<*n;k++){
  bool curved=false;
  if(k&&k+1<*n&&!p[k].stop&&fabsf(p[k-1].h-p[k].h)<.0001f&&fabsf(p[k+1].h-p[k].h)<.0001f){
   float ax=p[k].x-p[k-1].x,ay=p[k].y-p[k-1].y,bx=p[k+1].x-p[k].x,by=p[k+1].y-p[k].y;
   float la=hypotf(ax,ay),lb=hypotf(bx,by);
   if(la>.20f&&lb>.20f&&fabsf((ax*bx+ay*by)/(la*lb))<.01f){
    ax/=la;ay/=la;bx/=lb;by/=lb;
    float radius=.04f,start=atan2f(-by,-bx),sign=ax*by-ay*bx;
    const float candidates[]={.25f,.18f,.125f,.075f,.04f};
    bool found=false;
    for(unsigned candidate=0;candidate<5&&!found;candidate++){
     float v=fminf(candidates[candidate],fminf(la,lb)*.45f);
     float cx=p[k].x-ax*v+bx*v,cy=p[k].y-ay*v+by*v;found=true;
     for(unsigned j=0;j<=32;j++){
      float a=start+(sign>0?1:-1)*PI*.5f*j/32;
      if(!nav_map_footprint(cx+v*cosf(a),cy+v*sinf(a),p[k].h,.020f)){found=false;break;}
     }
     if(found)radius=v;
     if(!nav_hw_plan_service())return false;
    }
    if(!found)return false;
    float cx=p[k].x-ax*radius+bx*radius,cy=p[k].y-ay*radius+by*radius;
    unsigned samples=(unsigned)ceilf(PI*.5f*radius/.05f);if(samples<4)samples=4;
    for(unsigned j=0;j<=samples;j++){
     float a=start+(sign>0?1:-1)*PI*.5f*j/samples;
     if(!add(q,&used,cx+radius*cosf(a),cy+radius*sinf(a),p[k].h,0))return false;
    }
    curved=true;
   }
  }
  if(!curved&&!add(q,&used,p[k].x,p[k].y,p[k].h,p[k].stop))return false;
 }
 memcpy(p,q,used*sizeof *p);*n=used;return true;
}
bool nav_map_path(NavMapNode *p,unsigned *n,const char **why)
{
 *n=0;*why="map not received";if(!valid)return false;
 /* Nine physical junctions x four cardinal body headings. Reachability
  * includes sampled translation AND swept quarter-turn body occupancy.
  * A blocked station-side turn may instead occur at an earlier junction. */
 bool walk[12][4],turn[9][4];
 for(unsigned e=0;e<12;e++)for(unsigned h=0;h<4;h++){
  if(!nav_hw_plan_service()){*why="planning interrupted";return false;}
  walk[e][h]=!blocked(e);unsigned u=edge[e][0],v=edge[e][1];
  for(unsigned j=0;walk[e][h]&&j<=94;j++){
   float q=j/94.0f;
   if(!nav_map_footprint(jx(u)+(jx(v)-jx(u))*q,jy(u)+(jy(v)-jy(u))*q,h*PI*.5f,.015f))walk[e][h]=false;
  }
 }
 for(unsigned u=0;u<9;u++)for(unsigned h=0;h<4;h++){
  if(!nav_hw_plan_service()){*why="planning interrupted";return false;}
  turn[u][h]=true;
  for(unsigned j=0;turn[u][h]&&j<=45;j++)
   if(!nav_map_footprint(jx(u),jy(u),(h+j/45.0f)*PI*.5f,.015f))turn[u][h]=false;
 }
 unsigned current=2,heading=0;
 /* Same 0.1mm numerical tolerance as field footprint, not a new origin. */
 if(!isfinite(start_pose[0])||!isfinite(start_pose[1])||!isfinite(start_pose[2])||start_pose[0]<-.0001f||start_pose[0]>.08f||start_pose[1]<-.0001f||start_pose[1]>.08f||fabsf(start_pose[2])>.035f){*why="plan start outside known zone1 frame";return false;}
 /* Reach the actual shifted upper-right graph junction before shortcuts.
  * Otherwise QR-off routes cut diagonally toward the raw platter. */
 if(!add(p,n,fmaxf(0,start_pose[0]),fmaxf(0,start_pose[1]),start_pose[2],0)||!add(p,n,.125f,.125f,0,0)||!add(p,n,jx(2),jy(2),0,0))return false;
 const unsigned goal[8]={1,7,3,1,7,3,8,2};
 /* Include incoming travel direction: a shorter zigzag is not necessarily
  * quicker. Costs are conservative time heuristics, not timing guarantees. */
 enum {STATES=180};
 unsigned total=(direct_home?7U:8U)+(qr_checkpoint?1U:0U);
 for(unsigned visit=0;visit<total;visit++){
  if(!nav_hw_plan_service()){*why="planning interrupted";return false;}
  bool qr=qr_checkpoint&&visit==0;unsigned station=visit-(qr_checkpoint?1U:0U);
  bool home=!qr&&station==(direct_home?6U:7U);
  unsigned target=qr?5:home?2:goal[station],target_h=qr?1:(station==1||station==4?2:station==2||station==5?1:0);
  unsigned start=(current*4+heading)*5,finish=STATES;
  float dist[STATES];int parent[STATES];bool used[STATES]={false};
  for(unsigned i=0;i<STATES;i++){dist[i]=10000;parent[i]=-1;}dist[start]=0;
  for(unsigned it=0;it<STATES;it++){
   if(!(it&15U)&&!nav_hw_plan_service()){*why="planning interrupted";return false;}
   unsigned state=STATES;for(unsigned i=0;i<STATES;i++)if(!used[i]&&(state==STATES||dist[i]<dist[state]))state=i;
   if(state==STATES||dist[state]>=10000)break;
   used[state]=true;unsigned u=state/20,h=(state/5)%4,incoming=state%5;
   if(u==target&&h==target_h){finish=state;break;}
   for(unsigned e=0;e<12;e++)if(walk[e][h]&&(edge[e][0]==u||edge[e][1]==u)){
    unsigned v=edge[e][0]==u?edge[e][1]:edge[e][0];
    float dx=jx(v)-jx(u),dy=jy(v)-jy(u);
    unsigned direction=fabsf(dx)>=fabsf(dy)?(dx>0?1:2):(dy>0?3:4),next=(v*4+h)*5+direction;
    float transition=incoming&&incoming!=direction?(((incoming-1)/2==(direction-1)/2)?.80f:.40f):0;
    float cost=hypotf(dx,dy)/planning_speed+transition;
    if(dist[next]>dist[state]+cost){dist[next]=dist[state]+cost;parent[next]=(int)state;}
   }
   for(unsigned d=0;d<2;d++){
    unsigned nh=(h+(d?3:1))%4,next=(u*4+nh)*5+incoming;
    bool possible=d?turn[u][nh]:turn[u][h];float cost=PI*.5f/planning_yaw;
    if(possible&&dist[next]>dist[state]+cost){dist[next]=dist[state]+cost;parent[next]=(int)state;}
   }
  }
  if(finish==STATES){*why="no connected pose path to required station";return false;}
  unsigned reverse[STATES],len=0,state=finish;
  while(state!=start){if(len==STATES||parent[state]<0){*why="invalid pose-graph predecessor";return false;}reverse[len++]=state;state=(unsigned)parent[state];}
  while(len){state=reverse[--len];unsigned u=state/20,h=(state/5)%4;
   /* Work endpoints already use jx/jy compensation. Blue docks differ
    * from their graph junction: retain that junction before entering the
    * dock, or a last long diagonal can cut the raw platter on the way home. */
   if(!len&&u==target&&h==target_h&&(qr||station<6))continue;
   if(!add(p,n,jx(u),jy(u),h*PI*.5f,0)){*why="node capacity";return false;}
  }
  heading=target_h;float h=heading*PI*.5f,x=jx(target),y=jy(target);
  if(!qr&&station>=6){x=.020f;y=home?.020f:2.080f;}
  if(!add(p,n,x,y,h,qr?9:home?8:station+1)){*why="node capacity";return false;}
  /* Depart directly from the actual compensated endpoint; full geometry is checked by nav_route_plan. */
  current=target;
 }
 blend_turns(p,n);
 if(!fillet(p,n)){*why="fillet node capacity";return false;}
 *why="none";return true;
}
void nav_map_report(void)
{
 nav_logf("[MAP] valid=%u round=%u count=%u generation=%u; ROAD1..12 JUNCTION1..9; pole50x100mm, conservative ROI occupancy\r\n",valid,round_id,count,generation);
 for(unsigned k=0;k<count;k++)nav_logf("[OBSTACLE] kind=%u id=%u\r\n",obstacle[k].kind,obstacle[k].id);
 for(unsigned k=0;k<3;k++)nav_logf("[STATION_OFFSET] %s local_dx_mm=%d local_dy_mm=%d RAM_ONLY\r\n",k==0?"raw":k==1?"rough":"left",(int)roundf(offsets[k][0]*1000),(int)roundf(offsets[k][1]*1000));
 nav_logf("[BODY] front/rear/left/right=%d,%d,%d,%d mm; includes secured arm, not extended measurement pose\r\n",(int)(body[0]*1000),(int)(body[1]*1000),(int)(body[2]*1000),(int)(body[3]*1000));
}
bool nav_map_command(const char *line)
{
 if(!strcmp(line,"route finish direct")||!strcmp(line,"route finish via2")){direct_home=!strcmp(line,"route finish direct");generation++;nav_logf("[ROUTE_FINISH] direct_home=%u; replan required, final storage -> zone1\r\n",direct_home);return true;}
 if(!strcmp(line,"route qr on")||!strcmp(line,"route qr off")){qr_checkpoint=!strcmp(line,"route qr on");generation++;nav_logf("[QR_CHECKPOINT] enabled=%u; plan again; geometry only, no camera acquisition\r\n",qr_checkpoint);return true;}
 if(!strcmp(line,"map")){nav_map_report();return true;}
 if(!strcmp(line,"map empty")){nav_map_set(1,0,0);return true;}
 char name[16];int a,b,n=0,c,d;
 if(sscanf(line,"station offset %15s %d %d %n",name,&a,&b,&n)==3&&n&&!line[n]){
  unsigned k=!strcmp(name,"raw")?0:!strcmp(name,"rough")?1:!strcmp(name,"left")?2:3;
  if(k==3||a< -150||a>150||b< -150||b>150){nav_log("[E] offset station raw/rough/left; each -150..150mm\r\n");return true;}
  offsets[k][0]=a*.001f;offsets[k][1]=b*.001f;generation++;nav_map_report();return true;
 }
 n=0;if(sscanf(line,"footprint %d %d %d %d %n",&a,&b,&c,&d,&n)==4&&n&&!line[n]){
  if(a<150||b<150||c<150||d<150||a>350||b>350||c>350||d>350){nav_log("[E] body extents150..350mm; measure actual secured geometry\r\n");return true;}
  body[0]=a*.001f;body[1]=b*.001f;body[2]=c*.001f;body[3]=d*.001f;generation++;nav_map_report();return true;
 }
 return false;
}


static bool circle_box(float x,float y,float radius,float rx,float ry,float hx,float hy)
{float dx=fmaxf(0,fabsf(x-rx)-hx),dy=fmaxf(0,fabsf(y-ry)-hy);return dx*dx+dy*dy<=radius*radius;}
bool nav_map_arm_safe(float lx,float ly,float h,float radius,float ox,float oy)
{
 if(!isfinite(lx)||!isfinite(ly)||!isfinite(h)||!isfinite(radius)||radius<.15f||radius>.50f||!isfinite(ox)||!isfinite(oy)||fabsf(ox)>.15f||fabsf(oy)>.15f)return false;
 float x=2.25f-lx-cosf(h)*ox+sinf(h)*oy,y=2.25f-ly-sinf(h)*ox-cosf(h)*oy;
 radius+=.015f;
 if(x-radius<0||x+radius>2.4f||y-radius<0||y+radius>2.4f)return false;
 /* This sweep is checked while the chassis moves. Parked arm jobs use
  * their own taught motion and may reach into the target work area. */
 for(unsigned k=0;k<2;k++){
  const float *region=work_regions[k];
  if(circle_box(x,y,radius,region[0],region[1],region[2],region[3]))return false;
 }
 for(unsigned i=0;i<2;i++)for(unsigned j=0;j<2;j++)if(circle_box(x,y,radius,.775f+.85f*i,.775f+.85f*j,.225f,.225f))return false;
 for(unsigned k=0;k<count;k++){
  unsigned id=obstacle[k].kind==3?(obstacle[k].id-1)/3:obstacle[k].id-1;
  if(obstacle[k].kind==2){
   float qx=coord[id%3],qy=coord[2-id/3];
   float wx=id%3==1?.20f:.275f,wy=id/3==1?.20f:.275f;
   if(circle_box(x,y,radius,qx,qy,wx+.025f,wy+.025f))return false;
  }else{
   unsigned u=edge[id][0],v=edge[id][1];float ux=coord[u%3],uy=coord[2-u/3];
   bool horizontal=u/3==v/3;float wx=horizontal?.225f:(u%3==1?.20f:.275f),wy=horizontal?(u/3==1?.20f:.275f):.225f;
   /* Physical road ROIs follow yellow bounds550..1000 /1400..1850,
    * not endpoint midpoints (outer and inner roads have unequal widths). */
   float rx=horizontal?(u%3==0?.775f:1.625f):ux;
   float ry=horizontal?uy:(u/3==0?1.625f:.775f);
   if(obstacle[k].kind==3){
    unsigned section=(obstacle[k].id-1)%3;
    /* An end-section report includes its neighbouring junction: "near G"
     * must not be silently narrowed to a guessed pole centre in the road. */
    if(horizontal){
     float lo=rx-.225f+section*.15f,hi=lo+.15f;
     if(section==0)lo=coord[u%3]-(u%3==1?.20f:.275f);
     if(section==2)hi=coord[v%3]+(v%3==1?.20f:.275f);
     rx=(lo+hi)*.5f;wx=(hi-lo)*.5f;
    }else{
     float hi=ry+.225f-section*.15f,lo=hi-.15f;
     if(section==0)hi=coord[2-u/3]+(u/3==1?.20f:.275f);
     if(section==2)lo=coord[2-v/3]-(v/3==1?.20f:.275f);
     ry=(lo+hi)*.5f;wy=(hi-lo)*.5f;
    }
   }
   if(circle_box(x,y,radius,rx,ry,wx+.025f,wy+.025f))return false;
  }
 }return true;
}


/* Nine junctions and twelve roads; endpoints attach to the same corridor.
 * No fine grid, lateral obstacle pass or non-neighbour junction shortcut.
 * Swept body geometry includes lidar live and remembered circles. */
static bool detour_attachment(const NavMapNode *at,unsigned junction){
 unsigned nearest=0;float nearest_d=100;
 for(unsigned j=0;j<9;j++){
  float d=hypotf(at->x-graph_x(j),at->y-graph_y(j));
  if(d<nearest_d){nearest=j;nearest_d=d;}
 }
 /* Docks and taught stations attach to their nearest physical junction.
  * This prevents endpoints being used as shortcuts past a blocked junction. */
 if(nearest_d<=.25f)return junction==nearest;
 unsigned road=12;float road_d=100;
 for(unsigned e=0;e<12;e++){
  unsigned u=edge[e][0],v=edge[e][1];float ax=graph_x(u),ay=graph_y(u),dx=graph_x(v)-ax,dy=graph_y(v)-ay;
  float t=((at->x-ax)*dx+(at->y-ay)*dy)/(dx*dx+dy*dy);
  if(t<0||t>1)continue;
  float d=hypotf(at->x-ax-t*dx,at->y-ay-t*dy);
  if(d<road_d){road=e;road_d=d;}
 }
 return road<12&&road_d<=.20f&&!blocked(road)&&(junction==edge[road][0]||junction==edge[road][1]);
}
static bool detour_link(unsigned u,unsigned v,const NavMapNode *p){
 if(u>=1&&u<=9&&v>=1&&v<=9){
  for(unsigned e=0;e<12;e++)if((edge[e][0]==u-1&&edge[e][1]==v-1)||(edge[e][1]==u-1&&edge[e][0]==v-1))return !blocked(e);
  return false;
 }
 if(u>=1&&u<=9)return detour_attachment(&p[v],u-1);
 if(v>=1&&v<=9)return detour_attachment(&p[u],v-1);
 return false; /* Exact endpoints never form a visibility bypass. */
}
/* Straight upper-road travel stays50mm clear of the yellow/platter bounds.
 * Only an actual change of heading uses this deterministic move-turn-return
 * manoeuvre. Search remains9 junctions x6 headings, without a fine grid. */
static bool upper_turn_bay(const NavMapNode *a,const NavMapNode *b,NavMapNode *via){
 if(hypotf(a->x-graph_x(1),a->y-graph_y(1))>=.0001f||hypotf(b->x-graph_x(1),b->y-graph_y(1))>=.0001f)return false;
 if(fabsf(atan2f(sinf(b->h-a->h),cosf(b->h-a->h)))<.001f)return false;
 *via=(NavMapNode){graph_x(1),.275f,a->h,0};return true;
}
static bool detour_segment(const NavMapNode *a,const NavMapNode *b){
 float length=hypotf(b->x-a->x,b->y-a->y),dh=atan2f(sinf(b->h-a->h),cosf(b->h-a->h));
 unsigned samples=(unsigned)ceilf(fmaxf(length/.01f,fabsf(dh)/.025f));if(!samples)samples=1;
 for(unsigned j=0;j<=samples;j++){
  if(!(j&15U)&&!nav_hw_plan_service())return false;
  float q=(float)j/samples;
  if(!nav_map_footprint(a->x+(b->x-a->x)*q,a->y+(b->y-a->y)*q,a->h+dh*q,.015f))return false;
 }return true;
}
static bool detour_edge(const NavMapNode *a,const NavMapNode *b){
 NavMapNode via;
 if(upper_turn_bay(a,b,&via)){
  NavMapNode after=via;after.h=b->h;
  return detour_segment(a,&via)&&detour_segment(&via,&after)&&detour_segment(&after,b);
 }
 return detour_segment(a,b);
}
static bool detour_add(NavMapNode *out,unsigned *n,float x,float y,float h,unsigned stop){
 if(*n)h=out[*n-1].h+atan2f(sinf(h-out[*n-1].h),cosf(h-out[*n-1].h));
 return add(out,n,x,y,h,stop);
}
bool nav_map_detour(NavMapNode *out,unsigned *count_out,const NavMapNode *start,const NavMapNode *goals,unsigned goal_count,const char **why){
 enum {N=11,H=6,S=N*H};
 static int8_t edges[H][N][N],turns[N][H][H];
 /* Physical junctions/cardinal headings recur at every task leg. Reuse
  * only IDENTICALLY directed swept checks within this one frozen input.
  * Exact start/goal attachments and arbitrary headings remain uncached.
  * Nothing survives a new call, frame/config change, or live-map search. */
 static int8_t fixed_edges[4][9][9],fixed_turns[9][4][4];
 static float dist[S];static int16_t parent[S];static bool visited[S];
 if(!count_out||!why)return false;
 *count_out=0;*why="invalid detour arguments";
 if(!out||!start||!goals||goal_count<1||goal_count>10||!valid||!isfinite(start->x)||!isfinite(start->y)||!isfinite(start->h))return false;
 uint32_t token=nav_lidar_plan_token();unsigned map_generation=generation;
 memset(fixed_edges,-1,sizeof fixed_edges);memset(fixed_turns,-1,sizeof fixed_turns);
 NavMapNode from=*start;from.stop=0;
 if(!detour_add(out,count_out,from.x,from.y,from.h,0))return false;
 for(unsigned leg=0;leg<goal_count;leg++){
  NavMapNode goal=goals[leg];
  if(!isfinite(goal.x)||!isfinite(goal.y)||!isfinite(goal.h)||!goal.stop){*why="invalid detour task goal";return false;}
  if(goal.stop==9){
   static const float oy[]={0,.025f,-.025f,.050f,-.050f,.075f,-.075f};
   static const float ox[]={0,.025f,.050f,.075f,.100f};
   bool found=false;
   for(unsigned ix=0;ix<5&&!found;ix++)for(unsigned iy=0;iy<7;iy++){
    float x=goal.x+ox[ix],y=jy(5)+oy[iy];
    if(nav_map_footprint(x,y,goal.h,.03f)){
     goal.x=x;goal.y=y;found=true;
     if(ix||iy)nav_logf("[QR_DOCK] inward_mm=%d offset_y_mm=%d; final pose and actual approach checked, NO360 pivot\r\n",(int)(ox[ix]*1000),(int)(oy[iy]*1000));
     break;
    }
   }
   if(!found){*why="QR dock final footprint blocked";*count_out=0;return false;}
  }
  NavMapNode p[N];p[0]=from;p[10]=goal;
  for(unsigned j=0;j<9;j++)p[j+1]=(NavMapNode){graph_x(j),graph_y(j),0,0};
  float heading[H]={0,PI*.5f,PI,PI*1.5f,from.h,goal.h};
  memset(edges,-1,sizeof edges);memset(turns,-1,sizeof turns);
  for(unsigned j=0;j<S;j++){dist[j]=1e9f;parent[j]=-1;visited[j]=false;}
  unsigned first=4,finish=10*H+5;dist[first]=0;
  for(unsigned it=0;it<S;it++){
   if(!nav_hw_plan_service()){*why="detour planning interrupted";return false;}
   if(generation!=map_generation||(token&&nav_lidar_plan_token()!=token)){
    *why="detour map/frame changed during planning";*count_out=0;return false;
   }
   unsigned state=S;for(unsigned j=0;j<S;j++)if(!visited[j]&&(state==S||dist[j]<dist[state]))state=j;
   if(state==S||dist[state]>1e8f)break;
   if(state==finish)break;
   visited[state]=true;unsigned u=state/H,h=state%H;
   NavMapNode a={p[u].x,p[u].y,heading[h],0};
   for(unsigned v=0;v<N;v++)if(v!=u&&detour_link(u,v,p)){
    /* The right outer vertical road is lateral/blind from a WEST start.
     * Face that road before traversing it during the QR survey. Other roads
     * remain holonomic; a universal forward-only rule traps recovery at the
     * tight taught raw dock. A full QR scan below completes scene acquisition. */
    if(goal.stop==9){
     float dx=p[v].x-a.x,dy=p[v].y-a.y;
     if(a.x<.30f&&p[v].x<.30f&&fabsf(dy)>2*fabsf(dx)&&dx*dx+dy*dy>.0001f&&fabsf(atan2f(sinf(atan2f(dy,dx)-a.h),cosf(atan2f(dy,dx)-a.h)))>PI/3)continue;
    }
    unsigned next=v*H+h;
    if(edges[h][u][v]<0){
     int8_t *saved=token&&h<4&&u>=1&&u<=9&&v>=1&&v<=9?&fixed_edges[h][u-1][v-1]:NULL;
     if(saved&&*saved>=0)edges[h][u][v]=edges[h][v][u]=*saved;
     else{
      NavMapNode b={p[v].x,p[v].y,heading[h],0};edges[h][u][v]=edges[h][v][u]=(int8_t)detour_edge(&a,&b);
      if(saved)*saved=edges[h][u][v];
     }
    }
    float cost=hypotf(p[v].x-p[u].x,p[v].y-p[u].y)/planning_speed;
    if(edges[h][u][v]&&dist[next]>dist[state]+cost){dist[next]=dist[state]+cost;parent[next]=(int16_t)state;}
   }
   for(unsigned nh=0;nh<H;nh++)if(nh!=h){
    unsigned next=u*H+nh;
    if(turns[u][h][nh]<0){
     int8_t *saved=token&&u>=1&&u<=9&&h<4&&nh<4?&fixed_turns[u-1][h][nh]:NULL;
     if(saved&&*saved>=0)turns[u][h][nh]=turns[u][nh][h]=*saved;
     else{
      NavMapNode b={p[u].x,p[u].y,heading[nh],0};turns[u][h][nh]=turns[u][nh][h]=(int8_t)detour_edge(&a,&b);
      if(saved)*saved=turns[u][h][nh];
     }
    }
    float dh=fabsf(atan2f(sinf(heading[nh]-heading[h]),cosf(heading[nh]-heading[h]))),cost=dh/planning_yaw+.03f;
    NavMapNode b={p[u].x,p[u].y,heading[nh],0},via;
    if(upper_turn_bay(&a,&b,&via))cost+=2*hypotf(a.x-via.x,a.y-via.y)/planning_speed;
    if(turns[u][h][nh]&&dist[next]>dist[state]+cost){dist[next]=dist[state]+cost;parent[next]=(int16_t)state;}
   }
  }
  if(dist[finish]>1e8f){
   nav_logf("[DETOUR_NOPATH] leg=%u station=%u from_mm=%d,%d goal_mm=%d,%d endpoint_clear=%u; no margin relaxation\r\n",leg,goal.stop,(int)roundf(from.x*1000),(int)roundf(from.y*1000),(int)roundf(goal.x*1000),(int)roundf(goal.y*1000),nav_map_footprint(goal.x,goal.y,goal.h,.015f));
    nav_lidar_plan_blockers(goal.x,goal.y,goal.h,.015f,false);
   *why="no connected nine-junction route; no in-corridor bypass";*count_out=0;return false;
  }
  unsigned reverse[S],used=0,state=finish;
  while(state!=first){if(used==S||parent[state]<0){*why="invalid detour predecessor";return false;}reverse[used++]=state;state=(unsigned)parent[state];}
  while(used){state=reverse[--used];unsigned u=state/H,h=state%H;
   NavMapNode next={p[u].x,p[u].y,heading[h],state==finish?goal.stop:0},via;
   if(*count_out&&upper_turn_bay(&out[*count_out-1],&next,&via)){
    if(!detour_add(out,count_out,via.x,via.y,via.h,0)||!detour_add(out,count_out,via.x,via.y,next.h,0)){*why="turn bay node capacity";return false;}
   }
   if(!detour_add(out,count_out,p[u].x,p[u].y,heading[h],state==finish?goal.stop:0)){*why="detour node capacity";return false;}
  }
  if(!out[*count_out-1].stop)out[*count_out-1].stop=goal.stop;
  if(goal.stop==9){
   /* Do not insert four same-position 90-degree pivots here. The physical
    * QR board/支架 and nearby obstacles can make that pivot footprint
    * unreachable, and the user explicitly requested move-and-scan instead of
    * a stop-and-spin. The QR approach remains a normal moving approach; lidar
    * accumulation is active during the approach and the CAI freeze happens
    * at the actual QR stop. A true circular observation must be a separately
    * validated trajectory, not a hidden pivot at the dock. */
   out[*count_out-1].stop=9;
   nav_logf("[QR_APPROACH_SCAN] no stationary 4x90 pivot; collect QR anchors during approach, stop only at dock\r\n");
  }
  from=goal;
 }
 if(*count_out<2){*why="empty detour";return false;}
 *why="none";return true;
}

/* Deep interior of the already modelled raw-turntable centre interval.
 * A small random pole fully inside this region is already excluded by the
 * static map. This predicate never removes the existing fixture guard. */
bool nav_map_fixture_core(float lx,float ly,float inset){
 if(!isfinite(lx)||!isfinite(ly)||!isfinite(inset)||inset<0||inset>=.15f)return false;
 float mx=2.25f-lx,my=2.25f-ly;
 float dx=fmaxf(0,fmaxf(raw_min_x-mx,mx-raw_max_x)),dy=my-raw_y;
 float radius=raw_radius-inset;
 return dx*dx+dy*dy<radius*radius;
}
bool nav_map_static_covered(float lx,float ly,float r){
 if(!isfinite(lx)||!isfinite(ly)||!isfinite(r)||r<=0)return false;
 float x=2.25f-lx,y=2.25f-ly;
 for(unsigned k=0;k<2;k++){
  const float *region=work_regions[k];
  if(fabsf(x-region[0])+r<=region[2]&&fabsf(y-region[1])+r<=region[3])return true;
 }
 for(unsigned i=0;i<2;i++)for(unsigned j=0;j<2;j++){
  float ax=.55f+.85f*i,ay=.55f+.85f*j;
  if(x-r>=ax&&x+r<=ax+.45f&&y-r>=ay&&y+r<=ay+.45f)return true;
 }
 return false;
}

bool nav_map_sensed_path(NavMapNode *out,unsigned *n,const char **why){
 NavMapNode goals[9];unsigned count=0;
 if(qr_checkpoint)goals[count++]=(NavMapNode){jx(5),jy(5),PI*.5f,9};
 const unsigned visits[6]={1,7,3,1,7,3};
 for(unsigned i=0;i<6;i++){unsigned h=i==1||i==4?2:i==2||i==5?1:0;goals[count++]=(NavMapNode){jx(visits[i]),jy(visits[i]),h*PI*.5f,i+1};}
 if(!direct_home)goals[count++]=(NavMapNode){.020f,2.080f,0,7};
 goals[count++]=(NavMapNode){.020f,.020f,0,8};
 NavMapNode from={start_pose[0],start_pose[1],start_pose[2],0};
 return nav_map_detour(out,n,&from,goals,count,why);
}
/* Survey ends at the nominal QR dock only. It must not require any of the
 * later workstations to be reachable before gathering the initial scene. */
bool nav_map_qr_path(NavMapNode *out,unsigned *n,const char **why){
 NavMapNode from={start_pose[0],start_pose[1],start_pose[2],0};
 NavMapNode qr={jx(5),jy(5),PI*.5f,9};
 return nav_map_detour(out,n,&from,&qr,1,why);
}
bool nav_map_after_qr_path(NavMapNode *out,unsigned *n,const NavMapNode *start,const char **why){
 return nav_map_official_path(out,n,start,0,false,why);
}
bool nav_map_official_path(NavMapNode *out,unsigned *n,const NavMapNode *start,unsigned home,bool park_only,const char **why){
 NavMapNode goals[8];unsigned count=0;const unsigned visits[6]={1,7,3,1,7,3};
 for(unsigned i=0;!park_only&&i<6;i++){
  unsigned h=i==1||i==4?2:i==2||i==5?1:0;
  goals[count++]=(NavMapNode){jx(visits[i]),jy(visits[i]),h*PI*.5f,i+1};
 }
 if(!home&&!direct_home)goals[count++]=(NavMapNode){.020f,2.080f,0,7};
 goals[count++]=(NavMapNode){.020f,home==2?2.080f:.020f,0,8};
 return nav_map_detour(out,n,start,goals,count,why);
}
