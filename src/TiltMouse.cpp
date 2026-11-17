#include "TiltMouse.h"
#include <Arduino.h>
#include <cmath>
namespace TiltMouse {
namespace {
struct Vec {float x,y,z;};
Vec add(Vec a,Vec b){return {a.x+b.x,a.y+b.y,a.z+b.z};}
Vec scale(Vec a,float k){return {a.x*k,a.y*k,a.z*k};}
float dot(Vec a,Vec b){return a.x*b.x+a.y*b.y+a.z*b.z;}
Vec cross(Vec a,Vec b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
float norm(Vec a){return std::sqrt(dot(a,a));}
Vec unit(Vec a){return scale(a,1.0f/norm(a));}
bool finite(Vec a){return std::isfinite(a.x)&&std::isfinite(a.y)&&std::isfinite(a.z);}
bool running=false,calibrated=false,haveTime=false,quiet=false,timedOut=false;
bool movingX=false,movingY=false;
uint32_t previous=0,entered=0,quietAt=0,printed=0;
unsigned count=0;
Vec gravity{},neutral{},right{},down{},accSum{},gyroSum{},bias{},anchor{};
float remainderX=0,remainderY=0;
constexpr float Degrees=57.29578f, MaxCountsPerSecond=800;
// Fixed mounting: positive gravity projection on device X -> right, device Y -> down.
float speed(float angle,bool& moving){
  float magnitude=std::fabs(angle);
  if(magnitude<=3) moving=false;
  else if(magnitude>4) moving=true;
  if(!moving) return 0;
  float level=fmaxf(0,fminf(1,(magnitude-4)/21));
  return (angle<0?-1:1)*MaxCountsPerSecond*std::pow(level,1.5f);
}
int8_t counts(float velocity,float dt,float& remainder){
  if(velocity==0){remainder=0;return 0;}
  float value=velocity*dt+remainder;
  int whole=int(value);remainder=value-whole;
  if(whole>127){whole=127;remainder=0;}
  if(whole< -127){whole=-127;remainder=0;}
  return int8_t(whole);
}
}
void reset(){
  running=calibrated=haveTime=quiet=timedOut=false;count=0;
  movingX=movingY=false;remainderX=remainderY=0;
  accSum=gyroSum=bias={0,0,0};
}
Delta update(const Gesture::Sample& s,uint32_t now,bool active,bool blocked,bool debug){
  Delta result;
  if(!active){reset();return result;}
  Vec a{s.ax,s.ay,s.az},w{s.gx,s.gy,s.gz};
  float magnitude=norm(a);
  if(!finite(a)||!finite(w)||magnitude<0.1f||std::fabs(w.x)>34||std::fabs(w.y)>34||std::fabs(w.z)>34){
    if(debug && now-printed>=500){printed=now;Serial.println("# MOUSE INVALID SAMPLE: stopped, recalibration required");}
    reset();return result;
  }
  if(haveTime && now-previous>100){reset();if(debug)Serial.println("# MOUSE SAMPLE GAP: stopped, recalibrating");}
  if(!running){running=true;entered=now;if(debug)Serial.println("# MOUSE CALIBRATE_BEGIN: release buttons and hold neutral still 1s");}
  float dt=haveTime?(now-previous)*0.001f:0;
  previous=now;haveTime=true;
  Vec measured=scale(a,1/magnitude);
  if(!calibrated){
    bool still=!blocked && norm(w)*Degrees<15 && magnitude>8.826f && magnitude<10.788f;
    if(still && quiet && dot(anchor,measured)<0.998629535f) still=false;
    if(!still){quiet=false;count=0;accSum=gyroSum={0,0,0};}
    else {
      if(!quiet){quiet=true;quietAt=now;anchor=measured;}
      accSum=add(accSum,a);gyroSum=add(gyroSum,w);++count;
      if(now-quietAt>=1000 && count>=40){
        neutral=gravity=unit(accSum);bias=scale(gyroSum,1.0f/count);
        Vec reference{1,0,0};
        right=add(reference,scale(neutral,-dot(reference,neutral)));
        if(norm(right)<0.2f){reference={0,1,0};right=add(reference,scale(neutral,-dot(reference,neutral)));}
        right=unit(right);down=unit(cross(neutral,right));
        calibrated=true;remainderX=remainderY=0;
        if(debug)Serial.println("# MOUSE READY: tilt to move, return to neutral to stop");
      }
    }
    if(now-entered>=5000 && !calibrated && !timedOut){
      timedOut=true;if(debug)Serial.println("# MOUSE CALIBRATION WAIT: no movement output; hold still or n to retry");
    }
    return result;
  }
  // Integrate gravity in body coordinates, then correct with acceleration near 1g.
  w=add(w,scale(bias,-1));
  Vec predicted=add(gravity,scale(cross(w,gravity),-dt));
  if(norm(predicted)<0.1f){reset();return result;}
  gravity=unit(predicted);
  if(magnitude>8.335f && magnitude<11.278f){
    float weight=1-std::exp(-dt/0.15f);
    Vec fused=add(scale(gravity,1-weight),scale(measured,weight));
    if(norm(fused)>0.1f) gravity=unit(fused);
  }
  float denominator=dot(gravity,neutral);
  float angleX=std::atan2(dot(gravity,right),denominator)*Degrees;
  float angleY=std::atan2(dot(gravity,down),denominator)*Degrees;
  // Turning over is outside the velocity-control posture; require deliberate recalibration.
  if(denominator<0.5f){
    if(debug)Serial.println("# MOUSE POSTURE OUT OF RANGE: stopped, recalibrating");
    reset();return result;
  }
  float vx=speed(angleX,movingX),vy=speed(angleY,movingY);
  if(blocked){remainderX=remainderY=0;vx=vy=0;}
  else {result.x=counts(vx,dt,remainderX);result.y=counts(vy,dt,remainderY);}
  if(debug && now-printed>=250){
    printed=now;
    Serial.printf("# MOUSE tilt=[%.1f %.1f]deg speed=[%.0f %.0f]counts/s delta=[%d %d] blocked=%u\n",
      angleX,angleY,vx,vy,int(result.x),int(result.y),unsigned(blocked));
  }
  return result;
}
}
