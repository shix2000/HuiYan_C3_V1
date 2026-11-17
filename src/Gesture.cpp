#include "Gesture.h"
#include <cmath>
#include <Arduino.h>
namespace Gesture {
namespace {
enum class State { Calibrate, Arming, Ready, Candidate, Lockout };
State state=State::Calibrate;
uint32_t previous=0, quietAt=0, started=0, lastOutput=0;
bool haveTime=false, quiet=false;
constexpr uint32_t EndLowSpeedMs=60, CandidateWindowMs=500;
// Natural-wrist experimental profile, replacing PRD's initial 70deg/10deg proposal.
// Current logs stop at the old cap: 150deg requires full-motion and negative-sample validation.
constexpr float MaxRotationDeg=150.0f, NeutralToleranceDeg=25.0f;
constexpr float NeutralCos=0.906307787f; // cos(25deg), shared by control and diagnostics.
static_assert(MaxRotationDeg>20 && MaxRotationDeg<180, "Bounded rotation window required");
static_assert(NeutralToleranceDeg==25, "Update NeutralCos when changing tolerance");
uint32_t lowSpeedAt=0;
bool lowSpeed=false;
uint32_t phaseStarted=0, returnStarted=0;
bool confirmingReturn=false;
uint32_t neutralLostAt=0;
bool neutralLossPending=false;
uint32_t recoverySince=0, recoveryLastHint=0;
bool recoveryWaiting=false;
bool debugOutput=true;
bool autoRestoreEnabled=true, autoSampling=false;
uint32_t autoStarted=0;
float autoX=0,autoY=0,autoZ=0,autoPath=0;
float autoAx=0,autoAy=0,autoAz=0,autoGx=0,autoGy=0,autoGz=0;
unsigned autoCount=0;
constexpr uint32_t AutoWaitMs=5000, AutoStableMs=2000;
constexpr float AutoPoseCos=0.998629535f; // 3deg pose drift during the stable window.
void cancelAuto(const char* why){
  if(autoSampling && debugOutput) Serial.printf("# GST AUTO_CALIBRATE_CANCEL reason=%s\n",why);
  autoSampling=false;autoStarted=0;autoCount=0;autoPath=0;
}
constexpr uint32_t NeutralLossConfirmMs=300;
float nx=0,ny=0,nz=1,bx=0,by=0,bz=0;
float sx=0,sy=0,sz=0,ux=0,uy=0,uz=0;
unsigned count=0;
float angle=0,total=0,aligned=0,axisEnergy=0,allEnergy=0;
int direction=0;

const char* reason="initial";
const char* stateName(State s){
  switch(s){case State::Calibrate:return "CALIBRATE";case State::Arming:return "ARMING";
    case State::Ready:return "READY";case State::Candidate:return "CANDIDATE";default:return "LOCKOUT";}
}
struct Trace {
  const Sample& s; uint32_t now; State before;
  ~Trace(){
    static uint32_t printed=0;
    if(!debugOutput || (state==before && now-printed<200)) return;
    printed=now;
    float norm=std::sqrt(s.ax*s.ax+s.ay*s.ay+s.az*s.az);
    float dot=norm>0?(s.ax*nx+s.ay*ny+s.az*nz)/norm:0;
    float neutralAngle=std::acos(fmaxf(-1.0f,fminf(1.0f,dot)))*57.29578f;
    Serial.printf("# GST t=%lu %s->%s reason=%s a=[%.2f %.2f %.2f]m/s2 g=[%.1f %.1f %.1f]deg/s quiet_ms=%lu n=%u bias=[%.1f %.1f %.1f] angle=%.1f consistency=%.2f axis=%.2f neutral=%u neutral_deg=%.1f ref=[%.2f %.2f %.2f] candidate_ms=%lu low_ms=%lu neutral_loss_ms=%lu max_rotation_deg=%.0f neutral_limit_deg=%.0f direction=%d return_ms=%lu auto_restore=%u auto_stable_ms=%lu\n",
      (unsigned long)now,stateName(before),stateName(state),reason,s.ax,s.ay,s.az,
      s.gx*57.29578f,s.gy*57.29578f,s.gz*57.29578f,
      (unsigned long)(quiet?now-quietAt:0),count,bx,by,bz,angle,total>0?aligned/total:0,
      allEnergy>0?axisEnergy/allEnergy:0,unsigned(dot>=NeutralCos),neutralAngle,nx,ny,nz,
      (unsigned long)((before==State::Candidate || state==State::Candidate)?now-started:0),
      (unsigned long)(lowSpeed?now-lowSpeedAt:0),
      (unsigned long)(neutralLossPending?now-neutralLostAt:0),MaxRotationDeg,NeutralToleranceDeg,direction,
      (unsigned long)(confirmingReturn && (before==State::Candidate || state==State::Candidate)?now-returnStarted:0),
      unsigned(autoRestoreEnabled),(unsigned long)(autoSampling?now-autoStarted:0));
  }
};
float length(float x,float y,float z){return std::sqrt(x*x+y*y+z*z);}
void restartQuiet(){quiet=false;quietAt=0;}
}
void setDebug(bool enabled){debugOutput=enabled;}
bool autoRestore(){return autoRestoreEnabled;}
void setAutoRestore(bool enabled){
  cancelAuto("setting_changed");autoRestoreEnabled=enabled;
  recoveryWaiting=false;recoverySince=recoveryLastHint=0;
}
void reset(){cancelAuto("reset_or_output_gate");confirmingReturn=false;returnStarted=phaseStarted=0;recoveryWaiting=false;recoverySince=recoveryLastHint=0;neutralLossPending=false;neutralLostAt=0;lowSpeed=false;lowSpeedAt=0;reason="reset_hold_current_neutral_1s";angle=total=aligned=axisEnergy=allEnergy=0;direction=0;state=State::Calibrate;haveTime=false;restartQuiet();count=0;sx=sy=sz=ux=uy=uz=0;}
int update(const Sample& s,uint32_t now,bool enabled){
  Trace trace{s,now,state};
  if(!enabled){reason="output_gate_closed";reset();return 0;}
  float a=length(s.ax,s.ay,s.az);
  if(!std::isfinite(a) || !std::isfinite(s.gx) || !std::isfinite(s.gy) || !std::isfinite(s.gz) ||
     a<0.1f || std::fabs(s.gx)>34 || std::fabs(s.gy)>34 || std::fabs(s.gz)>34){reason="invalid_or_saturated_sample";reset();return 0;}
  if(haveTime && now-previous>100){reason="sample_gap_over_100ms";reset();}
  float dt=haveTime?(now-previous)*0.001f:0;
  previous=now;haveTime=true;
  float x=s.gx*57.29578f-bx,y=s.gy*57.29578f-by,z=s.gz*57.29578f-bz;
  bool still=length(x,y,z)<15 && a>8.826f && a<10.788f;
  if(state==State::Calibrate){
    // Initial bias estimate uses raw rates; never retain a previous mounting.
    still=length(s.gx,s.gy,s.gz)*57.29578f<15 && a>8.826f && a<10.788f;
    reason=still?"calibrating_hold_still_1s":"calibration_not_still";
    if(!still){restartQuiet();count=0;sx=sy=sz=ux=uy=uz=0;return 0;}
    if(!quiet){quiet=true;quietAt=now;}
    sx+=s.gx*57.29578f;sy+=s.gy*57.29578f;sz+=s.gz*57.29578f;
    ux+=s.ax;uy+=s.ay;uz+=s.az;++count;
    if(now-quietAt>=1000 && count>=40){
      bx=sx/count;by=sy/count;bz=sz/count;
      float n=length(ux,uy,uz);nx=ux/n;ny=uy/n;nz=uz/n;
      reason="calibration_complete";state=State::Arming;restartQuiet();
    }
    return 0;
  }
  float dot=(s.ax*nx+s.ay*ny+s.az*nz)/a;
  bool neutral=dot>=NeutralCos; // Reference changes only after manual or gated automatic calibration.
  if(state==State::Arming || state==State::Lockout){
    reason=!still?"waiting_still":!neutral?"waiting_return_neutral":"waiting_quiet_or_cooldown";
    // Rejected candidates return to ARMING, so recovery guidance must cover both states.
    // Only count continuous stationary time outside neutral; motion/return resets it.
    if(still && !neutral){
      if(!recoveryWaiting){recoveryWaiting=true;recoverySince=recoveryLastHint=now;}
      if(debugOutput && now-recoverySince>=5000 && now-recoveryLastHint>=5000){
        recoveryLastHint=now;
        float degrees=std::acos(fmaxf(-1.0f,fminf(1.0f,dot)))*57.29578f;
        Serial.printf("# GST RECOVERY state=%s neutral_deg=%.1f limit=%.0f: return to calibrated pose OR send n and hold desired neutral still until READY. Automatic restore requires enabled setting and stable pose.\n",stateName(state),degrees,NeutralToleranceDeg);
      }
    } else {recoveryWaiting=false;recoverySince=recoveryLastHint=0;}
    // Existing recovery timer is continuous stationary time outside the old neutral.
    // Once 5s elapses, collect 2s more; never output HID from this branch.
    if(autoRestoreEnabled && recoveryWaiting && now-recoverySince>=AutoWaitMs){
      float speed=length(x,y,z);
      if(!autoSampling){
        if(speed<8){
          autoSampling=true;autoStarted=now;autoCount=0;autoPath=0;
          autoX=s.ax/a;autoY=s.ay/a;autoZ=s.az/a;
          autoAx=autoAy=autoAz=autoGx=autoGy=autoGz=0;
          if(debugOutput) Serial.println("# GST AUTO_CALIBRATE_BEGIN: hold pose stable 2s; movement cancels");
        }
      }
      if(autoSampling){
        float poseDot=(s.ax*autoX+s.ay*autoY+s.az*autoZ)/a;
        autoPath+=speed*dt;
        if(!still || neutral || speed>=8 || poseDot<AutoPoseCos || autoPath>8){
          cancelAuto("movement_or_return_to_old_neutral");
          recoveryWaiting=false;recoverySince=recoveryLastHint=0;
          reason="auto_calibration_cancelled";return 0;
        }
        autoAx+=s.ax;autoAy+=s.ay;autoAz+=s.az;
        autoGx+=s.gx*57.29578f;autoGy+=s.gy*57.29578f;autoGz+=s.gz*57.29578f;++autoCount;
        reason="auto_calibrating_stable_2s";
        if(now-autoStarted>=AutoStableMs && autoCount>=80){
          // Apply the stable-window averages atomically within this loop tick.
          float norm=length(autoAx,autoAy,autoAz);
          float newNx=autoAx/norm,newNy=autoAy/norm,newNz=autoAz/norm;
          float newBx=autoGx/autoCount,newBy=autoGy/autoCount,newBz=autoGz/autoCount;
          unsigned samples=autoCount;
          autoSampling=false; // Completion is not a cancellation.
          reset();nx=newNx;ny=newNy;nz=newNz;bx=newBx;by=newBy;bz=newBz;
          count=samples;state=State::Arming;reason="auto_calibration_complete";
          // Still require the ordinary 300ms stationary arming period; no queued event.
          if(debugOutput) Serial.println("# GST AUTO_CALIBRATE_COMPLETE: new neutral saved for this session; wait for READY");
        }
        return 0;
      }
    } else if(autoSampling){
      cancelAuto("movement_or_return_or_disabled");
      recoveryWaiting=false;recoverySince=recoveryLastHint=0;
    }
    if(still && neutral){
      if(!quiet){quiet=true;quietAt=now;}
      if(now-quietAt>=300 && (state!=State::Lockout || now-lastOutput>=600)){
        state=State::Ready;recoveryWaiting=false;recoverySince=recoveryLastHint=0;neutralLossPending=false;neutralLostAt=0;reason="ready_neutral_confirmed";
        // Do not display metrics from a previous rejected gesture as a new candidate.
        angle=total=aligned=axisEnergy=allEnergy=0;
        direction=0;confirmingReturn=false;returnStarted=phaseStarted=0;lowSpeed=false;lowSpeedAt=0;restartQuiet();
      }
    } else restartQuiet();
    return 0;
  }
  if(state==State::Ready){
    reason="ready_wait_GY_120deg_s";
    if(std::fabs(y)<120){
      // Gravity is trustworthy only when stationary. Do not reject a fast onset
      // using acceleration contaminated by the gesture itself.
      if(still && !neutral){
        if(!neutralLossPending){neutralLossPending=true;neutralLostAt=now;}
        reason="ready_neutral_loss_pending_300ms";
        if(now-neutralLostAt>=NeutralLossConfirmMs){
          neutralLossPending=false;neutralLostAt=0;
          state=State::Arming;restartQuiet();
          reason="ready_lost_neutral_return_or_send_n";
        }
      } else {
        // Returning inside the neutral tolerance or starting to move breaks continuous stationary loss.
        neutralLossPending=false;neutralLostAt=0;
      }
      return 0;
    }
    neutralLossPending=false;neutralLostAt=0;
    direction=y>0?1:-1;started=phaseStarted=now;confirmingReturn=false;returnStarted=0;lowSpeed=false;lowSpeedAt=0;
    angle=total=aligned=axisEnergy=allEnergy=0;
    reason="rotation_started";state=State::Candidate;
  }
  if(state==State::Candidate){
    float d=y*direction;
    // Check deadline even while observing a preliminary twitch or the return stroke.
    if(now-started>CandidateWindowMs){
      reason="candidate_timeout";state=State::Arming;
      lowSpeed=false;confirmingReturn=false;restartQuiet();return 0;
    }
    if(confirmingReturn){
      // Freeze the qualified outbound metrics; return movement cannot become a second event.
      if(d>=-40){
        reason="return_confirmation_interrupted";state=State::Arming;
        confirmingReturn=false;lowSpeed=false;restartQuiet();return 0;
      }
      if(now-returnStarted<EndLowSpeedMs){reason="waiting_return_60ms";return 0;}
      reason="ACCEPT";state=State::Lockout;lastOutput=now;
      lowSpeed=false;restartQuiet();return direction;
    }
    if(d< -40){
      lowSpeed=false;lowSpeedAt=0;
      if(angle<20){
        // An under-sized precursor does not own direction. A new opposite burst
        // must meet the original 120deg/s start threshold; deadline never restarts.
        if(std::fabs(y)<120){reason="small_motion_wait_opposite_start";return 0;}
        direction=-direction;d=-d;phaseStarted=now;
        angle=total=aligned=axisEnergy=allEnergy=0;
        reason="small_motion_direction_reselected";
      } else {
        bool qualified=now-phaseStarted>=80 && angle<=MaxRotationDeg && total>0 &&
          aligned/total>=0.8f && allEnergy>0 && axisEnergy/allEnergy>=0.65f;
        if(!qualified){reason="reversal_before_valid_outbound";state=State::Arming;restartQuiet();return 0;}
        confirmingReturn=true;returnStarted=now;reason="waiting_return_60ms";return 0;
      }
    }
    angle+=d*dt;total+=std::fabs(y)*dt;if(d>0) aligned+=d*dt;
    axisEnergy+=y*y*dt;allEnergy+=(x*x+y*y+z*z)*dt;
    if(angle>MaxRotationDeg){
      reason="angle_over_limit";state=State::Arming;lowSpeed=false;restartQuiet();return 0;
    }
    if(std::fabs(y)>=40){
      lowSpeed=false;lowSpeedAt=0;
      // Preserve the direction-reselection diagnostic on its first sample.
      if(now!=phaseStarted) reason="candidate_rotating";
      return 0;
    }
    if(!lowSpeed){lowSpeed=true;lowSpeedAt=now;}
    if(now-lowSpeedAt<EndLowSpeedMs){reason="waiting_low_speed_60ms";return 0;}
    if(angle<20){reason="small_motion_wait_within_500ms";return 0;}
    bool valid=now-phaseStarted>=80 && angle<=MaxRotationDeg && total>0 &&
      aligned/total>=0.8f && allEnergy>0 && axisEnergy/allEnergy>=0.65f;
    reason=valid?"ACCEPT":now-phaseStarted<80?"duration_under_80ms":
      aligned/total<0.8f?"direction_inconsistent":"axis_energy_under_65pct";
    state=State::Lockout;lastOutput=now;lowSpeed=false;restartQuiet();
    return valid?direction:0;
  }
  return 0;
}
}
