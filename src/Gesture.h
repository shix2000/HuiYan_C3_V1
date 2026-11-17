#pragma once
#include <stdint.h>
namespace Gesture {
struct Sample { float ax,ay,az,gx,gy,gz; }; // SI: m/s2, rad/s
// Returns +1=next, -1=previous, 0=no event. Fixed mounting: positive GY=right.
int update(const Sample& s,uint32_t now,bool enabled);
void reset();
void setDebug(bool enabled);
void setAutoRestore(bool enabled);
bool autoRestore();
}
