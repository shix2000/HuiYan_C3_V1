#pragma once
#include "Gesture.h"
namespace TiltMouse {
struct Delta { int8_t x=0,y=0; };
void reset();
Delta update(const Gesture::Sample& sample,uint32_t now,bool active,bool blocked,bool debug);
}
