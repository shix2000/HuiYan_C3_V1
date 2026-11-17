#include "PairButton.h"
// Uncomment to enable raw levels and button diagnostics.
// #define HUIYAN_PAIR_BUTTON_DEBUG
namespace PairButton {
namespace {
bool rawLast=HIGH, stable=HIGH, armed=false, fired=false;
uint32_t changedAt=0, downAt=0, lastStatus=0;
constexpr uint32_t DebounceMs=25, HoldMs=2000;
}
void begin() {
  pinMode(Pin, INPUT_PULLUP);
  rawLast=stable=digitalRead(Pin);
  changedAt=millis(); lastStatus=changedAt; armed=false; fired=false;
#ifdef HUIYAN_PAIR_BUTTON_DEBUG
  Serial.printf("# PAIR_BTN INIT GPIO%u raw=%u (released=1, pressed=0)\n", Pin, unsigned(rawLast));
#endif
}
bool poll(uint32_t now) {
  bool raw=digitalRead(Pin);
  if(raw!=rawLast){
    rawLast=raw;changedAt=now;
#ifdef HUIYAN_PAIR_BUTTON_DEBUG
    Serial.printf("# PAIR_BTN RAW GPIO%u=%u t=%lu\n",Pin,unsigned(raw),(unsigned long)now);
#endif
  }
#ifdef HUIYAN_PAIR_BUTTON_DEBUG
  // Periodic snapshot also makes an unchanged/wrongly wired pin visible.
  if(now-lastStatus>=1000){
    lastStatus=now;
    Serial.printf("# PAIR_BTN STATUS raw=%u stable=%u armed=%u\n",
                  unsigned(raw),unsigned(stable),unsigned(armed));
  }
#endif
  if(now-changedAt<DebounceMs) return false;
  if(raw!=stable){
    stable=raw;
#ifdef HUIYAN_PAIR_BUTTON_DEBUG
    Serial.println(stable==LOW?"# PAIR_BTN DOWN":"# PAIR_BTN UP");
#endif
    if(stable==LOW){downAt=now;fired=false;}
  }
  if(stable==HIGH){armed=true;fired=false;return false;}
  if(armed && !fired && now-downAt>=HoldMs){
    fired=true;
#ifdef HUIYAN_PAIR_BUTTON_DEBUG
    Serial.println("# PAIR_BTN HOLD 2000ms: pairing requested");
#endif
    return true;
  }
  return false;
}
}
