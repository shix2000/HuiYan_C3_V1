#pragma once
#include <Arduino.h>
// Active-low momentary switch between GPIO6 and GND; internal pull-up.
namespace PairButton {
constexpr uint8_t Pin = 6;
void begin();
// One event after a debounced 2-second hold; release required before reuse.
bool poll(uint32_t now);
}
