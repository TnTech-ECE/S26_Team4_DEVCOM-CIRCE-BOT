#include "status_led.h"

#include <Arduino.h>

namespace {

constexpr uint32_t PULSE_MS = 30;
uint32_t g_on_since = 0;
bool g_on = false;

}  // namespace

void led_begin() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWriteFast(LED_BUILTIN, LOW);
}

void led_pulse() {
  digitalWriteFast(LED_BUILTIN, HIGH);
  g_on_since = millis();
  g_on = true;
}

void led_update() {
  if (g_on && millis() - g_on_since >= PULSE_MS) {
    digitalWriteFast(LED_BUILTIN, LOW);
    g_on = false;
  }
}
