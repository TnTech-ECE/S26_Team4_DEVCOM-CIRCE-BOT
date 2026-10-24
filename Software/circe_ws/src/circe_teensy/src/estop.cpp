#include "estop.h"

#include <Arduino.h>

#include "bridge.h"
#include "protocol.h"

namespace {

constexpr uint8_t ESTOP_PIN = 2;
// A press must hold this long to count: rejects a spike on a long cable without
// adding a delay anyone would notice.
constexpr uint32_t PRESS_CONFIRM_MS = 5;
// Release only arms the reset, so it can afford to wait out contact bounce.
constexpr uint32_t RELEASE_CONFIRM_MS = 50;
constexpr uint32_t REPORT_ACTIVE_MS = 200;
constexpr uint32_t REPORT_CLEAR_MS = 1000;

bool g_raw = true;          // last raw read, true = open circuit = pressed
uint32_t g_raw_since = 0;
bool g_pressed = true;      // debounced
bool g_latched = true;
bool g_dirty = true;        // state changed and the Pi has not been told yet
uint32_t g_last_report_ms = 0;

bool read_pressed() { return digitalReadFast(ESTOP_PIN) == HIGH; }

}  // namespace

void estop_begin() {
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  delayMicroseconds(50);  // let the pull-up charge the cable before the first read
  g_raw = g_pressed = g_latched = read_pressed();
  g_raw_since = millis();
  g_dirty = true;
}

void estop_update() {
  uint32_t now = millis();
  bool raw = read_pressed();
  if (raw != g_raw) {
    g_raw = raw;
    g_raw_since = now;
  }
  uint32_t confirm = raw ? PRESS_CONFIRM_MS : RELEASE_CONFIRM_MS;
  if (raw != g_pressed && now - g_raw_since >= confirm) {
    g_pressed = raw;
    if (g_pressed) g_latched = true;
    g_dirty = true;
  }

  uint32_t period = g_latched ? REPORT_ACTIVE_MS : REPORT_CLEAR_MS;
  if (g_dirty || now - g_last_report_ms >= period) {
    EstopState m;
    m.active = g_latched ? 1 : 0;
    m.button_pressed = g_pressed ? 1 : 0;
    // Retried every loop until it leaves, so a full USB buffer cannot lose a press.
    if (bridge_send(MSG_ESTOP, &m, sizeof(m))) {
      g_dirty = false;
      g_last_report_ms = now;
    }
  }
}

void estop_handle(uint8_t msg_id, const uint8_t * /*payload*/, size_t len) {
  if (msg_id != MSG_ESTOP_RESET || len != 0) return;
  if (!g_pressed) g_latched = false;
  // Answered either way, so a refused reset shows on the Pi at once.
  g_dirty = true;
}

bool estop_active() { return g_latched; }
