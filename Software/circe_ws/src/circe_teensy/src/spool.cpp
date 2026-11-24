#include "spool.h"

#include <Arduino.h>
#include <math.h>

#include "bridge.h"
#include "estop.h"
#include "heartbeat.h"
#include "protocol.h"

namespace {

// Drum geometry (from spool_drum.stl). Cable/motor values are ASSUMED until measured.
constexpr float CORE_RADIUS_M = 0.0381f;
constexpr float WIDTH_M = 0.154f;
constexpr float CABLE_DIAMETER_M = 0.006f;
constexpr float FULL_LENGTH_M = 100.0f;
constexpr float MAX_SPOOL_RAD_S = 15.7f;  // 150 rpm

constexpr float RESERVE_M = 3.0f;      // never pay out the tied-off end
constexpr float OVERSPEED = 1.03f;     // lead the rover slightly to avoid dragging the cable
constexpr float DEADBAND_MPS = 0.02f;
constexpr uint32_t SPEED_TIMEOUT_MS = 300;
constexpr uint32_t STATUS_PERIOD_MS = 200;

bool g_enabled = false;
float g_speed = 0.0f;
uint32_t g_last_speed_ms = 0;
bool g_speed_seen = false;
uint32_t g_last_status_ms = 0;

// Resets to full on every boot.
float g_cable_on_m = FULL_LENGTH_M;

bool speed_fresh() { return g_speed_seen && millis() - g_last_speed_ms < SPEED_TIMEOUT_MS; }

// Outer radius of a pack holding `length` of cable (square packing).
float pack_radius(float length) {
  return sqrtf(CORE_RADIUS_M * CORE_RADIUS_M +
               length * CABLE_DIAMETER_M * CABLE_DIAMETER_M / (static_cast<float>(M_PI) * WIDTH_M));
}

}  // namespace

void spool_update() {
  // Latched off; the Pi must re-send SPOOL_CMD.
  if (!heartbeat_pi_alive() || estop_active()) g_enabled = false;

  uint32_t now = millis();
  if (now - g_last_status_ms >= STATUS_PERIOD_MS) {
    g_last_status_ms = now;
    SpoolStatus m;
    m.cable_on_m = g_cable_on_m;
    m.radius_m = spool_radius_m();
    m.target_rad_s = spool_target_rad_s();
    m.flags = spool_flags();
    bridge_send(MSG_SPOOL_STATUS, &m, sizeof(m));
  }
}

void spool_handle(uint8_t msg_id, const uint8_t *payload, size_t len) {
  switch (msg_id) {
    case MSG_SPOOL_CMD: {
      SpoolCmd m;
      if (bridge_unpack(payload, len, m)) g_enabled = m.enable != 0;
      break;
    }
    case MSG_ODOM_SPEED: {
      OdomSpeed m;
      if (bridge_unpack(payload, len, m)) {
        g_speed = m.linear_x;
        g_last_speed_ms = millis();
        g_speed_seen = true;
      }
      break;
    }
    default:
      break;
  }
}

bool spool_enabled() { return g_enabled; }

float spool_speed() { return speed_fresh() ? g_speed : 0.0f; }

bool spool_may_run() {
  return g_enabled && heartbeat_pi_alive() && !estop_active() && speed_fresh();
}

float spool_cable_on_m() { return g_cable_on_m; }

float spool_radius_m() {
  float r = pack_radius(g_cable_on_m) - CABLE_DIAMETER_M / 2;
  float innermost = CORE_RADIUS_M + CABLE_DIAMETER_M / 2;
  return r > innermost ? r : innermost;
}

uint8_t spool_flags() {
  uint8_t f = 0;
  if (spool_may_run()) f |= SPOOL_RUN;
  if (g_cable_on_m <= RESERVE_M) f |= SPOOL_AT_RESERVE;
  if (g_cable_on_m >= FULL_LENGTH_M) f |= SPOOL_FULL;
  float v = spool_speed();
  if (fabsf(v) * OVERSPEED / spool_radius_m() > MAX_SPOOL_RAD_S) f |= SPOOL_SATURATED;
  return f;
}

float spool_target_rad_s() {
  if (!spool_may_run()) return 0.0f;
  float v = spool_speed();
  if (fabsf(v) < DEADBAND_MPS) return 0.0f;
  if (v > 0 && g_cable_on_m <= RESERVE_M) return 0.0f;
  if (v < 0 && g_cable_on_m >= FULL_LENGTH_M) return 0.0f;
  float w = v * OVERSPEED / spool_radius_m();
  if (w > MAX_SPOOL_RAD_S) w = MAX_SPOOL_RAD_S;
  if (w < -MAX_SPOOL_RAD_S) w = -MAX_SPOOL_RAD_S;
  return w;
}

void spool_add_rotation(float delta_rad) {
  g_cable_on_m -= spool_radius_m() * delta_rad;
  if (g_cable_on_m < 0.0f) g_cable_on_m = 0.0f;
  if (g_cable_on_m > FULL_LENGTH_M) g_cable_on_m = FULL_LENGTH_M;
}
