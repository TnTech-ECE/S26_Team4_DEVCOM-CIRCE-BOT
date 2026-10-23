#include "spool.h"

#include <Arduino.h>
#include <math.h>

#include "bridge.h"
#include "estop.h"
#include "heartbeat.h"
#include "protocol.h"

namespace {

// --- Drum geometry -----------------------------------------------------------------
// Measured from circe_description/meshes/spool_drum.stl (radial profile of the mesh).
constexpr float CORE_RADIUS_M = 0.0381f;  // 3 in core
constexpr float WIDTH_M = 0.154f;         // between the flanges; the "6 in" spool
// *** ASSUMED UNTIL MEASURED. *** Replace with the real cable and spool.
constexpr float CABLE_DIAMETER_M = 0.006f;  // typical Cat6 jacket, 5.5-6.5 mm
constexpr float FULL_LENGTH_M = 100.0f;     // cable on a freshly loaded drum
constexpr float MAX_SPOOL_RAD_S = 15.7f;    // gearmotor output, 150 rpm
// -----------------------------------------------------------------------------------

// Never pay out the last few metres: the cable's end is tied to the core, and the
// rover would tear it off.
constexpr float RESERVE_M = 3.0f;
// Pays out 3% faster than the rover. Too slow drags the drum and strains the cable
// (Cat6 is rated ~110 N); too fast drops slack behind a rover driving away from it.
// Winding in, too slow leaves slack under the reversing rover, so it leads there too.
constexpr float OVERSPEED = 1.03f;
// Below this the EKF's speed is noise around zero; the spool holds still.
constexpr float DEADBAND_MPS = 0.02f;

// The EKF publishes at tens of Hz; a gap this long means localization has stopped,
// and a spool still turning at the last speed would pay out or reel in blind.
constexpr uint32_t SPEED_TIMEOUT_MS = 300;
constexpr uint32_t STATUS_PERIOD_MS = 200;

bool g_enabled = false;
float g_speed = 0.0f;
uint32_t g_last_speed_ms = 0;
bool g_speed_seen = false;
uint32_t g_last_status_ms = 0;

// *** RESETS TO FULL ON EVERY BOOT. *** A Teensy reset mid-run loses the real figure,
// and every radius after it is wrong until the drum is reloaded.
float g_cable_on_m = FULL_LENGTH_M;

bool speed_fresh() { return g_speed_seen && millis() - g_last_speed_ms < SPEED_TIMEOUT_MS; }

// Outer radius of the wound pack holding `length` of cable, square packing:
// pi * W * (r^2 - r_core^2) = length * d^2.
float pack_radius(float length) {
  return sqrtf(CORE_RADIUS_M * CORE_RADIUS_M +
               length * CABLE_DIAMETER_M * CABLE_DIAMETER_M / (static_cast<float>(M_PI) * WIDTH_M));
}

}  // namespace

void spool_update() {
  // Latched off: a lost Pi or an e-stop must not leave the spool armed to resume by
  // itself once the cause clears. The Pi re-sends SPOOL_CMD to start again.
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
  // Cable leaves from the centre of the outer turn, half a diameter in from the pack.
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
  // Out of cable going forward, or a full drum going back: hold, and let the flags
  // tell the Pi to stop the rover.
  if (v > 0 && g_cable_on_m <= RESERVE_M) return 0.0f;
  if (v < 0 && g_cable_on_m >= FULL_LENGTH_M) return 0.0f;
  float w = v * OVERSPEED / spool_radius_m();
  // Saturated means the rover is outrunning the drum; SPOOL_SATURATED says so.
  if (w > MAX_SPOOL_RAD_S) w = MAX_SPOOL_RAD_S;
  if (w < -MAX_SPOOL_RAD_S) w = -MAX_SPOOL_RAD_S;
  return w;
}

void spool_add_rotation(float delta_rad) {
  g_cable_on_m -= spool_radius_m() * delta_rad;
  if (g_cable_on_m < 0.0f) g_cable_on_m = 0.0f;
  if (g_cable_on_m > FULL_LENGTH_M) g_cable_on_m = FULL_LENGTH_M;
}
