#include "heartbeat.h"

#include <Arduino.h>

#include "bridge.h"
#include "estop.h"
#include "protocol.h"
#include "spool.h"

namespace {

constexpr uint32_t HEARTBEAT_PERIOD_MS = 1000;
// *** TIED TO heartbeat_hz IN config/teensy_bridge.yaml (5 Hz). *** Three missed beats
// and the spool stops. Lower that rate and this trips on a healthy link.
constexpr uint32_t PI_TIMEOUT_MS = 750;

uint32_t g_last_sent_ms = 0;
uint32_t g_counter = 0;

uint32_t g_last_pi_counter = 0;
bool g_pi_seen = false;  // nothing is alive until the first beat
uint32_t g_last_pi_ms = 0;

}  // namespace

void heartbeat_update() {
  uint32_t now = millis();
  if (now - g_last_sent_ms < HEARTBEAT_PERIOD_MS) return;
  g_last_sent_ms = now;

  BridgeStats stats = bridge_stats();
  TeensyHeartbeat hb;
  hb.counter = g_counter;
  hb.uptime_ms = now;
  hb.rx_ok = stats.rx_ok;
  hb.rx_bad = stats.rx_bad;
  hb.tx_dropped = stats.tx_dropped;
  hb.last_pi_counter = g_last_pi_counter;
  hb.spool_enabled = spool_enabled() ? 1 : 0;
  hb.last_speed = spool_speed();
  hb.flags = (heartbeat_pi_alive() ? HB_PI_ALIVE : 0) | (estop_active() ? HB_ESTOP : 0) |
             (spool_may_run() ? HB_SPOOL_RUN : 0);
  // Counts only heartbeats that actually left, so a gap in the Pi's log is a real loss.
  if (bridge_send(MSG_TEENSY_HEARTBEAT, &hb, sizeof(hb))) g_counter++;
}

void heartbeat_handle(uint8_t msg_id, const uint8_t *payload, size_t len) {
  if (msg_id != MSG_PI_HEARTBEAT) return;
  PiHeartbeat m;
  if (!bridge_unpack(payload, len, m)) return;
  g_last_pi_counter = m.counter;
  g_last_pi_ms = millis();
  g_pi_seen = true;
}

bool heartbeat_pi_alive() { return g_pi_seen && millis() - g_last_pi_ms < PI_TIMEOUT_MS; }
