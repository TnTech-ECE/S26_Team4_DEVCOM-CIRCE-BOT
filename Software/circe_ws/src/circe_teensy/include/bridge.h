// Non-blocking USB serial link to the Pi (frames per protocol.h).
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

typedef void (*BridgeHandler)(uint8_t msg_id, const uint8_t *payload, size_t len);

struct BridgeStats {
  uint32_t rx_ok;
  uint16_t rx_bad;
  uint16_t tx_dropped;  // Pi's buffer full
};

void bridge_begin();
void bridge_on_message(BridgeHandler handler);
void bridge_poll();
bool bridge_send(uint8_t msg_id, const void *payload, size_t len);
BridgeStats bridge_stats();

// Copies the payload only if its size matches T.
template <typename T>
bool bridge_unpack(const uint8_t *payload, size_t len, T &out) {
  if (len != sizeof(T)) return false;
  memcpy(&out, payload, sizeof(T));
  return true;
}
