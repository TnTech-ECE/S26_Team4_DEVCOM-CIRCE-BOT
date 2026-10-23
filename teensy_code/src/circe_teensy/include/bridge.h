// USB serial link to the Pi 5. Frames per include/protocol.h.
//
//   setup(): bridge_begin(); bridge_on_message(handler);
//   loop():  bridge_poll();
//
// Never blocks: with the Pi's port closed, or its buffer full, bridge_send() returns
// false instead of waiting, so a missing Pi cannot stall the sensor loop.
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

typedef void (*BridgeHandler)(uint8_t msg_id, const uint8_t *payload, size_t len);

struct BridgeStats {
  uint32_t rx_ok;       // frames that decoded and passed the CRC
  uint16_t rx_bad;      // frames dropped: bad COBS, bad CRC, too long or too short
  uint16_t tx_dropped;  // sends refused because the Pi held the port but its buffer was full
};

void bridge_begin();
void bridge_on_message(BridgeHandler handler);
void bridge_poll();
bool bridge_send(uint8_t msg_id, const void *payload, size_t len);
BridgeStats bridge_stats();

// Copies the payload only if its length matches, so a mismatched protocol version
// is ignored rather than read as garbage.
template <typename T>
bool bridge_unpack(const uint8_t *payload, size_t len, T &out) {
  if (len != sizeof(T)) return false;
  memcpy(&out, payload, sizeof(T));
  return true;
}
