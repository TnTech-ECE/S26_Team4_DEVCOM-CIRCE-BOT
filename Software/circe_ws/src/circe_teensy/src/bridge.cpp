#include "bridge.h"

#include <Arduino.h>

#include "protocol.h"
#include "status_led.h"

namespace {

// COBS adds at most one byte per 254, plus the 0x00 delimiter.
constexpr size_t MAX_ENCODED = MAX_FRAME + MAX_FRAME / 254 + 2;

BridgeHandler g_handler = nullptr;
BridgeStats g_stats = {0, 0, 0};
uint8_t g_tx_seq = 0;

uint8_t g_rx_buf[MAX_ENCODED];
size_t g_rx_len = 0;
bool g_rx_overflow = false;  // discard until the next 0x00

uint16_t crc16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int b = 0; b < 8; b++) {
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
  }
  return crc;
}

// Returns the encoded length. `out` must hold len + len/254 + 1 bytes.
size_t cobs_encode(const uint8_t *in, size_t len, uint8_t *out) {
  size_t code_idx = 0;
  size_t o = 1;
  uint8_t code = 1;
  for (size_t i = 0; i < len; i++) {
    if (in[i] == 0) {
      out[code_idx] = code;
      code_idx = o++;
      code = 1;
    } else {
      out[o++] = in[i];
      if (++code == 0xFF) {
        out[code_idx] = code;
        code_idx = o++;
        code = 1;
      }
    }
  }
  out[code_idx] = code;
  return o;
}

// Returns the decoded length, or 0 on malformed input or if it would exceed out_cap.
size_t cobs_decode(const uint8_t *in, size_t len, uint8_t *out, size_t out_cap) {
  size_t i = 0;
  size_t o = 0;
  while (i < len) {
    uint8_t code = in[i++];
    if (code == 0 || i + code - 1 > len) return 0;
    for (uint8_t k = 1; k < code; k++) {
      if (o >= out_cap) return 0;
      out[o++] = in[i++];
    }
    if (code < 0xFF && i < len) {
      if (o >= out_cap) return 0;
      out[o++] = 0;
    }
  }
  return o;
}

void handle_encoded(const uint8_t *enc, size_t len) {
  uint8_t frame[MAX_FRAME];
  size_t n = cobs_decode(enc, len, frame, sizeof(frame));
  if (n < FRAME_OVERHEAD) {
    g_stats.rx_bad++;
    return;
  }
  uint16_t got = frame[n - 2] | (static_cast<uint16_t>(frame[n - 1]) << 8);
  if (crc16(frame, n - 2) != got) {
    g_stats.rx_bad++;
    return;
  }
  g_stats.rx_ok++;
  led_pulse();
  // frame[1] is the Pi's seq. Gaps are not counted here; the Pi logs its own sends.
  if (g_handler) g_handler(frame[0], frame + 2, n - FRAME_OVERHEAD);
}

}  // namespace

void bridge_begin() {
  // USB CDC ignores the baud rate; the link runs at USB speed regardless.
  Serial.begin(115200);
}

void bridge_on_message(BridgeHandler handler) { g_handler = handler; }

void bridge_poll() {
  while (Serial.available() > 0) {
    uint8_t b = Serial.read();
    if (b == 0x00) {
      if (g_rx_overflow) {
        g_stats.rx_bad++;
      } else if (g_rx_len > 0) {
        handle_encoded(g_rx_buf, g_rx_len);
      }
      g_rx_len = 0;
      g_rx_overflow = false;
    } else if (g_rx_len < sizeof(g_rx_buf)) {
      g_rx_buf[g_rx_len++] = b;
    } else {
      g_rx_overflow = true;
    }
  }
}

bool bridge_send(uint8_t msg_id, const void *payload, size_t len) {
  if (len > MAX_PAYLOAD) return false;
  // dtr() is true only while a program on the Pi holds the port open.
  if (!Serial.dtr()) return false;

  uint8_t frame[MAX_FRAME];
  frame[0] = msg_id;
  frame[1] = g_tx_seq;
  memcpy(frame + 2, payload, len);
  uint16_t crc = crc16(frame, len + 2);
  frame[len + 2] = crc & 0xFF;
  frame[len + 3] = crc >> 8;

  uint8_t enc[MAX_ENCODED];
  size_t n = cobs_encode(frame, len + FRAME_OVERHEAD, enc);
  enc[n++] = 0x00;

  if (Serial.availableForWrite() < static_cast<int>(n)) {
    // Not stamped with a seq, so the Pi cannot see this loss; the heartbeat reports it.
    g_stats.tx_dropped++;
    return false;
  }
  Serial.write(enc, n);
  g_tx_seq++;
  led_pulse();
  return true;
}

BridgeStats bridge_stats() { return g_stats; }
