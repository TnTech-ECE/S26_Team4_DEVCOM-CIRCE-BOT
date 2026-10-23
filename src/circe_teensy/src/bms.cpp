#include "bms.h"

#include <Arduino.h>

#include "bridge.h"
#include "protocol.h"

namespace {

constexpr uint32_t BMS_BAUD = 115200;
constexpr uint32_t POLL_MS = 1000;
constexpr uint32_t NO_DATA_MS = 3000;     // no valid response for this long -> BMS_NO_DATA
constexpr uint32_t SILENT_REPORT_MS = 1000;
constexpr uint32_t BYTE_GAP_MS = 100;     // a frame that stalls this long is abandoned
// 223 fixed bytes of data + 3 per cell, plus header and trailer, for 24 cells.
constexpr size_t MAX_RESPONSE = 400;

HardwareSerialIMXRT &g_port = Serial1;  // the Teensy 4 class: has addMemoryForRead()

// The whole response lands in ~25 ms at 115200; the core's 64-byte buffer would
// overflow if loop() is busy for more than ~5 ms of it.
uint8_t g_rx_extra[512];

// "Read all data" (command 0x06, source 0x02), checksum 0x0128.
const uint8_t READ_ALL[] = {0x4E, 0x57, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00, 0x06, 0x02, 0x00,
                            0x00, 0x00, 0x00, 0x00, 0x00, 0x68, 0x00, 0x00, 0x01, 0x28};

uint8_t g_buf[MAX_RESPONSE];
size_t g_pos = 0;
size_t g_frame_len = 0;  // total bytes, known once the length field is in
uint32_t g_last_byte_ms = 0;

uint32_t g_last_poll_ms = 0;
uint32_t g_last_ok_ms = 0;
uint32_t g_last_sent_ms = 0;

BmsStatus g_status = {};
BmsCells g_cells = {};

uint16_t be16(const uint8_t *p) { return (static_cast<uint16_t>(p[0]) << 8) | p[1]; }
uint32_t be32(const uint8_t *p) {
  return (static_cast<uint32_t>(be16(p)) << 16) | be16(p + 2);
}

// JK: 0..100 is degrees C, above 100 is negative (101 = -1, 140 = -40).
int8_t temperature(uint16_t raw) {
  return static_cast<int8_t>(raw > 100 ? 100 - static_cast<int>(raw) : raw);
}

// Payload length of a fixed-size field, or -1 for an ID we do not know, after which
// nothing that follows can be located. Lengths per esphome-jk-bms jk_bms.cpp.
int field_len(uint8_t id) {
  switch (id) {
    case 0x85: case 0x86: case 0x9D: case 0xA9: case 0xAB: case 0xAC: case 0xAE:
    case 0xAF: case 0xB1: case 0xB3: case 0xB8: case 0xC0:
      return 1;
    case 0x80: case 0x81: case 0x82: case 0x83: case 0x84: case 0x87: case 0x8A:
    case 0x8B: case 0x8C: case 0xAD: case 0xB0:
      return 2;
    case 0x89: case 0xAA: case 0xB5: case 0xB6: case 0xB9:
      return 4;
    case 0xB4: return 8;
    case 0xB2: return 10;
    case 0xB7: return 15;
    case 0xBA: return 24;
    default:
      // Protection settings, all two bytes.
      if ((id >= 0x8E && id <= 0x9C) || (id >= 0x9E && id <= 0xA8)) return 2;
      return -1;
  }
}

void report(uint32_t now) {
  if (g_status.status == BMS_OK) bridge_send(MSG_BMS_CELLS, &g_cells, sizeof(g_cells));
  bridge_send(MSG_BMS_STATUS, &g_status, sizeof(g_status));
  g_last_sent_ms = now;
}

// Walks the TLV data. Returns false if the fields every report needs are missing.
bool parse_data(const uint8_t *d, size_t n) {
  BmsStatus s = g_status;
  BmsCells cells = {};
  uint16_t raw_current = 0;
  uint8_t version = 0x01;  // newest layout, assumed if 0xC0 is never reached
  bool have_cells = false, have_voltage = false, have_current = false, have_soc = false;

  size_t i = 0;
  while (i < n) {
    uint8_t id = d[i];
    const uint8_t *v = d + i + 1;
    if (id == 0x79) {
      // Cell block: length byte, then (index:u8, mV:u16) per cell.
      if (i + 1 >= n) break;
      uint8_t len = d[i + 1];
      if (i + 2 + len > n) break;
      for (uint8_t k = 0; k + 2 < len && cells.count < BMS_MAX_CELLS; k += 3) {
        cells.cell_mv[cells.count++] = be16(d + i + 2 + k + 1);
      }
      have_cells = true;
      i += 2 + len;
      continue;
    }
    int len = field_len(id);
    if (len < 0 || i + 1 + len > n) break;
    switch (id) {
      case 0x80: s.temp_mos_c = temperature(be16(v)); break;
      case 0x81: s.temp_1_c = temperature(be16(v)); break;
      case 0x82: s.temp_2_c = temperature(be16(v)); break;
      case 0x83: s.voltage_cv = be16(v); have_voltage = true; break;
      case 0x84: raw_current = be16(v); have_current = true; break;
      case 0x85: s.soc_pct = v[0]; have_soc = true; break;
      case 0x87: s.cycles = be16(v); break;
      case 0x8A: s.cell_count = static_cast<uint8_t>(be16(v)); break;
      case 0x8B: s.alarms = be16(v); break;
      case 0x8C: s.mode = be16(v); break;
      case 0xAA: {
        uint32_t ah = be32(v);
        s.capacity_ah = static_cast<uint16_t>(ah > 0xFFFF ? 0xFFFF : ah);
        break;
      }
      case 0xAF: s.battery_type = v[0]; break;
      case 0xC0: version = v[0]; break;
      default: break;
    }
    i += 1 + len;
  }
  if (!have_cells || !have_voltage || !have_current || !have_soc) return false;

  // Protocol 0x01: bit 15 set = charging, the rest is magnitude in 0.01 A. Older
  // firmware encodes it otherwise; esphome-jk-bms reports 0 A for those. Compare the
  // sign against the JK app before trusting it.
  int32_t mag = raw_current & 0x7FFF;
  s.current_ca = static_cast<int16_t>((raw_current & 0x8000) ? mag : -mag);
  s.protocol_version = version;
  if (s.cell_count == 0) s.cell_count = cells.count;
  s.status = BMS_OK;
  g_status = s;
  g_cells = cells;
  return true;
}

void handle_frame(uint32_t now) {
  size_t len = g_frame_len - 2;  // the length field's value
  uint16_t sum = 0;
  for (size_t i = 0; i < len; i++) sum += g_buf[i];
  // Data runs from byte 11 to just before record(4) + 0x68 + checksum(4). esphome's
  // slice also takes the record number; walked as TLV, its bytes would read as IDs.
  if (sum != be16(g_buf + len) || g_buf[8] != 0x06 || len < 18 ||
      !parse_data(g_buf + 11, len - 18)) {
    g_status.bad_frames++;
    return;
  }
  g_last_ok_ms = now;
  report(now);
}

void feed(uint8_t b, uint32_t now) {
  if (g_pos > 0 && now - g_last_byte_ms > BYTE_GAP_MS) g_pos = 0;
  g_last_byte_ms = now;

  if (g_pos == 0 && b != 0x4E) return;  // hunting for the header
  if (g_pos == 1 && b != 0x57) {
    g_pos = (b == 0x4E) ? 1 : 0;  // 4E 4E 57: the second 4E may be the real start
    return;
  }
  g_buf[g_pos++] = b;
  if (g_pos == 4) {
    g_frame_len = be16(g_buf + 2) + 2;
    if (g_frame_len < 20 || g_frame_len > MAX_RESPONSE) {
      g_status.bad_frames++;
      g_pos = 0;
    }
    return;
  }
  if (g_pos > 4 && g_pos == g_frame_len) {
    handle_frame(now);
    g_pos = 0;
  }
}

}  // namespace

void bms_begin() {
  g_port.addMemoryForRead(g_rx_extra, sizeof(g_rx_extra));
  g_port.begin(BMS_BAUD);
  uint32_t now = millis();
  g_last_ok_ms = now;
  g_last_sent_ms = now;
  g_last_poll_ms = now - POLL_MS;  // first poll on the first update
  g_status.status = BMS_NO_DATA;
}

void bms_update() {
  uint32_t now = millis();
  while (g_port.available() > 0) feed(g_port.read(), now);

  if (now - g_last_poll_ms >= POLL_MS) {
    g_last_poll_ms = now;
    // 21 bytes fit the core's TX buffer, so this returns at once.
    g_port.write(READ_ALL, sizeof(READ_ALL));
  }

  // A dead or unwired BMS sends nothing, so silence is reported explicitly.
  if (now - g_last_ok_ms >= NO_DATA_MS && now - g_last_sent_ms >= SILENT_REPORT_MS) {
    g_status.status = BMS_NO_DATA;
    report(now);
  }
}
