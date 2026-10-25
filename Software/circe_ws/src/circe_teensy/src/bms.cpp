#include "bms.h"

#include <Arduino.h>

#include "bridge.h"
#include "protocol.h"

namespace {

constexpr uint32_t BMS_BAUD = 115200;
constexpr uint32_t POLL_MS = 1000;
constexpr uint32_t NO_DATA_MS = 3000;
constexpr uint32_t SILENT_REPORT_MS = 1000;
constexpr uint32_t BYTE_GAP_MS = 100;     // abandon a stalled frame
constexpr size_t MAX_RESPONSE = 400;
constexpr size_t RX_EXTRA = 512;

// "Read all data" request (cmd 0x06, source 0x03).
const uint8_t READ_ALL[] = {0x4E, 0x57, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00, 0x06, 0x03, 0x00,
                            0x00, 0x00, 0x00, 0x00, 0x00, 0x68, 0x00, 0x00, 0x01, 0x29};

struct Bms {
  HardwareSerialIMXRT *port;
  uint8_t id;

  uint8_t rx_extra[RX_EXTRA];

  uint8_t buf[MAX_RESPONSE];
  size_t pos;
  size_t frame_len;
  uint32_t last_byte_ms;

  uint32_t last_poll_ms;
  uint32_t last_ok_ms;
  uint32_t last_sent_ms;
  BmsStatus status;
  BmsCells cells;
};

// bms 0: Serial1 (RX 0, TX 1); bms 1: Serial2 (RX 7, TX 8). id = index into battery_topics.
Bms g_bms[] = {
  {&Serial1, 0},
  {&Serial2, 1},
};

uint16_t be16(const uint8_t *p) { return (static_cast<uint16_t>(p[0]) << 8) | p[1]; }
uint32_t be32(const uint8_t *p) {
  return (static_cast<uint32_t>(be16(p)) << 16) | be16(p + 2);
}

// JK temperature: >100 means negative.
int8_t temperature(uint16_t raw) {
  return static_cast<int8_t>(raw > 100 ? 100 - static_cast<int>(raw) : raw);
}

// Fixed field lengths per esphome-jk-bms; -1 = unknown ID.
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
      if ((id >= 0x8E && id <= 0x9C) || (id >= 0x9E && id <= 0xA8)) return 2;
      return -1;
  }
}

void report(Bms &b, uint32_t now) {
  if (b.status.status == BMS_OK) bridge_send(MSG_BMS_CELLS, &b.cells, sizeof(b.cells));
  bridge_send(MSG_BMS_STATUS, &b.status, sizeof(b.status));
  b.last_sent_ms = now;
}

// Walks the TLV data; false if required fields are missing.
bool parse_data(Bms &b, const uint8_t *d, size_t n) {
  BmsStatus s = b.status;
  BmsCells cells = {};
  cells.bms_id = b.id;
  uint16_t raw_current = 0;
  uint8_t version = 0x01;
  bool have_cells = false, have_voltage = false, have_current = false, have_soc = false;

  size_t i = 0;
  while (i < n) {
    uint8_t id = d[i];
    const uint8_t *v = d + i + 1;
    if (id == 0x79) {
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

  // Bit 15 = charging, rest = 0.01 A (protocol 0x01).
  int32_t mag = raw_current & 0x7FFF;
  s.current_ca = static_cast<int16_t>((raw_current & 0x8000) ? mag : -mag);
  s.protocol_version = version;
  if (s.cell_count == 0) s.cell_count = cells.count;
  s.status = BMS_OK;
  b.status = s;
  b.cells = cells;
  return true;
}

void handle_frame(Bms &b, uint32_t now) {
  size_t len = b.frame_len - 2;
  uint16_t sum = 0;
  for (size_t i = 0; i < len; i++) sum += b.buf[i];
  // Data is bytes 11 .. before record(4) + 0x68 + checksum(4).
  if (sum != be16(b.buf + len) || b.buf[8] != 0x06 || len < 18 ||
      !parse_data(b, b.buf + 11, len - 18)) {
    b.status.bad_frames++;
    return;
  }
  b.last_ok_ms = now;
  report(b, now);
}

void feed(Bms &b, uint8_t c, uint32_t now) {
  if (b.pos > 0 && now - b.last_byte_ms > BYTE_GAP_MS) b.pos = 0;
  b.last_byte_ms = now;

  if (b.pos == 0 && c != 0x4E) return;
  if (b.pos == 1 && c != 0x57) {
    b.pos = (c == 0x4E) ? 1 : 0;
    return;
  }
  b.buf[b.pos++] = c;
  if (b.pos == 4) {
    b.frame_len = be16(b.buf + 2) + 2;
    if (b.frame_len < 20 || b.frame_len > MAX_RESPONSE) {
      b.status.bad_frames++;
      b.pos = 0;
    }
    return;
  }
  if (b.pos > 4 && b.pos == b.frame_len) {
    handle_frame(b, now);
    b.pos = 0;
  }
}

}  // namespace

void bms_begin() {
  uint32_t now = millis();
  for (Bms &b : g_bms) {
    b.port->addMemoryForRead(b.rx_extra, sizeof(b.rx_extra));
    b.port->begin(BMS_BAUD);
    b.last_ok_ms = now;
    b.last_sent_ms = now;
    b.last_poll_ms = now - POLL_MS;
    b.status.bms_id = b.id;
    b.status.status = BMS_NO_DATA;
  }
}

void bms_update() {
  uint32_t now = millis();
  for (Bms &b : g_bms) {
    while (b.port->available() > 0) feed(b, b.port->read(), now);

    if (now - b.last_poll_ms >= POLL_MS) {
      b.last_poll_ms = now;
      b.port->write(READ_ALL, sizeof(READ_ALL));
    }

    if (now - b.last_ok_ms >= NO_DATA_MS && now - b.last_sent_ms >= SILENT_REPORT_MS) {
      b.status.status = BMS_NO_DATA;
      report(b, now);
    }
  }
}
