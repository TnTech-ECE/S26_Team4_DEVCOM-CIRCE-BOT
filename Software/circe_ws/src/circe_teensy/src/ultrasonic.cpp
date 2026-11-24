#include "ultrasonic.h"

#include <Arduino.h>

#include "bridge.h"
#include "protocol.h"

namespace {

// The A02YYUW only supports 9600 8N1.
constexpr uint32_t ULTRASONIC_BAUD = 9600;
constexpr uint32_t TIMEOUT_MS = 500;
constexpr uint16_t MIN_MM = 30;
constexpr uint16_t MAX_MM = 4500;

struct Sensor {
  HardwareSerial *port;
  uint8_t id;

  uint8_t frame[4];
  uint8_t pos;
  uint16_t last_mm;
  uint16_t bad_frames;
  uint32_t last_ok_ms;
  uint32_t last_timeout_report_ms;
};

// sensor 0: Serial5 (RX 21, TX 20); sensor 1: Serial4 (RX 16, TX 17)
Sensor g_sensors[] = {
  {&Serial5, 0},
  {&Serial4, 1},
};

void report(Sensor &s, uint8_t status) {
  UltrasonicReading r;
  r.sensor_id = s.id;
  r.status = status;
  r.distance_mm = s.last_mm;
  r.bad_frames = s.bad_frames;
  bridge_send(MSG_ULTRASONIC, &r, sizeof(r));
}

void feed(Sensor &s, uint8_t b, uint32_t now) {
  if (s.pos == 0 && b != 0xFF) return;  // wait for header
  s.frame[s.pos++] = b;
  if (s.pos < 4) return;
  s.pos = 0;

  uint8_t sum = (s.frame[0] + s.frame[1] + s.frame[2]) & 0xFF;
  if (sum != s.frame[3]) {
    s.bad_frames++;
    return;
  }
  s.last_mm = (static_cast<uint16_t>(s.frame[1]) << 8) | s.frame[2];
  s.last_ok_ms = now;
  report(s, (s.last_mm >= MIN_MM && s.last_mm <= MAX_MM) ? ULTRASONIC_OK
                                                       : ULTRASONIC_OUT_OF_RANGE);
}

}  // namespace

void ultrasonic_begin() {
  uint32_t now = millis();
  for (Sensor &s : g_sensors) {
    s.port->begin(ULTRASONIC_BAUD);
    s.last_ok_ms = now;
    s.last_timeout_report_ms = now;
  }
}

void ultrasonic_update() {
  uint32_t now = millis();
  for (Sensor &s : g_sensors) {
    while (s.port->available() > 0) feed(s, s.port->read(), now);

    // Report silence so the Pi doesn't trust a stale distance.
    if (now - s.last_ok_ms >= TIMEOUT_MS && now - s.last_timeout_report_ms >= TIMEOUT_MS) {
      s.last_timeout_report_ms = now;
      report(s, ULTRASONIC_TIMEOUT);
    }
  }
}
