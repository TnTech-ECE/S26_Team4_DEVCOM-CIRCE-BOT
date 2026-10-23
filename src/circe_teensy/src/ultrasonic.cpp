#include "ultrasonic.h"

#include <Arduino.h>

#include "bridge.h"
#include "protocol.h"

namespace {

// *** FIXED BY THE SENSOR, NOT A CHOICE. *** The A02YYUW only talks 9600 8N1; measured
// on CIRCE's sensor as a 104 us shortest pulse. At 115200 not one valid frame arrives.
constexpr uint32_t ULTRASONIC_BAUD = 9600;
constexpr uint32_t TIMEOUT_MS = 500;
constexpr uint16_t MIN_MM = 30;    // A02YYUW blind zone
constexpr uint16_t MAX_MM = 4500;  // A02YYUW rated range

struct Sensor {
  HardwareSerial *port;
  uint8_t id;

  // Parser and reporting state.
  uint8_t frame[4];
  uint8_t pos;
  uint16_t last_mm;
  uint16_t bad_frames;
  uint32_t last_ok_ms;
  uint32_t last_timeout_report_ms;
};

// Teensy RX <- sensor TX, Teensy TX -> sensor RX (mode select):
//   sensor 0  Serial5  RX5 pin 21, TX5 pin 20
//   sensor 1  Serial4  RX4 pin 16, TX4 pin 17
// Crossed wires show as TIMEOUT with bad_frames=0, and also put two outputs on the TX pin.
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
  if (s.pos == 0 && b != 0xFF) return;  // hunting for the header
  s.frame[s.pos++] = b;
  if (s.pos < 4) return;
  s.pos = 0;

  uint8_t sum = (s.frame[0] + s.frame[1] + s.frame[2]) & 0xFF;
  if (sum != s.frame[3]) {
    // Also what a wrong baud rate looks like: bytes arrive, nothing checks out.
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

    // A dead or unplugged sensor sends nothing at all, so silence has to be reported
    // explicitly or the Pi would keep trusting the last distance.
    if (now - s.last_ok_ms >= TIMEOUT_MS && now - s.last_timeout_report_ms >= TIMEOUT_MS) {
      s.last_timeout_report_ms = now;
      report(s, ULTRASONIC_TIMEOUT);
    }
  }
}
