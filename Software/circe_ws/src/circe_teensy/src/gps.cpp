#include "gps.h"

#include <Arduino.h>
#include <TinyGPS++.h>
#include <math.h>

#include "bridge.h"
#include "protocol.h"

namespace {

constexpr uint32_t GPS_BAUD = 9600;
// Rates the receiver may be at (power-up or after an earlier PMTK251).
constexpr uint32_t GPS_PROBE_BAUDS[] = {9600, 115200};
constexpr uint32_t NO_DATA_MS = 2000;
constexpr uint32_t RECONFIGURE_MS = 3000;  // silent this long -> resend baud command
constexpr uint32_t STALE_FIX_MS = 2000;
constexpr uint32_t SILENT_REPORT_MS = 1000;

HardwareSerialIMXRT &g_port = Serial3;
TinyGPSPlus g_gps;

// Extra RX buffer so a ~500-byte NMEA burst survives a loop() stall.
uint8_t g_rx_extra[512];

uint32_t g_last_sentence_ms = 0;
uint32_t g_last_sentences = 0;
uint32_t g_last_sent_ms = 0;
uint32_t g_last_configure_ms = 0;

void send_nmea(const char *body) {
  uint8_t sum = 0;
  for (const char *c = body; *c; c++) sum ^= static_cast<uint8_t>(*c);
  char tail[6];
  snprintf(tail, sizeof(tail), "*%02X\r\n", sum);
  g_port.write('$');
  g_port.write(body);
  g_port.write(tail);
}

// Send PMTK251 at every probe rate, then listen at GPS_BAUD (blocks ~25 ms).
void configure_baud(uint32_t now) {
  char cmd[20];
  snprintf(cmd, sizeof(cmd), "PMTK251,%lu", static_cast<unsigned long>(GPS_BAUD));
  for (uint32_t baud : GPS_PROBE_BAUDS) {
    g_port.begin(baud);
    send_nmea(cmd);
    g_port.flush();
  }
  g_port.begin(GPS_BAUD);
  g_last_configure_ms = now;
}

uint8_t status_now(uint32_t now) {
  if (now - g_last_sentence_ms >= NO_DATA_MS) return GPS_NO_DATA;
  // location.isValid() stays true once set, so also check age and fix quality.
  if (!g_gps.location.isValid() || g_gps.location.age() > STALE_FIX_MS ||
      g_gps.location.FixQuality() == TinyGPSLocation::Invalid) {
    return GPS_NO_FIX;
  }
  return GPS_FIX;
}

void report(uint32_t now) {
  GpsFix m;
  m.status = status_now(now);
  m.lat_e7 = static_cast<int32_t>(lround(g_gps.location.lat() * 1e7));
  m.lon_e7 = static_cast<int32_t>(lround(g_gps.location.lng() * 1e7));
  m.alt_m = g_gps.altitude.meters();
  m.hdop = g_gps.hdop.hdop();
  m.speed_mps = g_gps.speed.mps();
  m.course_deg = g_gps.course.deg();
  uint32_t age = g_gps.location.age();
  m.fix_age_ms = age;
  m.bad_sentences = static_cast<uint16_t>(min(g_gps.failedChecksum(), 0xFFFFu));
  m.satellites = static_cast<uint8_t>(g_gps.satellites.value());
  m.fix_quality = static_cast<uint8_t>(g_gps.location.FixQuality() - '0');
  bridge_send(MSG_GPS_FIX, &m, sizeof(m));
  g_last_sent_ms = now;
}

}  // namespace

void gps_begin() {
  g_port.addMemoryForRead(g_rx_extra, sizeof(g_rx_extra));
  g_port.begin(GPS_BAUD);
  uint32_t now = millis();
  g_last_sentence_ms = now;
  g_last_sent_ms = now;
  g_last_configure_ms = now;
}

void gps_update() {
  while (g_port.available() > 0) g_gps.encode(g_port.read());

  uint32_t now = millis();
  if (g_gps.passedChecksum() != g_last_sentences) {
    g_last_sentences = g_gps.passedChecksum();
    g_last_sentence_ms = now;
  }

  if (now - g_last_sentence_ms >= RECONFIGURE_MS && now - g_last_configure_ms >= RECONFIGURE_MS) {
    configure_baud(now);
  }

  // hdop updates once per GGA: one report per epoch.
  if (g_gps.hdop.isUpdated()) {
    report(now);
  } else if (now - g_last_sent_ms >= SILENT_REPORT_MS && status_now(now) == GPS_NO_DATA) {
    report(now);
  }
}
