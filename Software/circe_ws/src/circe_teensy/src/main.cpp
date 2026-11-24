// CIRCE Teensy 4.0 I/O board: wires the modules together. Nothing in loop() may block.
#include <Arduino.h>

#include "bms.h"
#include "bridge.h"
#include "estop.h"
#include "gps.h"
#include "heartbeat.h"
#include "spool.h"
#include "status_led.h"
#include "ultrasonic.h"

static void on_message(uint8_t msg_id, const uint8_t *payload, size_t len) {
  heartbeat_handle(msg_id, payload, len);
  estop_handle(msg_id, payload, len);
  spool_handle(msg_id, payload, len);
}

void setup() {
  led_begin();
  estop_begin();
  bridge_begin();
  bridge_on_message(on_message);
  ultrasonic_begin();
  gps_begin();
  bms_begin();
}

void loop() {
  bridge_poll();
  estop_update();
  heartbeat_update();
  spool_update();
  ultrasonic_update();
  gps_update();
  bms_update();
  led_update();
}
