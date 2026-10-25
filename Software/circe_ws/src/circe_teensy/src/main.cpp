// CIRCE Teensy 4.0 I/O board. Each feature lives in its own include/ + src/ pair;
// this file only wires them together. Nothing in loop() may block.
//
// Hardware UARTs: Serial1 + Serial2 BMS, Serial3 GPS, Serial4 + Serial5 ultrasonics.
// The spool motor controller needs Serial6 or Serial7 (pads underneath). Pin 2 e-stop,
// pin 13 status LED.
#include <Arduino.h>

#include "bms.h"
#include "bridge.h"
#include "estop.h"
#include "gps.h"
#include "heartbeat.h"
#include "spool.h"
#include "status_led.h"
#include "ultrasonic.h"

// Every module sees every frame and ignores the IDs it does not own.
static void on_message(uint8_t msg_id, const uint8_t *payload, size_t len) {
  heartbeat_handle(msg_id, payload, len);
  estop_handle(msg_id, payload, len);
  spool_handle(msg_id, payload, len);
}

void setup() {
  led_begin();
  // First, so the button is being watched before anything else can move.
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
