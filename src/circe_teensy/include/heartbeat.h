// Link liveness in both directions.
//
//   loop():  heartbeat_update();
//   bridge:  heartbeat_handle(msg_id, payload, len);
//
// Sends TEENSY_HEARTBEAT at 1 Hz, echoing the last PI_HEARTBEAT counter, the spool
// command and odometry speed it received, and the interlock flags.
//
// heartbeat_pi_alive() is the watchdog everything that moves must check: false once
// no PI_HEARTBEAT has arrived for PI_TIMEOUT_MS (src/heartbeat.cpp), which is what a
// crashed bridge node, a hung Pi or a pulled USB cable all look like from here.
#pragma once

#include <stddef.h>
#include <stdint.h>

void heartbeat_update();
void heartbeat_handle(uint8_t msg_id, const uint8_t *payload, size_t len);
bool heartbeat_pi_alive();
