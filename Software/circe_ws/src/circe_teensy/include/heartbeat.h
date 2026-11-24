// 1 Hz TEENSY_HEARTBEAT out; heartbeat_pi_alive() is false once PI_HEARTBEAT stops.
#pragma once

#include <stddef.h>
#include <stdint.h>

void heartbeat_update();
void heartbeat_handle(uint8_t msg_id, const uint8_t *payload, size_t len);
bool heartbeat_pi_alive();
