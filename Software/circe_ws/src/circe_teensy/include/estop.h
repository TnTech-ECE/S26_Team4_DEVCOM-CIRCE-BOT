// E-stop (NC contact, pin 2 to GND) -> MSG_ESTOP. Fails safe: open/broken reads as stop.
// Latches until MSG_ESTOP_RESET. Software report only; the button must also cut motor power.
#pragma once

#include <stddef.h>
#include <stdint.h>

void estop_begin();
void estop_update();
void estop_handle(uint8_t msg_id, const uint8_t *payload, size_t len);
bool estop_active();
