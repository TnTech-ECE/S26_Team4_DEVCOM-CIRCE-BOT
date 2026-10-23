// E-stop button, forwarded to the Pi as MSG_ESTOP (include/protocol.h).
//
//   setup(): estop_begin();
//   loop():  estop_update();
//   bridge:  estop_handle(msg_id, payload, len);
//
// Wiring: a NORMALLY-CLOSED contact between pin 2 and GND, with the internal pull-up.
// Closed reads LOW = run. Pressed, a broken wire or an unplugged connector all read
// HIGH = stop, so every wiring fault fails safe. With no button fitted the board
// boots latched: jumper pin 2 to GND on the bench.
//
// A press latches. Releasing the button does not clear it; MSG_ESTOP_RESET from the
// Pi does, and only while the button is released.
//
// *** THIS IS A REPORT, NOT A POWER CUT. *** The rover's drive motors stop because the
// Pi forwards this to the driver's /soft_estop/trigger. That path is USB and software;
// a hung Pi defeats it. A second NC contact on the button must break motor power.
#pragma once

#include <stddef.h>
#include <stdint.h>

void estop_begin();
void estop_update();
void estop_handle(uint8_t msg_id, const uint8_t *payload, size_t len);
bool estop_active();
