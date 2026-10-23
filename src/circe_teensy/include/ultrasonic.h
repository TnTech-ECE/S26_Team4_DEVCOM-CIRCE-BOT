// DFRobot A02YYUW ultrasonic rangers on hardware UARTs, forwarded to the Pi as
// MSG_ULTRASONIC (include/protocol.h).
//
//   setup(): ultrasonic_begin();
//   loop():  ultrasonic_update();
//
// The sensor streams 4-byte frames on its own: 0xFF, distance high, distance low, sum,
// where sum = (0xFF + high + low) & 0xFF and distance is in mm.
//
// Wiring per sensor: sensor TX -> Teensy RX, sensor RX <- Teensy TX. The sensor's "RX"
// is not a data input but its mode select: high or floating gives the filtered
// (processed) value, low gives the raw real-time value. The Teensy TX pin idles high,
// so the sensors run in processed mode.
//
// The sensors are listed in the table at the top of src/ultrasonic.cpp; a second one
// is one more line there.
#pragma once

void ultrasonic_begin();
void ultrasonic_update();
