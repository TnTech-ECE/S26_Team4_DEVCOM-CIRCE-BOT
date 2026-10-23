// YIC GT-504GG GNSS receiver (MediaTek, NMEA 0183) on Serial3 at 9600, parsed with
// TinyGPS++ and forwarded to the Pi as MSG_GPS_FIX (include/protocol.h).
//
//   setup(): gps_begin();
//   loop():  gps_update();
//
// Wiring: GPS TXD (white) -> pin 15 (RX3), GPS RXD (green) <- pin 14 (TX3).
//
// *** THE BAUD RATE IS RE-ASSERTED WHENEVER THE RECEIVER GOES QUIET. ***
// GPS_BAUD in src/gps.cpp is the rate the Teensy listens at. If no valid sentence
// arrives for 3 s, $PMTK251,<GPS_BAUD> is sent at every rate the receiver might be at
// (its 9600 power-up rate, or 115200 from an earlier command, held by its backup
// capacitor), and listening resumes at GPS_BAUD.
//
// *** NEEDS THE -N2 OR -JST VARIANT, WHERE GREEN IS RXD. *** On the GT-504GG-N, green
// is PPS, an output: it cannot take the baud command, and pin 14 would fight it.
#pragma once

void gps_begin();
void gps_update();
