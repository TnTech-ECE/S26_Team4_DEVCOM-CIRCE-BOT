// GT-504GG GNSS (NMEA) on Serial3 -> MSG_GPS_FIX. Needs the -N2/-JST variant (green = RXD).
// Wiring: GPS TX -> pin 15 (RX3), GPS RX <- pin 14 (TX3).
#pragma once

void gps_begin();
void gps_update();
