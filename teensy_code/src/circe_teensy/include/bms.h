// JK BMS (JK-BD4A8S4P) on Serial1 at 115200, polled once a second and forwarded to the
// Pi as MSG_BMS_CELLS + MSG_BMS_STATUS (include/protocol.h).
//
//   setup(): bms_begin();
//   loop():  bms_update();
//
// Protocol per syssi/esphome-jk-bms (components/jk_modbus, jk_bms): the "read all"
// request below, answered by one ~270-byte frame
//   4E 57 | len:u16 BE | terminal:4 | cmd | source | type | TLV data ... | record:4 | 68 |
//   checksum:u32 BE
// where len counts every byte after the 4E 57 and the checksum is the 16-bit sum of all
// bytes before it. Data is ID-tagged, multi-byte values big-endian.
//
// *** THIS MODEL IS ONLY CONFIRMED OVER BLUETOOTH. *** esphome-jk-bms lists other BD
// boards on the same hardware revision (11.xw) as answering on UART-TTL, not this one.
// If the log shows BMS_STATUS NO_DATA with bad_frames=0, it is not answering at all.
//
// Wiring: the BMS's 4-pin JST 1.25 mm socket is GND, RX, TX, VBAT.
//   BMS TX -> pin 0 (RX1), BMS RX <- pin 1 (TX1), GND to GND.
//   *** VBAT IS THE FULL PACK VOLTAGE (~20-33 V). NEVER CONNECT IT TO THE TEENSY. ***
// The socket's GND is the pack's B-, on the battery side of the BMS's cut-off MOSFETs.
// If the Teensy's ground is also on the load side (P-), this wire bypasses them. Put
// a digital isolator (ADuM1201, ISO7721) between the BMS and the Teensy.
#pragma once

void bms_begin();
void bms_update();
