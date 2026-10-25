// Two JK BMSs (JK-BD4A8S4P), one per hardware UART at 115200, each polled once a second
// and forwarded to the Pi as MSG_BMS_CELLS + MSG_BMS_STATUS (include/protocol.h),
// tagged with its bms_id.
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
// *** UART ON THIS MODEL IS NOT YET TESTED ON CIRCE. *** chrissank/JKBMSInterface
// reports reading a JK-BD4A8S4P on its 4-pin GPS port with this same request.
// If the log shows BMS_STATUS NO_DATA with bad_frames=0, it is not answering at all.
//
// The BMSs are listed in the table at the top of src/bms.cpp.
//
// Wiring: the BMS's 4-pin JST 1.25 mm socket is GND, RX, TX, VBAT.
//   bms 0  BMS TX -> pin 0 (RX1), BMS RX <- pin 1 (TX1), GND to GND.
//   bms 1  BMS TX -> pin 7 (RX2), BMS RX <- pin 8 (TX2), GND to GND.
//   *** VBAT IS THE FULL PACK VOLTAGE (~20-33 V). NEVER CONNECT IT TO THE TEENSY. ***
// The socket's GND is the pack's B-, on the battery side of the BMS's cut-off MOSFETs.
// If the Teensy's ground is also on the load side (P-), this wire bypasses them. Put
// a digital isolator (ADuM1201, ISO7721) between the BMS and the Teensy.
// *** WITH PACKS IN SERIES, ISOLATE BOTH. *** The upper pack's B- sits at the lower
// pack's full voltage; tying both GND pins to the Teensy shorts the lower pack.
#pragma once

void bms_begin();
void bms_update();
