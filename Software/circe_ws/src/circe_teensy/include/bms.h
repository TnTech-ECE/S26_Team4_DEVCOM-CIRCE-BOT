// Two JK BMSs on Serial1/Serial2 at 115200, polled at 1 Hz -> MSG_BMS_CELLS + MSG_BMS_STATUS.
// Protocol per syssi/esphome-jk-bms. Never wire VBAT to the Teensy; isolate the UARTs.
#pragma once

void bms_begin();
void bms_update();
