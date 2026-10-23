// The onboard LED (pin 13) as a traffic light: led_pulse() lights it briefly.
//
// Non-blocking: led_update() in loop() turns it off when the pulse expires.
// Pin 13 is also SPI SCK. Move the indicator to another pin before adding SPI.
#pragma once

void led_begin();
void led_pulse();
void led_update();
