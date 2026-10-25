// Cable spool: Pi commands, interlocks, and rover speed -> spool speed via wound radius.
// spool_may_run() requires: enabled, Pi alive, e-stop clear, fresh ODOM_SPEED.
// Positive = paying cable out.
#pragma once

#include <stddef.h>
#include <stdint.h>

void spool_update();
void spool_handle(uint8_t msg_id, const uint8_t *payload, size_t len);
bool spool_enabled();
float spool_speed();  // last ODOM_SPEED, m/s; 0 once stale
bool spool_may_run();

float spool_cable_on_m();
float spool_radius_m();
float spool_target_rad_s();  // 0 whenever it must not turn
uint8_t spool_flags();

void spool_add_rotation(float delta_rad);
