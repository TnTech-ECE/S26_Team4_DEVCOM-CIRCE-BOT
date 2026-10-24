// Cable spool: the commands from the Pi, the interlocks that gate the motor, and the
// wound-radius model that turns rover speed into spool speed.
//
//   loop():  spool_update();
//   bridge:  spool_handle(msg_id, payload, len);
//
// spool_may_run() is true only while ALL of these hold:
//   - SPOOL_CMD enabled it
//   - the Pi is alive (heartbeat_pi_alive())
//   - the e-stop is clear (estop_active())
//   - an ODOM_SPEED arrived within SPEED_TIMEOUT_MS (src/spool.cpp)
// Any one failing stops the motor; none of them restarts it on its own except the
// speed, which resumes with the next odometry message.
//
// Sign convention, everywhere: positive = paying cable OUT, which is what the rover
// driving forward needs. Reversing winds it back in.
//
// The spool turns at rover_speed / wound_radius, so it speeds up as it empties: ~42 rpm
// full, ~96 rpm near the core at the rover's 0.4 m/s cap. The radius comes from how
// much cable is on the drum, which the encoder keeps current through
// spool_add_rotation(). See the ASSUMED constants at the top of src/spool.cpp.
//
// No motor yet: the controller driver calls spool_target_rad_s() for its setpoint and
// feeds encoder travel to spool_add_rotation(). Until then this tracks state only, and
// SPOOL_STATUS reports what the motor would be told.
#pragma once

#include <stddef.h>
#include <stdint.h>

void spool_update();
void spool_handle(uint8_t msg_id, const uint8_t *payload, size_t len);
bool spool_enabled();
float spool_speed();  // last ODOM_SPEED, m/s; 0 once stale
bool spool_may_run();

float spool_cable_on_m();    // cable still on the drum
float spool_radius_m();      // radius the cable is leaving from right now
float spool_target_rad_s();  // motor setpoint at the drum; 0 whenever it must not turn
uint8_t spool_flags();       // SpoolFlags (include/protocol.h)

// Drum rotation since the last call, radians, positive = paying out.
void spool_add_rotation(float delta_rad);
