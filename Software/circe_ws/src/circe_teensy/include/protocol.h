// Wire protocol between the Teensy and the Pi 5 bridge node.
//
// *** THIS IS DEFINED TWICE. *** The Python mirror is the constants block at the top
// of scripts/teensy_bridge.py. Change a message here and change it there, or the two
// sides disagree silently: a CRC still passes on a payload the other side unpacks wrong.
//
// Frame on the wire:
//   COBS( msg_id:u8 | seq:u8 | payload | crc16:u16 LE ) + 0x00
//
// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over msg_id, seq and payload.
// Payloads are packed little-endian structs, which is the Teensy's native layout.
#pragma once

#include <stddef.h>
#include <stdint.h>

// Largest decoded frame (msg_id + seq + payload + crc). Anything bigger is dropped.
constexpr size_t MAX_FRAME = 64;
constexpr size_t FRAME_OVERHEAD = 4;  // msg_id, seq, crc16
constexpr size_t MAX_PAYLOAD = MAX_FRAME - FRAME_OVERHEAD;

enum MsgId : uint8_t {
  // Teensy -> Pi
  MSG_TEENSY_HEARTBEAT = 0x01,
  MSG_ULTRASONIC = 0x20,
  MSG_GPS_FIX = 0x30,
  MSG_ESTOP = 0x40,
  MSG_SPOOL_STATUS = 0x41,
  MSG_BMS_STATUS = 0x50,
  MSG_BMS_CELLS = 0x51,
  // Pi -> Teensy
  MSG_PI_HEARTBEAT = 0x02,
  MSG_SPOOL_CMD = 0x10,
  MSG_ODOM_SPEED = 0x11,
  MSG_ESTOP_RESET = 0x12,  // empty payload
};

#pragma pack(push, 1)

// TeensyHeartbeat.flags
enum HeartbeatFlags : uint8_t {
  HB_PI_ALIVE = 1 << 0,   // a PI_HEARTBEAT arrived within PI_TIMEOUT_MS (src/heartbeat.cpp)
  HB_ESTOP = 1 << 1,      // e-stop latched
  HB_SPOOL_RUN = 1 << 2,  // every spool interlock is clear (src/spool.cpp)
};

// 1 Hz. Echoes what the Teensy last received, so the Pi's log alone proves a round trip.
struct TeensyHeartbeat {
  uint32_t counter;
  uint32_t uptime_ms;
  uint32_t rx_ok;
  uint16_t rx_bad;
  uint16_t tx_dropped;  // frames not sent because the USB buffer was full; seq skips none
  uint32_t last_pi_counter;
  uint8_t spool_enabled;
  float last_speed;
  uint8_t flags;  // HeartbeatFlags
};

// On every change, then every 200 ms while latched and every 1 s while clear.
struct EstopState {
  uint8_t active;          // latched: set by a press, cleared only by ESTOP_RESET
  uint8_t button_pressed;  // debounced input right now; a broken wire reads as pressed
};

// 5 Hz. What the spool model would tell the motor (src/spool.cpp).
enum SpoolFlags : uint8_t {
  SPOOL_RUN = 1 << 0,         // every interlock clear
  SPOOL_AT_RESERVE = 1 << 1,  // out of cable: will not pay out further
  SPOOL_FULL = 1 << 2,        // will not wind in further
  SPOOL_SATURATED = 1 << 3,   // rover faster than the drum can follow at this radius
};

struct SpoolStatus {
  float cable_on_m;
  float radius_m;
  float target_rad_s;  // positive = paying out
  uint8_t flags;       // SpoolFlags
};

// JK BMS, per BMS: one per poll (1 Hz), or every 1 s while that BMS is silent. Raw JK
// units, so the Teensy does no float maths and the Pi owns the conversion.
enum BmsStatusCode : uint8_t {
  BMS_OK = 0,
  BMS_NO_DATA = 1,  // no valid response for 3 s; every other field is the last good one
};

struct BmsStatus {
  uint8_t bms_id;          // the table in src/bms.cpp; index into battery_topics on the Pi
  uint16_t voltage_cv;     // pack, 0.01 V
  int16_t current_ca;      // 0.01 A, positive = charging
  uint16_t alarms;         // JK field 0x8B bitmask, names in scripts/teensy_bridge.py
  uint16_t mode;           // JK field 0x8C: bit0 charge MOS, bit1 discharge MOS, bit2 balancing
  uint16_t cycles;
  uint16_t capacity_ah;    // as configured in the BMS
  uint16_t bad_frames;     // running count of checksum and parse failures
  int8_t temp_mos_c;
  int8_t temp_1_c;
  int8_t temp_2_c;
  uint8_t soc_pct;
  uint8_t cell_count;
  uint8_t battery_type;    // 0 LiFePO4, 1 Li-ion, 2 LTO
  uint8_t protocol_version;
  uint8_t status;          // BmsStatusCode
};

// Sent just before each BMS_OK status, so the Pi pairs it with the status that follows
// from the same bms_id.
constexpr size_t BMS_MAX_CELLS = 24;
struct BmsCells {
  uint8_t bms_id;
  uint8_t count;
  uint16_t cell_mv[BMS_MAX_CELLS];  // only the first `count` are meaningful
};

// One per A02YYUW reading (~10 Hz per sensor), or every 500 ms while a sensor is silent.
enum UltrasonicStatus : uint8_t {
  ULTRASONIC_OK = 0,
  ULTRASONIC_OUT_OF_RANGE = 1,  // valid frame, distance outside 30..4500 mm
  ULTRASONIC_TIMEOUT = 2,       // no valid frame for 500 ms; distance_mm is the last good one
};

struct UltrasonicReading {
  uint8_t sensor_id;
  uint8_t status;        // UltrasonicStatus
  uint16_t distance_mm;
  uint16_t bad_frames;   // running count of checksum failures on this sensor
};

// One per GGA sentence (1 Hz by default), or every 1 s while the receiver is silent.
enum GpsStatus : uint8_t {
  GPS_FIX = 0,      // position valid and fresh
  GPS_NO_FIX = 1,   // NMEA arriving, but no valid position (searching, or lost)
  GPS_NO_DATA = 2,  // no NMEA for 2 s: unpowered, unplugged or wrong baud
};

struct GpsFix {
  int32_t lat_e7;        // degrees * 1e7, WGS84 (1e-7 deg ~ 1.1 cm)
  int32_t lon_e7;
  float alt_m;           // above mean sea level (GGA), NOT the WGS84 ellipsoid
  float hdop;
  float speed_mps;       // over ground, from RMC
  float course_deg;      // over ground, true north, from RMC; meaningless when stopped
  uint32_t fix_age_ms;   // since the position was last updated
  uint16_t bad_sentences;  // running count of NMEA checksum failures
  uint8_t satellites;
  uint8_t fix_quality;   // GGA field 6: 0 none, 1 GPS, 2 DGPS/SBAS, ...
  uint8_t status;        // GpsStatus
};

struct PiHeartbeat {
  uint32_t counter;
};

struct SpoolCmd {
  uint8_t enable;  // 1 = start reeling, 0 = stop
};

struct OdomSpeed {
  float linear_x;  // m/s, the rover's forward speed
};

#pragma pack(pop)

// Python struct formats in scripts/teensy_bridge.py must give these same sizes.
static_assert(sizeof(TeensyHeartbeat) == 26, "TeensyHeartbeat: '<IIIHHIBfB'");
static_assert(sizeof(UltrasonicReading) == 6, "UltrasonicReading: '<BBHH'");
static_assert(sizeof(GpsFix) == 33, "GpsFix: '<iiffffIHBBB'");
static_assert(sizeof(EstopState) == 2, "EstopState: '<BB'");
static_assert(sizeof(SpoolStatus) == 13, "SpoolStatus: '<fffB'");
static_assert(sizeof(BmsStatus) == 23, "BmsStatus: '<BHhHHHHHbbbBBBBB'");
static_assert(sizeof(BmsCells) == 50, "BmsCells: '<BB24H'");
static_assert(sizeof(PiHeartbeat) == 4, "PiHeartbeat: '<I'");
static_assert(sizeof(SpoolCmd) == 1, "SpoolCmd: '<B'");
static_assert(sizeof(OdomSpeed) == 4, "OdomSpeed: '<f'");
static_assert(sizeof(BmsCells) <= MAX_PAYLOAD, "payload exceeds MAX_FRAME");
