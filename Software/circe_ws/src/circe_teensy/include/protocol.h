// Teensy <-> Pi wire protocol. Mirrored in scripts/teensy_bridge.py; keep both in sync.
// Frame: COBS(msg_id:u8 | seq:u8 | payload | crc16-CCITT-FALSE:u16 LE) + 0x00
#pragma once

#include <stddef.h>
#include <stdint.h>

constexpr size_t MAX_FRAME = 64;
constexpr size_t FRAME_OVERHEAD = 4;
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
  MSG_ESTOP_RESET = 0x12,
};

#pragma pack(push, 1)

enum HeartbeatFlags : uint8_t {
  HB_PI_ALIVE = 1 << 0,
  HB_ESTOP = 1 << 1,
  HB_SPOOL_RUN = 1 << 2,
};

struct TeensyHeartbeat {
  uint32_t counter;
  uint32_t uptime_ms;
  uint32_t rx_ok;
  uint16_t rx_bad;
  uint16_t tx_dropped;
  uint32_t last_pi_counter;
  uint8_t spool_enabled;
  float last_speed;
  uint8_t flags;
};

struct EstopState {
  uint8_t active;          // latched
  uint8_t button_pressed;
};

enum SpoolFlags : uint8_t {
  SPOOL_RUN = 1 << 0,
  SPOOL_AT_RESERVE = 1 << 1,
  SPOOL_FULL = 1 << 2,
  SPOOL_SATURATED = 1 << 3,   // rover outrunning the drum
};

struct SpoolStatus {
  float cable_on_m;
  float radius_m;
  float target_rad_s;  // positive = paying out
  uint8_t flags;
};

// BMS fields are raw JK units; the Pi converts.
enum BmsStatusCode : uint8_t {
  BMS_OK = 0,
  BMS_NO_DATA = 1,
};

struct BmsStatus {
  uint8_t bms_id;
  uint16_t voltage_cv;     // 0.01 V
  int16_t current_ca;      // 0.01 A, positive = charging
  uint16_t alarms;
  uint16_t mode;
  uint16_t cycles;
  uint16_t capacity_ah;
  uint16_t bad_frames;
  int8_t temp_mos_c;
  int8_t temp_1_c;
  int8_t temp_2_c;
  uint8_t soc_pct;
  uint8_t cell_count;
  uint8_t battery_type;
  uint8_t protocol_version;
  uint8_t status;
};

constexpr size_t BMS_MAX_CELLS = 24;
struct BmsCells {
  uint8_t bms_id;
  uint8_t count;
  uint16_t cell_mv[BMS_MAX_CELLS];
};

enum UltrasonicStatus : uint8_t {
  ULTRASONIC_OK = 0,
  ULTRASONIC_OUT_OF_RANGE = 1,
  ULTRASONIC_TIMEOUT = 2,
};

struct UltrasonicReading {
  uint8_t sensor_id;
  uint8_t status;
  uint16_t distance_mm;
  uint16_t bad_frames;
};

enum GpsStatus : uint8_t {
  GPS_FIX = 0,
  GPS_NO_FIX = 1,
  GPS_NO_DATA = 2,
};

struct GpsFix {
  int32_t lat_e7;        // degrees * 1e7
  int32_t lon_e7;
  float alt_m;           // above MSL
  float hdop;
  float speed_mps;       // over ground, from RMC
  float course_deg;
  uint32_t fix_age_ms;
  uint16_t bad_sentences;
  uint8_t satellites;
  uint8_t fix_quality;   // GGA field 6
  uint8_t status;
};

struct PiHeartbeat {
  uint32_t counter;
};

struct SpoolCmd {
  uint8_t enable;
};

struct OdomSpeed {
  float linear_x;  // m/s
};

#pragma pack(pop)

// Must match the Python struct formats in scripts/teensy_bridge.py.
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
