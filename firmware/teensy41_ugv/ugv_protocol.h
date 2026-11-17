// ugv_protocol.h -- Teensy <-> Jetson wire protocol.
//
// Portable C++ (no Arduino dependencies) so it is compiled and tested on a PC
// against the Python implementation (src/ugv_hardware/ugv_hardware/protocol.py).
// Keep the two in sync; test/test_protocol.py cross-checks them.
//
// Frame (all multi-byte values little-endian):
//   0xA5 0x5A | type u8 | len u8 | seq u8 | payload[len] | crc16 u16
// crc16 = CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over type..payload.
// seq is a per-direction counter (wraps at 256) so the receiver can count
// dropped frames.
#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace ugv {

constexpr uint8_t  SYNC0 = 0xA5;
constexpr uint8_t  SYNC1 = 0x5A;
constexpr uint8_t  MAX_PAYLOAD = 120;
constexpr size_t   FRAME_OVERHEAD = 7;  // 2 sync + type + len + seq + 2 crc
constexpr uint8_t  N_WHEELS = 4;        // order: 0=FL 1=FR 2=RL 3=RR

enum MsgType : uint8_t {
  // Teensy -> Jetson
  MSG_TELEMETRY      = 0x01,
  MSG_CONFIG_REPORT  = 0x02,
  // Jetson -> Teensy
  MSG_CMD_WHEELS     = 0x10,   // also the heartbeat: send at >= 20 Hz
  MSG_CONFIG_SET     = 0x11,
  MSG_CLEAR_FAULT    = 0x12,
  MSG_REQUEST_CONFIG = 0x13,
  MSG_CMD_SERVOS     = 0x14,   // pan/tilt servo targets (degrees)
};

enum StatusBits : uint16_t {
  ST_ESTOP         = 1 << 0,  // e-stop input active (latched until released + CLEAR_FAULT)
  ST_CMD_WATCHDOG  = 1 << 1,  // no CMD_WHEELS within watchdog_ms -> motors stopped
  ST_IMU_STALE     = 1 << 2,  // no IMU sample recently
  ST_ENC_FAULT     = 1 << 3,  // an encoder failed to read or lost its magnet
  ST_MOTOR_FAULT   = 1 << 4,  // stall: driven hard but not turning (latched)
  ST_CONFIG_VALID  = 1 << 5,  // a CONFIG_SET has been received since boot
  ST_LOW_BATTERY   = 1 << 6,
  ST_ARMED         = 1 << 7,  // motors are actually allowed to move right now
  ST_OPEN_LOOP     = 1 << 8,  // firmware built without encoders: feed-forward only, no PID
};

enum CmdFlags : uint8_t { CMD_ENABLE = 1 << 0 };

#pragma pack(push, 1)
struct Telemetry {            // 71 bytes
  uint32_t t_us;              // Teensy micros() when this packet was built
  int32_t  enc_ticks[4];      // cumulative, + = wheel turning forward
  float    wheel_vel[4];      // rad/s, filtered
  float    gyro[3];           // rad/s, IMU axes (frame imu_link)
  float    accel[3];          // m/s^2, IMU axes
  uint32_t imu_t_us;          // micros() of the newest IMU sample
  uint16_t status;            // StatusBits
  uint16_t cmd_age_ms;        // time since the last CMD_WHEELS (saturates)
  uint16_t batt_mv;           // 0 = not measured
  uint8_t  enc_ok_mask;       // bit i set = encoder i healthy
};

struct CmdWheels {            // 9 bytes. Left = FL+RL, right = FR+RR.
  float   left_rad_s;
  float   right_rad_s;
  uint8_t flags;              // CmdFlags
};

struct CmdServos {            // 4 bytes. Same meaning as the old servo1:/servo2: tokens.
  int16_t servo1_deg;
  int16_t servo2_deg;
};

struct ConfigSet {            // 24 bytes (also used for CONFIG_REPORT)
  float    ticks_per_wheel_rev;  // encoder ticks per ONE WHEEL revolution
  float    kp, ki, kd;           // wheel-velocity PID (duty per rad/s)
  float    max_wheel_rad_s;      // speed at ~100 % duty, also the command clamp
  uint16_t watchdog_ms;
  uint8_t  enc_invert_mask;      // bit i inverts encoder i
  uint8_t  motor_invert_mask;    // bit i inverts motor i direction
};
#pragma pack(pop)

static_assert(sizeof(Telemetry) == 71, "Telemetry layout changed");
static_assert(sizeof(CmdWheels) == 9, "CmdWheels layout changed");
static_assert(sizeof(ConfigSet) == 24, "ConfigSet layout changed");
static_assert(sizeof(CmdServos) == 4, "CmdServos layout changed");

inline uint16_t crc16(const uint8_t* data, size_t len, uint16_t crc = 0xFFFF) {
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int b = 0; b < 8; b++) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

// Writes a complete frame to out (needs len + FRAME_OVERHEAD bytes).
// Returns the frame length, or 0 if the payload is too large.
inline size_t encode_frame(uint8_t type, uint8_t seq, const void* payload, uint8_t len,
                           uint8_t* out) {
  if (len > MAX_PAYLOAD) return 0;
  out[0] = SYNC0;
  out[1] = SYNC1;
  out[2] = type;
  out[3] = len;
  out[4] = seq;
  if (len) memcpy(out + 5, payload, len);
  uint16_t c = crc16(out + 2, 3u + len);
  out[5 + len] = (uint8_t)(c & 0xFF);
  out[6 + len] = (uint8_t)(c >> 8);
  return (size_t)len + FRAME_OVERHEAD;
}

// Byte-at-a-time parser; no heap, safe in an interrupt-free main loop.
class Parser {
 public:
  uint32_t crc_errors = 0;
  uint32_t bad_len = 0;

  // Returns true when a complete, CRC-valid frame is available via type()/seq()/...
  bool feed(uint8_t b) {
    switch (state_) {
      case S0:   if (b == SYNC0) state_ = S1; break;
      case S1:   if (b == SYNC1) state_ = TYPE;
                 else if (b != SYNC0) state_ = S0;  // A5 A5 5A still syncs
                 break;
      case TYPE: type_ = b; state_ = LEN; break;
      case LEN:
        len_ = b;
        if (len_ > MAX_PAYLOAD) { bad_len++; state_ = S0; }
        else state_ = SEQ;
        break;
      case SEQ:  seq_ = b; idx_ = 0; state_ = (len_ ? PAYLOAD : CRC0); break;
      case PAYLOAD:
        buf_[idx_++] = b;
        if (idx_ >= len_) state_ = CRC0;
        break;
      case CRC0: crc_rx_ = b; state_ = CRC1; break;
      case CRC1: {
        crc_rx_ |= (uint16_t)b << 8;
        uint8_t hdr[3] = {type_, len_, seq_};
        uint16_t c = crc16(hdr, 3);
        c = crc16(buf_, len_, c);
        state_ = S0;
        if (c == crc_rx_) return true;
        crc_errors++;
        break;
      }
    }
    return false;
  }

  uint8_t type() const { return type_; }
  uint8_t seq() const { return seq_; }
  uint8_t len() const { return len_; }
  const uint8_t* payload() const { return buf_; }

 private:
  enum State : uint8_t { S0, S1, TYPE, LEN, SEQ, PAYLOAD, CRC0, CRC1 };
  State state_ = S0;
  uint8_t type_ = 0, len_ = 0, seq_ = 0, idx_ = 0;
  uint16_t crc_rx_ = 0;
  uint8_t buf_[MAX_PAYLOAD];
};

}  // namespace ugv
