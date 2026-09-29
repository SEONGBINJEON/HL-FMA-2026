#ifndef HL_FMA_MANUAL_CONTROL_CORE_H
#define HL_FMA_MANUAL_CONTROL_CORE_H

#include <stddef.h>
#include <stdint.h>

namespace hl_manual {

enum Action : uint8_t { DISARM = 0, ARM = 1, CONTROL = 2 };
enum State : uint8_t { DISARMED = 0, ARMED = 1 };
enum Reason : uint8_t {
  BOOT = 0, STOP = 1, TIMEOUT = 2, BAD_FRAME = 3, STALE = 4,
  INTERLOCK = 5, ARM_REJECTED = 6, REVERSAL = 7, STEER_LIMIT = 8,
  READY = 9, ACTIVE = 10, NEUTRAL = 11
};
enum Flag : uint16_t {
  BENCH = 1, DRIVE_ENABLED = 2, STEER_ENABLED = 4,
  INTERLOCK_CONFIGURED = 8, INTERLOCK_OK = 16, BOOT_ID_VALID = 32
};

static const uint32_t REQUEST_MS = 150;
static const uint32_t STATUS_MS = 50;
static const uint32_t PARTIAL_MS = 30;
static const uint32_t NEUTRAL_MS = 500;
static const uint32_t STEER_MS = 200;
static const size_t MAX_FRAME = 32;
static const size_t COMMAND_SIZE = 18;
static const size_t STATUS_SIZE = 32;
static const uint32_t BOOT_MAGIC = 0x484C4D31UL;

uint16_t crc16(const uint8_t *data, size_t length);
uint32_t readU32(const uint8_t *p);
void writeU32(uint8_t *p, uint32_t value);
// Pure EEPROM transformation. Bytes 0..11 exclusively hold magic/count/~count.
// False means no output may be written and boot identity must remain invalid.
bool nextBootRecord(const uint8_t previous[12], uint8_t next[12], uint32_t &id);

struct Config {
  bool actuation_enabled;
  bool interlock_configured;
  bool steering_configured;
  uint16_t steering_min;
  uint16_t steering_max;
  int8_t positive_steer_adc_direction;
  Config();
};

struct Snapshot {
  uint32_t boot_id;
  uint32_t request_id;
  State state;
  Reason reason;
  int8_t requested_drive;
  int8_t requested_steer;
  int8_t actual_drive;
  int8_t actual_steer;
  uint16_t flags;
};

class Core {
 public:
  Core(uint32_t boot_id, bool boot_valid, const Config &config = Config());
  void tick(uint32_t now);
  void setInputs(uint16_t steering_adc, bool interlock_ok, uint32_t now);
  // Call after tick; caller must also bound its own serial read work per loop.
  void feed(uint8_t byte, uint32_t now);
  void inputOverload(uint32_t now);
  bool statusDue(uint32_t now) const;
  // Caller must reserve space for the entire frame before invoking this method.
  // Retransmissions retain the original request timestamp, never renew a lease.
  bool makeStatus(uint8_t out[STATUS_SIZE], uint32_t now, int32_t encoder_count);
  Snapshot snapshot() const;

 private:
#ifdef HL_MANUAL_TEST
  friend struct TestAccess;
#endif
  Config config_;
  uint32_t boot_id_;
  bool boot_valid_;
  bool permanent_lock_;
  State state_;
  Reason reason_;
  uint32_t request_id_;
  uint32_t request_at_;
  bool pending_;
  bool lease_valid_;
  uint32_t lease_at_;
  bool neutral_valid_;
  uint32_t neutral_at_;
  bool deadman_seen_;
  int8_t drive_;
  int8_t steer_;
  int8_t drive_sign_seen_;
  int8_t steer_sign_seen_;
  bool steering_timed_;
  uint32_t steering_at_;
  uint16_t steering_adc_;
  bool interlock_ok_;
  bool status_sent_;
  uint32_t status_at_;
  uint8_t rx_[MAX_FRAME];
  uint8_t rx_used_;
  uint32_t rx_at_;
  bool rx_aborted_;

  void stop(Reason reason);
  void stopAsynchronously(Reason reason);
  void processCommand(uint32_t now);
  bool steeringConfigurationValid() const;
  bool steeringAllowed(int8_t direction) const;
  uint16_t flags() const;
};

}  // namespace hl_manual
#endif
