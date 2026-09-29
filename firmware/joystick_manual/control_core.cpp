#include "control_core.h"

namespace hl_manual {

uint16_t crc16(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit)
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
  }
  return crc;
}

uint32_t readU32(const uint8_t *p) {
  return static_cast<uint32_t>(p[0]) |
         (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}

void writeU32(uint8_t *p, uint32_t value) {
  for (uint8_t i = 0; i < 4; ++i) p[i] = static_cast<uint8_t>(value >> (8 * i));
}

bool nextBootRecord(const uint8_t previous[12], uint8_t next[12], uint32_t &id) {
  id = 0;
  bool erased = true;
  for (uint8_t i = 0; i < 12; ++i) erased = erased && previous[i] == 0xFF;
  uint32_t count = 0;
  if (!erased) {
    count = readU32(previous + 4);
    if (readU32(previous) != BOOT_MAGIC || count == 0 ||
        readU32(previous + 8) != ~count || count == UINT32_MAX)
      return false;
  }
  id = count + 1;
  writeU32(next, BOOT_MAGIC);
  writeU32(next + 4, id);
  writeU32(next + 8, ~id);
  return true;
}

Config::Config()
    : actuation_enabled(false), interlock_configured(false),
      steering_configured(false), steering_min(0), steering_max(0),
      positive_steer_adc_direction(0) {}

Core::Core(uint32_t boot_id, bool boot_valid, const Config &config)
    : config_(config), boot_id_(boot_id), boot_valid_(boot_valid && boot_id != 0),
      permanent_lock_(!boot_valid_), state_(DISARMED), reason_(BOOT),
      request_id_(0), request_at_(0), pending_(false), lease_valid_(false),
      lease_at_(0), neutral_valid_(false), neutral_at_(0), deadman_seen_(false),
      drive_pwm_(60), drive_(0), steer_(0), drive_sign_seen_(0), steer_sign_seen_(0),
      steering_adc_(0),
      interlock_ok_(false), status_sent_(false), status_at_(0), rx_used_(0),
      rx_at_(0), rx_aborted_(false) {}

void Core::stop(Reason reason) {
  state_ = DISARMED;
  reason_ = reason;
  drive_ = steer_ = 0;
  drive_sign_seen_ = steer_sign_seen_ = 0;
  pending_ = lease_valid_ = neutral_valid_ = deadman_seen_ = false;
  rx_used_ = 0;
  rx_aborted_ = false;
}

void Core::stopAsynchronously(Reason reason) {
  // A safety deadline can fall between serial bytes. Invalidate all control
  // state immediately, but finish parsing the already-started frame as a
  // discarded frame. Otherwise its valid tail is misread as unrelated garbage.
  // It retains the original 30ms partial-frame deadline and cannot grant a
  // command lease, neutral proof, or ARM. No new buffering or waiting is added.
  const uint8_t partial_used = rx_used_;
  stop(reason);
  rx_used_ = partial_used;
  rx_aborted_ = partial_used != 0;
}

bool Core::steeringConfigurationValid() const {
  return config_.steering_configured && config_.steering_min < config_.steering_max &&
         config_.steering_max <= 1023 &&
         (config_.positive_steer_adc_direction == 1 ||
          config_.positive_steer_adc_direction == -1);
}

bool Core::steeringAllowed(int8_t direction) const {
  if (!steeringConfigurationValid() || steering_adc_ < config_.steering_min ||
      steering_adc_ > config_.steering_max) return false;
  int8_t adc_direction = direction * config_.positive_steer_adc_direction;
  if (adc_direction > 0 && steering_adc_ >= config_.steering_max) return false;
  if (adc_direction < 0 && steering_adc_ <= config_.steering_min) return false;
  return true;
}

void Core::tick(uint32_t now) {
  if ((lease_valid_ && static_cast<uint32_t>(now - lease_at_) >= REQUEST_MS) ||
      (pending_ && static_cast<uint32_t>(now - request_at_) >= REQUEST_MS)) {
    stopAsynchronously(TIMEOUT);
    return;
  }
  if (rx_used_ && static_cast<uint32_t>(now - rx_at_) >= PARTIAL_MS) {
    stop(BAD_FRAME);
    return;
  }
  if (neutral_valid_ && static_cast<uint32_t>(now - neutral_at_) >= NEUTRAL_MS)
    neutral_valid_ = false;
  if (state_ == ARMED && config_.actuation_enabled &&
      (!config_.interlock_configured || !interlock_ok_)) {
    stopAsynchronously(INTERLOCK);
    return;
  }
  if (state_ == ARMED && steer_ && config_.actuation_enabled && !steeringAllowed(steer_))
    stopAsynchronously(STEER_LIMIT);
}

void Core::setInputs(uint16_t steering_adc, bool interlock_ok, uint32_t now) {
  steering_adc_ = steering_adc;
  interlock_ok_ = interlock_ok;
  tick(now);
}

void Core::inputOverload(uint32_t now) {
  tick(now);
  stop(BAD_FRAME);
}

void Core::feed(uint8_t byte, uint32_t now) {
  // Deadline checks precede handling even a valid delayed command.
  tick(now);
  if (!rx_used_) {
    if (byte != 0xF1) { stop(BAD_FRAME); return; }
    rx_[0] = byte;
    rx_used_ = 1;
    rx_at_ = now;
    return;
  }
  if (rx_used_ == 1 && byte != 0xA5) { stop(BAD_FRAME); return; }
  if (rx_used_ >= MAX_FRAME) { stop(BAD_FRAME); return; }
  rx_[rx_used_++] = byte;
  if (rx_used_ == 3 && rx_[2] != 2) { stop(BAD_FRAME); return; }
  if (rx_used_ == 4 && rx_[3] != 13) { stop(BAD_FRAME); return; }
  if (rx_used_ != COMMAND_SIZE) return;
  const uint16_t received = static_cast<uint16_t>(rx_[17]) |
                            (static_cast<uint16_t>(rx_[18]) << 8);
  if (received != crc16(rx_ + 2, 15)) { stop(BAD_FRAME); return; }
  // A valid explicit DISARM always works. Other commands from a frame that
  // crossed a stop transition are discarded even if a new request now exists.
  if (!rx_aborted_ || rx_[12] == DISARM) processCommand(now);
  rx_used_ = 0;
  rx_aborted_ = false;
}

void Core::processCommand(uint32_t now) {
  const uint8_t action = rx_[12];
  // A well-framed stop is effective even when identity/ranges are stale.
  if (action == DISARM) { stop(STOP); return; }
  const int8_t drive = static_cast<int8_t>(rx_[13]);
  const int8_t steer = static_cast<int8_t>(rx_[14]);
  const uint8_t deadman = rx_[15];
  if (action > CONTROL || drive < -1 || drive > 1 || steer < -1 || steer > 1 ||
      deadman > 1) { stop(BAD_FRAME); return; }
  if (permanent_lock_ || !boot_valid_ || readU32(rx_ + 4) != boot_id_ ||
      readU32(rx_ + 8) != request_id_ || !pending_ ||
      static_cast<uint32_t>(now - request_at_) >= REQUEST_MS) {
    stop(STALE);
    return;
  }
  pending_ = false;
  if (action == ARM) {
    if (state_ != DISARMED || !neutral_valid_ ||
        static_cast<uint32_t>(now - neutral_at_) >= NEUTRAL_MS ||
        drive || steer || deadman) { stop(ARM_REJECTED); return; }
    if (config_.actuation_enabled && (!config_.interlock_configured || !interlock_ok_)) {
      stop(INTERLOCK);
      return;
    }
    drive_pwm_ = rx_[16];
    state_ = ARMED;
    reason_ = READY;
    neutral_valid_ = false;
    lease_valid_ = true;
    lease_at_ = request_at_;
    return;
  }
  if (state_ != ARMED) {
    if (drive == 0 && steer == 0 && deadman == 0) {
      drive_pwm_ = rx_[16];
      neutral_valid_ = true;
      neutral_at_ = now;
      reason_ = NEUTRAL;
    } else stop(ARM_REJECTED);
    return;
  }
  if (!deadman) {
    if (drive || steer) { stop(STOP); return; }
    // A neutral command keeps the prepared session alive while guaranteeing
    // zero requested output. It also proves a release before another axis or
    // direction can start.
    drive_pwm_ = rx_[16];
    drive_ = steer_ = 0;
    drive_sign_seen_ = steer_sign_seen_ = 0;
    deadman_seen_ = false;
    reason_ = READY;
    lease_valid_ = true;
    lease_at_ = request_at_;
    return;
  }
  deadman_seen_ = true;
  // Each axis may be added or removed independently. A sign reversal still
  // requires at least one accepted command with that axis at zero.
  if ((drive && drive_sign_seen_ && drive != drive_sign_seen_) ||
      (steer && steer_sign_seen_ && steer != steer_sign_seen_)) {
    stop(REVERSAL);
    return;
  }
  if (!drive) drive_sign_seen_ = 0;
  if (!steer) steer_sign_seen_ = 0;
  if (steer && config_.actuation_enabled && !steeringAllowed(steer)) {
    stop(STEER_LIMIT);
    return;
  }
  if (drive) drive_sign_seen_ = drive;
  if (steer) steer_sign_seen_ = steer;
  drive_pwm_ = rx_[16];
  drive_ = drive;
  steer_ = steer;
  lease_at_ = request_at_;
  lease_valid_ = true;
  reason_ = (drive || steer) ? ACTIVE : NEUTRAL;
}

uint16_t Core::flags() const {
  uint16_t result = config_.actuation_enabled ? 0 : BENCH;
  if (config_.interlock_configured) result |= INTERLOCK_CONFIGURED;
  if (config_.interlock_configured && interlock_ok_) result |= INTERLOCK_OK;
  if (boot_valid_ && !permanent_lock_) result |= BOOT_ID_VALID;
  if (config_.actuation_enabled && config_.interlock_configured && boot_valid_ &&
      !permanent_lock_) {
    result |= DRIVE_ENABLED;
    if (steeringConfigurationValid()) result |= STEER_ENABLED;
  }
  return result;
}

Snapshot Core::snapshot() const {
  Snapshot out;
  out.boot_id = boot_id_;
  out.request_id = request_id_;
  out.state = state_;
  out.reason = reason_;
  out.requested_drive = drive_;
  out.requested_steer = steer_;
  out.flags = flags();
  const bool permit = state_ == ARMED && lease_valid_ && interlock_ok_;
  out.actual_drive = permit && (out.flags & DRIVE_ENABLED) ? drive_ : 0;
  out.actual_steer = permit && (out.flags & STEER_ENABLED) && steeringAllowed(steer_) ? steer_ : 0;
  return out;
}

bool Core::statusDue(uint32_t now) const {
  return !status_sent_ || static_cast<uint32_t>(now - status_at_) >= STATUS_MS;
}

bool Core::makeStatus(uint8_t out[STATUS_SIZE], uint32_t now, int32_t encoder_count) {
  tick(now);
  if (!statusDue(now)) return false;
  if (!pending_ && !permanent_lock_) {
    if (request_id_ == UINT32_MAX) {
      permanent_lock_ = true;
      stop(STALE);
    } else {
      ++request_id_;
      request_at_ = now;
      pending_ = true;
    }
  }
  const Snapshot view = snapshot();
  out[0] = 0xF1;
  out[1] = 0xA5;
  out[2] = 129;
  out[3] = 26;
  writeU32(out + 4, view.boot_id);
  writeU32(out + 8, view.request_id);
  writeU32(out + 12, now);
  writeU32(out + 16, static_cast<uint32_t>(encoder_count));
  out[20] = static_cast<uint8_t>(steering_adc_);
  out[21] = static_cast<uint8_t>(steering_adc_ >> 8);
  out[22] = view.state;
  out[23] = view.reason;
  out[24] = static_cast<uint8_t>(view.requested_drive);
  out[25] = static_cast<uint8_t>(view.requested_steer);
  out[26] = static_cast<uint8_t>(view.actual_drive);
  out[27] = static_cast<uint8_t>(view.actual_steer);
  out[28] = static_cast<uint8_t>(view.flags);
  out[29] = static_cast<uint8_t>(view.flags >> 8);
  const uint16_t crc = crc16(out + 2, 28);
  out[30] = static_cast<uint8_t>(crc);
  out[31] = static_cast<uint8_t>(crc >> 8);
  status_at_ = now;
  status_sent_ = true;
  return true;
}

}  // namespace hl_manual
