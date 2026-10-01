#include "daisy_blind.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "esphome/core/application.h"
#include "esphome/core/log.h"
#include "esphome/core/preferences_rtc.h"

#ifdef USE_ESP8266
#include <core_esp8266_waveform.h>
extern "C" {
#include <spi_flash.h>
}
extern "C" uint32_t _SPIFFS_end;  // NOLINT: ESPHome's preference sector sits here
#endif

namespace esphome {
namespace daisy_blind {

static const char *const TAG = "daisy_blind";

static const uint32_t PACKET_MAGIC = 0x4442534E;  // "DBSN"
static const uint8_t PROTO_VERSION = 2;
static const uint8_t TYPE_BEACON = 1;
static const uint8_t TYPE_COMMAND = 2;
static const uint8_t CMD_GOTO = 0;
static const uint8_t CMD_HOME = 1;
static const uint8_t CMD_STOP = 2;
static const uint8_t FLAG_MOVING = 1;
static const uint8_t FLAG_STARTED = 2;
static const uint8_t FLAG_ONE_AT_A_TIME = 4;

static const size_t MAX_PEERS = 8;
static const uint32_t PEER_TIMEOUT_MS = 6000;
static const uint32_t BEACON_IDLE_MS = 1000;
static const uint32_t BEACON_MOVING_MS = 300;
static const uint32_t CYCLE_MS = 1000;        // one full firing cycle, shared evenly by movers
static const uint32_t GUARD_MS = 20;          // dead time at each slot edge
static const uint32_t HOLD_MS = 250;          // wait after a command so peers can announce
static const uint32_t DRIVER_WAKE_MS = 2;     // A4988 needs 1 ms after SLEEP/ENABLE
static const uint32_t IDLE_SLEEP_MS = 300;    // hold time before de-energising
static const uint32_t PUBLISH_INTERVAL_MS = 500;
static const float START_SPS = 180.0f;        // step rate a ramp begins and ends at (higher = gentler at the ends)
static const float ACCEL_SPS2 = 1500.0f;      // acceleration, steps per second squared
static const int32_t MAX_MANUAL_STEPS = 50000;

static const uint32_t STATE_MAGIC = 0x31534244;  // "DBS1"
static const uint8_t STATE_VERSION_V1 = 1;
static const uint8_t STATE_VERSION = 2;
static const uint32_t STATE_KEY_V1 = 0x6D0B1D57;  // fixed; never tied to entity names
static const uint32_t STATE_KEY = 0x6D0B1D58;
static const uint8_t STATE_FLAG_POSITION_VALID = 1;
static const uint8_t STATE_FLAG_INVERT = 2;
static const uint8_t STATE_FLAG_GROUP_CONTROL = 4;
static const uint8_t STATE_FLAG_HOTSPOT_LED = 8;
static const uint8_t STATE_FLAG_ONE_AT_A_TIME = 16;
static const uint8_t STATE_FLAG_HOME_KNOWN = 32;
static const uint8_t STATE_FLAG_HOME_ON_POWER_LOSS = 64;
static const uint32_t STATE_WORDS_V1 = (sizeof(StoredStateV1) + 3) / 4;
static const uint32_t STATE_WORDS = (sizeof(StoredState) + 3) / 4;
// The record's size is part of its identity: changing it would orphan stored state.
static_assert(sizeof(StoredStateV1) == 56, "StoredStateV1 layout must never change");
static_assert(sizeof(StoredState) == 68, "StoredState must stay 68 bytes; use the reserved bytes");
static const uint32_t STATE_SAVE_INTERVAL_MS = 1000;  // throttle while moving
static const uint32_t PREF_SECTOR_WORDS = 128;         // ESPHome flash preference area (restore_from_flash)
static const uint32_t TORQUE_PERIOD_US = 50;           // 20 kHz enable-pin pulsing, above hearing
static const uint8_t TORQUE_MIN = 10;
static const uint8_t HOMING_TORQUE_DEFAULT = 30;
static const int32_t HOMING_OVERTRAVEL_DEFAULT = 150;
static const uint32_t BOOT_HOME_MIN_MS = 20000;   // let WiFi connect and peers be heard first
static const uint32_t BOOT_HOME_PEERS_MS = 5000;  // after the coordination socket opens
static const uint32_t BOOT_HOME_MAX_MS = 90000;   // home anyway if WiFi never comes up

template<typename T> static uint32_t state_crc(const T &st) {
  // FNV-1a over everything except the crc field itself.
  const uint8_t *b = reinterpret_cast<const uint8_t *>(&st);
  uint32_t h = 2166136261UL;
  for (size_t i = 0; i < offsetof(T, crc); i++) {
    h ^= b[i];
    h *= 16777619UL;
  }
  return h;
}

template<typename T> static bool state_ok(const T &st, uint8_t version) {
  return st.magic == STATE_MAGIC && st.version == version && st.crc == state_crc(st);
}

static StoredState upgrade_v1(const StoredStateV1 &o) {
  StoredState n{};
  n.magic = STATE_MAGIC;
  n.version = STATE_VERSION;
  n.flags = o.flags;
  n.group = o.group;
  n.order = o.order;
  n.seq = o.seq;
  n.position = o.position;
  n.open_steps = o.open_steps;
  n.closed_steps = o.closed_steps;
  n.speed = o.speed;
  n.nudge = o.nudge;
  memcpy(n.label, o.label, NAME_LEN);
  n.torque = 100;
  n.crc = state_crc(n);
  return n;
}

using cover::COVER_OPERATION_CLOSING;
using cover::COVER_OPERATION_IDLE;
using cover::COVER_OPERATION_OPENING;

static bool mac_less(const uint8_t *a, const uint8_t *b) { return memcmp(a, b, 6) < 0; }

// ---------------------------------------------------------------- setup / loop

void DaisyBlind::setup() {
  step_pin_->setup();
  step_pin_->digital_write(false);
  dir_pin_->setup();
  dir_pin_->digital_write(false);
  if (sleep_pin_ != nullptr) {
    sleep_pin_->setup();
    sleep_pin_->digital_write(false);  // asleep / disabled
  }

  get_mac_address_raw(mac_);

  // Allocation order matters on ESP8266: keep the legacy record first so the
  // version being upgraded from can still be read, then the new record.
  legacy_pref_ = global_preferences->make_preference<int32_t>(this->get_object_id_hash() ^ 0x44424C31);
  state_pref_ = global_preferences->make_preference<StoredState>(STATE_KEY, true);

  if (load_state_()) {
    ESP_LOGI(TAG, "Loaded state: position %d steps%s, open %d, closed %d", (int) position_,
             position_valid_ ? "" : " (UNKNOWN)", (int) open_steps_, (int) closed_steps_);
  } else {
    // First boot of this firmware: take settings from the entities (restored
    // just before this component) and the position from the legacy record.
    int32_t legacy = 0;
    if (legacy_pref_.load(&legacy)) {
      position_ = legacy;
      position_valid_ = true;
      ESP_LOGI(TAG, "Migrated position %d steps from previous firmware", (int) position_);
    } else {
      position_valid_ = false;
      ESP_LOGW(TAG, "No stored position: cover moves are blocked until you mark the blind closed or open");
    }
    // Nothing is written until the YAML boot hook has copied every restored
    // setting in and called finish_migration(), so a half-migrated record can
    // never be saved.
    migrating_ = true;
  }
  loaded_ = true;
  target_ = position_;

  this->position = pos_pct_();
  this->current_operation = COVER_OPERATION_IDLE;
  this->publish_state(false);

  // Remember whether this was a real power-up (not a restart or OTA update),
  // so "Home after power loss" only runs after the power actually went away.
  boot_ms_ = millis();
#ifdef USE_ESP8266
  const uint32_t reason = ESP.getResetInfoPtr()->reason;
  boot_home_pending_ = (reason == REASON_DEFAULT_RST || reason == REASON_EXT_SYS_RST);
  ESP_LOGI(TAG, "Reset reason: %s", ESP.getResetReason().c_str());
#endif
}

void DaisyBlind::loop() {
  const uint32_t now = millis();

  if (!udp_started_ && WiFi.isConnected()) {
    udp_started_ = udp_.begin(port_) != 0;
    if (udp_started_)
      ESP_LOGI(TAG, "Coordination listening on UDP %u", port_);
  }
  if (udp_started_) {
    receive_packets_(now);
    expire_peers_(now);
    const uint32_t interval = wants_move_() ? BEACON_MOVING_MS : BEACON_IDLE_MS;
    if (now - last_beacon_ms_ >= interval)
      send_beacon_(now);
  }

  if (boot_home_pending_) {
    const uint32_t up = now - boot_ms_;
    static uint32_t udp_since = 0;
    if (udp_started_ && udp_since == 0)
      udp_since = now | 1;
    const bool ready = up >= BOOT_HOME_MIN_MS && udp_since != 0 && now - udp_since >= BOOT_HOME_PEERS_MS;
    if (ready || up >= BOOT_HOME_MAX_MS) {
      boot_home_pending_ = false;
      if (home_on_power_loss_ && !homing_ && !migrating_) {
        ESP_LOGI(TAG, "Power-up: homing to re-establish the position");
        start_home_(now);
      }
    }
  }

  run_motor_(now);
}

void DaisyBlind::dump_config() {
  LOG_COVER("", "Daisy Blind", this);
  LOG_PIN("  Step pin: ", step_pin_);
  LOG_PIN("  Dir pin: ", dir_pin_);
  LOG_PIN("  Sleep/enable pin: ", sleep_pin_);
  ESP_LOGCONFIG(TAG, "  Coordination UDP port: %u", port_);
  ESP_LOGCONFIG(TAG, "  Torque limit: %u%%, homing torque %u%%", torque_limit_, homing_torque_);
  ESP_LOGCONFIG(TAG, "  Position: %d steps (open=%d closed=%d)", (int) position_, (int) open_steps_,
                (int) closed_steps_);
}

cover::CoverTraits DaisyBlind::get_traits() {
  auto traits = cover::CoverTraits();
  traits.set_is_assumed_state(false);
  traits.set_supports_position(true);
  traits.set_supports_stop(true);
  traits.set_supports_toggle(true);
  traits.set_supports_tilt(false);
  return traits;
}

// ---------------------------------------------------------------- cover control

void DaisyBlind::control(const cover::CoverCall &call) {
  if (call.get_stop()) {
    stop();
    if (group_control_)
      send_command_(CMD_STOP, 0);
    return;
  }
  if (call.get_toggle().has_value()) {
    if (wants_move_()) {
      stop();
      if (group_control_)
        send_command_(CMD_STOP, 0);
    } else {
      float pct = pos_pct_() < 0.5f ? 1.0f : 0.0f;
      apply_target_pct_(pct);
      if (group_control_)
        send_command_(CMD_GOTO, (uint8_t) lroundf(pct * 100.0f));
    }
    return;
  }
  if (call.get_position().has_value()) {
    float pct = *call.get_position();
    apply_target_pct_(pct);
    if (group_control_)
      send_command_(CMD_GOTO, (uint8_t) lroundf(pct * 100.0f));
  }
}

void DaisyBlind::apply_target_pct_(float pct) {
  if (homing_) {
    // Remember it: the blind goes there instead of its old position once homed.
    float q = pct < 0.0f ? 0.0f : (pct > 1.0f ? 1.0f : pct);
    restore_target_ = closed_steps_ + (int32_t) lroundf(q * (float) (open_steps_ - closed_steps_));
    ESP_LOGI(TAG, "Homing in progress: will go to %d%% afterwards", (int) lroundf(q * 100.0f));
    return;
  }
  if (!position_valid_) {
    ESP_LOGW(TAG, "Position unknown: refusing to move. Nudge the blind fully closed or open, then press "
                  "\"Mark as fully closed\" or \"Mark as fully open\".");
    this->current_operation = COVER_OPERATION_IDLE;
    publish_(false, millis());
    return;
  }
  if (pct < 0.0f)
    pct = 0.0f;
  if (pct > 1.0f)
    pct = 1.0f;
  const int32_t span = open_steps_ - closed_steps_;
  if (span <= 0) {
    ESP_LOGW(TAG, "Open limit must be greater than closed limit; ignoring move");
    return;
  }
  const int32_t target = closed_steps_ + (int32_t) lroundf(pct * (float) span);
  ESP_LOGI(TAG, "Target %d%% -> %d steps (now %d)", (int) lroundf(pct * 100.0f), (int) target, (int) position_);
  begin_move_(target, millis());
}

void DaisyBlind::move_to_steps(int32_t steps) {
  if (homing_) {
    ESP_LOGW(TAG, "Homing in progress; manual move ignored");
    return;
  }
  if (steps > MAX_MANUAL_STEPS)
    steps = MAX_MANUAL_STEPS;
  if (steps < -MAX_MANUAL_STEPS)
    steps = -MAX_MANUAL_STEPS;
  ESP_LOGI(TAG, "Manual target %d steps (now %d)", (int) steps, (int) position_);
  begin_move_(steps, millis());
}

void DaisyBlind::nudge(int32_t delta) {
  // Nudge relative to where the blind is heading, so repeated presses add up.
  move_to_steps(target_ + delta);
}

void DaisyBlind::begin_move_(int32_t target, uint32_t now) {
  const bool was_moving = wants_move_();
  target_ = target;
  if (target_ > position_) {
    this->current_operation = COVER_OPERATION_OPENING;
  } else if (target_ < position_) {
    this->current_operation = COVER_OPERATION_CLOSING;
  } else {
    this->current_operation = COVER_OPERATION_IDLE;
  }
  if (!was_moving && wants_move_()) {
    // Give peers that received the same command a moment to announce
    // themselves before anybody starts drawing current.
    hold_until_ = now + HOLD_MS;
    started_ = false;
  }
  publish_(false, now);
  if (udp_started_)
    send_beacon_(now);
}

void DaisyBlind::stop() {
  abort_home_("stopped");
  target_ = position_;
  this->current_operation = COVER_OPERATION_IDLE;
  if (state_dirty_)
    save_state_(true);
  publish_(true, millis());
  if (udp_started_)
    send_beacon_(millis());
}

float DaisyBlind::pos_pct_() const {
  const int32_t span = open_steps_ - closed_steps_;
  if (span <= 0)
    return 0.0f;
  float pct = (float) (position_ - closed_steps_) / (float) span;
  return pct < 0.0f ? 0.0f : (pct > 1.0f ? 1.0f : pct);
}

void DaisyBlind::refresh_position_() {
  this->position = pos_pct_();
  if (!wants_move_())
    this->publish_state(false);
}

void DaisyBlind::publish_(bool save, uint32_t now) {
  this->position = pos_pct_();
  this->publish_state(save);
  last_publish_ms_ = now;
}

// ---------------------------------------------------------------- persistence

bool DaisyBlind::load_state_() {
  StoredState best{};
  bool found = false;

  StoredState st{};
  bool in_slot = false;
  if (state_pref_.load(&st) && state_ok(st, STATE_VERSION)) {
    best = st;
    found = true;
    in_slot = true;
  }

#ifdef USE_ESP8266
  // Scan the raw preference sector so the record is found even if the order of
  // other settings changed between firmware versions. Newest sequence wins.
  // A version 1 record is used only if no version 2 record exists.
  static uint32_t raw[PREF_SECTOR_WORDS];
  const uint32_t addr = (uint32_t) &_SPIFFS_end - 0x40200000UL;
  StoredStateV1 best_v1{};
  bool found_v1 = false;
  if (spi_flash_read(addr, raw, sizeof(raw)) == SPI_FLASH_RESULT_OK) {
    for (uint32_t off = 0; off < PREF_SECTOR_WORDS; off++) {
      if (off + STATE_WORDS < PREF_SECTOR_WORDS) {
        StoredState cand{};
        if (rtc_pref_decode(raw + off, STATE_KEY, STATE_WORDS, reinterpret_cast<uint8_t *>(&cand), sizeof(cand)) &&
            state_ok(cand, STATE_VERSION) && (!found || (int32_t) (cand.seq - best.seq) > 0)) {
          ESP_LOGI(TAG, "Found state record at word %u (seq %u)", (unsigned) off, (unsigned) cand.seq);
          best = cand;
          found = true;
          in_slot = false;
        }
      }
      if (off + STATE_WORDS_V1 < PREF_SECTOR_WORDS) {
        StoredStateV1 cand{};
        if (rtc_pref_decode(raw + off, STATE_KEY_V1, STATE_WORDS_V1, reinterpret_cast<uint8_t *>(&cand),
                            sizeof(cand)) &&
            state_ok(cand, STATE_VERSION_V1) && (!found_v1 || (int32_t) (cand.seq - best_v1.seq) > 0)) {
          best_v1 = cand;
          found_v1 = true;
        }
      }
    }
  }
  // Use a version 1 record when there is no version 2 record, or when it is
  // newer (the blind was rolled back to older firmware and changed since).
  if (found_v1 && (!found || (int32_t) (best_v1.seq - best.seq) > 0)) {
    ESP_LOGI(TAG, "Upgrading version 1 state record (seq %u)", (unsigned) best_v1.seq);
    best = upgrade_v1(best_v1);
    found = true;
    in_slot = false;
  }
#endif

  if (!found)
    return false;

  state_seq_ = best.seq;
  position_ = best.position;
  position_valid_ = (best.flags & STATE_FLAG_POSITION_VALID) != 0;
  open_steps_ = best.open_steps;
  closed_steps_ = best.closed_steps;
  speed_sps_ = best.speed < 1 ? 1 : best.speed;
  nudge_size_ = best.nudge < 1 ? 1 : best.nudge;
  group_ = best.group < 1 ? 1 : best.group;
  order_ = best.order < 1 ? 1 : (best.order > 8 ? 8 : best.order);
  invert_direction_ = (best.flags & STATE_FLAG_INVERT) != 0;
  group_control_ = (best.flags & STATE_FLAG_GROUP_CONTROL) != 0;
  hotspot_led_ = (best.flags & STATE_FLAG_HOTSPOT_LED) != 0;
  one_at_a_time_ = (best.flags & STATE_FLAG_ONE_AT_A_TIME) != 0;
  best.label[NAME_LEN - 1] = 0;
  label_ = best.label;
  torque_limit_ = best.torque < TORQUE_MIN ? TORQUE_MIN : (best.torque > 100 ? 100 : best.torque);
  homing_torque_ = best.homing_torque == 0 ? HOMING_TORQUE_DEFAULT
                                           : (best.homing_torque < TORQUE_MIN ? TORQUE_MIN
                                                                              : (best.homing_torque > 100 ? 100 : best.homing_torque));
  homing_overtravel_ = best.homing_overtravel == 0 ? HOMING_OVERTRAVEL_DEFAULT : best.homing_overtravel;
  home_known_ = (best.flags & STATE_FLAG_HOME_KNOWN) != 0;
  home_pos_ = best.home_pos;
  home_on_power_loss_ = (best.flags & STATE_FLAG_HOME_ON_POWER_LOSS) != 0;
  dir_dirty_ = true;
  loaded_ = true;
  if (in_slot) {
    // Already where this firmware expects it; remember it to avoid rewriting.
    StoredState content = best;
    content.seq = 0;
    last_content_ = state_crc(content);
    have_saved_ = true;
  } else {
    // Found elsewhere in the sector: rewrite into this firmware's slot.
    ESP_LOGI(TAG, "Moving state record into this firmware's slot");
    save_state_(true);
  }
  return true;
}

void DaisyBlind::save_state_(bool flush) {
  if (!loaded_ || migrating_)
    return;
  StoredState st{};
  st.magic = STATE_MAGIC;
  st.version = STATE_VERSION;
  st.flags = (position_valid_ ? STATE_FLAG_POSITION_VALID : 0) | (invert_direction_ ? STATE_FLAG_INVERT : 0) |
             (group_control_ ? STATE_FLAG_GROUP_CONTROL : 0) | (hotspot_led_ ? STATE_FLAG_HOTSPOT_LED : 0) |
             (one_at_a_time_ ? STATE_FLAG_ONE_AT_A_TIME : 0) | (home_known_ ? STATE_FLAG_HOME_KNOWN : 0) |
             (home_on_power_loss_ ? STATE_FLAG_HOME_ON_POWER_LOSS : 0);
  st.group = group_;
  st.order = order_;
  st.seq = 0;
  st.position = position_;
  st.open_steps = open_steps_;
  st.closed_steps = closed_steps_;
  st.speed = (uint16_t) speed_sps_;
  st.nudge = (uint16_t) nudge_size_;
  snprintf(st.label, NAME_LEN, "%s", label_.c_str());
  st.torque = torque_limit_;
  st.home_pos = home_pos_;
  st.homing_torque = homing_torque_;
  st.homing_overtravel = (uint16_t) homing_overtravel_;
  // Skip the write if nothing changed (e.g. entities re-published at boot).
  const uint32_t content = state_crc(st);
  if (content == last_content_ && have_saved_) {
    state_dirty_ = false;
    return;
  }
  st.seq = ++state_seq_;
  st.crc = state_crc(st);
  if (!state_pref_.save(&st)) {
    ESP_LOGE(TAG, "Failed to save state record");
    return;
  }
  last_content_ = content;
  have_saved_ = true;
  state_dirty_ = false;
  last_state_save_ms_ = millis();
  if (flush)
    global_preferences->sync();
}

void DaisyBlind::settings_changed_() {
  if (loaded_ && !migrating_)
    save_state_(true);
}

void DaisyBlind::finish_migration() {
  if (!migrating_)
    return;
  migrating_ = false;
  ESP_LOGI(TAG, "Migrated settings: open %d, closed %d, invert %s, group %u, order %u", (int) open_steps_,
           (int) closed_steps_, invert_direction_ ? "on" : "off", group_, order_);
  save_state_(true);
}

void DaisyBlind::set_position_known_(int32_t steps) {
  abort_home_("marked");
  target_ = position_ = steps;
  position_valid_ = true;
  this->current_operation = COVER_OPERATION_IDLE;
  save_state_(true);
  publish_(true, millis());
  if (udp_started_)
    send_beacon_(millis());
}

void DaisyBlind::mark_closed() {
  ESP_LOGI(TAG, "Marked fully closed at %d steps", (int) closed_steps_);
  set_position_known_(closed_steps_);
}

void DaisyBlind::mark_open() {
  ESP_LOGI(TAG, "Marked fully open at %d steps", (int) open_steps_);
  set_position_known_(open_steps_);
}

void DaisyBlind::on_safe_shutdown() {
  // Reboot for an update or restart: stop cleanly and make sure the exact
  // position reaches flash before the power goes.
  homing_ = false;  // position already saved as unknown when homing began
  target_ = position_;
  set_driver_awake_(false, millis());
  save_state_(true);
}

// ---------------------------------------------------------------- motor

void DaisyBlind::apply_enable_pin_(bool awake) {
  if (sleep_pin_ == nullptr)
    return;
#ifdef USE_ESP8266
  const uint8_t pin = sleep_pin_->get_pin();
  const uint8_t torque = homing_ ? homing_torque_ : torque_limit_;
  if (awake && torque < 100) {
    // Pulse the driver's enable input so the coils are powered only part of
    // the time: less average current, less torque. 20 kHz is above hearing and
    // far faster than the coil current can follow, so the motor sees a steady
    // reduced current rather than pulses.
    const uint32_t on_us = (TORQUE_PERIOD_US * torque + 50) / 100;
    const uint32_t off_us = TORQUE_PERIOD_US - on_us;
    const bool active_high = !sleep_pin_->is_inverted();
    if (active_high) {
      startWaveform(pin, on_us, off_us, 0);
    } else {
      startWaveform(pin, off_us, on_us, 0);  // high = disabled for an active-low ENABLE
    }
    return;
  }
  stopWaveform(pin);
#endif
  sleep_pin_->digital_write(awake);
}

void DaisyBlind::set_torque_limit(int32_t pct) {
  const uint8_t v = (uint8_t) (pct < TORQUE_MIN ? TORQUE_MIN : (pct > 100 ? 100 : pct));
  if (v == torque_limit_)
    return;
  torque_limit_ = v;
  if (sleep_pin_ == nullptr && v < 100)
    ESP_LOGW(TAG, "Torque limit needs sleep_pin (the A4988 ENABLE pin); it has no effect");
  if (driver_awake_)
    apply_enable_pin_(true);
  settings_changed_();
}

void DaisyBlind::set_homing_torque(int32_t pct) {
  const uint8_t v = (uint8_t) (pct < TORQUE_MIN ? TORQUE_MIN : (pct > 100 ? 100 : pct));
  if (v == homing_torque_)
    return;
  homing_torque_ = v;
  if (homing_ && driver_awake_)
    apply_enable_pin_(true);
  settings_changed_();
}

void DaisyBlind::set_homing_overtravel(int32_t steps) {
  homing_overtravel_ = steps < 10 ? 10 : (steps > 5000 ? 5000 : steps);
  settings_changed_();
}

// ---------------------------------------------------------------- homing

void DaisyBlind::home() { start_home_(millis()); }

void DaisyBlind::home_group() {
  start_home_(millis());
  send_command_(CMD_HOME, 0);
}

void DaisyBlind::start_home_(uint32_t now) {
  if (homing_)
    return;
  if (sleep_pin_ == nullptr) {
    ESP_LOGW(TAG, "Homing needs sleep_pin (the A4988 ENABLE pin) for its torque limit; not homing");
    return;
  }
  const int32_t lo = closed_steps_ < open_steps_ ? closed_steps_ : open_steps_;
  const int32_t hi = closed_steps_ < open_steps_ ? open_steps_ : closed_steps_;
  // Return to where the blind last was (or was heading), kept within the limits.
  int32_t back = wants_move_() ? target_ : position_;
  restore_target_ = back < lo ? lo : (back > hi ? hi : back);

  // Far enough to reach the stop from anywhere it could plausibly be.
  int32_t distance;
  if (position_valid_ && home_known_ && position_ >= home_pos_) {
    distance = (position_ - home_pos_) + homing_overtravel_;
  } else {
    int32_t span = hi - lo;
    if (home_known_ && home_pos_ < lo)
      span += lo - home_pos_;
    distance = span + homing_overtravel_;
  }

  homing_ = true;
  homing_remaining_ = distance;
  target_ = position_;
  // If power is lost mid-homing the position must not be trusted afterwards.
  position_valid_ = false;
  save_state_(true);
  hold_until_ = now + HOLD_MS;
  started_ = false;
  dir_dirty_ = true;
  if (driver_awake_)
    apply_enable_pin_(true);
  this->current_operation = COVER_OPERATION_CLOSING;
  ESP_LOGI(TAG, "Homing: up to %d steps toward closed at %u%% torque, then back to %d", (int) distance,
           homing_torque_, (int) restore_target_);
  publish_(false, now);
  if (udp_started_)
    send_beacon_(now);
}

void DaisyBlind::finish_home_(uint32_t now) {
  homing_ = false;
  if (!home_known_) {
    // First home: the stop becomes the closed limit's reference point.
    home_pos_ = closed_steps_;
    home_known_ = true;
    ESP_LOGI(TAG, "First home: end stop recorded at the closed limit (%d)", (int) home_pos_);
  }
  position_ = home_pos_;
  position_valid_ = true;
  target_ = restore_target_;
  cur_sps_ = START_SPS;  // reverse gently, not at full speed
  if (driver_awake_)
    apply_enable_pin_(true);  // back to the normal torque limit
  dir_dirty_ = true;
  save_state_(true);
  ESP_LOGI(TAG, "Homed at %d; returning to %d", (int) position_, (int) target_);
  this->current_operation = target_ > position_ ? COVER_OPERATION_OPENING : COVER_OPERATION_IDLE;
  publish_(false, now);
}

void DaisyBlind::abort_home_(const char *why) {
  if (!homing_)
    return;
  homing_ = false;
  target_ = position_;
  if (driver_awake_)
    apply_enable_pin_(true);
  ESP_LOGW(TAG, "Homing stopped (%s); position unknown until homed or marked", why);
  save_state_(true);
}

void DaisyBlind::set_driver_awake_(bool awake, uint32_t now) {
  if (awake == driver_awake_)
    return;
  driver_awake_ = awake;
  apply_enable_pin_(awake);
  if (awake) {
    wake_until_ = now + DRIVER_WAKE_MS;
    cur_sps_ = START_SPS;  // every burst of motion ramps up from rest
    last_step_us_ = micros();
  }
}

void DaisyBlind::finish_move_(uint32_t now) {
  set_driver_awake_(false, now);
  started_ = false;
  if (dirty_ || state_dirty_) {
    dirty_ = false;
    save_state_(true);  // flush now: a finished move must survive a power cut
    ESP_LOGI(TAG, "Move complete at %d steps (%d%%)", (int) position_, (int) lroundf(pos_pct_() * 100.0f));
  }
  this->current_operation = COVER_OPERATION_IDLE;
  publish_(true, now);
  if (udp_started_)
    send_beacon_(now);
  high_freq_.stop();
}

void DaisyBlind::run_motor_(uint32_t now) {
  if (!wants_move_()) {
    // Hold briefly so the mechanism settles, then de-energise and persist.
    if ((driver_awake_ || dirty_ || this->current_operation != COVER_OPERATION_IDLE) &&
        now - last_step_ms_ >= IDLE_SLEEP_MS) {
      finish_move_(now);
    }
    return;
  }

  high_freq_.start();

  // Persist progress during long moves. The record is cached in RAM every
  // second and written to flash by ESPHome within flash_write_interval.
  if (state_dirty_ && now - last_state_save_ms_ >= STATE_SAVE_INTERVAL_MS)
    save_state_(false);

  if (!may_step_(now)) {
    set_driver_awake_(false, now);
    return;
  }

  if (!driver_awake_) {
    set_driver_awake_(true, now);
    return;
  }
  if ((int32_t) (now - wake_until_) < 0)
    return;

  // Trapezoidal speed profile: ramp up from START_SPS, cruise at the configured
  // speed, and ramp down so the final steps land gently on the target.
  const float max_sps = (float) speed_sps_;
  // Homing runs at full speed into the stop (a stepper is weakest when fast),
  // so it never decelerates.
  const int32_t remaining = homing_ ? 1000000 : (target_ > position_ ? target_ - position_ : position_ - target_);
  float sps = cur_sps_;
  if (sps < START_SPS)
    sps = START_SPS;
  const float stop_sps = sqrtf(2.0f * ACCEL_SPS2 * (float) remaining + START_SPS * START_SPS);
  if (sps > stop_sps)
    sps = stop_sps;
  if (sps > max_sps)
    sps = max_sps;
  const uint32_t interval_us = (uint32_t) (1000000.0f / sps);
  const uint32_t now_us = micros();
  if (now_us - last_step_us_ < interval_us)
    return;
  // Don't accumulate a backlog if the loop stalled; just step now.
  if (now_us - last_step_us_ > 2 * interval_us) {
    last_step_us_ = now_us;
  } else {
    last_step_us_ += interval_us;
  }
  // Accelerate for the next step: v += a * dt.
  cur_sps_ = sps + ACCEL_SPS2 * ((float) interval_us / 1000000.0f);

  const bool opening = homing_ ? false : target_ > position_;
  if (opening != last_dir_opening_ || dir_dirty_) {
    dir_pin_->digital_write(opening != invert_direction_);
    last_dir_opening_ = opening;
    dir_dirty_ = false;
    delayMicroseconds(5);
  }

  step_pin_->digital_write(true);
  delayMicroseconds(5);
  step_pin_->digital_write(false);
  last_step_ms_ = now;
  started_ = true;

  if (homing_) {
    // Steps into the stop don't move the blind; the count resets at the end.
    if (--homing_remaining_ <= 0)
      finish_home_(now);
  } else {
    position_ += opening ? 1 : -1;
    dirty_ = true;
    state_dirty_ = true;
  }

  if (now - last_publish_ms_ >= PUBLISH_INTERVAL_MS) {
    this->current_operation = opening ? COVER_OPERATION_OPENING : COVER_OPERATION_CLOSING;
    publish_(false, now);
  }
}

// ---------------------------------------------------------------- coordination

std::vector<DaisyBlind::Participant> DaisyBlind::participants_(uint32_t now, bool movers_only,
                                                                bool &one_at_a_time) {
  std::vector<Participant> list;
  one_at_a_time = false;
  if (!movers_only || wants_move_()) {
    list.push_back(Participant{mac_, order_, started_, one_at_a_time_, nullptr});
    one_at_a_time |= one_at_a_time_;
  }
  for (const auto &p : peers_) {
    if (p.group != group_)
      continue;
    if (movers_only && !(p.flags & FLAG_MOVING))
      continue;
    list.push_back(Participant{p.mac, p.order, (p.flags & FLAG_STARTED) != 0, (p.flags & FLAG_ONE_AT_A_TIME) != 0,
                               &p});
    one_at_a_time |= (p.flags & FLAG_ONE_AT_A_TIME) != 0;
  }
  const bool seq = one_at_a_time && movers_only;
  std::sort(list.begin(), list.end(), [seq](const Participant &a, const Participant &b) {
    // One-at-a-time: whoever is already in motion keeps going; then firing order; then MAC.
    if (seq && a.started != b.started)
      return a.started;
    if (a.order != b.order)
      return a.order < b.order;
    return mac_less(a.mac, b.mac);
  });
  return list;
}

bool DaisyBlind::may_step_(uint32_t now) {
  if ((int32_t) (now - hold_until_) < 0)
    return false;
  bool one_at_a_time = false;
  auto list = participants_(now, true, one_at_a_time);
  if (list.size() <= 1)
    return true;

  size_t my_index = 0;
  for (size_t i = 0; i < list.size(); i++) {
    if (list[i].peer == nullptr) {
      my_index = i;
      break;
    }
  }

  if (one_at_a_time)
    return my_index == 0;

  // Take turns: the cycle is divided evenly between every blind that is moving.
  const uint32_t count = list.size();
  const uint32_t slot_ms = CYCLE_MS / count;
  const uint32_t t = (now + sync_offset_) % CYCLE_MS;
  const uint32_t slot = t / slot_ms;
  const uint32_t in_slot = t % slot_ms;
  uint32_t guard = GUARD_MS;
  if (guard > slot_ms / 5)
    guard = slot_ms / 5;
  return slot == my_index && in_slot >= guard && in_slot < slot_ms - guard;
}

bool DaisyBlind::is_clock_master_(const uint8_t *mac, uint32_t now) {
  // The clock master is simply the lowest MAC in the group; independent of firing order.
  const uint8_t *lowest = mac_;
  for (const auto &p : peers_) {
    if (p.group == group_ && mac_less(p.mac, lowest))
      lowest = p.mac;
  }
  return memcmp(lowest, mac, 6) == 0;
}

void DaisyBlind::expire_peers_(uint32_t now) {
  peers_.erase(std::remove_if(peers_.begin(), peers_.end(),
                              [now](const Peer &p) { return now - p.last_seen > PEER_TIMEOUT_MS; }),
               peers_.end());
}

Peer *DaisyBlind::upsert_peer_(const Packet &p, uint32_t now) {
  Peer *slot = nullptr;
  for (auto &peer : peers_) {
    if (memcmp(peer.mac, p.mac, 6) == 0) {
      slot = &peer;
      break;
    }
  }
  if (slot == nullptr) {
    if (peers_.size() >= MAX_PEERS) {
      slot = &peers_.front();
      for (auto &peer : peers_)
        if (peer.last_seen < slot->last_seen)
          slot = &peer;
    } else {
      peers_.emplace_back();
      slot = &peers_.back();
    }
    memcpy(slot->mac, p.mac, 6);
    ESP_LOGI(TAG, "Discovered peer %.*s (group %u, order %u)", (int) NAME_LEN, p.name, p.group, p.order);
  }
  slot->group = p.group;
  slot->order = p.order;
  slot->flags = p.flags;
  slot->percent = p.percent;
  slot->last_seen = now;
  memcpy(slot->name, p.name, NAME_LEN);
  memcpy(slot->label, p.label, NAME_LEN);
  slot->name[NAME_LEN - 1] = 0;
  slot->label[NAME_LEN - 1] = 0;
  return slot;
}

void DaisyBlind::receive_packets_(uint32_t now) {
  for (int i = 0; i < 8; i++) {
    int size = udp_.parsePacket();
    if (size <= 0)
      break;
    if (size != (int) sizeof(Packet)) {
      udp_.flush();
      continue;
    }
    Packet p;
    udp_.read(reinterpret_cast<unsigned char *>(&p), sizeof(p));
    if (p.magic != PACKET_MAGIC || p.version != PROTO_VERSION)
      continue;
    if (memcmp(p.mac, mac_, 6) == 0)
      continue;  // our own broadcast echoed back

    upsert_peer_(p, now);
    if (p.group != group_)
      continue;

    if (p.type == TYPE_BEACON) {
      if (is_clock_master_(p.mac, now)) {
        sync_offset_ = p.t_sync - now;
        last_sync_ms_ = now;
      }
    } else if (p.type == TYPE_COMMAND) {
      handle_command_(p);
    }
  }
}

void DaisyBlind::handle_command_(const Packet &p) {
  if (p.cmd == CMD_HOME) {
    // An explicit "home the group" press is honoured regardless of Group control.
    ESP_LOGI(TAG, "Group home requested by %.*s", (int) NAME_LEN, p.name);
    start_home_(millis());
    return;
  }
  if (!group_control_)
    return;
  switch (p.cmd) {
    case CMD_GOTO:
      ESP_LOGI(TAG, "Group move to %u%% requested by %.*s", p.cmd_percent, (int) NAME_LEN, p.name);
      apply_target_pct_((float) p.cmd_percent / 100.0f);
      break;
    case CMD_STOP:
      stop();
      break;
    default:
      break;
  }
}

void DaisyBlind::fill_packet_(Packet &p, uint8_t type, uint32_t now) {
  memset(&p, 0, sizeof(p));
  p.magic = PACKET_MAGIC;
  p.version = PROTO_VERSION;
  p.type = type;
  memcpy(p.mac, mac_, 6);
  p.group = group_;
  p.order = order_;
  p.flags = (wants_move_() ? FLAG_MOVING : 0) | (started_ ? FLAG_STARTED : 0) |
            (one_at_a_time_ ? FLAG_ONE_AT_A_TIME : 0);
  p.percent = (uint8_t) lroundf(pos_pct_() * 100.0f);
  p.t_sync = now + sync_offset_;
  p.position = position_;
  snprintf(p.name, NAME_LEN, "%s", App.get_name().c_str());
  snprintf(p.label, NAME_LEN, "%s", label_.c_str());
}

void DaisyBlind::send_packet_(const Packet &p) {
  if (!udp_started_ || !WiFi.isConnected())
    return;
  const uint32_t ip = (uint32_t) WiFi.localIP();
  const uint32_t mask = (uint32_t) WiFi.subnetMask();
  ::IPAddress bcast(ip | ~mask);
  udp_.beginPacket(bcast, port_);
  udp_.write(reinterpret_cast<const uint8_t *>(&p), sizeof(p));
  udp_.endPacket();
}

void DaisyBlind::send_beacon_(uint32_t now) {
  last_beacon_ms_ = now;
  Packet p;
  fill_packet_(p, TYPE_BEACON, now);
  send_packet_(p);
}

void DaisyBlind::send_command_(uint8_t cmd, uint8_t percent) {
  Packet p;
  fill_packet_(p, TYPE_COMMAND, millis());
  p.cmd = cmd;
  p.cmd_percent = percent;
  send_packet_(p);
}

// ---------------------------------------------------------------- readouts

std::string DaisyBlind::peer_summary() {
  std::string out;
  int n = 0;
  for (const auto &p : peers_) {
    if (p.group != group_)
      continue;
    n++;
    char buf[112];
    const char *moving = (p.flags & FLAG_MOVING) ? " moving" : "";
    if (p.label[0] != 0) {
      snprintf(buf, sizeof(buf), "%s#%u %s (%s) %u%%%s", n > 1 ? " | " : "", p.order, p.label, p.name, p.percent,
               moving);
    } else {
      snprintf(buf, sizeof(buf), "%s#%u %s %u%%%s", n > 1 ? " | " : "", p.order, p.name, p.percent, moving);
    }
    out += buf;
    if (out.size() > 200) {
      out += " ...";
      break;
    }
  }
  char head[48];
  if (n == 0) {
    snprintf(head, sizeof(head), "No peers in group %u", group_);
    return head;
  }
  snprintf(head, sizeof(head), "%d peer%s: ", n, n == 1 ? "" : "s");
  return std::string(head) + out;
}

std::string DaisyBlind::sync_status() {
  if (homing_)
    return "Homing";
  const uint32_t now = millis();
  bool one_at_a_time = false;
  auto list = participants_(now, false, one_at_a_time);
  size_t my_index = 0;
  for (size_t i = 0; i < list.size(); i++) {
    if (list[i].peer == nullptr) {
      my_index = i;
      break;
    }
  }
  char buf[160];
  if (list.size() <= 1) {
    snprintf(buf, sizeof(buf), "Alone in group %u: moves unrestricted", group_);
    return buf;
  }
  const bool master_is_me = is_clock_master_(mac_, now);
  const char *mode = one_at_a_time ? "one at a time" : "taking turns";
  snprintf(buf, sizeof(buf), "Fires %u of %u (%s), firing order %u, clock from %s%s", (unsigned) (my_index + 1),
           (unsigned) list.size(), mode, order_, master_is_me ? "this device" : "peer",
           (!master_is_me && now - last_sync_ms_ > 5000) ? " (stale)" : "");
  return buf;
}

}  // namespace daisy_blind
}  // namespace esphome
