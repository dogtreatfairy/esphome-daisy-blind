#include "daisy_blind.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "esphome/core/application.h"
#include "esphome/core/log.h"

namespace esphome {
namespace daisy_blind {

static const char *const TAG = "daisy_blind";

static const uint32_t PACKET_MAGIC = 0x4442534E;  // "DBSN"
static const uint8_t PROTO_VERSION = 2;
static const uint8_t TYPE_BEACON = 1;
static const uint8_t TYPE_COMMAND = 2;
static const uint8_t CMD_GOTO = 0;
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
static const uint32_t SAVE_INTERVAL_MS = 1000;
static const int32_t MAX_MANUAL_STEPS = 50000;

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

  pref_ = global_preferences->make_preference<int32_t>(this->get_object_id_hash() ^ 0x44424C31);
  int32_t saved = 0;
  if (pref_.load(&saved)) {
    position_ = saved;
    ESP_LOGI(TAG, "Restored position %d steps", (int) position_);
  }
  target_ = position_;

  this->position = pos_pct_();
  this->current_operation = COVER_OPERATION_IDLE;
  this->publish_state(false);
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

  run_motor_(now);
}

void DaisyBlind::dump_config() {
  LOG_COVER("", "Daisy Blind", this);
  LOG_PIN("  Step pin: ", step_pin_);
  LOG_PIN("  Dir pin: ", dir_pin_);
  LOG_PIN("  Sleep/enable pin: ", sleep_pin_);
  ESP_LOGCONFIG(TAG, "  Coordination UDP port: %u", port_);
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
  target_ = position_;
  this->current_operation = COVER_OPERATION_IDLE;
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

// ---------------------------------------------------------------- motor

void DaisyBlind::set_driver_awake_(bool awake, uint32_t now) {
  if (awake == driver_awake_)
    return;
  driver_awake_ = awake;
  if (sleep_pin_ != nullptr)
    sleep_pin_->digital_write(awake);
  if (awake)
    wake_until_ = now + DRIVER_WAKE_MS;
}

void DaisyBlind::save_position_() {
  pref_.save(&position_);
  dirty_ = false;
  last_save_ms_ = millis();
}

void DaisyBlind::finish_move_(uint32_t now) {
  set_driver_awake_(false, now);
  started_ = false;
  if (dirty_) {
    save_position_();
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

  // Persist progress during long moves so a power cut loses at most a second.
  if (dirty_ && now - last_save_ms_ >= SAVE_INTERVAL_MS)
    save_position_();

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

  const uint32_t interval_us = 1000000UL / (uint32_t) speed_sps_;
  const uint32_t now_us = micros();
  if (now_us - last_step_us_ < interval_us)
    return;
  // Don't accumulate a backlog if the loop stalled; just step now.
  if (now_us - last_step_us_ > 2 * interval_us) {
    last_step_us_ = now_us;
  } else {
    last_step_us_ += interval_us;
  }

  const bool opening = target_ > position_;
  if (opening != last_dir_opening_ || last_step_ms_ == 0) {
    dir_pin_->digital_write(opening != invert_direction_);
    last_dir_opening_ = opening;
    delayMicroseconds(5);
  }

  step_pin_->digital_write(true);
  delayMicroseconds(5);
  step_pin_->digital_write(false);
  last_step_ms_ = now;
  started_ = true;

  position_ += opening ? 1 : -1;
  dirty_ = true;

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
