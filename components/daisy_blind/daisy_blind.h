#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/preferences.h"
#include "esphome/components/cover/cover.h"

#ifdef USE_ESP8266
#include <ESP8266WiFi.h>
#else
#include <WiFi.h>
#endif
#include <WiFiUdp.h>

namespace esphome {
namespace daisy_blind {

static const size_t NAME_LEN = 24;

// Beacon / command datagram, broadcast on the LAN.
struct __attribute__((packed)) Packet {
  uint32_t magic;
  uint8_t version;
  uint8_t type;         // 1 = beacon, 2 = command
  uint8_t mac[6];
  uint8_t group;
  uint8_t order;        // firing order 1..8
  uint8_t flags;        // bit0 moving, bit1 started stepping, bit2 wants one-at-a-time
  uint8_t percent;      // 0..100 open
  uint8_t cmd;          // commands: 0 = go to percent, 2 = stop
  uint8_t cmd_percent;
  uint32_t t_sync;      // sender's synchronised millisecond clock
  int32_t position;     // steps
  char name[NAME_LEN];  // hostname
  char label[NAME_LEN]; // user label
};

struct Peer {
  uint8_t mac[6];
  uint8_t group;
  uint8_t order;
  uint8_t flags;
  uint8_t percent;
  uint32_t last_seen;
  char name[NAME_LEN];
  char label[NAME_LEN];
};

class DaisyBlind : public cover::Cover, public Component {
 public:
  // --- wiring (from YAML) ---
  void set_step_pin(GPIOPin *pin) { step_pin_ = pin; }
  void set_dir_pin(GPIOPin *pin) { dir_pin_ = pin; }
  void set_sleep_pin(GPIOPin *pin) { sleep_pin_ = pin; }
  void set_port(uint16_t port) { port_ = port; }

  // --- runtime settings (from template entities) ---
  void set_open_steps(int32_t v) { open_steps_ = v; refresh_position_(); }
  void set_closed_steps(int32_t v) { closed_steps_ = v; refresh_position_(); }
  void set_speed(int32_t sps) { speed_sps_ = sps < 1 ? 1 : (sps > 5000 ? 5000 : sps); }
  void set_group(int32_t g) { group_ = (uint8_t) (g < 1 ? 1 : (g > 255 ? 255 : g)); }
  void set_order(int32_t o) { order_ = (uint8_t) (o < 1 ? 1 : (o > 8 ? 8 : o)); }
  void set_one_at_a_time(bool v) { one_at_a_time_ = v; }
  void set_invert_direction(bool v) { invert_direction_ = v; }
  void set_group_control(bool v) { group_control_ = v; }
  void set_label(const std::string &label) { label_ = label; }

  // --- actions ---
  void stop();
  void move_to_steps(int32_t steps);
  void nudge(int32_t delta);

  // --- readouts ---
  int32_t get_position_steps() const { return position_; }
  bool is_moving() const { return wants_move_(); }
  std::string peer_summary();
  std::string sync_status();

  // --- ESPHome plumbing ---
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }
  cover::CoverTraits get_traits() override;

 protected:
  void control(const cover::CoverCall &call) override;

  // motion
  bool wants_move_() const { return target_ != position_; }
  void run_motor_(uint32_t now);
  void set_driver_awake_(bool awake, uint32_t now);
  void begin_move_(int32_t target, uint32_t now);
  void apply_target_pct_(float pct);
  void finish_move_(uint32_t now);
  void save_position_();
  float pos_pct_() const;
  void refresh_position_();
  void publish_(bool save, uint32_t now);

  // coordination / network
  struct Participant {
    const uint8_t *mac;
    uint8_t order;
    bool started;
    bool one_at_a_time;
    const Peer *peer;  // nullptr == this device
  };
  void receive_packets_(uint32_t now);
  void expire_peers_(uint32_t now);
  Peer *upsert_peer_(const Packet &p, uint32_t now);
  void handle_command_(const Packet &p);
  void send_beacon_(uint32_t now);
  void send_command_(uint8_t cmd, uint8_t percent);
  void fill_packet_(Packet &p, uint8_t type, uint32_t now);
  void send_packet_(const Packet &p);
  std::vector<Participant> participants_(uint32_t now, bool movers_only, bool &one_at_a_time);
  bool may_step_(uint32_t now);
  bool is_clock_master_(const uint8_t *mac, uint32_t now);

  GPIOPin *step_pin_{nullptr};
  GPIOPin *dir_pin_{nullptr};
  GPIOPin *sleep_pin_{nullptr};
  uint16_t port_{44820};

  // settings
  int32_t open_steps_{750};
  int32_t closed_steps_{0};
  int32_t speed_sps_{250};
  uint8_t group_{1};
  uint8_t order_{1};
  bool one_at_a_time_{false};
  bool invert_direction_{false};
  bool group_control_{false};
  std::string label_;

  // motion state
  int32_t position_{0};
  int32_t target_{0};
  bool started_{false};
  bool driver_awake_{false};
  bool last_dir_opening_{true};
  bool dirty_{false};
  uint32_t hold_until_{0};
  uint32_t wake_until_{0};
  uint32_t last_step_us_{0};
  uint32_t last_step_ms_{0};
  uint32_t last_publish_ms_{0};
  uint32_t last_save_ms_{0};
  HighFrequencyLoopRequester high_freq_;
  ESPPreferenceObject pref_;

  // network state
  WiFiUDP udp_;
  bool udp_started_{false};
  uint8_t mac_[6]{};
  uint32_t sync_offset_{0};
  uint32_t last_beacon_ms_{0};
  uint32_t last_sync_ms_{0};
  std::vector<Peer> peers_;
};

}  // namespace daisy_blind
}  // namespace esphome
