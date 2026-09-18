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

// One UDP datagram, broadcast on the LAN.  Same layout for beacons and commands.
struct __attribute__((packed)) Packet {
  uint32_t magic;
  uint8_t version;
  uint8_t type;        // 1 = beacon, 2 = command
  uint8_t mac[6];
  uint8_t group;
  uint8_t slot_mode;   // 0 = auto, 1..4 = fixed slot
  uint8_t moving;      // 1 while the motor has work to do
  uint8_t percent;     // 0..100 open, 255 = unknown
  uint8_t cmd;         // commands only: 0 = go to percent, 1 = home, 2 = stop
  uint8_t cmd_percent; // commands only
  uint32_t t_sync;     // sender's synchronised millisecond clock
  int32_t position;    // steps from home
  char name[NAME_LEN]; // hostname, e.g. blind-a1b2c3
  char label[NAME_LEN];// user label
};

struct Peer {
  uint8_t mac[6];
  uint8_t group;
  uint8_t slot_mode;
  uint8_t moving;
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
  void set_homing_steps(int32_t v) { homing_steps_ = v < 1 ? 1 : v; }
  void set_speed(int32_t sps) { speed_sps_ = sps < 1 ? 1 : (sps > 5000 ? 5000 : sps); }
  void set_slot_ms(int32_t ms) { slot_ms_ = ms < 50 ? 50 : ms; }
  void set_group(int32_t g) { group_ = (uint8_t) (g < 0 ? 0 : (g > 255 ? 255 : g)); }
  void set_slot_mode(int32_t mode) { slot_mode_ = (uint8_t) (mode < 0 ? 0 : (mode > 4 ? 4 : mode)); }
  void set_invert_direction(bool v) { invert_direction_ = v; }
  void set_sync_enabled(bool v) { sync_enabled_ = v; }
  void set_sleep_between_slots(bool v) { sleep_between_slots_ = v; }
  void set_home_on_boot(bool v) { home_on_boot_ = v; }
  void set_group_control(bool v) { group_control_ = v; }
  void set_label(const std::string &label) { label_ = label; }

  // --- actions ---
  void home();
  void home_group();
  void stop();
  void move_to_steps(int32_t steps);

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
  bool wants_move_() const { return homing_ || target_ != position_; }
  void run_motor_(uint32_t now);
  void set_driver_awake_(bool awake, uint32_t now);
  void apply_target_pct_(float pct);
  void start_home_();
  void finish_move_(uint32_t now);
  float pos_pct_() const;
  void refresh_position_();
  void publish_(bool save, uint32_t now);

  // sync / network
  struct Participant {
    const uint8_t *mac;
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
  std::vector<Participant> participants_(uint32_t now, bool movers_only);
  bool slot_gate_(uint32_t now);
  bool is_clock_master_(const uint8_t *mac, uint32_t now);

  GPIOPin *step_pin_{nullptr};
  GPIOPin *dir_pin_{nullptr};
  GPIOPin *sleep_pin_{nullptr};
  uint16_t port_{44820};

  // settings
  int32_t open_steps_{750};
  int32_t closed_steps_{0};
  int32_t homing_steps_{1500};
  int32_t speed_sps_{250};
  int32_t slot_ms_{250};
  uint8_t group_{1};
  uint8_t slot_mode_{0};
  bool invert_direction_{false};
  bool sync_enabled_{true};
  bool sleep_between_slots_{true};
  bool home_on_boot_{true};
  bool group_control_{false};
  std::string label_;

  // motion state
  int32_t position_{0};
  int32_t target_{0};
  bool homing_{false};
  int32_t homing_remaining_{0};
  bool driver_awake_{false};
  bool last_dir_opening_{true};
  bool dirty_{false};
  uint32_t wake_until_{0};
  uint32_t last_step_us_{0};
  uint32_t last_step_ms_{0};
  uint32_t last_publish_ms_{0};
  HighFrequencyLoopRequester high_freq_;
  ESPPreferenceObject pref_;

  // boot
  uint32_t boot_ms_{0};
  bool cold_boot_{false};
  bool boot_home_pending_{false};

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
