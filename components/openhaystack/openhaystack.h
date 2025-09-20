#pragma once

#include "esphome/core/component.h"

#include <array>
#include <vector>

#ifdef USE_ESP32

#include <esp_gap_ble_api.h>

namespace esphome {
namespace openhaystack {

class OpenHaystack : public Component {
 public:
  explicit OpenHaystack(const std::array<uint8_t, 28> &advertising_key) {
    this->advertising_keys_.push_back(advertising_key);
  }

  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;

  void add_additional_key(const std::array<uint8_t, 28> &advertising_key) {
    this->advertising_keys_.push_back(advertising_key);
  }

  void set_rotation_interval(uint32_t interval_ms) { this->rotation_interval_ms_ = interval_ms; }

 protected:
  static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
  static void set_addr_from_key(esp_bd_addr_t addr, const uint8_t *public_key);
  static void set_payload_from_key(uint8_t *payload, const uint8_t *public_key);
  static void ble_setup();
  static void configure_advertisement();

  void apply_current_key_();
  void refresh_advertisement_();
  void handle_advertising_stopped_();
  void schedule_key_rotation_();

  std::vector<std::array<uint8_t, 28>> advertising_keys_;
  esp_bd_addr_t random_address_ = {0xFF, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
  uint8_t adv_data_[31] = {
    0x1e, /* Length (30) */
    0xff, /* Manufacturer Specific Data (type 0xff) */
    0x4c, 0x00, /* Company ID (Apple) */
    0x12, 0x19, /* Offline Finding type and length */
    0x00, /* State */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, /* First two bits */
    0x00, /* Hint (0x00) */
  };

  size_t current_key_index_ = 0;
  size_t pending_key_index_ = 0;
  bool has_pending_key_ = false;
  bool advertising_active_ = false;
  uint32_t rotation_interval_ms_ = 0;
};

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
extern OpenHaystack *global_openhaystack;

}  // namespace openhaystack
}  // namespace esphome

#endif
