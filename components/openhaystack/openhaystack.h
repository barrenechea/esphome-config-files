#pragma once

#include "esphome/core/component.h"

#include <array>

#ifdef USE_ESP32

#include <esp_gap_ble_api.h>

#include "crypto_context.h"

namespace esphome {
namespace openhaystack {

class OpenHaystack : public Component {
 public:
  static constexpr size_t ADVERTISING_KEY_SIZE = 28;
  static constexpr size_t RANDOM_ADDRESS_SIZE = ESP_BD_ADDR_LEN;
  static constexpr size_t ADV_PAYLOAD_SIZE = 31;
  static constexpr size_t ADV_PAYLOAD_KEY_OFFSET = 7;
  static constexpr size_t ADV_PAYLOAD_KEY_LENGTH = 22;
  static constexpr size_t ADV_PAYLOAD_KEY_TRAILING_BITS_INDEX = 29;
  static constexpr size_t MASTER_PRIVATE_KEY_SIZE = 28;
  static constexpr size_t MASTER_SYMMETRIC_KEY_SIZE = 32;
  static constexpr size_t ANTI_TRACKING_COMPONENT_SIZE = 36;
  static constexpr size_t DERIVED_SHARED_DATA_SIZE = ANTI_TRACKING_COMPONENT_SIZE * 2;
  static constexpr size_t MASTER_KEY_DIGEST_SIZE = 16;
  static constexpr const char *const NVS_NAMESPACE = "openhstk";
  static constexpr const char *const NVS_KEY_MASTER_DIGEST = "master_hash";
  static constexpr const char *const NVS_KEY_CURRENT_PRIV = "curr_priv";
  static constexpr const char *const NVS_KEY_CURRENT_SYM = "curr_sym";
  static constexpr const char *const NVS_KEY_COUNTER = "counter";
  using AdvertisingKey = std::array<uint8_t, ADVERTISING_KEY_SIZE>;

  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;

  void set_master_keys(const std::array<uint8_t, MASTER_PRIVATE_KEY_SIZE> &master_private_key,
                       const std::array<uint8_t, MASTER_SYMMETRIC_KEY_SIZE> &master_symmetric_key);

  static constexpr uint32_t ROTATION_INTERVAL_MS = 15U * 60U * 1000U;
 protected:
  static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
  static void set_addr_from_key(uint8_t *addr, const uint8_t *public_key);
  static void set_payload_from_key(uint8_t *payload, const uint8_t *public_key);
  static void ble_setup();
  static void configure_advertisement();

  bool apply_current_key_();
  void refresh_advertisement_();
  void handle_advertising_stopped_();
  void schedule_key_rotation_();
  bool initialize_master_keys_();
  bool derive_next_master_key_();
  bool derive_public_key_from_private_(const std::array<uint8_t, MASTER_PRIVATE_KEY_SIZE> &private_key,
                                       AdvertisingKey &advertising_key_out);
  bool calculate_derived_private_key_(const std::array<uint8_t, DERIVED_SHARED_DATA_SIZE> &shared_data,
                                      std::array<uint8_t, MASTER_PRIVATE_KEY_SIZE> &out_private_key);
  bool ensure_nvs_initialized_();
  bool save_persisted_state_();
  bool load_persisted_state_();
  void clear_persisted_state_();
  bool compute_master_key_digest_(std::array<uint8_t, MASTER_KEY_DIGEST_SIZE> &digest_out) const;
  static bool kdf_(const uint8_t *input,
                   size_t input_length,
                   const char *label,
                   uint8_t *output,
                   size_t output_length);

  AdvertisingKey current_advertising_key_{};
  std::array<uint8_t, RANDOM_ADDRESS_SIZE> random_address_{{0xFF, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF}};
  std::array<uint8_t, ADV_PAYLOAD_SIZE> adv_data_{{
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
  }};

  bool advertising_active_ = false;
  bool awaiting_refresh_after_stop_ = false;
  bool has_master_keys_ = false;
  bool master_initialized_ = false;
  std::array<uint8_t, MASTER_PRIVATE_KEY_SIZE> master_private_key_{};
  std::array<uint8_t, MASTER_SYMMETRIC_KEY_SIZE> master_symmetric_key_{};
  std::array<uint8_t, MASTER_PRIVATE_KEY_SIZE> current_private_key_{};
  std::array<uint8_t, MASTER_SYMMETRIC_KEY_SIZE> current_symmetric_key_{};
  uint32_t derived_key_counter_ = 0;
  uint16_t adv_interval_min_ = 0x0C80;
  uint16_t adv_interval_max_ = 0x0C80;
  CryptoContext crypto_context_;
};

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
extern OpenHaystack *global_openhaystack;

}  // namespace openhaystack
}  // namespace esphome

#endif
