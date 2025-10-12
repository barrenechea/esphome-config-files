#include "openhaystack.h"
#include "esphome/core/log.h"

#ifdef USE_ESP32

#include <nvs.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <esp_bt_main.h>
#include <esp_bt.h>
#include <esp_err.h>
#include <esp_gap_ble_api.h>
#include <esp_idf_version.h>
#include <algorithm>
#include <array>
#include <inttypes.h>
#include <cstring>
#include "esphome/core/hal.h"

#ifdef USE_ARDUINO
#include <esp32-hal-bt.h>
#endif

namespace esphome {
namespace openhaystack {

static const char *const TAG = "openhaystack";

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
static esp_ble_adv_params_t ble_adv_params = {
    .adv_int_min = 0x0C80, // 2s
    .adv_int_max = 0x0C80, // 2s
    .adv_type = ADV_TYPE_NONCONN_IND,
    .own_addr_type = BLE_ADDR_TYPE_RANDOM,
    .peer_addr = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    .peer_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

void OpenHaystack::dump_config() {
  ESP_LOGCONFIG(TAG, "OpenHaystack:");
  ESP_LOGCONFIG(TAG,
                "  Bluetooth MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                this->random_address_[0],
                this->random_address_[1],
                this->random_address_[2],
                this->random_address_[3],
                this->random_address_[4],
                this->random_address_[5]
  );
  if (this->has_master_keys_) {
    ESP_LOGCONFIG(TAG, "  Master key derivation: enabled");
    ESP_LOGCONFIG(TAG, "  Derived key counter: %u", static_cast<unsigned>(this->derived_key_counter_));
  }
  if (!this->has_master_keys_) {
    ESP_LOGCONFIG(TAG, "  Master keys: not configured");
  }
  uint32_t interval_ms = static_cast<uint32_t>(this->adv_interval_min_ * 625U / 1000U);
  ESP_LOGCONFIG(TAG, "  Advertising interval: %" PRIu32 " ms", interval_ms);
}

void OpenHaystack::setup() {
  ESP_LOGCONFIG(TAG, "Setting up OpenHaystack device...");
  global_openhaystack = this;

  if (!this->has_master_keys_) {
    ESP_LOGE(TAG, "Master keys not configured");
    return;
  }

  if (!this->ensure_nvs_initialized_()) {
    ESP_LOGE(TAG, "Failed to initialize NVS");
    return;
  }

  if (!this->initialize_master_keys_()) {
    ESP_LOGE(TAG, "Failed to initialize master keys");
    return;
  }

  if (!this->apply_current_key_()) {
    ESP_LOGE(TAG, "Failed to apply initial advertising key");
    return;
  }

  ble_setup();
}

float OpenHaystack::get_setup_priority() const { return setup_priority::BLUETOOTH; }

void OpenHaystack::set_addr_from_key(uint8_t *addr, const uint8_t *public_key) {
  addr[0] = public_key[0] | 0b11000000;
  std::copy_n(&public_key[1], 5, &addr[1]);
}

void OpenHaystack::set_payload_from_key(uint8_t *payload, const uint8_t *public_key) {
  /* copy last 22 bytes */
  memcpy(&payload[OpenHaystack::ADV_PAYLOAD_KEY_OFFSET], &public_key[6], OpenHaystack::ADV_PAYLOAD_KEY_LENGTH);
  /* append two bits of public key */
  payload[OpenHaystack::ADV_PAYLOAD_KEY_TRAILING_BITS_INDEX] = public_key[0] >> 6;
}

void OpenHaystack::ble_setup() {
  if (global_openhaystack == nullptr) {
    ESP_LOGE(TAG, "OpenHaystack instance not initialized");
    return;
  }

  ble_adv_params.adv_int_min = global_openhaystack->adv_interval_min_;
  ble_adv_params.adv_int_max = global_openhaystack->adv_interval_max_;

  if (!global_openhaystack->ensure_nvs_initialized_()) {
    ESP_LOGE(TAG, "Failed to initialize NVS for BLE controller");
    return;
  }

  esp_err_t err;
  err = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    ESP_LOGE(TAG, "esp_bt_controller_mem_release failed: %s", esp_err_to_name(err));
    return;
  }

#ifdef USE_ARDUINO
  if (!btStart()) {
    ESP_LOGE(TAG, "btStart failed: %d", esp_bt_controller_get_status());
    return;
  }
#else
  if (esp_bt_controller_get_status() != ESP_BT_CONTROLLER_STATUS_ENABLED) {
    // start bt controller
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE) {
      esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
      err = esp_bt_controller_init(&cfg);
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_bt_controller_init failed: %s", esp_err_to_name(err));
        return;
      }
      uint32_t wait_ms = 0;
      while (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE) {
        delay(10);
        wait_ms += 10;
        if (wait_ms >= 1000) {
          ESP_LOGW(TAG, "BT controller still idle after waiting, continuing anyway");
          break;
        }
      }
    }
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED) {
      err = esp_bt_controller_enable(ESP_BT_MODE_BLE);
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_bt_controller_enable failed: %s", esp_err_to_name(err));
        return;
      }
    }
    if (esp_bt_controller_get_status() != ESP_BT_CONTROLLER_STATUS_ENABLED) {
      ESP_LOGE(TAG, "esp bt controller enable failed");
      return;
    }
  }
#endif

  err = esp_bluedroid_init();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_bluedroid_init failed: %s", esp_err_to_name(err));
    return;
  }
  err = esp_bluedroid_enable();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_bluedroid_enable failed: %s", esp_err_to_name(err));
    return;
  }

  err = esp_ble_gap_register_callback(OpenHaystack::gap_event_handler);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_ble_gap_register_callback failed: %s", esp_err_to_name(err));
    return;
  }

  global_openhaystack->refresh_advertisement_();
}

void OpenHaystack::gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  esp_err_t err;
  switch (event) {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT: {
      err = esp_ble_gap_start_advertising(&ble_adv_params);
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ble_gap_start_advertising failed: %s", esp_err_to_name(err));
      }
      break;
    }
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT: {
      esp_bt_status_t status = param->adv_start_cmpl.status;
      if (status != ESP_BT_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "BLE adv start failed: %s", esp_err_to_name(static_cast<esp_err_t>(status)));
      } else if (global_openhaystack != nullptr) {
        global_openhaystack->advertising_active_ = true;
      }
      break;
    }
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT: {
      esp_bt_status_t status = param->adv_stop_cmpl.status;
      if (status != ESP_BT_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "BLE adv stop failed: %s", esp_err_to_name(static_cast<esp_err_t>(status)));
      } else if (global_openhaystack != nullptr) {
        global_openhaystack->handle_advertising_stopped_();
      }
      break;
    }
    default:
      break;
  }
}

bool OpenHaystack::apply_current_key_() {
  if (!this->master_initialized_) {
    ESP_LOGE(TAG, "Master keys have not been initialized");
    return false;
  }

  set_addr_from_key(this->random_address_.data(), this->current_advertising_key_.data());
  set_payload_from_key(this->adv_data_.data(), this->current_advertising_key_.data());
  return true;
}

void OpenHaystack::refresh_advertisement_() {
  if (!this->apply_current_key_()) {
    ESP_LOGE(TAG, "Unable to refresh advertisement: no valid key");
    return;
  }
  configure_advertisement();
}

void OpenHaystack::handle_advertising_stopped_() {
  this->advertising_active_ = false;
  if (this->awaiting_refresh_after_stop_) {
    ESP_LOGD(TAG, "Derived advertising key applied (counter=%u); restarting advertising",
             static_cast<unsigned>(this->derived_key_counter_));
    this->awaiting_refresh_after_stop_ = false;
    this->refresh_advertisement_();
  } else {
    configure_advertisement();
  }
}

void OpenHaystack::schedule_key_rotation() {
  if (!this->master_initialized_) {
    ESP_LOGW(TAG, "Cannot rotate keys: master keys not initialized yet");
    return;
  }
  if (!this->derive_next_master_key_()) {
    ESP_LOGE(TAG, "Failed to derive next advertising key");
    return;
  }
  if (this->advertising_active_) {
    this->awaiting_refresh_after_stop_ = true;
    esp_err_t err = esp_ble_gap_stop_advertising();
    if (err == ESP_OK) {
      return;
    }
    if (err == ESP_ERR_INVALID_STATE) {
      this->advertising_active_ = false;
      this->awaiting_refresh_after_stop_ = false;
      this->refresh_advertisement_();
      return;
    }
    ESP_LOGE(TAG, "esp_ble_gap_stop_advertising failed: %s", esp_err_to_name(err));
    this->awaiting_refresh_after_stop_ = false;
  } else {
    this->refresh_advertisement_();
  }
}

void OpenHaystack::set_master_keys(const std::array<uint8_t, MASTER_PRIVATE_KEY_SIZE> &master_private_key,
                                   const std::array<uint8_t, MASTER_SYMMETRIC_KEY_SIZE> &master_symmetric_key) {
  this->master_private_key_ = master_private_key;
  this->master_symmetric_key_ = master_symmetric_key;
  this->has_master_keys_ = true;
  this->master_initialized_ = false;
  this->derived_key_counter_ = 0;
  this->current_private_key_.fill(0);
  this->current_symmetric_key_.fill(0);
  this->current_advertising_key_.fill(0);
}

bool OpenHaystack::initialize_master_keys_() {
  if (!this->has_master_keys_)
    return false;

  this->current_private_key_ = this->master_private_key_;
  this->current_symmetric_key_ = this->master_symmetric_key_;
  this->derived_key_counter_ = 0;

  AdvertisingKey advertising_key{};
  bool restored_state = this->load_persisted_state_();

  if (!this->derive_public_key_from_private_(this->current_private_key_, advertising_key)) {
    if (restored_state) {
      ESP_LOGW(TAG, "Persisted private key invalid, clearing stored state");
      this->clear_persisted_state_();
      this->current_private_key_ = this->master_private_key_;
      this->current_symmetric_key_ = this->master_symmetric_key_;
      this->derived_key_counter_ = 0;
      restored_state = false;
    }
    if (!this->derive_public_key_from_private_(this->current_private_key_, advertising_key)) {
      return false;
    }
  }

  this->current_advertising_key_ = advertising_key;
  this->master_initialized_ = true;

  if (restored_state) {
    ESP_LOGI(TAG, "Restored derived key state from NVS (counter=%u)", static_cast<unsigned>(this->derived_key_counter_));
  }

  if (!this->save_persisted_state_()) {
    ESP_LOGW(TAG, "Unable to persist OpenHaystack state after initialization");
  }

  return true;
}

bool OpenHaystack::derive_next_master_key_() {
  if (!this->has_master_keys_)
    return false;

  std::array<uint8_t, MASTER_SYMMETRIC_KEY_SIZE> next_symmetric{};
  if (!kdf_(this->current_symmetric_key_.data(), this->current_symmetric_key_.size(), "update", next_symmetric.data(), next_symmetric.size())) {
    return false;
  }

  std::array<uint8_t, DERIVED_SHARED_DATA_SIZE> shared_data{};
  if (!kdf_(next_symmetric.data(), next_symmetric.size(), "diversify", shared_data.data(), shared_data.size())) {
    shared_data.fill(0);
    return false;
  }

  std::array<uint8_t, MASTER_PRIVATE_KEY_SIZE> derived_private{};
  if (!this->calculate_derived_private_key_(shared_data, derived_private)) {
    shared_data.fill(0);
    derived_private.fill(0);
    return false;
  }

  AdvertisingKey derived_public{};
  if (!this->derive_public_key_from_private_(derived_private, derived_public)) {
    shared_data.fill(0);
    derived_private.fill(0);
    return false;
  }

  this->current_symmetric_key_ = next_symmetric;
  this->current_private_key_ = derived_private;
  this->current_advertising_key_ = derived_public;
  this->derived_key_counter_ += 1;

  if (!this->save_persisted_state_()) {
    ESP_LOGW(TAG, "Failed to persist derived key state");
  }

  shared_data.fill(0);
  derived_private.fill(0);
  next_symmetric.fill(0);

  return true;
}

bool OpenHaystack::derive_public_key_from_private_(const std::array<uint8_t, MASTER_PRIVATE_KEY_SIZE> &private_key,
                                                   AdvertisingKey &advertising_key_out) {
  advertising_key_out.fill(0);
  return this->crypto_context_.derive_public_key(private_key.data(), private_key.size(),
                                                 advertising_key_out.data(), advertising_key_out.size());
}

bool OpenHaystack::calculate_derived_private_key_(const std::array<uint8_t, DERIVED_SHARED_DATA_SIZE> &shared_data,
                                                  std::array<uint8_t, MASTER_PRIVATE_KEY_SIZE> &out_private_key) {
  return this->crypto_context_.calculate_derived_private_key(shared_data.data(),
                                                             shared_data.size(),
                                                             this->master_private_key_.data(),
                                                             this->master_private_key_.size(),
                                                             out_private_key.data(),
                                                             out_private_key.size());
}

bool OpenHaystack::ensure_nvs_initialized_() {
  static bool nvs_ready = false;
  if (nvs_ready)
    return true;

  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_LOGW(TAG, "nvs_flash_init reported %s, erasing NVS partition", esp_err_to_name(err));
    esp_err_t deinit_err = nvs_flash_deinit();
    if (deinit_err != ESP_OK && deinit_err != ESP_ERR_NVS_NOT_INITIALIZED) {
      ESP_LOGE(TAG, "nvs_flash_deinit failed: %s", esp_err_to_name(deinit_err));
      return false;
    }
    err = nvs_flash_erase();
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "nvs_flash_erase failed: %s", esp_err_to_name(err));
      return false;
    }
    err = nvs_flash_init();
  }

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "nvs_flash_init failed: %s", esp_err_to_name(err));
    return false;
  }

  nvs_ready = true;
  return true;
}

bool OpenHaystack::compute_master_key_digest_(std::array<uint8_t, MASTER_KEY_DIGEST_SIZE> &digest_out) const {
  return CryptoContext::compute_master_key_digest(this->master_private_key_.data(),
                                                  this->master_private_key_.size(),
                                                  this->master_symmetric_key_.data(),
                                                  this->master_symmetric_key_.size(),
                                                  digest_out.data(),
                                                  digest_out.size());
}

bool OpenHaystack::save_persisted_state_() {
  std::array<uint8_t, MASTER_KEY_DIGEST_SIZE> master_digest{};
  if (!this->compute_master_key_digest_(master_digest)) {
    ESP_LOGE(TAG, "Failed to compute master key digest for persistence");
    return false;
  }

  nvs_handle_t handle = 0;
  esp_err_t err = nvs_open(OpenHaystack::NVS_NAMESPACE, NVS_READWRITE, &handle);
  if (err == ESP_ERR_NVS_NOT_INITIALIZED) {
    if (!this->ensure_nvs_initialized_())
      return false;
    err = nvs_open(OpenHaystack::NVS_NAMESPACE, NVS_READWRITE, &handle);
  }
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(err));
    master_digest.fill(0);
    return false;
  }

  bool ok = true;

  do {
    err = nvs_set_blob(handle, OpenHaystack::NVS_KEY_MASTER_DIGEST, master_digest.data(), master_digest.size());
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "nvs_set_blob(master_digest) failed: %s", esp_err_to_name(err));
      ok = false;
      break;
    }
    err = nvs_set_blob(handle, OpenHaystack::NVS_KEY_CURRENT_PRIV, this->current_private_key_.data(), this->current_private_key_.size());
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "nvs_set_blob(current_private) failed: %s", esp_err_to_name(err));
      ok = false;
      break;
    }
    err = nvs_set_blob(handle, OpenHaystack::NVS_KEY_CURRENT_SYM, this->current_symmetric_key_.data(), this->current_symmetric_key_.size());
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "nvs_set_blob(current_symmetric) failed: %s", esp_err_to_name(err));
      ok = false;
      break;
    }
    err = nvs_set_u32(handle, OpenHaystack::NVS_KEY_COUNTER, this->derived_key_counter_);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "nvs_set_u32(counter) failed: %s", esp_err_to_name(err));
      ok = false;
      break;
    }
    err = nvs_commit(handle);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "nvs_commit failed: %s", esp_err_to_name(err));
      ok = false;
    }
  } while (false);

  nvs_close(handle);
  master_digest.fill(0);

  return ok;
}

void OpenHaystack::clear_persisted_state_() {
  nvs_handle_t handle = 0;
  esp_err_t err = nvs_open(OpenHaystack::NVS_NAMESPACE, NVS_READWRITE, &handle);
  if (err != ESP_OK) {
    return;
  }

  if (nvs_erase_all(handle) != ESP_OK) {
    nvs_close(handle);
    return;
  }
  nvs_commit(handle);
  nvs_close(handle);
}

bool OpenHaystack::load_persisted_state_() {
  if (!this->ensure_nvs_initialized_())
    return false;

  nvs_handle_t handle = 0;
  esp_err_t err = nvs_open(OpenHaystack::NVS_NAMESPACE, NVS_READONLY, &handle);
  if (err != ESP_OK) {
    return false;
  }

  bool ok = true;
  bool digest_mismatch = false;

  std::array<uint8_t, MASTER_KEY_DIGEST_SIZE> stored_digest{};
  size_t digest_len = stored_digest.size();
  err = nvs_get_blob(handle, OpenHaystack::NVS_KEY_MASTER_DIGEST, stored_digest.data(), &digest_len);
  if (err != ESP_OK || digest_len != stored_digest.size()) {
    ok = false;
  }

  std::array<uint8_t, MASTER_KEY_DIGEST_SIZE> current_digest{};
  if (ok && !this->compute_master_key_digest_(current_digest)) {
    ok = false;
  }

  if (ok && stored_digest != current_digest) {
    digest_mismatch = true;
    ok = false;
  }

  if (ok) {
    size_t priv_len = this->current_private_key_.size();
    err = nvs_get_blob(handle, OpenHaystack::NVS_KEY_CURRENT_PRIV, this->current_private_key_.data(), &priv_len);
    if (err != ESP_OK || priv_len != this->current_private_key_.size()) {
      ok = false;
    }
  }

  if (ok) {
    size_t sym_len = this->current_symmetric_key_.size();
    err = nvs_get_blob(handle, OpenHaystack::NVS_KEY_CURRENT_SYM, this->current_symmetric_key_.data(), &sym_len);
    if (err != ESP_OK || sym_len != this->current_symmetric_key_.size()) {
      ok = false;
    }
  }

  if (ok) {
    uint32_t counter = 0;
    err = nvs_get_u32(handle, OpenHaystack::NVS_KEY_COUNTER, &counter);
    if (err != ESP_OK) {
      ok = false;
    } else {
      this->derived_key_counter_ = counter;
    }
  }

  nvs_close(handle);

  if (!ok) {
    if (digest_mismatch) {
      ESP_LOGI(TAG, "Master keys changed, clearing persisted OpenHaystack state");
      this->clear_persisted_state_();
    }
    this->current_private_key_ = this->master_private_key_;
    this->current_symmetric_key_ = this->master_symmetric_key_;
    this->derived_key_counter_ = 0;
  }

  stored_digest.fill(0);
  current_digest.fill(0);

  return ok;
}

bool OpenHaystack::kdf_(const uint8_t *input,
                        size_t input_length,
                        const char *label,
                        uint8_t *output,
                        size_t output_length) {
  return CryptoContext::kdf(input, input_length, label, output, output_length);
}

void OpenHaystack::configure_advertisement() {
  if (global_openhaystack == nullptr)
    return;

  esp_err_t err = esp_ble_gap_set_rand_addr(global_openhaystack->random_address_.data());
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_ble_gap_set_rand_addr failed: %s", esp_err_to_name(err));
    return;
  }

  err = esp_ble_gap_config_adv_data_raw(global_openhaystack->adv_data_.data(), global_openhaystack->adv_data_.size());
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_ble_gap_config_adv_data_raw failed: %s", esp_err_to_name(err));
  }
}

OpenHaystack *global_openhaystack = nullptr;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

}  // namespace openhaystack
}  // namespace esphome

#endif
