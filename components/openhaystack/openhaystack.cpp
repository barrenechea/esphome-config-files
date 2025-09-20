#include "openhaystack.h"
#include "esphome/core/log.h"

#ifdef USE_ESP32

#include <nvs_flash.h>
#include <esp_bt_main.h>
#include <esp_bt.h>
#include <esp_err.h>
#include <esp_gap_ble_api.h>
#include <algorithm>
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
  if (!this->advertising_keys_.empty()) {
    ESP_LOGCONFIG(TAG, "  Advertising keys configured: %zu", this->advertising_keys_.size());
    if (this->advertising_keys_.size() > 1) {
      if (this->rotation_interval_ms_ > 0) {
        ESP_LOGCONFIG(TAG, "  Key rotation interval: %ums", this->rotation_interval_ms_);
      } else {
        ESP_LOGCONFIG(TAG, "  Key rotation interval: disabled");
      }
    }
  } else {
    ESP_LOGCONFIG(TAG, "  Advertising key: not configured");
  }
}

void OpenHaystack::setup() {
  ESP_LOGCONFIG(TAG, "Setting up OpenHaystack device...");
  global_openhaystack = this;

  if (this->advertising_keys_.empty()) {
    ESP_LOGE(TAG, "No advertising keys configured");
    return;
  }

  this->apply_current_key_();
  ble_setup();

  if (this->rotation_interval_ms_ > 0) {
    if (this->advertising_keys_.size() > 1) {
      this->set_interval("openhaystack_key_rotation", this->rotation_interval_ms_, [this]() { this->schedule_key_rotation_(); });
    } else {
      ESP_LOGW(TAG, "Key rotation requested but only one key provided. Rotation disabled.");
    }
  }
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

  // Initialize non-volatile storage for the bluetooth controller
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_LOGW(TAG, "nvs_flash_init reported %s, erasing NVS partition", esp_err_to_name(err));
    esp_err_t deinit_err = nvs_flash_deinit();
    if (deinit_err != ESP_OK && deinit_err != ESP_ERR_NVS_NOT_INITIALIZED) {
      ESP_LOGE(TAG, "nvs_flash_deinit failed: %s", esp_err_to_name(deinit_err));
      return;
    }
    err = nvs_flash_erase();
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "nvs_flash_erase failed: %s", esp_err_to_name(err));
      return;
    }
    err = nvs_flash_init();
  }
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "nvs_flash_init failed: %s", esp_err_to_name(err));
    return;
  }

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
        ESP_LOGD(TAG, "BLE advertising started");
      }
      break;
    }
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT: {
      esp_bt_status_t status = param->adv_stop_cmpl.status;
      if (status != ESP_BT_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "BLE adv stop failed: %s", esp_err_to_name(static_cast<esp_err_t>(status)));
      } else if (global_openhaystack != nullptr) {
        global_openhaystack->handle_advertising_stopped_();
        ESP_LOGD(TAG, "BLE stopped advertising successfully");
      }
      break;
    }
    default:
      break;
  }
}

void OpenHaystack::apply_current_key_() {
  if (this->advertising_keys_.empty())
    return;
  const auto &key = this->advertising_keys_[this->current_key_index_];
  set_addr_from_key(this->random_address_.data(), key.data());
  set_payload_from_key(this->adv_data_.data(), key.data());
}

void OpenHaystack::refresh_advertisement_() {
  this->apply_current_key_();
  configure_advertisement();
}

void OpenHaystack::handle_advertising_stopped_() {
  this->advertising_active_ = false;
  if (this->has_pending_key_) {
    this->current_key_index_ = this->pending_key_index_;
    this->has_pending_key_ = false;
    this->refresh_advertisement_();
  } else {
    // Resume advertising with current key if it was stopped unexpectedly.
    configure_advertisement();
  }
}

void OpenHaystack::schedule_key_rotation_() {
  if (this->advertising_keys_.size() <= 1)
    return;

  if (this->has_pending_key_) {
    ESP_LOGD(TAG, "Key rotation already pending");
    return;
  }

  size_t next_index = (this->current_key_index_ + 1) % this->advertising_keys_.size();
  this->pending_key_index_ = next_index;
  this->has_pending_key_ = true;

  if (this->advertising_active_) {
    esp_err_t err = esp_ble_gap_stop_advertising();
    if (err == ESP_OK) {
      ESP_LOGD(TAG, "Stopping advertising to rotate key");
      return;
    }
    if (err == ESP_ERR_INVALID_STATE) {
      this->advertising_active_ = false;
      this->handle_advertising_stopped_();
      return;
    }
    ESP_LOGE(TAG, "esp_ble_gap_stop_advertising failed: %s", esp_err_to_name(err));
    this->has_pending_key_ = false;
  } else {
    this->handle_advertising_stopped_();
  }
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
