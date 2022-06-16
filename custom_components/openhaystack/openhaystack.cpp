#include "openhaystack.h"
#include "esphome/core/log.h"

#ifdef USE_ESP32

#include <nvs_flash.h>
#include <freertos/FreeRTOS.h>
#include <esp_bt_main.h>
#include <esp_bt.h>
#include <freertos/task.h>
#include <esp_gap_ble_api.h>
#include <cstring>
#include "esphome/core/hal.h"

#ifdef USE_ARDUINO
#include <esp32-hal-bt.h>
#endif

namespace esphome {
namespace openhaystack {

static const char *const TAG = "openhaystack";

/** Random device address */
static esp_bd_addr_t rnd_addr = { 0xFF, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF };

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
static esp_ble_adv_params_t ble_adv_params = {
    .adv_int_min = 0x0640, // 1s
    .adv_int_max = 0x0C80, // 2s
    .adv_type = ADV_TYPE_NONCONN_IND,
    .own_addr_type = BLE_ADDR_TYPE_RANDOM,
    .peer_addr = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    .peer_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

/** Advertisement payload */
static uint8_t ble_adv_data[31] = {
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

#define ENDIAN_CHANGE_U16(x) ((((x) &0xFF00) >> 8) + (((x) &0xFF) << 8))

void set_addr_from_key(esp_bd_addr_t addr, uint8_t *public_key) {
	addr[0] = public_key[0] | 0b11000000;
	addr[1] = public_key[1];
	addr[2] = public_key[2];
	addr[3] = public_key[3];
	addr[4] = public_key[4];
	addr[5] = public_key[5];
}

void set_payload_from_key(uint8_t *payload, uint8_t *public_key) {
    /* copy last 22 bytes */
	memcpy(&payload[7], &public_key[6], 22);
	/* append two bits of public key */
	payload[29] = public_key[0] >> 6;
}

void OpenHaystack::dump_config() {
  ESP_LOGCONFIG(TAG, "OpenHaystack:");
  ESP_LOGCONFIG(TAG, "  Bluetooth MAC: %02X:%02X:%02X:%02X:%02X:%02X", rnd_addr[0], rnd_addr[1], rnd_addr[2], rnd_addr[3], rnd_addr[4], rnd_addr[5]);
  ESP_LOGCONFIG(TAG,
                "  Advertising Key (first six digits): %02X %02X %02X %02X %02X %02X",
                this->advertising_key_[0],
                this->advertising_key_[1],
                this->advertising_key_[2],
                this->advertising_key_[3],
                this->advertising_key_[4],
                this->advertising_key_[5]
  );
}

void OpenHaystack::setup() {
  ESP_LOGCONFIG(TAG, "Setting up OpenHaystack device...");
  global_openhaystack = this;

  xTaskCreatePinnedToCore(OpenHaystack::ble_core_task,
                          "ble_task",  // name
                          10000,       // stack size (in words)
                          nullptr,     // input params
                          1,           // priority
                          nullptr,     // Handle, not needed
                          0            // core
  );
}

float OpenHaystack::get_setup_priority() const { return setup_priority::BLUETOOTH; }
void OpenHaystack::ble_core_task(void *params) {
  ble_setup();

  while (true) {
    delay(1000);  // NOLINT
  }
}

void OpenHaystack::ble_setup() {
  // Initialize non-volatile storage for the bluetooth controller
  esp_err_t err = nvs_flash_init();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "nvs_flash_init failed: %d", err);
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
      while (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE)
        ;
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

  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);

  err = esp_bluedroid_init();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_bluedroid_init failed: %d", err);
    return;
  }
  err = esp_bluedroid_enable();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_bluedroid_enable failed: %d", err);
    return;
  }

  set_addr_from_key(rnd_addr, global_openhaystack->advertising_key_.data());
  set_payload_from_key(ble_adv_data, global_openhaystack->advertising_key_.data());

  err = esp_ble_gap_register_callback(OpenHaystack::gap_event_handler);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_ble_gap_register_callback failed: %d", err);
    return;
  }
  err = esp_ble_gap_set_rand_addr(rnd_addr);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_ble_gap_set_rand_addr failed: %s", esp_err_to_name(err));
    return;
  }

  esp_ble_gap_config_adv_data_raw((uint8_t *) &ble_adv_data, sizeof(ble_adv_data));
}

void OpenHaystack::gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  esp_err_t err;
  switch (event) {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT: {
      err = esp_ble_gap_start_advertising(&ble_adv_params);
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ble_gap_start_advertising failed: %d", err);
      }
      break;
    }
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT: {
      err = param->adv_start_cmpl.status;
      if (err != ESP_BT_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "BLE adv start failed: %s", esp_err_to_name(err));
      }
      break;
    }
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT: {
      err = param->adv_start_cmpl.status;
      if (err != ESP_BT_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "BLE adv stop failed: %s", esp_err_to_name(err));
      } else {
        ESP_LOGD(TAG, "BLE stopped advertising successfully");
      }
      break;
    }
    default:
      break;
  }
}

OpenHaystack *global_openhaystack = nullptr;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

}  // namespace openhaystack
}  // namespace esphome

#endif
