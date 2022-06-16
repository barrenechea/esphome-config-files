#pragma once

#include "esphome/core/component.h"

#ifdef USE_ESP32

#include <esp_gap_ble_api.h>

namespace esphome {
namespace openhaystack {

class OpenHaystack : public Component {
 public:
  explicit OpenHaystack(const std::array<uint8_t, 28> &advertising_key) : advertising_key_(advertising_key) {}

  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;

 protected:
  static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
  static void ble_core_task(void *params);
  static void ble_setup();

  std::array<uint8_t, 28> advertising_key_;
};

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
extern OpenHaystack *global_openhaystack;

}  // namespace openhaystack
}  // namespace esphome

#endif
