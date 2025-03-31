#pragma once

#include "esphome/core/component.h"
#include "esphome/components/esp32_ble/ble.h"
#include "esphome/components/api/custom_api_device.h"

#include <freertos/semphr.h>
#include <esp_gap_ble_api.h>
#include <list>

namespace esphome {

namespace ble_adv_proxy {

static constexpr size_t MAX_PACKET_LEN = 31;

class BleAdvParam {
 public:
  BleAdvParam(const std::string &hex_string, uint32_t duration);
  BleAdvParam(const uint8_t *buf, size_t len, uint32_t duration);
  BleAdvParam(BleAdvParam &&) = default;
  BleAdvParam &operator=(BleAdvParam &&) = default;

  uint32_t duration_{100};
  uint8_t buf_[MAX_PACKET_LEN]{0};
  size_t len_{0};
};

/**
  BleAdvProxy:
 */
class BleAdvProxy : public Component,
                    public esp32_ble::GAPEventHandler,
                    public Parented<esp32_ble::ESP32BLE>,
                    public api::CustomAPIDevice {
 public:
  // component handling
  void setup() override;
  void loop() override;

  void set_use_max_tx_power(bool use_max_tx_power) { this->use_max_tx_power_ = use_max_tx_power; }
  void on_advertise(std::string raw, float duration);
  void on_raw_recv(const BleAdvParam &param);
  void secure_add_recv_packet(BleAdvParam &&packet);

 protected:
  /**
    Performing RAW ADV
   */
  std::list<BleAdvParam> send_packets_;
  uint32_t adv_stop_time_ = 0;

  esp_ble_adv_params_t adv_params_ = {
      .adv_int_min = 0x20,
      .adv_int_max = 0x20,
      .adv_type = ADV_TYPE_NONCONN_IND,
      .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
      .peer_addr = {0x00},
      .peer_addr_type = BLE_ADDR_TYPE_PUBLIC,
      .channel_map = ADV_CHNL_ALL,
      .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
  };

  bool use_max_tx_power_ = false;
  bool max_tx_power_setup_done_ = false;
  void setup_max_tx_power();

  /**
    Listening to ADV
   */
  void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
  SemaphoreHandle_t scan_result_lock_;
  std::list<BleAdvParam> recv_packets_;
  std::list<BleAdvParam> dupe_packets_;

  /*
  API Discovery
  */
  void send_discovery_event();
  bool api_was_connected_ = false;
  uint32_t next_discovery_ = 0;
  uint8_t nb_short_sent_ = 0;
};

}  // namespace ble_adv_proxy
}  // namespace esphome
