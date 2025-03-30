#include "ble_adv_proxy.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include <esp_err.h>
#include <esp_bt_device.h>

namespace esphome {
namespace ble_adv_proxy {

static constexpr const char *TAG = "ble_adv_proxy";
static constexpr const char *DISCOVERY_EVENT = "esphome.ble_adv.discovery";
static constexpr const char *ADV_RECV_EVENT = "esphome.ble_adv.raw_adv";
static constexpr const char *ADV_SVC = "adv_svc";
static constexpr const char *CONF_RAW = "raw";
static constexpr const char *CONF_DURATION = "duration";
static constexpr const char *CONF_NAME = "name";
static constexpr const char *CONF_MAC = "mac";
static constexpr const char *CONF_ADV_EVT = "adv_recv_event";
static constexpr const char *CONF_PUB_SVC = "publish_adv_svc";

void BleAdvProxy::setup() {
  register_service(&BleAdvProxy::on_advertise, ADV_SVC, {CONF_RAW, CONF_DURATION});
  this->scan_result_lock_ = xSemaphoreCreateMutex();
}

void BleAdvProxy::on_advertise(std::string raw, float duration) {
  BleAdvParam param;
  param.len_ = std::min(MAX_PACKET_LEN, raw.size() / 2);
  esphome::parse_hex(raw, param.buf_, param.len_);
  param.duration_ = duration;
  ESP_LOGD(TAG, "send adv - %s", esphome::format_hex_pretty(param.buf_, param.len_).c_str());
  this->send_packets_.emplace_back(std::move(param));
}

void BleAdvProxy::on_raw_recv(const BleAdvParam &param) {
  ESP_LOGD(TAG, "recv raw - %s", esphome::format_hex_pretty(param.buf_, param.len_).c_str());
  if (!this->is_connected()) {
    ESP_LOGD(TAG, "No clients connected to API server, received adv ignored.");
    return;
  }

  this->fire_homeassistant_event(ADV_RECV_EVENT, {{CONF_RAW, esphome::format_hex(param.buf_, param.len_)}});
}

void BleAdvProxy::setup_max_tx_power() {
  // The standard interfaces for esp32 are limited to ESP_PWR_LVL_P9, whereas some other interfaces for ESP32-C2 / C3 /
  // .. are able to go up to ESP_PWR_LVL_P20. This function will simply try to setup the max value by increasing by 1
  // each time, and checking if it gives an error
  if (this->max_tx_power_setup_done_ || !this->use_max_tx_power_) {
    return;
  }

  esp_power_level_t lev_init = esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_ADV);
  ESP_LOGD(TAG, "Advertising TX Power enum value (NOT dBm) before max setup: %d", lev_init);

  esp_err_t ret_code = ESP_OK;
  esp_power_level_t lev_code = ESP_PWR_LVL_P9;
  while ((ret_code == ESP_OK) && (lev_code < 0xFF)) {
    ret_code = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, lev_code);
    lev_code = (esp_power_level_t) ((uint8_t) (lev_code + 1));
  }

  esp_power_level_t lev_final = esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_ADV);
  ESP_LOGD(TAG, "Advertising TX Power enum value (NOT dBm) after max setup: %d", lev_final);

  this->max_tx_power_setup_done_ = true;
}

std::string get_mac() {
  const uint8_t *mac = esp_bt_dev_get_address();
  return str_snprintf("%02X:%02X:%02X:%02X:%02X:%02X", 17, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

std::string build_svc_name(const char *svc_name) {
  // same as done in home assistant core 'esphome.manager.build_service_name':
  // https://github.com/home-assistant/core/blob/dev/homeassistant/components/esphome/manager.py#L774
  std::string app_name = App.get_name();
  std::replace(app_name.begin(), app_name.end(), '-', '_');
  return str_sprintf("%s_%s", app_name.c_str(), svc_name);
}

void BleAdvProxy::send_discovery_event() {
  ESP_LOGD(TAG, "Sending discovery event");
  static const std::map<std::string, std::string> KV = {{CONF_ADV_EVT, ADV_RECV_EVENT},
                                                        {CONF_MAC, get_mac()},
                                                        {CONF_NAME, App.get_name()},
                                                        {CONF_PUB_SVC, build_svc_name(ADV_SVC)}};

  this->fire_homeassistant_event(DISCOVERY_EVENT, KV);
}

void BleAdvProxy::loop() {
  if (!this->get_parent()->is_active()) {
    // esp32_ble::ESP32BLE not ready: do not process any action
    return;
  }

  // Handle discovery
  if (!this->api_was_connected_ && this->is_connected()) {
    // Reconnection occured: setup discovery in a few seconds
    this->next_discovery_ = millis() + 3 * 1000;
    this->nb_short_sent_ = 10;
  }
  this->api_was_connected_ = this->is_connected();
  if (this->api_was_connected_ && this->next_discovery_ < millis()) {
    if (this->nb_short_sent_ > 0) {
      this->nb_short_sent_ -= 1;
      this->next_discovery_ = millis() + 3 * 1000;
    } else {
      this->next_discovery_ = millis() + 60 * 1000;
    }
    this->send_discovery_event();
  }

  // Cleanup expired packets
  this->dupe_packets_.remove_if([&](BleAdvParam &p) { return p.duration_ < millis(); });

  // swap packet list to further process it outside of the lock
  std::list<BleAdvParam> new_packets;
  if (xSemaphoreTake(this->scan_result_lock_, 5L / portTICK_PERIOD_MS)) {
    std::swap(this->recv_packets_, new_packets);
    xSemaphoreGive(this->scan_result_lock_);
  } else {
    ESP_LOGW(TAG, "loop - failed to take lock");
  }

  // handle new packets
  for (auto &param : new_packets) {
    auto idx = std::find_if(this->dupe_packets_.begin(), this->dupe_packets_.end(), [&](BleAdvParam &p) {
      return (p.len_ == param.len_) && std::equal(p.buf_, p.buf_ + p.len_, param.buf_);
    });
    if (idx == this->dupe_packets_.end()) {
      this->on_raw_recv(param);
      this->dupe_packets_.emplace_back(std::move(param));
    }
  }

  // Process advertising
  if (this->adv_stop_time_ == 0) {
    // No packet is being advertised, advertise the front one
    if (!this->send_packets_.empty()) {
      BleAdvParam &packet = this->send_packets_.front();
      this->setup_max_tx_power();
      ESP_ERROR_CHECK_WITHOUT_ABORT(esp_ble_gap_config_adv_data_raw(packet.buf_, packet.len_));
      ESP_ERROR_CHECK_WITHOUT_ABORT(esp_ble_gap_start_advertising(&(this->adv_params_)));
      this->adv_stop_time_ = millis() + this->send_packets_.front().duration_;
    }
  } else {
    // Packet is being advertised, stop advertising and remove packet
    if (millis() > this->adv_stop_time_) {
      ESP_ERROR_CHECK_WITHOUT_ABORT(esp_ble_gap_stop_advertising());
      this->adv_stop_time_ = 0;
      this->send_packets_.pop_front();
    }
  }
}

// We let the configuration of the scanning to esp32_ble_tracker, towards with stop / start
// We only gather directly the raw events
void BleAdvProxy::gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  if (event == ESP_GAP_BLE_SCAN_RESULT_EVT && param->scan_rst.adv_data_len <= MAX_PACKET_LEN) {
    BleAdvParam packet;
    packet.len_ = param->scan_rst.adv_data_len;
    std::copy(param->scan_rst.ble_adv, param->scan_rst.ble_adv + packet.len_, packet.buf_);
    packet.duration_ = millis() + 60 * 1000;
    if (xSemaphoreTake(this->scan_result_lock_, 5L / portTICK_PERIOD_MS)) {
      this->recv_packets_.emplace_back(std::move(packet));
      xSemaphoreGive(this->scan_result_lock_);
    } else {
      ESP_LOGW(TAG, "evt - failed to take lock");
    }
  }
}

}  // namespace ble_adv_proxy
}  // namespace esphome
