#include "ble_adv_proxy.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include <esp_err.h>
#include <esp_bt_device.h>

#ifdef ESP_PWR_LVL_P20
#define MAX_TX_POWER ESP_PWR_LVL_P20
#elif ESP_PWR_LVL_P18
#define MAX_TX_POWER ESP_PWR_LVL_P18
#elif ESP_PWR_LVL_P15
#define MAX_TX_POWER ESP_PWR_LVL_P15
#elif ESP_PWR_LVL_P12
#define MAX_TX_POWER ESP_PWR_LVL_P12
#else
#define MAX_TX_POWER ESP_PWR_LVL_P9
#endif

namespace esphome {
namespace ble_adv_proxy {

static constexpr const char *TAG = "ble_adv_proxy";
static constexpr const char *DISCOVERY_EVENT = "esphome.ble_adv.discovery";
static constexpr const char *ADV_RECV_EVENT = "esphome.ble_adv.raw_adv";
static constexpr const char *SETUP_SVC = "setup_svc";
static constexpr const char *CONF_IGN_ADVS = "ignored_advs";
static constexpr const char *CONF_IGN_DURATION = "ignored_duration";
static constexpr const char *ADV_SVC = "adv_svc";
static constexpr const char *CONF_RAW = "raw";
static constexpr const char *CONF_DURATION = "duration";
static constexpr const char *CONF_NAME = "name";
static constexpr const char *CONF_MAC = "mac";
static constexpr const char *CONF_ADV_EVT = "adv_recv_event";
static constexpr const char *CONF_PUB_SVC = "publish_adv_svc";

static constexpr const uint32_t DISCOVERY_EMIT_INTERVAL = 60 * 1000;
static constexpr const uint32_t DISCOVERY_EMIT_INTERVAL_SHORT = 3 * 1000;
static constexpr const uint8_t DISCOVERY_EMIT_NB_SHORT = 10;

BleAdvParam::BleAdvParam(const std::string &hex_string, uint32_t duration)
    : duration_(duration), len_(std::min(MAX_PACKET_LEN, hex_string.size() / 2)) {
  esphome::parse_hex(hex_string, this->buf_, this->len_);
}

BleAdvParam::BleAdvParam(const uint8_t *buf, size_t len, uint32_t duration)
    : duration_(duration), len_(std::min(MAX_PACKET_LEN, len)) {
  std::copy(buf, buf + this->len_, this->buf_);
}

void BleAdvProxy::setup() {
  this->register_service(&BleAdvProxy::on_setup, SETUP_SVC, {CONF_IGN_DURATION, CONF_IGN_ADVS});
  this->register_service(&BleAdvProxy::on_advertise, ADV_SVC, {CONF_RAW, CONF_DURATION});
  this->scan_result_lock_ = xSemaphoreCreateMutex();
}

void BleAdvProxy::on_setup(float ign_duration, std::vector<std::string> ignored_advs) {
  this->dupe_ignore_duration_ = ign_duration;
  for (auto &ignored_adv : ignored_advs) {
    ESP_LOGI(TAG, "Permanently ignoring ADVs starting with: %s", ignored_adv.c_str());
    this->check_add_dupe_packet(BleAdvParam(ignored_adv, 0));
  }
}

void BleAdvProxy::on_advertise(std::string raw, float duration) {
  ESP_LOGD(TAG, "send adv - %s", raw.c_str());
  this->send_packets_.emplace_back(raw, duration);
  // Prevent this packet from being re sent to HA host in case re received
  this->check_add_dupe_packet(BleAdvParam(raw, millis() + this->dupe_ignore_duration_));
}

bool BleAdvProxy::check_add_dupe_packet(BleAdvParam &&packet) {
  // Check the recently received advs
  auto idx = std::find_if(this->dupe_packets_.begin(), this->dupe_packets_.end(), [&](BleAdvParam &p) {
    return (p.len_ <= packet.len_) && std::equal(p.buf_, p.buf_ + p.len_, packet.buf_);
  });
  if (idx != this->dupe_packets_.end()) {
    if (idx->duration_ > 0) {
      idx->duration_ = packet.duration_;  // Existing Packet with specified deletion time: Update the deletion time
    }
    return false;
  }
  this->dupe_packets_.emplace_back(std::move(packet));
  return true;
}

void BleAdvProxy::on_raw_recv(const BleAdvParam &param) {
  std::string raw = esphome::format_hex(param.buf_, param.len_);
  ESP_LOGD(TAG, "recv raw - %s", raw.c_str());
  if (!this->is_connected()) {
    ESP_LOGD(TAG, "No clients connected to API server, received adv ignored.");
    return;
  }
  this->fire_homeassistant_event(ADV_RECV_EVENT, {{CONF_RAW, std::move(raw)}});
}

void BleAdvProxy::setup_max_tx_power() {
  if (this->max_tx_power_setup_done_ || !this->use_max_tx_power_) {
    return;
  }

  esp_power_level_t lev_init = esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_ADV);
  ESP_LOGD(TAG, "Advertising TX Power enum value (NOT dBm) before max setup: %d", lev_init);

  if (lev_init != MAX_TX_POWER) {
    ESP_LOGI(TAG, "Advertising TX Power setup attempt to: %d", MAX_TX_POWER);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, MAX_TX_POWER);
    esp_power_level_t lev_final = esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_ADV);
    ESP_LOGI(TAG, "Advertising TX Power enum value (NOT dBm) after max setup: %d", lev_final);
  } else {
    ESP_LOGI(TAG, "Advertising TX Power already at max: %d", MAX_TX_POWER);
  }

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
    this->next_discovery_ = millis() + DISCOVERY_EMIT_INTERVAL_SHORT;
    this->nb_short_sent_ = DISCOVERY_EMIT_NB_SHORT;
  }
  this->api_was_connected_ = this->is_connected();
  if (this->api_was_connected_ && this->next_discovery_ < millis()) {
    if (this->nb_short_sent_ > 0) {
      this->nb_short_sent_ -= 1;
      this->next_discovery_ = millis() + DISCOVERY_EMIT_INTERVAL_SHORT;
    } else {
      this->next_discovery_ = millis() + DISCOVERY_EMIT_INTERVAL;
    }
    this->send_discovery_event();
  }

  // Cleanup expired packets
  this->dupe_packets_.remove_if([&](BleAdvParam &p) { return p.duration_ > 0 && p.duration_ < millis(); });

  // swap packet list to further process it outside of the lock
  std::list<BleAdvParam> new_packets;
  if (xSemaphoreTake(this->scan_result_lock_, 5L / portTICK_PERIOD_MS)) {
    std::swap(this->recv_packets_, new_packets);
    xSemaphoreGive(this->scan_result_lock_);
  } else {
    ESP_LOGW(TAG, "loop - failed to take lock");
  }

  // handle new packets
  for (auto &packet : new_packets) {
    if (this->check_add_dupe_packet(std::move(packet))) {
      this->on_raw_recv(this->dupe_packets_.back());
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
      this->adv_stop_time_ = millis() + packet.duration_;
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
void BleAdvProxy::gap_scan_event_handler(const esp32_ble::BLEScanResult &sr) {
  if (sr.adv_data_len <= MAX_PACKET_LEN) {
    if (xSemaphoreTake(this->scan_result_lock_, 5L / portTICK_PERIOD_MS)) {
      this->recv_packets_.emplace_back(sr.ble_adv, sr.adv_data_len, millis() + this->dupe_ignore_duration_);
      xSemaphoreGive(this->scan_result_lock_);
    } else {
      ESP_LOGW(TAG, "evt - failed to take lock");
    }
  }
}

}  // namespace ble_adv_proxy
}  // namespace esphome
