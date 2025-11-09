#include "ble_adv_proxy.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include <esp_err.h>
#include <esp_bt.h>
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
static constexpr const char *ADV_RECV_EVENT = "esphome.ble_adv.raw_adv";
static constexpr const char *SETUP_SVC_V0 = "setup_svc_v0";
static constexpr const char *CONF_IGN_ADVS = "ignored_advs";
static constexpr const char *CONF_IGN_CIDS = "ignored_cids";
static constexpr const char *CONF_IGN_MACS = "ignored_macs";
static constexpr const char *CONF_IGN_DURATION = "ignored_duration";
static constexpr const char *ADV_SVC_V0 = "adv_svc";  // legacy name / service
static constexpr const char *ADV_SVC_V1 = "adv_svc_v1";
static constexpr const char *CONF_RAW = "raw";
static constexpr const char *CONF_ORIGIN = "orig";
static constexpr const char *CONF_DURATION = "duration";
static constexpr const char *CONF_REPEAT = "repeat";

static constexpr const uint8_t REPEAT_NB = 3;
static constexpr const uint8_t MIN_ADV = 0x20;
static constexpr const uint8_t MIN_VIABLE_PACKET_LEN = 5;

BleAdvParam::BleAdvParam(const std::string &hex_string, uint32_t duration)
    : duration_(duration), len_(std::min(MAX_PACKET_LEN, hex_string.size() / 2)) {
  esphome::parse_hex(hex_string, this->buf_, this->len_);
}

BleAdvParam::BleAdvParam(const uint8_t *buf, size_t len, const esp_bd_addr_t &orig, uint32_t duration)
    : duration_(duration), len_(std::min(MAX_PACKET_LEN, len)) {
  std::copy(buf, buf + this->len_, this->buf_);
  std::copy(orig, orig + ESP_BD_ADDR_LEN, this->orig_);
}

void BleAdvProxy::setup() {
  this->register_service(&BleAdvProxy::on_setup_v0, SETUP_SVC_V0, {CONF_IGN_DURATION, CONF_IGN_CIDS, CONF_IGN_MACS});
  this->register_service(&BleAdvProxy::on_advertise_v0, ADV_SVC_V0, {CONF_RAW, CONF_DURATION});
  this->register_service(&BleAdvProxy::on_advertise_v1, ADV_SVC_V1,
                         {CONF_RAW, CONF_DURATION, CONF_REPEAT, CONF_IGN_ADVS, CONF_IGN_DURATION});
  this->scan_result_lock_ = xSemaphoreCreateMutex();
  if (this->sensor_name_->state.empty()) {
    this->sensor_name_->state = App.get_name();
  }
  this->sensor_name_->publish_state(this->sensor_name_->state);
}

void BleAdvProxy::dump_config() {
  ESP_LOGCONFIG(TAG, "BleAdvProxy '%s'", this->sensor_name_->state.c_str());
  ESP_LOGCONFIG(TAG, "  Use Max TxPower: %s", this->use_max_tx_power_ ? "True" : "False");
}

void BleAdvProxy::on_setup_v0(float ign_duration, std::vector<float> ignored_cids,
                              std::vector<std::string> ignored_macs) {
  this->dupe_ignore_duration_ = ign_duration;
  this->dupe_packets_.clear();
  this->ign_cids_.clear();
  for (auto &ign_cid : ignored_cids) {
    this->ign_cids_.emplace_back(uint16_t(ign_cid));
  }
  ESP_LOGI(TAG, "SETUP - %d Company IDs Permanently ignored.", this->ign_cids_.size());
  this->ign_macs_.clear();
  std::swap(ignored_macs, this->ign_macs_);
  ESP_LOGI(TAG, "SETUP - %d MACs Permanently ignored.", this->ign_macs_.size());
  this->setup_done_ = true;
}

void BleAdvProxy::on_advertise_v0(std::string raw, float duration) {
  this->on_advertise_v1(raw, duration / REPEAT_NB, REPEAT_NB, {raw}, this->dupe_ignore_duration_);
}

void BleAdvProxy::on_advertise_v1(std::string raw, float duration, float repeat, std::vector<std::string> ignored_advs,
                                  float ign_duration) {
  this->setup_done_ = true;  // Flag setup done as best effort
  uint8_t int_repeat = uint8_t(repeat);
  uint32_t int_duration = uint32_t(duration);
  uint32_t int_ign_duration = uint32_t(ign_duration);
  ESP_LOGD(TAG, "send adv - %s, duration %dms, repeat: %d", raw.c_str(), int_duration, int_repeat);
  for (uint8_t i = 0; i < int_repeat; ++i) {
    this->send_packets_.emplace_back(raw, int_duration);
  }
  // Prevent ignored packets from being re sent to HA host in case received
  for (auto &ignored_adv : ignored_advs) {
    // ESP_LOGD(TAG, "Ignoring ADV for %ds: %s", int_ign_duration / 1000, ignored_adv.c_str());
    this->check_add_dupe_packet(BleAdvParam(ignored_adv, millis() + int_ign_duration));
  }
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

std::string get_str_mac(const uint8_t *mac) {
  return str_snprintf("%02X:%02X:%02X:%02X:%02X:%02X", 17, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void BleAdvProxy::on_raw_recv(const BleAdvParam &param, const std::string &str_mac) {
  std::string raw = esphome::format_hex(param.buf_, param.len_);
  ESP_LOGD(TAG, "[%s] recv raw - %s", str_mac.c_str(), raw.c_str());
  if (!this->is_connected()) {
    ESP_LOGD(TAG, "No clients connected to API server, received adv ignored.");
    return;
  }
  this->fire_homeassistant_event(ADV_RECV_EVENT, {{CONF_RAW, std::move(raw)}, {CONF_ORIGIN, std::move(str_mac)}});
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

void BleAdvProxy::loop() {
  if (!this->get_parent()->is_active() || !this->setup_done_) {
    // esp32_ble::ESP32BLE not ready or Setup not done yet: do not process any action
    return;
  }

  if (!this->is_connected()) {  // API not connected (disconnection occured), re wait for setup
    this->setup_done_ = false;
    return;
  }

  // Cleanup expired packets
  this->dupe_packets_.remove_if([&](BleAdvParam &p) { return p.duration_ > 0 && p.duration_ < millis(); });

  // swap packet list to further process it outside of the lock
  std::list<esp32_ble::BLEScanResult> new_packets;
  if (xSemaphoreTake(this->scan_result_lock_, 5L / portTICK_PERIOD_MS)) {
    std::swap(this->recv_packets_, new_packets);
    xSemaphoreGive(this->scan_result_lock_);
  } else {
    ESP_LOGW(TAG, "loop - failed to take lock");
  }

  // handle new packets, exclude if one of the following is true:
  // - len is too small
  // - company ID is part of ignored company ids
  // - mac is part of ignored macs
  // - is dupe of previously received
  for (auto &sr : new_packets) {
    uint16_t cid = (sr.ble_adv[3] << 8) + sr.ble_adv[2];
    std::string str_mac = get_str_mac(sr.bda);
    if (std::find(this->ign_cids_.begin(), this->ign_cids_.end(), cid) == this->ign_cids_.end() &&
        std::find(this->ign_macs_.begin(), this->ign_macs_.end(), str_mac) == this->ign_macs_.end() &&
        this->check_add_dupe_packet(
            BleAdvParam(sr.ble_adv, sr.adv_data_len, sr.bda, millis() + this->dupe_ignore_duration_))) {
      this->on_raw_recv(this->dupe_packets_.back(), str_mac);
    }
  }

  // Process advertising
  if (this->adv_stop_time_ == 0) {
    // No packet is being advertised, advertise the front one
    if (!this->send_packets_.empty()) {
      BleAdvParam &packet = this->send_packets_.front();
      this->setup_max_tx_power();
      ESP_ERROR_CHECK_WITHOUT_ABORT(esp_ble_gap_config_adv_data_raw(packet.buf_, packet.len_));
      uint8_t adv_time = std::max(MIN_ADV, uint8_t(1.6 * packet.duration_));
      this->adv_params_.adv_int_min = adv_time;
      this->adv_params_.adv_int_max = adv_time;
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
  if (this->setup_done_ && sr.adv_data_len <= MAX_PACKET_LEN && sr.adv_data_len >= MIN_VIABLE_PACKET_LEN) {
    if (xSemaphoreTake(this->scan_result_lock_, 5L / portTICK_PERIOD_MS)) {
      this->recv_packets_.emplace_back(sr);
      xSemaphoreGive(this->scan_result_lock_);
    } else {
      ESP_LOGW(TAG, "evt - failed to take lock");
    }
  }
}

}  // namespace ble_adv_proxy
}  // namespace esphome
