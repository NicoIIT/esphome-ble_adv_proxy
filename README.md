# esphome-ble_adv_proxy
Custom ESPHome component ble_adv_proxy

This component is to be used to transform your ESP into a bluetooth adv proxy that can be used with [ha-ble-adv component](https://github.com/NicoIIT/ha-ble-adv).

Contrarily to the existing bluetooth_proxy component, this component is handling Raw Advertising exchanges in both directions which is mandatory in order to control devices using BLE Advertising.

It can be used on top of an existing standard Bluetooth ESPHome Proxy by adding the config:

```yaml
(...)
esp32_ble_tracker:
  scan_parameters:
    # We currently use the defaults to ensure Bluetooth
    # can co-exist with WiFi In the future we may be able to
    # enable the built-in coexistence logic in ESP-IDF
    active: true

bluetooth_proxy:
  active: true

# Added for ble_adv_proxy
ble_adv_proxy:

external_components:
  source: github://NicoIIT/esphome-ble_adv_proxy
```

It can also be used standalone without bluetooth_proxy or esp32_ble_tracker, still the `api` component is mandatory.

## Variables
The following variables are available:
- **use_max_tx_power** (Optional, Default: False): Try to use the max TX Power for the Advertising stack, as defined in Espressif [doc](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/bluetooth/controller_vhci.html#_CPPv417esp_power_level_t). Setup to 'true' if your ESP32 is far from your device and have difficulties to communicate with it.
