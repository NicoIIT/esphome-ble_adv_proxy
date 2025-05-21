# ESPHome custom component ble_adv_proxy

This component transforms your ESP32 into a bluetooth adv proxy that can be used with [ha-ble-adv component](https://github.com/NicoIIT/ha-ble-adv).

Contrarily to the existing bluetooth_proxy component, this component is handling Raw Advertising exchanges in both directions which is mandatory in order to control devices using BLE Advertising.

The easiest solution to build an ESPHome based binary including this component is to add this feature to an existing [standard Bluetooth ESPHome Proxy](https://esphome.io/components/bluetooth_proxy.html):
1. Create your standard Bluetooth ESPHome proxy following the standard guide, and have it available in HA. **No help will be provided on this part, you can find tons of help and tutorials on the net**
2. Add this to the yaml config:

```yaml
ble_adv_proxy:
  use_max_tx_power: true # see below, remove in case of build issues

external_components:
  source: github://NicoIIT/esphome-ble_adv_proxy
```

It can also be used with an existing config including ble_adv_manager / ble_adv_remote / ble_adv_controller, this will ease migrations from those components to the HA integration.

## Variables
The following variables are available:
- **use_max_tx_power** (Optional, Default: False): Try to use the max TX Power for the Advertising stack, as defined in Espressif [doc](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/bluetooth/controller_vhci.html#_CPPv417esp_power_level_t). Setup to 'true' if your ESP32 is far from your device and have difficulties to communicate with it.

## FAQ
### I have a lot of warnings from esp32_ble_tracker, but I did not even add this component! How can I get ride of them ?
```
[23:09:41][W][esp32_ble_tracker:125]: Too many BLE events to process. Some devices may not show up.
[23:09:41][W][esp32_ble_tracker:125]: Too many BLE events to process. Some devices may not show up.
```
The `esp32_ble_tracker` is forced included by this component in order to handle the scanning part, it is also included in default Bluetooth Proxy config.
Still the default scan parameters defined in this component may not be accurate to handle the needs of our devices that publish a LOT of Advertising messages in a short amount of time. You can decrease them in order to get ride of the warnings by defining the esp32_ble_tracker and its scan parameters:

```yaml
esp32_ble_tracker:
  scan_parameters:
    window: 20ms
    interval: 20ms
```
Please be aware that the more you decrease those parameters, the more CPU will be used and the more issue you may face with Wifi / BLE co-existence.