import logging

import esphome.codegen as cg
from esphome.components.esp32_ble import CONF_BLE_ID, ESP32BLE
from esphome.components.text_sensor import new_text_sensor, text_sensor_schema
import esphome.config_validation as cv
from esphome.const import (
    CONF_ENTITY_CATEGORY,
    CONF_ID,
    CONF_NAME,
    ENTITY_CATEGORY_DIAGNOSTIC,
)

# A wonderful HACK to avoid the need for users to define the 'api' option 'custom_services' to True:
# we patch the CONFIG_SCHEMA of the 'api' component to setup the default value of 'custom_services' to True
# This can break anytime, but anyway ESPHome changes regularly break all our users, so not less risky...
try:
    from esphome.components.api import (
        CONF_CUSTOM_SERVICES as API_CONF_CUSTOM_SERVICES,
        CONF_HOMEASSISTANT_SERVICES as API_CONF_HOMEASSISTANT_SERVICES,
        CONFIG_SCHEMA as API_CONFIG_SCHEMA,
    )

    vals = list(API_CONFIG_SCHEMA.validators)
    vals[0] = vals[0].extend(
        {
            cv.Optional(API_CONF_CUSTOM_SERVICES, default=True): cv.boolean,
            cv.Optional(API_CONF_HOMEASSISTANT_SERVICES, default=True): cv.boolean,
        }
    )
    API_CONFIG_SCHEMA.validators = tuple(vals)
except BaseException:
    logging.warning(
        "Unable to define api custom_services to True, please refer to the doc to do it manually."
    )
# End of workaround


AUTO_LOAD = ["esp32_ble", "esp32_ble_tracker", "api", "text_sensor"]
DEPENDENCIES = ["esp32", "api"]
MULTI_CONF = False

bleadvproxy_ns = cg.esphome_ns.namespace("ble_adv_proxy")
BleAdvProxy = bleadvproxy_ns.class_("BleAdvProxy", cg.Component)

CONF_BLE_ADV_USE_MAX_TX_POWER = "use_max_tx_power"
CONF_NAME_SENSOR = "name_sensor"
CONF_VAL_NAME_SENSOR = "ble_adv_proxy_name"

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(BleAdvProxy),
            cv.GenerateID(CONF_BLE_ID): cv.use_id(ESP32BLE),
            cv.Optional(CONF_BLE_ADV_USE_MAX_TX_POWER, default=False): cv.boolean,
            cv.Optional(
                CONF_NAME_SENSOR,
                default={
                    CONF_ID: CONF_VAL_NAME_SENSOR,
                    CONF_NAME: CONF_VAL_NAME_SENSOR,
                    CONF_ENTITY_CATEGORY: ENTITY_CATEGORY_DIAGNOSTIC,
                },
            ): text_sensor_schema(),
        }
    ),
    # Creation of esp32_ble::GAPScanEventHandler: 2025.6.2
    # API_CONF_HOMEASSISTANT_SERVICES: 2025.8.0
    cv.require_esphome_version(2025, 8, 0),
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    cg.add(var.set_setup_priority(300))  # start after Bluetooth
    await cg.register_component(var, config)
    cg.add(var.set_use_max_tx_power(config[CONF_BLE_ADV_USE_MAX_TX_POWER]))
    parent = await cg.get_variable(config[CONF_BLE_ID])
    cg.add(parent.register_gap_scan_event_handler(var))
    cg.add(var.set_parent(parent))
    sens = await new_text_sensor(config[CONF_NAME_SENSOR])
    cg.add(var.set_sensor_name(sens))
