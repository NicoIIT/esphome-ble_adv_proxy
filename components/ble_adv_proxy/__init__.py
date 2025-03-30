import esphome.codegen as cg
from esphome.components.esp32_ble import CONF_BLE_ID, ESP32BLE
import esphome.config_validation as cv
from esphome.const import CONF_ID

AUTO_LOAD = ["esp32_ble", "esp32_ble_tracker", "api"]
DEPENDENCIES = ["esp32", "api"]
MULTI_CONF = False

bleadvproxy_ns = cg.esphome_ns.namespace("ble_adv_proxy")
BleAdvProxy = bleadvproxy_ns.class_("BleAdvProxy", cg.Component)

CONF_BLE_ADV_USE_MAX_TX_POWER = "use_max_tx_power"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(BleAdvProxy),
        cv.GenerateID(CONF_BLE_ID): cv.use_id(ESP32BLE),
        cv.Optional(CONF_BLE_ADV_USE_MAX_TX_POWER, default=False): cv.boolean,
    }
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    cg.add(var.set_setup_priority(300))  # start after Bluetooth
    await cg.register_component(var, config)
    cg.add(var.set_use_max_tx_power(config[CONF_BLE_ADV_USE_MAX_TX_POWER]))
    parent = await cg.get_variable(config[CONF_BLE_ID])
    cg.add(parent.register_gap_event_handler(var))
    cg.add(var.set_parent(parent))
