import base64
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_KEY
from esphome.core import CORE
from esphome.components.esp32 import add_idf_sdkconfig_option

DEPENDENCIES = ["esp32"]
CONFLICTS_WITH = ["esp32_ble_tracker", "esp32_ble_beacon"]
CONF_KEYS = 'keys'
CONF_INTERVAL = 'key_switch_interval'
CONF_SAVE = 'save_key_index_to_flash'

openhaystack_ns = cg.esphome_ns.namespace("openhaystack")
OpenHaystack = openhaystack_ns.class_("OpenHaystack", cg.Component)


def _validate_keys(value):
    keys = value[CONF_KEYS]
    for k in keys:
        try:
            key = base64.b64decode(k)
            if len(key) != 28:
                raise cv.Invalid(
                    k+" invalid key size. Make sure you're using the base64 advertisement key."
                )
        except base64.binascii.Error as base64_error:
            raise cv.Invalid(
                k+" invalid key. Make sure you're using the base64 advertisement key."
            ) from base64_error

    return value


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(OpenHaystack),
            cv.Required(CONF_KEYS): cv.ensure_list(cv.string),
            cv.Optional(CONF_INTERVAL, default=3600): cv.int_range(min=10),
            cv.Optional(CONF_SAVE, default=False): cv.boolean,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    _validate_keys,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    for encoded_adv_pub_key in config[CONF_KEYS]:
        adv_pub_key = base64.b64decode(encoded_adv_pub_key).hex()
        adv_pub_key_arr = [
            int(adv_pub_key[i : i + 2], 16) for i in range(0, len(adv_pub_key), 2)
        ]
        cg.add(var.add_adv_key(adv_pub_key_arr))
    if CONF_INTERVAL in config:
        cg.add(var.set_switch_key_interval(config[CONF_INTERVAL]))
    if CONF_SAVE in config:
        cg.add(var.set_save_key_index(config[CONF_SAVE]))
    if CORE.using_esp_idf:
        add_idf_sdkconfig_option("CONFIG_BT_ENABLED", True)
