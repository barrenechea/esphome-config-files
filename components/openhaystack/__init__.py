import base64
import logging

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_KEY
from esphome.core import CORE, TimePeriod
from esphome.components.esp32 import add_idf_sdkconfig_option

DEPENDENCIES = ["esp32"]
CONFLICTS_WITH = ["esp32_ble_tracker", "esp32_ble_beacon"]

openhaystack_ns = cg.esphome_ns.namespace("openhaystack")
OpenHaystack = openhaystack_ns.class_("OpenHaystack", cg.Component)

CONF_KEYS = "keys"
CONF_ROTATION_INTERVAL = "rotation_interval"

_LOGGER = logging.getLogger(__name__)
_LEGACY_KEY_WARNING_ISSUED = False


def _decode_key(key: str, index: int) -> list[int]:
    try:
        decoded = base64.b64decode(key)
    except base64.binascii.Error as base64_error:
        raise cv.Invalid(
            f"Key #{index + 1} is not valid base64. Make sure you're using the base64 advertisement key."
        ) from base64_error

    if len(decoded) != 28:
        raise cv.Invalid(
            f"Key #{index + 1} has invalid length {len(decoded)} bytes. Expected 28-byte advertisement key."
        )

    return list(decoded)


def _normalize_config(value):
    global _LEGACY_KEY_WARNING_ISSUED
    keys = []

    if CONF_KEY in value:
        if not _LEGACY_KEY_WARNING_ISSUED:
            _LOGGER.warning(
                "The 'openhaystack.key' option is deprecated; use 'keys' instead."
            )
            _LEGACY_KEY_WARNING_ISSUED = True
        keys.append(value[CONF_KEY])
    if CONF_KEYS in value:
        keys.extend(value[CONF_KEYS])

    if not keys:
        raise cv.Invalid("Provide at least one advertisement key using 'key' or 'keys'.")

    value[CONF_KEYS] = keys
    value.pop(CONF_KEY, None)

    return value


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(OpenHaystack),
            cv.Optional(CONF_KEY): cv.string,
            cv.Optional(CONF_KEYS): cv.ensure_list(cv.string),
            cv.Optional(CONF_ROTATION_INTERVAL): cv.All(
                cv.positive_time_period_milliseconds,
                cv.Range(min=TimePeriod(milliseconds=100)),
            ),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    _normalize_config,
)


async def to_code(config):
    encoded_keys = config[CONF_KEYS]
    decoded_keys = [_decode_key(key, idx) for idx, key in enumerate(encoded_keys)]

    first_key = decoded_keys[0]
    first_key_expr = cg.RawExpression(
        "std::array<uint8_t, 28>{" + ", ".join(f"0x{byte:02X}" for byte in first_key) + "}"
    )
    var = cg.new_Pvariable(config[CONF_ID], first_key_expr)

    for key_bytes in decoded_keys[1:]:
        key_expr = cg.RawExpression(
            "std::array<uint8_t, 28>{" + ", ".join(f"0x{byte:02X}" for byte in key_bytes) + "}"
        )
        cg.add(var.add_additional_key(key_expr))

    if (interval := config.get(CONF_ROTATION_INTERVAL)) is not None:
        cg.add(var.set_rotation_interval(int(interval.total_milliseconds)))

    await cg.register_component(var, config)

    if CORE.using_esp_idf:
        add_idf_sdkconfig_option("CONFIG_BT_ENABLED", True)
        add_idf_sdkconfig_option("CONFIG_BT_BLE_42_FEATURES_SUPPORTED", True)
        add_idf_sdkconfig_option("CONFIG_BT_BLE_50_FEATURES_SUPPORTED", False)
