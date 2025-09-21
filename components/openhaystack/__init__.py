import base64

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome.core import CORE, TimePeriod
from esphome.components.esp32 import add_idf_sdkconfig_option

DEPENDENCIES = ["esp32"]
CONFLICTS_WITH = ["esp32_ble_tracker", "esp32_ble_beacon"]

openhaystack_ns = cg.esphome_ns.namespace("openhaystack")
OpenHaystack = openhaystack_ns.class_("OpenHaystack", cg.Component)

CONF_ROTATION_INTERVAL = "rotation_interval"
CONF_MASTER_PRIVATE_KEY = "master_private_key"
CONF_MASTER_SYMMETRIC_KEY = "master_symmetric_key"
CONF_MASTER_PUBLIC_KEY = "master_public_key"


def _decode_fixed_length_base64(value: str, *, expected_length: int, label: str) -> list[int]:
    try:
        decoded = base64.b64decode(value)
    except base64.binascii.Error as base64_error:
        raise cv.Invalid(f"{label} is not valid base64") from base64_error

    if len(decoded) != expected_length:
        raise cv.Invalid(
            f"{label} has invalid length {len(decoded)} bytes. Expected {expected_length} bytes."
        )

    return list(decoded)


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(OpenHaystack),
        cv.Required(CONF_MASTER_PRIVATE_KEY): cv.string,
        cv.Required(CONF_MASTER_SYMMETRIC_KEY): cv.string,
        cv.Optional(CONF_MASTER_PUBLIC_KEY): cv.string,
        cv.Optional(CONF_ROTATION_INTERVAL): cv.All(
            cv.positive_time_period_milliseconds,
            cv.Range(min=TimePeriod(milliseconds=100)),
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    private_bytes = _decode_fixed_length_base64(
        config[CONF_MASTER_PRIVATE_KEY], expected_length=28, label="Master private key"
    )
    symmetric_bytes = _decode_fixed_length_base64(
        config[CONF_MASTER_SYMMETRIC_KEY], expected_length=32, label="Master symmetric key"
    )

    var = cg.new_Pvariable(config[CONF_ID])

    private_expr = cg.RawExpression(
        "std::array<uint8_t, 28>{" + ", ".join(f"0x{byte:02X}" for byte in private_bytes) + "}"
    )
    symmetric_expr = cg.RawExpression(
        "std::array<uint8_t, 32>{" + ", ".join(f"0x{byte:02X}" for byte in symmetric_bytes) + "}"
    )
    cg.add(var.set_master_keys(private_expr, symmetric_expr))

    if (public_value := config.get(CONF_MASTER_PUBLIC_KEY)) is not None:
        public_bytes = _decode_fixed_length_base64(
            public_value,
            expected_length=57,
            label="Master public key (uncompressed)",
        )
        public_expr = cg.RawExpression(
            "std::array<uint8_t, 57>{"
            + ", ".join(f"0x{byte:02X}" for byte in public_bytes)
            + "}"
        )
        cg.add(var.set_master_public_key(public_expr))

    if (interval := config.get(CONF_ROTATION_INTERVAL)) is not None:
        cg.add(var.set_rotation_interval(int(interval.total_milliseconds)))

    await cg.register_component(var, config)

    if CORE.using_esp_idf:
        add_idf_sdkconfig_option("CONFIG_BT_ENABLED", True)
        add_idf_sdkconfig_option("CONFIG_BT_BLE_42_FEATURES_SUPPORTED", True)
        add_idf_sdkconfig_option("CONFIG_BT_BLE_50_FEATURES_SUPPORTED", False)
        add_idf_sdkconfig_option("CONFIG_MBEDTLS_ECP_C", True)
        add_idf_sdkconfig_option("CONFIG_MBEDTLS_ECP_DP_SECP224R1_ENABLED", True)
        add_idf_sdkconfig_option("CONFIG_MBEDTLS_ENTROPY_C", True)
        add_idf_sdkconfig_option("CONFIG_MBEDTLS_CTR_DRBG_C", True)
