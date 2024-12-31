import esphome.codegen as cg
from esphome.components import binary_sensor, output, text_sensor, uart
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_UART_ID,
    DEVICE_CLASS_CONNECTIVITY,
    ENTITY_CATEGORY_DIAGNOSTIC,
)

DEPENDENCIES = ["uart", "network"]

# Namespace for the component
esph_hm_rf_eth_ns = cg.esphome_ns.namespace("esph_hm")
esp_esph_hm_rf_eth = esph_hm_rf_eth_ns.class_("esph_hm", cg.PollingComponent)

# Configuration options
# CONF_UART_ID = "uart_id"
CONF_RESET_OUTPUT = "reset_output"
CONF_RED_LED = "red_led"
CONF_GREEN_LED = "green_led"
CONF_BLUE_LED = "blue_led"
CONF_CONNECTED = "connected"
CONF_RADIO_MODULE_TYPE = "radio_module_type"
CONF_FIRMWARE_VERSION = "firmware_version"
CONF_SERIAL = "serial"
CONF_SGTIN = "SGTIN"


CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(esp_esph_hm_rf_eth),
            cv.Required(CONF_UART_ID): cv.use_id(uart.IDFUARTComponent),
            cv.Required(CONF_RESET_OUTPUT): cv.use_id(output.BinaryOutput),
            cv.Optional(CONF_RED_LED): cv.use_id(output.BinaryOutput),
            cv.Optional(CONF_GREEN_LED): cv.use_id(output.BinaryOutput),
            cv.Optional(CONF_BLUE_LED): cv.use_id(output.BinaryOutput),
            cv.Optional(CONF_CONNECTED): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_CONNECTIVITY,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_RADIO_MODULE_TYPE): text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC
            ),
            cv.Optional(CONF_FIRMWARE_VERSION): text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC
            ),
            cv.Optional(CONF_SERIAL): text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC
            ),
            cv.Optional(CONF_SGTIN): text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC
            ),
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.polling_component_schema("10s"))
)

FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    "esp_hm",
    baud_rate=115200,
    require_tx=True,
    require_rx=True,
    data_bits=8,
    parity=None,
    stop_bits=1,
)


async def to_code(config):
    uart_component = await cg.get_variable(config[CONF_UART_ID])
    reset_output = await cg.get_variable(config[CONF_RESET_OUTPUT])
    var = cg.new_Pvariable(config[CONF_ID], uart_component, reset_output)

    if CONF_RED_LED in config:
        red = await cg.get_variable(config[CONF_RED_LED])
        cg.add(var.set_red_led(red))

    if CONF_GREEN_LED in config:
        green = await cg.get_variable(config[CONF_GREEN_LED])
        cg.add(var.set_green_led(green))

    if CONF_BLUE_LED in config:
        blue = await cg.get_variable(config[CONF_BLUE_LED])
        cg.add(var.set_blue_led(blue))

    if CONF_CONNECTED in config:
        connected_sensor = await binary_sensor.new_binary_sensor(config[CONF_CONNECTED])
        cg.add(var.set_connected_sensor(connected_sensor))

    if CONF_RADIO_MODULE_TYPE in config:
        radio_module_type_sensor = await text_sensor.new_text_sensor(
            config[CONF_RADIO_MODULE_TYPE]
        )
        cg.add(var.set_radio_module_sensor(radio_module_type_sensor))

    if CONF_FIRMWARE_VERSION in config:
        firmware_sensor = await text_sensor.new_text_sensor(
            config[CONF_FIRMWARE_VERSION]
        )
        cg.add(var.set_firmware_sensor(firmware_sensor))

    if CONF_SERIAL in config:
        serial_sensor = await text_sensor.new_text_sensor(config[CONF_SERIAL])
        cg.add(var.set_serial_sensor(serial_sensor))

    if CONF_SGTIN in config:
        SGTIN_sensor = await text_sensor.new_text_sensor(config[CONF_SGTIN])
        cg.add(var.set_SGTIN_sensor(SGTIN_sensor))

    await cg.register_component(var, config)
