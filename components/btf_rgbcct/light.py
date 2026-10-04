from esphome import pins
import esphome.codegen as cg
from esphome.components import esp32, esp32_rmt, light
from esphome.components.const import CONF_USE_PSRAM
from esphome.components.esp32 import include_builtin_idf_component
import esphome.config_validation as cv
from esphome.const import (
    CONF_COLD_WHITE_COLOR_TEMPERATURE,
    CONF_COLOR_INTERLOCK,
    CONF_CONSTANT_BRIGHTNESS,
    CONF_INVERTED,
    CONF_NUM_LEDS,
    CONF_NUMBER,
    CONF_OUTPUT_ID,
    CONF_PIN,
    CONF_RMT_SYMBOLS,
    CONF_WARM_WHITE_COLOR_TEMPERATURE,
)

DEPENDENCIES = ["esp32"]

CONF_POWER_PIN = "power_pin"

btf_rgbcct_ns = cg.esphome_ns.namespace("btf_rgbcct")
BtfRgbcctLight = btf_rgbcct_ns.class_("BtfRgbcctLight", light.AddressableLight)

# Wire format (6 bytes per segment: R, G, B | WW, CW, unused) and bit timings
# are fixed for this strip in the C++ component.
CONFIG_SCHEMA = cv.All(
    esp32.only_on_variant(
        unsupported=list(esp32_rmt.VARIANTS_NO_RMT),
        msg_prefix="BTF RGBCCT light",
    ),
    light.ADDRESSABLE_LIGHT_SCHEMA.extend(
        {
            cv.GenerateID(CONF_OUTPUT_ID): cv.declare_id(BtfRgbcctLight),
            cv.Required(CONF_PIN): pins.internal_gpio_output_pin_schema,
            # Number of segments (one chip pair each)
            cv.Required(CONF_NUM_LEDS): cv.positive_not_null_int,
            # Plain on/off switch of the strip supply (not a PWM output)
            cv.Optional(CONF_POWER_PIN): pins.gpio_output_pin_schema,
            cv.Optional(
                CONF_COLD_WHITE_COLOR_TEMPERATURE, default="6500K"
            ): cv.color_temperature,
            cv.Optional(
                CONF_WARM_WHITE_COLOR_TEMPERATURE, default="3000K"
            ): cv.color_temperature,
            cv.Optional(CONF_CONSTANT_BRIGHTNESS, default=True): cv.boolean,
            cv.Optional(CONF_COLOR_INTERLOCK, default=True): cv.boolean,
            cv.SplitDefault(
                CONF_RMT_SYMBOLS,
                esp32=192,
                esp32_c3=96,
                esp32_c5=96,
                esp32_c6=96,
                esp32_h2=96,
                esp32_p4=192,
                esp32_s2=192,
                esp32_s3=192,
            ): cv.int_range(min=2),
            cv.Optional(CONF_USE_PSRAM, default=True): cv.boolean,
        }
    ).extend(cv.COMPONENT_SCHEMA),
)


async def to_code(config):
    # Re-enable ESP-IDF's RMT driver (excluded by default to save compile time)
    include_builtin_idf_component("esp_driver_rmt")

    var = cg.new_Pvariable(config[CONF_OUTPUT_ID])
    await light.register_light(var, config)
    await cg.register_component(var, config)

    cg.add(var.set_num_leds(config[CONF_NUM_LEDS]))
    cg.add(var.set_pin(config[CONF_PIN][CONF_NUMBER]))
    if config[CONF_PIN][CONF_INVERTED]:
        cg.add(var.set_inverted(True))
    if CONF_POWER_PIN in config:
        power_pin = await cg.gpio_pin_expression(config[CONF_POWER_PIN])
        cg.add(var.set_power_pin(power_pin))
    cg.add(var.set_cold_white_temperature(config[CONF_COLD_WHITE_COLOR_TEMPERATURE]))
    cg.add(var.set_warm_white_temperature(config[CONF_WARM_WHITE_COLOR_TEMPERATURE]))
    cg.add(var.set_constant_brightness(config[CONF_CONSTANT_BRIGHTNESS]))
    cg.add(var.set_color_interlock(config[CONF_COLOR_INTERLOCK]))
    cg.add(var.set_rmt_symbols(config[CONF_RMT_SYMBOLS]))
    cg.add(var.set_use_psram(config[CONF_USE_PSRAM]))
