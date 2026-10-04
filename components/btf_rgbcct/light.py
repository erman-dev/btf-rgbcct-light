from esphome import pins
import esphome.codegen as cg
from esphome.components import esp32, esp32_rmt, light
from esphome.components.const import CONF_USE_PSRAM
from esphome.components.esp32_rmt_led_strip import light as rmt_led_strip
import esphome.config_validation as cv
from esphome.const import (
    CONF_CHIPSET,
    CONF_COLD_WHITE_COLOR_TEMPERATURE,
    CONF_COLOR_INTERLOCK,
    CONF_CONSTANT_BRIGHTNESS,
    CONF_IS_RGBW,
    CONF_MAX_REFRESH_RATE,
    CONF_NUM_LEDS,
    CONF_OUTPUT_ID,
    CONF_PIN,
    CONF_RGB_ORDER,
    CONF_RMT_SYMBOLS,
    CONF_WARM_WHITE_COLOR_TEMPERATURE,
)

AUTO_LOAD = ["esp32_rmt_led_strip"]
DEPENDENCIES = ["esp32"]

CONF_WHITE_ORDER = "white_order"

btf_rgbcct_ns = cg.esphome_ns.namespace("btf_rgbcct")
BtfRgbcctLight = btf_rgbcct_ns.class_(
    "BtfRgbcctLight", rmt_led_strip.ESP32RMTLEDStripLightOutput
)

WhiteOrder = btf_rgbcct_ns.enum("WhiteOrder")
WHITE_ORDERS = {
    "WW_CW": WhiteOrder.WHITE_ORDER_WW_CW,
    "CW_WW": WhiteOrder.WHITE_ORDER_CW_WW,
}

CONFIG_SCHEMA = cv.All(
    esp32.only_on_variant(
        unsupported=list(esp32_rmt.VARIANTS_NO_RMT),
        msg_prefix="BTF RGBCCT light",
    ),
    light.ADDRESSABLE_LIGHT_SCHEMA.extend(
        {
            cv.GenerateID(CONF_OUTPUT_ID): cv.declare_id(BtfRgbcctLight),
            cv.Required(CONF_PIN): pins.internal_gpio_output_pin_schema,
            # Real 5-byte pixels, not 3-byte raw LEDs
            cv.Required(CONF_NUM_LEDS): cv.positive_not_null_int,
            cv.Optional(CONF_RGB_ORDER, default="RGB"): cv.enum(
                rmt_led_strip.RGB_ORDERS, upper=True
            ),
            cv.Optional(CONF_WHITE_ORDER, default="WW_CW"): cv.enum(
                WHITE_ORDERS, upper=True
            ),
            cv.Optional(CONF_CHIPSET, default="WS2812"): cv.one_of(
                *rmt_led_strip.CHIPSETS, upper=True
            ),
            cv.Optional(
                CONF_COLD_WHITE_COLOR_TEMPERATURE, default="6500K"
            ): cv.color_temperature,
            cv.Optional(
                CONF_WARM_WHITE_COLOR_TEMPERATURE, default="3000K"
            ): cv.color_temperature,
            cv.Optional(CONF_CONSTANT_BRIGHTNESS, default=False): cv.boolean,
            cv.Optional(CONF_COLOR_INTERLOCK, default=False): cv.boolean,
            cv.Optional(CONF_MAX_REFRESH_RATE): cv.positive_time_period_microseconds,
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
    pixels = config[CONF_NUM_LEDS]
    # The parent driver sends num_leds * 3 bytes: use enough raw LEDs for 5 bytes per pixel.
    rmt_config = {
        **config,
        CONF_NUM_LEDS: (pixels * 5 + 2) // 3,
        CONF_IS_RGBW: False,
        rmt_led_strip.CONF_IS_WRGB: False,
        rmt_led_strip.CONF_RESET_HIGH: 0,
        rmt_led_strip.CONF_RESET_LOW: 0,
    }
    await rmt_led_strip.to_code(rmt_config)

    var = await cg.get_variable(config[CONF_OUTPUT_ID])
    cg.add(var.set_pixels(pixels))
    cg.add(var.set_white_order(config[CONF_WHITE_ORDER]))
    cg.add(var.set_cold_white_temperature(config[CONF_COLD_WHITE_COLOR_TEMPERATURE]))
    cg.add(var.set_warm_white_temperature(config[CONF_WARM_WHITE_COLOR_TEMPERATURE]))
    cg.add(var.set_constant_brightness(config[CONF_CONSTANT_BRIGHTNESS]))
    cg.add(var.set_color_interlock(config[CONF_COLOR_INTERLOCK]))
