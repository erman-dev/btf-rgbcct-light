from esphome import pins
import esphome.codegen as cg
from esphome.components import esp32, esp32_rmt, light
from esphome.components.const import CONF_USE_PSRAM
from esphome.components.esp32 import include_builtin_idf_component
import esphome.config_validation as cv
from esphome.const import (
    CONF_CHIPSET,
    CONF_COLD_WHITE_COLOR_TEMPERATURE,
    CONF_COLOR_INTERLOCK,
    CONF_CONSTANT_BRIGHTNESS,
    CONF_INVERTED,
    CONF_MAX_REFRESH_RATE,
    CONF_NUM_LEDS,
    CONF_NUMBER,
    CONF_OUTPUT_ID,
    CONF_PIN,
    CONF_RGB_ORDER,
    CONF_RMT_SYMBOLS,
    CONF_WARM_WHITE_COLOR_TEMPERATURE,
)

DEPENDENCIES = ["esp32"]

CONF_WHITE_ORDER = "white_order"
CONF_BIT0_HIGH = "bit0_high"
CONF_BIT0_LOW = "bit0_low"
CONF_BIT1_HIGH = "bit1_high"
CONF_BIT1_LOW = "bit1_low"
CONF_RESET_TIME = "reset_time"

btf_rgbcct_ns = cg.esphome_ns.namespace("btf_rgbcct")
BtfRgbcctLight = btf_rgbcct_ns.class_(
    "BtfRgbcctLight", light.AddressableLight
)

RGBOrder = btf_rgbcct_ns.enum("RGBOrder")
RGB_ORDERS = {
    "RGB": RGBOrder.ORDER_RGB,
    "RBG": RGBOrder.ORDER_RBG,
    "GRB": RGBOrder.ORDER_GRB,
    "GBR": RGBOrder.ORDER_GBR,
    "BGR": RGBOrder.ORDER_BGR,
    "BRG": RGBOrder.ORDER_BRG,
}

WhiteOrder = btf_rgbcct_ns.enum("WhiteOrder")
WHITE_ORDERS = {
    "WW_CW": WhiteOrder.WHITE_ORDER_WW_CW,
    "CW_WW": WhiteOrder.WHITE_ORDER_CW_WW,
}

# bit0_high, bit0_low, bit1_high, bit1_low in ns (same as esp32_rmt_led_strip)
CHIPSETS = {
    "WS2812": (400, 1000, 1000, 400),
    "WS2811": (300, 1090, 1090, 320),
    "SK6812": (300, 900, 600, 600),
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
            cv.Required(CONF_NUM_LEDS): cv.positive_not_null_int,
            cv.Optional(CONF_RGB_ORDER, default="RGB"): cv.enum(RGB_ORDERS, upper=True),
            cv.Optional(CONF_WHITE_ORDER, default="WW_CW"): cv.enum(
                WHITE_ORDERS, upper=True
            ),
            cv.Optional(CONF_CHIPSET): cv.one_of(*CHIPSETS, upper=True),
            cv.Inclusive(CONF_BIT0_HIGH, "custom"): cv.positive_time_period_nanoseconds,
            cv.Inclusive(CONF_BIT0_LOW, "custom"): cv.positive_time_period_nanoseconds,
            cv.Inclusive(CONF_BIT1_HIGH, "custom"): cv.positive_time_period_nanoseconds,
            cv.Inclusive(CONF_BIT1_LOW, "custom"): cv.positive_time_period_nanoseconds,
            # WS2805 latches after >280 us low
            cv.Optional(
                CONF_RESET_TIME, default="300us"
            ): cv.positive_time_period_nanoseconds,
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
    cv.has_at_most_one_key(CONF_CHIPSET, CONF_BIT0_HIGH),
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
    if CONF_MAX_REFRESH_RATE in config:
        cg.add(var.set_max_refresh_rate(config[CONF_MAX_REFRESH_RATE]))

    if CONF_BIT0_HIGH in config:
        timings = (
            config[CONF_BIT0_HIGH],
            config[CONF_BIT0_LOW],
            config[CONF_BIT1_HIGH],
            config[CONF_BIT1_LOW],
        )
    else:
        timings = CHIPSETS[config.get(CONF_CHIPSET, "WS2812")]
    cg.add(var.set_led_params(*timings, config[CONF_RESET_TIME]))

    cg.add(var.set_rgb_order(config[CONF_RGB_ORDER]))
    cg.add(var.set_white_order(config[CONF_WHITE_ORDER]))
    cg.add(var.set_cold_white_temperature(config[CONF_COLD_WHITE_COLOR_TEMPERATURE]))
    cg.add(var.set_warm_white_temperature(config[CONF_WARM_WHITE_COLOR_TEMPERATURE]))
    cg.add(var.set_constant_brightness(config[CONF_CONSTANT_BRIGHTNESS]))
    cg.add(var.set_color_interlock(config[CONF_COLOR_INTERLOCK]))
    cg.add(var.set_rmt_symbols(config[CONF_RMT_SYMBOLS]))
    cg.add(var.set_use_psram(config[CONF_USE_PSRAM]))
