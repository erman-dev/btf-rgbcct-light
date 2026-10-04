#pragma once

#ifdef USE_ESP32

#include "esphome/components/esp32_rmt_led_strip/led_strip.h"
#include "esphome/components/light/addressable_light.h"

namespace esphome::btf_rgbcct {

enum WhiteOrder : uint8_t {
  WHITE_ORDER_WW_CW,
  WHITE_ORDER_CW_WW,
};

/// RGB + warm white + cold white addressable strip (5 bytes per pixel, e.g. WS2805).
///
/// Reuses the RMT driver of esp32_rmt_led_strip: the parent is configured with enough
/// 3-byte "raw" LEDs to hold 5 bytes per real pixel, and this class packs its own
/// per-pixel R, G, B, W + color temperature buffers into that wire buffer before sending.
///
/// Effects see a normal RGBW pixel. W is the white brightness, split into WW/CW by a
/// per-pixel color temperature that follows the light's color temperature slider.
class BtfRgbcctLight : public esp32_rmt_led_strip::ESP32RMTLEDStripLightOutput {
 public:
  void setup() override;
  void write_state(light::LightState *state) override;
  void update_state(light::LightState *state) override;
  void dump_config() override;
  std::unique_ptr<light::LightTransformer> create_default_transition() override;

  int32_t size() const override { return this->pixels_; }
  light::LightTraits get_traits() override;

  void set_pixels(uint16_t pixels) { this->pixels_ = pixels; }
  void set_white_order(WhiteOrder order) { this->white_order_ = order; }
  void set_cold_white_temperature(float mireds) { this->cold_white_mireds_ = mireds; }
  void set_warm_white_temperature(float mireds) { this->warm_white_mireds_ = mireds; }
  void set_constant_brightness(bool constant_brightness) { this->constant_brightness_ = constant_brightness; }
  void set_color_interlock(bool color_interlock) { this->color_interlock_ = color_interlock; }

  /// Per-pixel color temperature, for use in addressable_lambda effects.
  /// It is reset to the light's color temperature when that changes.
  void set_pixel_color_temperature(int32_t index, float kelvin);
  /// Per-pixel warm share of the white channel: 0.0 = cold only, 1.0 = warm only.
  void set_pixel_warm_fraction(int32_t index, float warm);

  /// Light color values to a pixel color, without brightness (W = white brightness).
  Color color_from_values(const light::LightColorValues &val) const;
  /// Warm share of white for these values, or `fallback` if the color mode has no color temperature.
  float warm_fraction_from_values(const light::LightColorValues &val, float fallback) const;
  float get_global_warm_fraction() const { return this->global_warm_; }
  void set_global_warm_fraction(float warm);

 protected:
  light::ESPColorView get_view_internal(int32_t index) const override;

  uint16_t pixels_{0};
  uint8_t *rgbw_{nullptr};  // 4 bytes per pixel: R, G, B, W (after gamma + color correction)
  uint8_t *warm_{nullptr};  // 1 byte per pixel: warm share of W, 0..255
  float global_warm_{0.5f};
  float cold_white_mireds_{153.0f};
  float warm_white_mireds_{333.0f};
  WhiteOrder white_order_{WHITE_ORDER_WW_CW};
  bool constant_brightness_{false};
  bool color_interlock_{false};
};

/// Addressable transition that also fades the color temperature and uses the RGBCCT color mapping.
class BtfRgbcctTransformer : public light::AddressableLightTransformer {
 public:
  explicit BtfRgbcctTransformer(BtfRgbcctLight &light) : AddressableLightTransformer(light), parent_(light) {}

  void start() override;
  optional<light::LightColorValues> apply() override;

 protected:
  BtfRgbcctLight &parent_;
  float start_warm_{0.5f};
  float target_warm_{0.5f};
};

}  // namespace esphome::btf_rgbcct

#endif  // USE_ESP32
