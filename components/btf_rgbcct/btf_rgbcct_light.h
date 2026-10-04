#pragma once

#ifdef USE_ESP32

#include "esphome/components/light/addressable_light.h"
#include "esphome/core/color.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"

#include <esp_idf_version.h>
#include <driver/rmt_tx.h>

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 3, 0)
#error "btf_rgbcct requires ESP-IDF 5.3 or newer"
#endif

namespace esphome::btf_rgbcct {

enum RGBOrder : uint8_t {
  ORDER_RGB,
  ORDER_RBG,
  ORDER_GRB,
  ORDER_GBR,
  ORDER_BGR,
  ORDER_BRG,
};

enum WhiteOrder : uint8_t {
  WHITE_ORDER_WW_CW,
  WHITE_ORDER_CW_WW,
};

struct LedParams {
  rmt_symbol_word_t bit0;
  rmt_symbol_word_t bit1;
  rmt_symbol_word_t reset;
};

/// RGB + warm white + cold white addressable strip (5 or 6 bytes per pixel), sent over RMT.
///
/// Effects see a normal RGBW pixel. W is the white brightness, split into WW/CW by a
/// per-pixel color temperature that follows the light's color temperature slider.
class BtfRgbcctLight : public light::AddressableLight {
 public:
  void setup() override;
  void write_state(light::LightState *state) override;
  void update_state(light::LightState *state) override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  std::unique_ptr<light::LightTransformer> create_default_transition() override;

  int32_t size() const override { return this->num_leds_; }
  light::LightTraits get_traits() override;
  void clear_effect_data() override {
    if (this->effect_data_ != nullptr)
      memset(this->effect_data_, 0, this->num_leds_);
  }

  void set_pin(uint8_t pin) { this->pin_ = pin; }
  void set_inverted(bool inverted) { this->inverted_ = inverted; }
  void set_num_leds(uint16_t num_leds) { this->num_leds_ = num_leds; }
  void set_rmt_symbols(uint32_t rmt_symbols) { this->rmt_symbols_ = rmt_symbols; }
  void set_use_psram(bool use_psram) { this->use_psram_ = use_psram; }
  void set_max_refresh_rate(uint32_t interval_us) { this->max_refresh_rate_ = interval_us; }
  void set_led_params(uint32_t bit0_high, uint32_t bit0_low, uint32_t bit1_high, uint32_t bit1_low,
                      uint32_t reset_time);
  void set_bytes_per_pixel(uint8_t bytes) { this->bytes_per_pixel_ = bytes; }
  void set_rgb_order(RGBOrder order) { this->rgb_order_ = order; }
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
  void pack_wire_buffer_();
  size_t wire_size_() const { return this->num_leds_ * this->bytes_per_pixel_; }

  uint8_t *rgbw_{nullptr};         // 4 bytes per pixel: R, G, B, W (after gamma + color correction)
  uint8_t *warm_{nullptr};         // 1 byte per pixel: warm share of W, 0..255
  uint8_t *effect_data_{nullptr};  // 1 byte per pixel
  uint8_t *rmt_buf_{nullptr};      // bytes_per_pixel_ per pixel in wire order (6th byte unused), owned by RMT while sending

  LedParams params_{};
  rmt_channel_handle_t channel_{nullptr};
  rmt_encoder_handle_t encoder_{nullptr};
  uint32_t rmt_symbols_{48};
  uint32_t last_refresh_{0};
  optional<uint32_t> max_refresh_rate_{};
  uint16_t num_leds_{0};
  uint8_t pin_{0};
  uint8_t bytes_per_pixel_{5};
  bool inverted_{false};
  bool use_psram_{false};
  bool was_lit_{false};

  float global_warm_{0.5f};
  float cold_white_mireds_{153.0f};
  float warm_white_mireds_{333.0f};
  RGBOrder rgb_order_{ORDER_RGB};
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
