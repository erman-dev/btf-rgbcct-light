#include "btf_rgbcct_light.h"

#ifdef USE_ESP32

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome::btf_rgbcct {

static const char *const TAG = "btf_rgbcct";

static constexpr size_t WIRE_BYTES_PER_PIXEL = 5;

void BtfRgbcctLight::setup() {
  // Parent allocates the wire buffer (num_leds_ raw 3-byte LEDs >= 5 bytes per pixel) and sets up RMT.
  ESP32RMTLEDStripLightOutput::setup();
  if (this->is_failed())
    return;

  RAMAllocator<uint8_t> allocator(this->use_psram_ ? 0 : RAMAllocator<uint8_t>::ALLOC_INTERNAL);
  this->rgbw_ = allocator.allocate(this->pixels_ * 4);
  this->warm_ = allocator.allocate(this->pixels_);
  if (this->rgbw_ == nullptr || this->warm_ == nullptr) {
    ESP_LOGE(TAG, "Cannot allocate pixel buffers!");
    this->mark_failed();
    return;
  }
  memset(this->rgbw_, 0, this->pixels_ * 4);
  memset(this->warm_, light::to_uint8_scale(this->global_warm_), this->pixels_);
}

light::LightTraits BtfRgbcctLight::get_traits() {
  auto traits = light::LightTraits();
  if (this->color_interlock_) {
    traits.set_supported_color_modes({light::ColorMode::RGB, light::ColorMode::COLOR_TEMPERATURE});
  } else {
    traits.set_supported_color_modes({light::ColorMode::RGB_COLOR_TEMPERATURE, light::ColorMode::COLOR_TEMPERATURE});
  }
  traits.set_min_mireds(this->cold_white_mireds_);
  traits.set_max_mireds(this->warm_white_mireds_);
  return traits;
}

Color BtfRgbcctLight::color_from_values(const light::LightColorValues &val) const {
  const auto mode = val.get_color_mode();
  const bool has_rgb = mode & light::ColorCapability::RGB;
  Color color(0, 0, 0, 0);
  if (has_rgb) {
    color.red = light::to_uint8_scale(val.get_color_brightness() * val.get_red());
    color.green = light::to_uint8_scale(val.get_color_brightness() * val.get_green());
    color.blue = light::to_uint8_scale(val.get_color_brightness() * val.get_blue());
  }
  if (mode & light::ColorCapability::COLOR_TEMPERATURE) {
    // Same as LightColorValues::as_ct(): white-only mode has no separate white level.
    color.white = light::to_uint8_scale(has_rgb ? val.get_white() : 1.0f);
  }
  return color;
}

float BtfRgbcctLight::warm_fraction_from_values(const light::LightColorValues &val, float fallback) const {
  if (!(val.get_color_mode() & light::ColorCapability::COLOR_TEMPERATURE))
    return fallback;
  float range = this->warm_white_mireds_ - this->cold_white_mireds_;
  if (range <= 0.0f)
    return fallback;
  return clamp((val.get_color_temperature() - this->cold_white_mireds_) / range, 0.0f, 1.0f);
}

void BtfRgbcctLight::set_global_warm_fraction(float warm) {
  uint8_t old_raw = light::to_uint8_scale(this->global_warm_);
  this->global_warm_ = warm;
  uint8_t raw = light::to_uint8_scale(warm);
  if (raw == old_raw || this->warm_ == nullptr)
    return;
  memset(this->warm_, raw, this->pixels_);
  this->schedule_show();
}

void BtfRgbcctLight::set_pixel_warm_fraction(int32_t index, float warm) {
  if (this->warm_ == nullptr)
    return;
  this->warm_[light::interpret_index(index, this->pixels_)] = light::to_uint8_scale(clamp(warm, 0.0f, 1.0f));
}

void BtfRgbcctLight::set_pixel_color_temperature(int32_t index, float kelvin) {
  if (kelvin <= 0.0f || this->warm_white_mireds_ <= this->cold_white_mireds_)
    return;
  float mireds = 1000000.0f / kelvin;
  this->set_pixel_warm_fraction(
      index, (mireds - this->cold_white_mireds_) / (this->warm_white_mireds_ - this->cold_white_mireds_));
}

void BtfRgbcctLight::update_state(light::LightState *state) {
  // Same as AddressableLight::update_state(), but with the RGBCCT color mapping.
  auto val = state->current_values;
  this->set_global_warm_fraction(this->warm_fraction_from_values(val, this->global_warm_));
  this->correction_.set_local_brightness(light::to_uint8_scale(val.get_brightness() * val.get_state()));

  if (this->is_effect_active())
    return;

  this->all() = this->color_from_values(val);
  this->schedule_show();
}

std::unique_ptr<light::LightTransformer> BtfRgbcctLight::create_default_transition() {
  return make_unique<BtfRgbcctTransformer>(*this);
}

void BtfRgbcctLight::write_state(light::LightState *state) {
  if (this->rgbw_ == nullptr)
    return;

  // Wire offsets of R, G, B inside the first 3 bytes of a pixel.
  uint8_t r = 0, g = 1, b = 2;
  switch (this->rgb_order_) {
    case esp32_rmt_led_strip::ORDER_RGB:
      r = 0, g = 1, b = 2;
      break;
    case esp32_rmt_led_strip::ORDER_RBG:
      r = 0, g = 2, b = 1;
      break;
    case esp32_rmt_led_strip::ORDER_GRB:
      r = 1, g = 0, b = 2;
      break;
    case esp32_rmt_led_strip::ORDER_GBR:
      r = 2, g = 0, b = 1;
      break;
    case esp32_rmt_led_strip::ORDER_BGR:
      r = 2, g = 1, b = 0;
      break;
    case esp32_rmt_led_strip::ORDER_BRG:
      r = 1, g = 2, b = 0;
      break;
  }
  const uint8_t ww_pos = this->white_order_ == WHITE_ORDER_WW_CW ? 3 : 4;
  const uint8_t cw_pos = this->white_order_ == WHITE_ORDER_WW_CW ? 4 : 3;

  for (uint16_t p = 0; p < this->pixels_; p++) {
    const uint8_t *px = &this->rgbw_[p * 4];
    uint8_t *out = &this->buf_[p * WIRE_BYTES_PER_PIXEL];
    const uint16_t w = px[3];
    const uint16_t k = this->warm_[p];
    uint16_t ww, cw;
    if (this->constant_brightness_) {
      // WW + CW = W
      ww = (w * k + 127) / 255;
      cw = w - ww;
    } else if (k >= 128) {
      // Dominant channel at W, the other scaled by its ratio (both at W in the middle)
      ww = w;
      cw = (w * (255 - k) + k / 2) / k;
    } else {
      cw = w;
      ww = (w * k + (255 - k) / 2) / (255 - k);
    }
    out[r] = px[0];
    out[g] = px[1];
    out[b] = px[2];
    out[ww_pos] = ww;
    out[cw_pos] = cw;
  }
  // Zero the padding bytes of the last raw LED.
  size_t used = this->pixels_ * WIRE_BYTES_PER_PIXEL;
  memset(this->buf_ + used, 0, this->num_leds_ * 3 - used);

  ESP32RMTLEDStripLightOutput::write_state(state);
}

light::ESPColorView BtfRgbcctLight::get_view_internal(int32_t index) const {
  uint8_t *px = &this->rgbw_[index * 4];
  return {px, px + 1, px + 2, px + 3, &this->effect_data_[index], &this->correction_};
}

void BtfRgbcctLight::dump_config() {
  ESP32RMTLEDStripLightOutput::dump_config();
  ESP_LOGCONFIG(TAG,
                "BTF RGBCCT:\n"
                "  Pixels: %u (5 bytes each)\n"
                "  White order: %s\n"
                "  Cold/warm white: %.0f K / %.0f K\n"
                "  Constant brightness: %s\n"
                "  Color interlock: %s",
                this->pixels_, this->white_order_ == WHITE_ORDER_WW_CW ? "WW, CW" : "CW, WW",
                1000000.0f / this->cold_white_mireds_, 1000000.0f / this->warm_white_mireds_,
                YESNO(this->constant_brightness_), YESNO(this->color_interlock_));
}

void BtfRgbcctTransformer::start() {
  AddressableLightTransformer::start();
  this->start_warm_ = this->parent_.get_global_warm_fraction();
  this->target_warm_ = this->parent_.warm_fraction_from_values(this->target_values_, this->start_warm_);
  if (this->parent_.is_effect_active())
    return;
  // Replace the parent's RGBW target with the RGBCCT mapping.
  this->target_color_ = this->parent_.color_from_values(this->target_values_);
  this->target_color_ *= light::to_uint8_scale(this->target_values_.get_brightness() * this->target_values_.get_state());
}

optional<light::LightColorValues> BtfRgbcctTransformer::apply() {
  auto result = AddressableLightTransformer::apply();
  // With an effect running, the parent returns interpolated values and update_state() handles the color temperature.
  if (!this->parent_.is_effect_active()) {
    float progress = LightTransformer::smoothed_progress(this->get_progress_());
    this->parent_.set_global_warm_fraction(this->start_warm_ + (this->target_warm_ - this->start_warm_) * progress);
  }
  return result;
}

}  // namespace esphome::btf_rgbcct

#endif  // USE_ESP32
