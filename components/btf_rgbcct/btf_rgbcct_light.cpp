#include "btf_rgbcct_light.h"

#ifdef USE_ESP32

#include <algorithm>
#include <cinttypes>

#include <esp_attr.h>
#include <esp_clk_tree.h>

#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome::btf_rgbcct {

static const char *const TAG = "btf_rgbcct";

static constexpr size_t WIRE_BYTES_PER_PIXEL = 5;
static constexpr size_t RMT_SYMBOLS_PER_BYTE = 8;

// RMT default clock source frequency, varies by variant (80 MHz on most, 32 MHz on H2).
static uint32_t rmt_resolution_hz() {
  uint32_t freq;
  esp_clk_tree_src_get_freq_hz((soc_module_clk_t) RMT_CLK_SRC_DEFAULT, ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &freq);
  return freq;
}

// Bytes -> RMT symbols (MSB first), followed by one reset symbol.
static size_t IRAM_ATTR HOT encoder_callback(const void *data, size_t size, size_t symbols_written, size_t symbols_free,
                                             rmt_symbol_word_t *symbols, bool *done, void *arg) {
  auto *params = static_cast<LedParams *>(arg);
  const auto *bytes = static_cast<const uint8_t *>(data);
  size_t index = symbols_written / RMT_SYMBOLS_PER_BYTE;

  if (index < size) {
    if (symbols_free < RMT_SYMBOLS_PER_BYTE)
      return 0;
    for (size_t i = 0; i < RMT_SYMBOLS_PER_BYTE; i++)
      symbols[i] = bytes[index] & (1 << (7 - i)) ? params->bit1 : params->bit0;
    return RMT_SYMBOLS_PER_BYTE;
  }

  if (symbols_free < 1)
    return 0;
  symbols[0] = params->reset;
  *done = true;
  return 1;
}

void BtfRgbcctLight::setup() {
  RAMAllocator<uint8_t> allocator(this->use_psram_ ? 0 : RAMAllocator<uint8_t>::ALLOC_INTERNAL);
  this->rgbw_ = allocator.allocate(this->num_leds_ * 4);
  this->warm_ = allocator.allocate(this->num_leds_);
  this->effect_data_ = allocator.allocate(this->num_leds_);
  this->rmt_buf_ = allocator.allocate(this->num_leds_ * WIRE_BYTES_PER_PIXEL);
  if (this->rgbw_ == nullptr || this->warm_ == nullptr || this->effect_data_ == nullptr || this->rmt_buf_ == nullptr) {
    ESP_LOGE(TAG, "Cannot allocate LED buffers!");
    this->mark_failed();
    return;
  }
  memset(this->rgbw_, 0, this->num_leds_ * 4);
  memset(this->warm_, light::to_uint8_scale(this->global_warm_), this->num_leds_);
  memset(this->effect_data_, 0, this->num_leds_);
  memset(this->rmt_buf_, 0, this->num_leds_ * WIRE_BYTES_PER_PIXEL);

  rmt_tx_channel_config_t channel;
  memset(&channel, 0, sizeof(channel));
  channel.clk_src = RMT_CLK_SRC_DEFAULT;
  channel.resolution_hz = rmt_resolution_hz();
  channel.gpio_num = gpio_num_t(this->pin_);
  channel.mem_block_symbols = this->rmt_symbols_;
  channel.trans_queue_depth = 1;
  channel.flags.invert_out = this->inverted_;
  if (rmt_new_tx_channel(&channel, &this->channel_) != ESP_OK) {
    ESP_LOGE(TAG, "Channel creation failed");
    this->mark_failed();
    return;
  }

  rmt_simple_encoder_config_t encoder;
  memset(&encoder, 0, sizeof(encoder));
  encoder.callback = encoder_callback;
  encoder.arg = &this->params_;
  encoder.min_chunk_size = RMT_SYMBOLS_PER_BYTE;
  if (rmt_new_simple_encoder(&encoder, &this->encoder_) != ESP_OK) {
    ESP_LOGE(TAG, "Encoder creation failed");
    this->mark_failed();
    return;
  }

  if (rmt_enable(this->channel_) != ESP_OK) {
    ESP_LOGE(TAG, "Enabling channel failed");
    this->mark_failed();
    return;
  }
}

void BtfRgbcctLight::set_led_params(uint32_t bit0_high, uint32_t bit0_low, uint32_t bit1_high, uint32_t bit1_low,
                                    uint32_t reset_time) {
  float ratio = (float) rmt_resolution_hz() / 1e09f;
  this->params_.bit0.duration0 = (uint32_t) (ratio * bit0_high);
  this->params_.bit0.level0 = 1;
  this->params_.bit0.duration1 = (uint32_t) (ratio * bit0_low);
  this->params_.bit0.level1 = 0;
  this->params_.bit1.duration0 = (uint32_t) (ratio * bit1_high);
  this->params_.bit1.level0 = 1;
  this->params_.bit1.duration1 = (uint32_t) (ratio * bit1_low);
  this->params_.bit1.level1 = 0;
  // Reset/latch: low for the whole time, split over both halves (a zero duration would end the transmission).
  uint32_t half = std::max<uint32_t>(1, (uint32_t) (ratio * reset_time / 2));
  this->params_.reset.duration0 = half;
  this->params_.reset.level0 = 0;
  this->params_.reset.duration1 = half;
  this->params_.reset.level1 = 0;
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
  memset(this->warm_, raw, this->num_leds_);
  this->schedule_show();
}

void BtfRgbcctLight::set_pixel_warm_fraction(int32_t index, float warm) {
  if (this->warm_ == nullptr)
    return;
  this->warm_[light::interpret_index(index, this->num_leds_)] = light::to_uint8_scale(clamp(warm, 0.0f, 1.0f));
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
  if (this->channel_ == nullptr)
    return;

  // Protect from refreshing too often; retry next loop so the change isn't lost.
  uint32_t now = micros();
  uint32_t rate = this->max_refresh_rate_.value_or(0);
  if (rate != 0 && (now - this->last_refresh_) < rate) {
    this->schedule_show();
    return;
  }
  this->last_refresh_ = now;
  this->mark_shown_();

  // The previous frame is still read by RMT until it is done.
  if (rmt_tx_wait_all_done(this->channel_, 1000) != ESP_OK) {
    ESP_LOGE(TAG, "RMT TX timeout");
    this->status_set_warning();
    return;
  }

  this->pack_wire_buffer_();

  rmt_transmit_config_t config;
  memset(&config, 0, sizeof(config));
  if (rmt_transmit(this->channel_, this->encoder_, this->rmt_buf_, this->num_leds_ * WIRE_BYTES_PER_PIXEL, &config) !=
      ESP_OK) {
    ESP_LOGE(TAG, "RMT TX error");
    this->status_set_warning();
    return;
  }
  this->status_clear_warning();
}

void BtfRgbcctLight::pack_wire_buffer_() {
  // Wire offsets of R, G, B inside the first 3 bytes of a pixel.
  uint8_t r = 0, g = 1, b = 2;
  switch (this->rgb_order_) {
    case ORDER_RGB:
      r = 0, g = 1, b = 2;
      break;
    case ORDER_RBG:
      r = 0, g = 2, b = 1;
      break;
    case ORDER_GRB:
      r = 1, g = 0, b = 2;
      break;
    case ORDER_GBR:
      r = 2, g = 0, b = 1;
      break;
    case ORDER_BGR:
      r = 2, g = 1, b = 0;
      break;
    case ORDER_BRG:
      r = 1, g = 2, b = 0;
      break;
  }
  const uint8_t ww_pos = this->white_order_ == WHITE_ORDER_WW_CW ? 3 : 4;
  const uint8_t cw_pos = this->white_order_ == WHITE_ORDER_WW_CW ? 4 : 3;

  for (uint16_t p = 0; p < this->num_leds_; p++) {
    const uint8_t *px = &this->rgbw_[p * 4];
    uint8_t *out = &this->rmt_buf_[p * WIRE_BYTES_PER_PIXEL];
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
}

light::ESPColorView BtfRgbcctLight::get_view_internal(int32_t index) const {
  uint8_t *px = &this->rgbw_[index * 4];
  return {px, px + 1, px + 2, px + 3, &this->effect_data_[index], &this->correction_};
}

void BtfRgbcctLight::dump_config() {
  ESP_LOGCONFIG(TAG,
                "BTF RGBCCT light:\n"
                "  Pin: %u\n"
                "  RMT symbols: %" PRIu32 "\n"
                "  Max refresh rate: %" PRIu32 " us\n"
                "  Pixels: %u (5 bytes each)\n"
                "  White order: %s\n"
                "  Cold/warm white: %.0f K / %.0f K\n"
                "  Constant brightness: %s\n"
                "  Color interlock: %s",
                this->pin_, this->rmt_symbols_, this->max_refresh_rate_.value_or(0), this->num_leds_,
                this->white_order_ == WHITE_ORDER_WW_CW ? "WW, CW" : "CW, WW",
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
