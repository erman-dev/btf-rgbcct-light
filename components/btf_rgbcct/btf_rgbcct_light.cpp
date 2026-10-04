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

static constexpr size_t RMT_SYMBOLS_PER_BYTE = 8;
static constexpr uint32_t POWER_ON_RESEND_MS = 200;
// power_pin: time for the strip supply to settle before data is sent, and how long the strip
// stays powered after it went dark.
static constexpr uint32_t POWER_ON_DELAY_MS = 100;
static constexpr uint32_t POWER_OFF_DELAY_MS = 2000;

// Each segment is driven by two 3-channel chips: R, G, B | WW, CW, unused.
static constexpr size_t BYTES_PER_PIXEL = 6;
// WS2812-style bit timings (ns) and latch time, as tested on the strip.
static constexpr uint32_t BIT0_HIGH_NS = 400;
static constexpr uint32_t BIT0_LOW_NS = 1000;
static constexpr uint32_t BIT1_HIGH_NS = 1000;
static constexpr uint32_t BIT1_LOW_NS = 400;
static constexpr uint32_t RESET_NS = 300000;

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
  this->init_led_params_();

  RAMAllocator<uint8_t> allocator(this->use_psram_ ? 0 : RAMAllocator<uint8_t>::ALLOC_INTERNAL);
  this->rgbw_ = allocator.allocate(this->num_leds_ * 4);
  this->warm_ = allocator.allocate(this->num_leds_);
  this->effect_data_ = allocator.allocate(this->num_leds_);
  this->rmt_buf_ = allocator.allocate(this->wire_size_());
  if (this->rgbw_ == nullptr || this->warm_ == nullptr || this->effect_data_ == nullptr || this->rmt_buf_ == nullptr) {
    ESP_LOGE(TAG, "Cannot allocate LED buffers!");
    this->mark_failed();
    return;
  }
  memset(this->rgbw_, 0, this->num_leds_ * 4);
  memset(this->warm_, light::to_uint8_scale(this->global_warm_), this->num_leds_);
  memset(this->effect_data_, 0, this->num_leds_);
  memset(this->rmt_buf_, 0, this->wire_size_());

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

  if (this->power_pin_ != nullptr) {
    // Strip unpowered and data line released until the light is turned on.
    this->power_pin_->setup();
    this->power_pin_->digital_write(false);
    this->release_data_pin_();
    return;
  }

  if (!this->start_channel_()) {
    this->mark_failed();
    return;
  }
  // Blank the strip right away: overwrites whatever boot-time noise on the data line latched.
  this->transmit_();
}

bool BtfRgbcctLight::start_channel_() {
  if (this->channel_ != nullptr)
    return true;
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
    this->channel_ = nullptr;
    return false;
  }
  if (rmt_enable(this->channel_) != ESP_OK) {
    ESP_LOGE(TAG, "Enabling channel failed");
    rmt_del_channel(this->channel_);
    this->channel_ = nullptr;
    return false;
  }
  return true;
}

void BtfRgbcctLight::release_data_pin_() {
  if (this->channel_ != nullptr) {
    rmt_tx_wait_all_done(this->channel_, 100);
    rmt_disable(this->channel_);
    rmt_del_channel(this->channel_);
    this->channel_ = nullptr;
  }
  // Input with weak pull-down: no current into an unpowered strip, no noise on the line.
  gpio_config_t io;
  memset(&io, 0, sizeof(io));
  io.pin_bit_mask = 1ULL << this->pin_;
  io.mode = GPIO_MODE_INPUT;
  io.pull_down_en = GPIO_PULLDOWN_ENABLE;
  io.pull_up_en = GPIO_PULLUP_DISABLE;
  gpio_config(&io);
}

void BtfRgbcctLight::power_off_() {
  this->power_off_pending_ = false;
  // Release data first: with a low-side switch, a driven data line would feed the chips once ground is cut.
  this->release_data_pin_();
  this->power_pin_->digital_write(false);
  this->power_state_ = PowerState::OFF;
  ESP_LOGD(TAG, "Strip power off");
}

void BtfRgbcctLight::init_led_params_() {
  const uint32_t bit0_high = BIT0_HIGH_NS, bit0_low = BIT0_LOW_NS;
  const uint32_t bit1_high = BIT1_HIGH_NS, bit1_low = BIT1_LOW_NS;
  const uint32_t reset_time = RESET_NS;
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
  if (this->rmt_buf_ == nullptr || this->encoder_ == nullptr)
    return;

  bool lit = false;
  for (size_t i = 0; i < this->num_leds_ * 4u && !lit; i++)
    lit = this->rgbw_[i] != 0;

  if (this->power_pin_ != nullptr) {
    if (lit) {
      if (this->power_off_pending_) {
        this->cancel_timeout("power_off");
        this->power_off_pending_ = false;
      }
      if (this->power_state_ == PowerState::OFF) {
        // Power first, data once the supply has settled.
        ESP_LOGD(TAG, "Strip power on");
        this->power_pin_->digital_write(true);
        this->power_state_ = PowerState::STARTING;
        this->set_timeout("power_on", POWER_ON_DELAY_MS, [this]() {
          if (!this->start_channel_()) {
            this->status_set_warning();
            return;
          }
          this->power_state_ = PowerState::ON;
          this->schedule_show();
        });
        return;
      }
      if (this->power_state_ == PowerState::STARTING)
        return;  // sent once powered
    } else {
      if (this->power_state_ == PowerState::STARTING) {
        this->cancel_timeout("power_on");
        this->power_pin_->digital_write(false);
        this->power_state_ = PowerState::OFF;
        return;
      }
      if (this->power_state_ == PowerState::OFF)
        return;
      // Send the dark frame below, then cut the supply.
      if (!this->power_off_pending_) {
        this->power_off_pending_ = true;
        this->set_timeout("power_off", POWER_OFF_DELAY_MS, [this]() { this->power_off_(); });
      }
    }
  } else {
    this->mark_shown_();
    // When the strip goes from dark to lit, a power_supply may only be switching on now and the
    // chips miss this frame. Send it again once the supply is up.
    if (lit && !this->was_lit_)
      this->set_timeout("resend", POWER_ON_RESEND_MS, [this]() { this->schedule_show(); });
    this->was_lit_ = lit;
  }

  this->transmit_();
}

void BtfRgbcctLight::transmit_() {
  if (this->channel_ == nullptr)
    return;

  // The previous frame is still read by RMT until it is done.
  if (rmt_tx_wait_all_done(this->channel_, 1000) != ESP_OK) {
    ESP_LOGE(TAG, "RMT TX timeout");
    this->status_set_warning();
    return;
  }

  this->pack_wire_buffer_();
  ESP_LOGV(TAG, "Pixel 0 wire bytes: %02X %02X %02X %02X %02X", this->rmt_buf_[0], this->rmt_buf_[1],
           this->rmt_buf_[2], this->rmt_buf_[3], this->rmt_buf_[4]);

  rmt_transmit_config_t config;
  memset(&config, 0, sizeof(config));
  if (rmt_transmit(this->channel_, this->encoder_, this->rmt_buf_, this->wire_size_(), &config) != ESP_OK) {
    ESP_LOGE(TAG, "RMT TX error");
    this->status_set_warning();
    return;
  }
  this->status_clear_warning();
}

size_t BtfRgbcctLight::wire_size_() const { return this->num_leds_ * BYTES_PER_PIXEL; }

void BtfRgbcctLight::pack_wire_buffer_() {
  for (uint16_t p = 0; p < this->num_leds_; p++) {
    const uint8_t *px = &this->rgbw_[p * 4];
    uint8_t *out = &this->rmt_buf_[p * BYTES_PER_PIXEL];
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
    // Wire order: R, G, B | WW, CW, unused (stays 0)
    out[0] = px[0];
    out[1] = px[1];
    out[2] = px[2];
    out[3] = ww;
    out[4] = cw;
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
                "  Pixels: %u\n"
                "  Cold/warm white: %.0f K / %.0f K\n"
                "  Constant brightness: %s\n"
                "  Color interlock: %s",
                this->pin_, this->rmt_symbols_, this->num_leds_,
                1000000.0f / this->cold_white_mireds_, 1000000.0f / this->warm_white_mireds_,
                YESNO(this->constant_brightness_), YESNO(this->color_interlock_));
  LOG_PIN("  Power pin: ", this->power_pin_);
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
