#include "light_controller.hpp"
#include "board_pins.hpp"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"

#include <algorithm>
#include <cmath>

static const char* TAG = "light";

static led_strip_handle_t s_led = nullptr;

LightController& LightController::instance() {
    static LightController inst;
    return inst;
}

void LightController::init() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (led_ready_) {
        return;
    }

    led_strip_config_t strip_config = {};
    strip_config.strip_gpio_num = BOARD_RGB_LED;
    strip_config.max_leds = 1;
    strip_config.led_model = LED_MODEL_WS2812;
#if defined(LED_STRIP_COLOR_COMPONENT_FMT_GRB)
    strip_config.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB;
#elif defined(LED_PIXEL_FORMAT_GRB)
    strip_config.led_pixel_format = LED_PIXEL_FORMAT_GRB;
#endif

    led_strip_rmt_config_t rmt_config = {};
    rmt_config.resolution_hz = 10 * 1000 * 1000;

    esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &s_led);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "led_strip_new_rmt_device failed: %s", esp_err_to_name(err));
        return;
    }
    led_ready_ = true;
    apply_locked();
}

void LightController::set_on(bool on) {
    Listener cb;
    State snapshot;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (state_.on == on) {
            return;
        }
        state_.on = on;
        apply_locked();
        cb = listener_;
        snapshot = state_;
    }
    if (cb) {
        cb(snapshot);
    }
}

void LightController::set_brightness(int brightness) {
    brightness = std::clamp(brightness, 0, 100);
    Listener cb;
    State snapshot;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (state_.brightness == brightness) {
            return;
        }
        state_.brightness = brightness;
        apply_locked();
        cb = listener_;
        snapshot = state_;
    }
    if (cb) {
        cb(snapshot);
    }
}

void LightController::set_color_temp(uint32_t mireds) {
    mireds = std::clamp<uint32_t>(mireds, 140, 500);
    Listener cb;
    State snapshot;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (state_.color_temp_mireds == mireds) {
            return;
        }
        state_.color_temp_mireds = mireds;
        apply_locked();
        cb = listener_;
        snapshot = state_;
    }
    if (cb) {
        cb(snapshot);
    }
}

void LightController::toggle_color_temp() {
    Listener cb;
    State snapshot;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const bool warm = state_.color_temp_mireds >= ((LIGHT_WARM_MIREDS + LIGHT_COOL_MIREDS) / 2);
        state_.color_temp_mireds = warm ? LIGHT_COOL_MIREDS : LIGHT_WARM_MIREDS;
        apply_locked();
        cb = listener_;
        snapshot = state_;
    }
    if (cb) {
        cb(snapshot);
    }
}

int LightController::snap_up(int brightness) {
    if (brightness >= 100) {
        return 100;
    }
    return std::min(100, ((brightness / LIGHT_PWM_STEP) + 1) * LIGHT_PWM_STEP);
}

int LightController::snap_down(int brightness) {
    if (brightness <= 0) {
        return 0;
    }
    const int stepped = ((brightness - 1) / LIGHT_PWM_STEP) * LIGHT_PWM_STEP;
    return std::max(0, stepped);
}

void LightController::encoder_brighter() {
    Listener cb;
    State snapshot;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const int next = snap_up(state_.brightness);
        const bool next_on = next > 0;
        if (state_.brightness == next && state_.on == next_on) {
            return;
        }
        state_.brightness = next;
        state_.on = next_on;
        apply_locked();
        cb = listener_;
        snapshot = state_;
    }
    if (cb) {
        cb(snapshot);
    }
}

void LightController::encoder_dimmer() {
    Listener cb;
    State snapshot;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const int next = snap_down(state_.brightness);
        const bool next_on = next > 0;
        if (state_.brightness == next && state_.on == next_on) {
            return;
        }
        state_.brightness = next;
        state_.on = next_on;
        apply_locked();
        cb = listener_;
        snapshot = state_;
    }
    if (cb) {
        cb(snapshot);
    }
}

void LightController::identify() {
    ESP_LOGI(TAG, "Identify");
    uint8_t r, g, b;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        compute_rgb_locked(r, g, b);
    }
    if (!s_led) {
        return;
    }
    for (int i = 0; i < 3; ++i) {
        led_strip_set_pixel(s_led, 0, 0, 0, 0);
        led_strip_refresh(s_led);
        vTaskDelay(pdMS_TO_TICKS(150));
        led_strip_set_pixel(s_led, 0, r, g, b);
        led_strip_refresh(s_led);
        vTaskDelay(pdMS_TO_TICKS(150));
    }
}

LightController::State LightController::state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

void LightController::set_listener(Listener listener) {
    std::lock_guard<std::mutex> lock(mutex_);
    listener_ = std::move(listener);
}

void LightController::compute_rgb_locked(uint8_t& r, uint8_t& g, uint8_t& b) const {
    const int pwm = (state_.on ? state_.brightness : 0);
    const uint8_t scale = static_cast<uint8_t>(0xFF * pwm / 100);

    // knob_panel: cool = white; warm = (0xFF, 0xFF, 0x33) scaled by pwm.
    const float t = std::clamp(
        (static_cast<float>(state_.color_temp_mireds) - 140.0f) / 360.0f,
        0.0f,
        1.0f);
    r = scale;
    g = scale;
    b = static_cast<uint8_t>(std::lround(scale * (1.0f - t * (1.0f - 0x33 / 255.0f))));
}

void LightController::apply_locked() {
    if (!led_ready_ || !s_led) {
        return;
    }
    uint8_t r, g, b;
    compute_rgb_locked(r, g, b);
    ESP_LOGI(TAG, "LED on=%d brightness=%d mireds=%lu rgb=(%u,%u,%u)",
             state_.on, state_.brightness, static_cast<unsigned long>(state_.color_temp_mireds),
             r, g, b);
    led_strip_set_pixel(s_led, 0, r, g, b);
    led_strip_refresh(s_led);
}
