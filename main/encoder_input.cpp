#include "encoder_input.hpp"
#include "board_pins.hpp"
#include "light_controller.hpp"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "encoder";

static constexpr int kPollMs = 2;
static constexpr int kDebounceMs = 30;
static constexpr int kClickMaxMs = 800;
static constexpr int kLongPressMs = 3000;

// One mechanical detent is typically four quadrature steps.
static constexpr int kStepsPerDetent = 4;

static void (*s_factory_reset)() = nullptr;

// Standard 2-bit gray-code quadrature table.
static const int8_t kQuadTable[16] = {
    0, -1,  1,  0,
    1,  0,  0, -1,
   -1,  0,  0,  1,
    0,  1, -1,  0
};

static void encoder_task(void* /*arg*/) {
    gpio_config_t io = {};
    io.pin_bit_mask = (1ULL << BOARD_ENCODER_A) |
                      (1ULL << BOARD_ENCODER_B) |
                      (1ULL << BOARD_ENCODER_SW);
    io.mode = GPIO_MODE_INPUT;
    io.pull_up_en = GPIO_PULLUP_ENABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io);

    uint8_t enc_prev = static_cast<uint8_t>(
        (gpio_get_level(BOARD_ENCODER_A) << 1) | gpio_get_level(BOARD_ENCODER_B));
    int enc_accum = 0;

    int64_t press_start_ms = 0;
    bool pressed = false;
    bool long_fired = false;

    while (true) {
        const uint8_t a = static_cast<uint8_t>(gpio_get_level(BOARD_ENCODER_A));
        const uint8_t b = static_cast<uint8_t>(gpio_get_level(BOARD_ENCODER_B));
        const uint8_t curr = static_cast<uint8_t>((a << 1) | b);
        enc_accum += kQuadTable[(enc_prev << 2) | curr];
        enc_prev = curr;
        if (enc_accum >= kStepsPerDetent) {
            enc_accum = 0;
            LightController::instance().encoder_brighter();
        } else if (enc_accum <= -kStepsPerDetent) {
            enc_accum = 0;
            LightController::instance().encoder_dimmer();
        }

        const bool level_pressed = gpio_get_level(BOARD_ENCODER_SW) == 0;
        const int64_t now = esp_timer_get_time() / 1000;

        if (level_pressed && !pressed) {
            vTaskDelay(pdMS_TO_TICKS(kDebounceMs));
            if (gpio_get_level(BOARD_ENCODER_SW) == 0) {
                pressed = true;
                long_fired = false;
                press_start_ms = now;
            }
        } else if (level_pressed && pressed && !long_fired) {
            if ((now - press_start_ms) >= kLongPressMs) {
                long_fired = true;
                ESP_LOGW(TAG, "Long press: HomeKit factory reset");
                if (s_factory_reset) {
                    s_factory_reset();
                }
            }
        } else if (!level_pressed && pressed) {
            const int64_t held = now - press_start_ms;
            pressed = false;
            if (!long_fired && held >= kDebounceMs && held < kClickMaxMs) {
                LightController::instance().toggle_color_temp();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(kPollMs));
    }
}

void encoder_input_start_with_reset(void (*factory_reset)()) {
    s_factory_reset = factory_reset;
    // Do not create a second iot_knob: bsp_display_start_with_config already
    // attaches one to the same A/B GPIOs. Poll the pins instead.
    xTaskCreate(encoder_task, "enc_in", 3072, nullptr, 5, nullptr);
    ESP_LOGI(TAG, "Encoder A=%d B=%d SW=%d (GPIO poll, no second knob)",
             BOARD_ENCODER_A, BOARD_ENCODER_B, BOARD_ENCODER_SW);
}

void encoder_input_start() {
    encoder_input_start_with_reset(nullptr);
}
