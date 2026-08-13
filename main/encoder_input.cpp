#include "encoder_input.hpp"
#include "board_pins.hpp"
#include "light_controller.hpp"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "iot_knob.h"

static const char* TAG = "encoder";

static constexpr int kDebounceMs = 30;
static constexpr int kClickMaxMs = 800;
static constexpr int kLongPressMs = 3000;

static void (*s_factory_reset)() = nullptr;

static void knob_left_cb(void* /*arg*/, void* /*data*/) {
    LightController::instance().encoder_dimmer();
}

static void knob_right_cb(void* /*arg*/, void* /*data*/) {
    LightController::instance().encoder_brighter();
}

static void button_task(void* /*arg*/) {
    int64_t press_start_ms = 0;
    bool pressed = false;
    bool long_fired = false;

    gpio_config_t io = {};
    io.pin_bit_mask = 1ULL << BOARD_ENCODER_SW;
    io.mode = GPIO_MODE_INPUT;
    io.pull_up_en = GPIO_PULLUP_ENABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io);

    while (true) {
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

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void encoder_input_start_with_reset(void (*factory_reset)()) {
    s_factory_reset = factory_reset;

    knob_config_t cfg = {};
    cfg.default_direction = 0;
    cfg.gpio_encoder_a = BOARD_ENCODER_A;
    cfg.gpio_encoder_b = BOARD_ENCODER_B;

    knob_handle_t knob = iot_knob_create(&cfg);
    if (knob == nullptr) {
        ESP_LOGE(TAG, "iot_knob_create failed");
    } else {
        iot_knob_register_cb(knob, KNOB_LEFT, knob_left_cb, nullptr);
        iot_knob_register_cb(knob, KNOB_RIGHT, knob_right_cb, nullptr);
    }

    xTaskCreate(button_task, "enc_btn", 3072, nullptr, 5, nullptr);
    ESP_LOGI(TAG, "Encoder A=%d B=%d SW=%d", BOARD_ENCODER_A, BOARD_ENCODER_B, BOARD_ENCODER_SW);
}

void encoder_input_start() {
    encoder_input_start_with_reset(nullptr);
}
