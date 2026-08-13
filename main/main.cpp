#include <esp_event.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "Esp32Ble.hpp"
#include "Esp32Crypto.hpp"
#include "Esp32Platform.hpp"
#include "Esp32Storage.hpp"
#include "hap/AccessoryServer.hpp"
#include "hap/core/Accessory.hpp"
#include "hap/types/CharacteristicTypes.hpp"
#include "hap/types/ServiceTypes.hpp"

#include <functional>
#include <memory>
#include <string>
#include <type_traits>
#include <variant>

#include "board_pins.hpp"
#include "encoder_input.hpp"
#include "light_controller.hpp"
#include "light_ui.hpp"
#include "setup_code.hpp"

static const char* TAG = "HAP_LCDkit";

static hap::AccessoryServer* s_server = nullptr;
static std::shared_ptr<hap::core::Characteristic> s_on_char;
static std::shared_ptr<hap::core::Characteristic> s_brightness_char;
static std::shared_ptr<hap::core::Characteristic> s_color_temp_char;
static bool s_updating_from_hap = false;

static std::shared_ptr<hap::core::Characteristic> find_characteristic(
    const std::shared_ptr<hap::core::Service>& service, uint64_t type) {
    for (const auto& ch : service->characteristics()) {
        if (ch->type() == type) {
            return ch;
        }
    }
    return nullptr;
}

static void hap_sync_from_light(const LightController::State& state) {
    if (s_updating_from_hap) {
        return;
    }
    if (s_on_char) {
        s_on_char->set_value(state.on);
    }
    if (s_brightness_char) {
        s_brightness_char->set_value(static_cast<int32_t>(state.brightness));
    }
    if (s_color_temp_char) {
        s_color_temp_char->set_value(state.color_temp_mireds);
    }
    light_ui_refresh();
}

static void factory_reset_hap() {
    if (s_server) {
        ESP_LOGW(TAG, "Factory reset HomeKit pairing");
        s_server->factory_reset();
        light_ui_set_paired(false);
    }
}

static void hap_work_task(void* arg) {
    auto* queue = static_cast<QueueHandle_t>(arg);
    while (true) {
        std::function<void()>* job = nullptr;
        if (xQueueReceive(queue, &job, portMAX_DELAY) == pdTRUE && job) {
            (*job)();
            delete job;
        }
    }
}

extern "C" void app_main() {
    ESP_LOGI(TAG, "ESP32-C3-LCDkit HAP-BLE light starting");

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    auto* work_queue = xQueueCreate(16, sizeof(std::function<void()>*));
    xTaskCreate(hap_work_task, "hap_work", 4096, work_queue, 5, nullptr);
    hap::core::Characteristic::set_dispatcher([work_queue](std::function<void()> fn) {
        auto* job = new std::function<void()>(std::move(fn));
        if (xQueueSend(work_queue, &job, 0) != pdTRUE) {
            delete job;
        }
    });

    LightController::instance().init();
    const std::string setup_code = hap_setup_code_from_mac();
    const std::string serial = hap_serial_from_mac();
    light_ui_set_setup_code(setup_code.c_str());
    light_ui_start();
    encoder_input_start_with_reset(factory_reset_hap);

    static Esp32System system_impl;
    static Esp32Storage storage_impl;
    static Esp32Crypto crypto_impl;
    static Esp32Ble ble_impl(&storage_impl);

    hap::AccessoryServer::Config config;
    config.system = &system_impl;
    config.storage = &storage_impl;
    config.crypto = &crypto_impl;
    config.ble = &ble_impl;
    config.network = nullptr;
    config.device_name = HAP_DEVICE_NAME;
    config.setup_code = setup_code;
    config.category_id = hap::core::AccessoryCategory::Lightbulb;
    config.on_identify = []() {
        LightController::instance().identify();
    };
    config.on_pairings_changed = [](const hap::PairingEvent& event) {
        ESP_LOGI(TAG, "Pairing event %d id=%s", static_cast<int>(event.type), event.pairing_id.c_str());
        light_ui_set_paired(event.type == hap::PairingEventType::Added);
    };

    static hap::AccessoryServer server(std::move(config));
    s_server = &server;

    auto accessory = std::make_shared<hap::core::Accessory>(1);
    auto info_service = hap::service::AccessoryInformationBuilder()
        .name(HAP_DEVICE_NAME)
        .manufacturer("Espressif")
        .model("ESP32-C3-LCDkit")
        .serial_number(serial)
        .firmware_revision("1.0.0")
        .hardware_revision("ESP32-C3-MINI-1")
        .on_identify([]() {
            LightController::instance().identify();
        })
        .build();
    accessory->add_service(info_service);

    auto light_builder = hap::service::LightBulbBuilder();
    light_builder.with_brightness()
        .with_color_temperature()
        .with_name("Knob Light")
        .on_change([](bool on) {
            ESP_LOGI(TAG, "HomeKit On=%d", on);
            s_updating_from_hap = true;
            LightController::instance().set_on(on);
            s_updating_from_hap = false;
            light_ui_refresh();
        })
        .on_brightness_change([](int brightness) {
            ESP_LOGI(TAG, "HomeKit Brightness=%d", brightness);
            s_updating_from_hap = true;
            LightController::instance().set_brightness(brightness);
            s_updating_from_hap = false;
            light_ui_refresh();
        });

    auto light_service = light_builder.build();
    s_on_char = find_characteristic(light_service, hap::characteristic::kType_On);
    s_brightness_char = find_characteristic(light_service, hap::characteristic::kType_Brightness);
    s_color_temp_char = find_characteristic(light_service, hap::characteristic::kType_ColorTemperature);

    if (s_color_temp_char) {
        s_color_temp_char->set_write_callback([](const hap::core::Value& value) -> hap::core::WriteResponse {
            uint32_t mireds = 370;
            std::visit([&](auto&& arg) {
                using T = std::decay_t<decltype(arg)>;
                if constexpr (std::is_arithmetic_v<T>) {
                    mireds = static_cast<uint32_t>(arg);
                }
            }, value);
            ESP_LOGI(TAG, "HomeKit ColorTemperature=%lu", static_cast<unsigned long>(mireds));
            s_updating_from_hap = true;
            LightController::instance().set_color_temp(mireds);
            s_updating_from_hap = false;
            light_ui_refresh();
            return std::nullopt;
        });
    }

    const auto initial = LightController::instance().state();
    if (s_on_char) {
        s_on_char->set_value(initial.on);
    }
    if (s_brightness_char) {
        s_brightness_char->set_value(static_cast<int32_t>(initial.brightness));
    }
    if (s_color_temp_char) {
        s_color_temp_char->set_value(initial.color_temp_mireds);
    }

    accessory->add_service(light_service);
    server.add_accessory(accessory);

    LightController::instance().set_listener([](const LightController::State& state) {
        hap_sync_from_light(state);
    });

    ESP_LOGI(TAG, "HAP-BLE advertising as '%s', setup code %s", HAP_DEVICE_NAME, setup_code.c_str());
    server.start();

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(100));
        server.tick();
    }
}
