#include <esp_event.h>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <host/ble_att.h>
#include <nvs_flash.h>
#include <services/gap/ble_svc_gap.h>
#include <services/gatt/ble_svc_gatt.h>
#include <sodium.h>

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
#include "hap_pairing.hpp"
#include "light_controller.hpp"
#include "light_ui.hpp"
#include "setup_code.hpp"

static const char* TAG = "HAP_LCDkit";

class LcdkitBle : public Esp32Ble {
public:
    using Esp32Ble::Esp32Ble;

    uint16_t att_mtu(uint16_t connection_id) const override {
        const uint16_t mtu = ble_att_mtu(connection_id);
        return mtu >= 23 ? mtu : 23;
    }
};

static hap::AccessoryServer* s_server = nullptr;
static std::shared_ptr<hap::core::Characteristic> s_on_char;
static std::shared_ptr<hap::core::Characteristic> s_brightness_char;
static std::shared_ptr<hap::core::Characteristic> s_color_temp_char;
static bool s_updating_from_hap = false;
static LightController::State s_last_hap_state{};
static bool s_have_last_hap_state = false;

static std::shared_ptr<hap::core::Characteristic> find_characteristic(
    const std::shared_ptr<hap::core::Service>& service, uint64_t type) {
    for (const auto& ch : service->characteristics()) {
        if (ch->type() == type) {
            return ch;
        }
    }
    return nullptr;
}

static void remember_hap_state(const LightController::State& state) {
    s_last_hap_state = state;
    s_have_last_hap_state = true;
}

static void hap_sync_from_light(const LightController::State& state) {
    if (s_updating_from_hap) {
        remember_hap_state(state);
        return;
    }
    const bool sync_on = !s_have_last_hap_state || s_last_hap_state.on != state.on;
    const bool sync_bri =
        !s_have_last_hap_state || s_last_hap_state.brightness != state.brightness;
    const bool sync_ct = !s_have_last_hap_state ||
                         s_last_hap_state.color_temp_mireds != state.color_temp_mireds;
    if (s_on_char && sync_on) {
        s_on_char->set_value(state.on);
    }
    if (s_brightness_char && sync_bri) {
        s_brightness_char->set_value(static_cast<int32_t>(state.brightness));
    }
    if (s_color_temp_char && sync_ct) {
        s_color_temp_char->set_value(state.color_temp_mireds);
    }
    remember_hap_state(state);
    light_ui_refresh();
}

static void factory_reset_hap() {
    if (s_server) {
        ESP_LOGW(TAG, "Factory reset HomeKit pairing");
        s_server->factory_reset();
        light_ui_set_paired(false);
        ESP_LOGW(TAG, "Rebooting so HomeKit advertises unpaired (SF=1), same Device ID");
        vTaskDelay(pdMS_TO_TICKS(200));
        esp_restart();
    }
}

static void log_pairing_banner(bool paired, const std::string& setup_code) {
    if (paired) {
        ESP_LOGW(TAG, "================================================");
        ESP_LOGW(TAG, "HomeKit BLE already paired (SF=0)");
        ESP_LOGW(TAG, "iPhone will NOT show this as a new accessory.");
        ESP_LOGW(TAG, "Hold encoder knob 3s to unpair, or hold it while");
        ESP_LOGW(TAG, "powering on. Remove the accessory in Home first.");
        ESP_LOGW(TAG, "================================================");
    } else {
        ESP_LOGI(TAG, "================================================");
        ESP_LOGI(TAG, "HomeKit BLE unpaired (SF=1) — iPhone can add it");
        ESP_LOGI(TAG, "Device: %s", HAP_DEVICE_NAME);
        ESP_LOGI(TAG, "Setup code: %s", setup_code.c_str());
        ESP_LOGI(TAG, "Home -> Add Accessory -> More Options");
        ESP_LOGI(TAG, "================================================");
    }
}

static void log_heap(const char* where) {
    ESP_LOGI(TAG, "heap %s: free=%u largest=%u",
             where,
             static_cast<unsigned>(esp_get_free_heap_size()),
             static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)));
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
    log_heap("boot");

    // NVS before anything that may persist pairing / BLE address.
    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES || nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_ret);
    hap_wipe_legacy_nvs();

    if (sodium_init() < 0) {
        ESP_LOGE(TAG, "sodium_init failed");
    }

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

    // Bring up HAP-BLE before LVGL. The BSP default display buffers used ~96 KB
    // and left too little heap for GATT registration, which abort()ed and
    // rebooted (screen flicker + iPhone cannot discover the accessory).
    static Esp32System system_impl;
    static Esp32Storage storage_impl;
    static Esp32Crypto crypto_impl;

    // Leftover or incomplete pairing_list makes HAP advertise SF=0, so Home
    // will not show this as a new accessory. Sanitize before start. Pair-Setup
    // without Pair-Verify still advertises SF=1 so Add Accessory can finish.
    if (hap_encoder_sw_held(1500)) {
        ESP_LOGW(TAG, "Encoder held at boot: clearing HomeKit pairings");
        hap_clear_controller_pairings(storage_impl);
    }
    bool paired = hap_sanitize_pairings(storage_impl);
    // HAP-BLE Device ID must be a static random address, equal to AdvA.
    // Using the factory public MAC here makes Home show 未响应 after pairing.
    if (!hap_align_ble_identity(storage_impl)) {
        paired = false;
    }
    static LcdkitBle ble_impl(&storage_impl);
    light_ui_set_paired(paired);
    log_pairing_banner(paired, setup_code);

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_svc_gap_device_name_set(HAP_DEVICE_NAME);
    log_heap("after nimble_port_init");

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
    config.on_pairings_changed = [work_queue, &storage_impl](const hap::PairingEvent& event) {
        auto* job = new std::function<void()>([event, &storage_impl]() {
            bool paired = event.type == hap::PairingEventType::Added;
            if (event.type == hap::PairingEventType::Removed) {
                auto list = storage_impl.get("pairing_list");
                if (list && list->size() > 2) {
                    const std::string raw(list->begin(), list->end());
                    paired = raw != "[]";
                } else {
                    paired = false;
                }
            }
            ESP_LOGI(TAG, "Pairing event %d id=%s paired=%d",
                     static_cast<int>(event.type), event.pairing_id.c_str(), paired ? 1 : 0);
            light_ui_set_paired(paired);
            if (paired) {
                ESP_LOGW(TAG, "Paired. Leave the accessory settings page,");
                ESP_LOGW(TAG, "turn OFF iPhone Wi-Fi, then tap the light tile.");
                ESP_LOGW(TAG, "A distant HomePod/Apple TV makes Home show 未响应.");
            } else {
                ESP_LOGW(TAG, "HomeKit unpaired — iPhone can add this accessory again");
                log_pairing_banner(false, hap_setup_code_from_mac());
            }
        });
        if (xQueueSend(work_queue, &job, 0) != pdTRUE) {
            delete job;
        }
    };

    static hap::AccessoryServer server(std::move(config));
    s_server = &server;

    auto accessory = std::make_shared<hap::core::Accessory>(1);
    auto info_service = hap::service::AccessoryInformationBuilder()
        .name(HAP_DEVICE_NAME)
        .manufacturer("Aidaegis")
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
            remember_hap_state(LightController::instance().state());
            s_updating_from_hap = false;
            light_ui_refresh();
        })
        .on_brightness_change([](int brightness) {
            ESP_LOGI(TAG, "HomeKit Brightness=%d", brightness);
            s_updating_from_hap = true;
            LightController::instance().set_brightness(brightness);
            remember_hap_state(LightController::instance().state());
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
            remember_hap_state(LightController::instance().state());
            s_updating_from_hap = false;
            light_ui_refresh();
            return std::nullopt;
        });
    }

    const auto initial = LightController::instance().state();
    remember_hap_state(initial);
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

    ESP_LOGW(TAG, "HomeKit sync-rev=11 — rebuild/flash this tree, not 33b40b7");
    ESP_LOGI(TAG, "HAP-BLE advertising as '%s', setup code %s", HAP_DEVICE_NAME, setup_code.c_str());
    log_heap("before hap start");
    server.start();
    log_heap("after hap start");

    light_ui_start();
    encoder_input_start_with_reset(factory_reset_hap);
    log_heap("after ui");

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(20));
        server.tick();
    }
}
