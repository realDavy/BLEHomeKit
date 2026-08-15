#include "hap_pairing.hpp"
#include "board_pins.hpp"

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

#include <algorithm>
#include <cctype>
#include <string>
#include <vector>

static const char* TAG = "hap_pair";

static std::string trim_copy(std::string s) {
    auto not_space = [](unsigned char c) { return !std::isspace(c); };
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
    s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
    return s;
}

static bool parse_pairing_ids(const std::string& list, std::vector<std::string>& ids) {
    const std::string s = trim_copy(list);
    if (s.empty() || s.front() != '[' || s.back() != ']') {
        return false;
    }

    size_t i = 1;
    const size_t n = s.size() - 1;
    bool expect_value = true;
    while (i < n) {
        while (i < n && std::isspace(static_cast<unsigned char>(s[i]))) {
            ++i;
        }
        if (i >= n) {
            break;
        }
        if (s[i] == ',') {
            if (expect_value) {
                return false;
            }
            expect_value = true;
            ++i;
            continue;
        }
        if (s[i] != '"') {
            return false;
        }
        ++i;
        std::string id;
        while (i < n && s[i] != '"') {
            if (s[i] == '\\') {
                ++i;
                if (i >= n) {
                    return false;
                }
            }
            id.push_back(s[i++]);
        }
        if (i >= n || s[i] != '"' || id.empty()) {
            return false;
        }
        ++i;
        ids.push_back(std::move(id));
        expect_value = false;
    }
    return !expect_value || ids.empty();
}

bool hap_encoder_sw_held(int hold_ms) {
    gpio_config_t io = {};
    io.pin_bit_mask = 1ULL << BOARD_ENCODER_SW;
    io.mode = GPIO_MODE_INPUT;
    io.pull_up_en = GPIO_PULLUP_ENABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io);

    vTaskDelay(pdMS_TO_TICKS(50));
    if (gpio_get_level(BOARD_ENCODER_SW) != 0) {
        return false;
    }

    const int step_ms = 50;
    int waited = 0;
    while (waited < hold_ms) {
        vTaskDelay(pdMS_TO_TICKS(step_ms));
        waited += step_ms;
        if (gpio_get_level(BOARD_ENCODER_SW) != 0) {
            return false;
        }
    }
    return true;
}

static bool hap_nvs_has_pairing_list() {
    nvs_handle_t handle;
    if (nvs_open("hap_storage", NVS_READONLY, &handle) != ESP_OK) {
        return false;
    }
    size_t len = 0;
    const esp_err_t err = nvs_get_blob(handle, "pairing_list", nullptr, &len);
    nvs_close(handle);
    return err == ESP_OK && len > 0;
}

static void hap_nvs_erase_hap_storage() {
    nvs_handle_t handle;
    esp_err_t err = nvs_open("hap_storage", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "hap_storage open failed: %s", esp_err_to_name(err));
        return;
    }
    err = nvs_erase_all(handle);
    ESP_LOGW(TAG, "nvs_erase_all(hap_storage)=%s", esp_err_to_name(err));
    err = nvs_commit(handle);
    ESP_LOGW(TAG, "nvs_commit(hap_storage)=%s", esp_err_to_name(err));
    nvs_close(handle);
}

void hap_wipe_legacy_nvs() {
    const bool had_list = hap_nvs_has_pairing_list();
    ESP_LOGW(TAG, "hap_wipe: pairing_list %s", had_list ? "PRESENT" : "absent");

    nvs_handle_t boot;
    if (nvs_open("hap_boot", NVS_READWRITE, &boot) != ESP_OK) {
        ESP_LOGE(TAG, "hap_boot open failed");
        if (had_list) {
            hap_nvs_erase_hap_storage();
        }
        return;
    }

    uint8_t wipe3 = 0;
    nvs_get_u8(boot, "wipe3", &wipe3);

    if (had_list && wipe3 != 1) {
        ESP_LOGW(TAG, "Wiping hap_storage (wipe3) so HomeKit advertises SF=1");
        hap_nvs_erase_hap_storage();
        if (hap_nvs_has_pairing_list()) {
            ESP_LOGE(TAG, "pairing_list STILL present after nvs_erase_all");
        } else {
            ESP_LOGW(TAG, "pairing_list removed; advertising should be SF=1");
            nvs_set_u8(boot, "wipe3", 1);
            nvs_commit(boot);
        }
    } else if (had_list) {
        ESP_LOGI(TAG, "Keeping HomeKit pairing (wipe3 already done)");
    } else {
        ESP_LOGI(TAG, "No NVS pairing_list (SF=1)");
        if (wipe3 != 1) {
            nvs_set_u8(boot, "wipe3", 1);
            nvs_commit(boot);
        }
    }
    nvs_close(boot);
}

void hap_clear_controller_pairings(hap::platform::Storage& storage) {
    auto list = storage.get("pairing_list");
    if (list && !list->empty()) {
        const std::string raw(list->begin(), list->end());
        std::vector<std::string> ids;
        if (parse_pairing_ids(raw, ids)) {
            for (const auto& id : ids) {
                storage.remove(std::string("pairing_") + id);
            }
        }
        storage.remove("pairing_list");
    }
    storage.remove("gsn");
    ESP_LOGW(TAG, "Cleared HomeKit controller pairings (identity kept)");
}

bool hap_sanitize_pairings(hap::platform::Storage& storage) {
    auto list = storage.get("pairing_list");
    if (!list || list->empty()) {
        ESP_LOGI(TAG, "HAP pairings: none (SF=1)");
        return false;
    }

    const std::string raw(list->begin(), list->end());
    ESP_LOGI(TAG, "HAP pairing_list (%u bytes): %s",
             static_cast<unsigned>(raw.size()), raw.c_str());

    std::vector<std::string> ids;
    if (!parse_pairing_ids(raw, ids) || ids.empty()) {
        ESP_LOGW(TAG, "pairing_list is empty or invalid; advertising unpaired (SF=1)");
        hap_clear_controller_pairings(storage);
        return false;
    }

    bool all_ok = true;
    for (const auto& id : ids) {
        auto ltpk = storage.get(std::string("pairing_") + id);
        if (!ltpk || ltpk->size() != 32) {
            ESP_LOGW(TAG, "pairing_%s missing or not 32 bytes", id.c_str());
            all_ok = false;
        }
    }
    if (!all_ok) {
        ESP_LOGW(TAG, "Incomplete pairings cleared; advertising unpaired (SF=1)");
        hap_clear_controller_pairings(storage);
        return false;
    }

    ESP_LOGI(TAG, "HAP pairings: %u valid controller(s) (SF=0)",
             static_cast<unsigned>(ids.size()));
    return true;
}
