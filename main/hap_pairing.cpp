#include "hap_pairing.hpp"
#include "board_pins.hpp"

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>
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

void hap_wipe_legacy_nvs() {
    if (hap_nvs_has_pairing_list()) {
        ESP_LOGI(TAG, "NVS pairing_list present (will persist across reboot)");
    } else {
        ESP_LOGI(TAG, "NVS pairing_list absent");
    }
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
    storage.remove("pair_verified");
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

    auto verified = storage.get("pair_verified");
    const bool advertise_paired = !verified || verified->empty() || (*verified)[0] == '1';
    if (!verified || verified->empty()) {
        // Legacy pairing already completed Add Accessory before this flag existed.
        storage.set("pair_verified", std::vector<uint8_t>{'1'});
    }
    ESP_LOGI(TAG, "HAP pairings: %u valid controller(s) (%s)",
             static_cast<unsigned>(ids.size()),
             advertise_paired ? "SF=0 after Pair Verify" : "SF=1 until Pair Verify");
    return advertise_paired;
}

static bool parse_device_id(const std::string& id, uint8_t out[6]) {
    unsigned b[6] = {};
    if (std::sscanf(id.c_str(), "%02X:%02X:%02X:%02X:%02X:%02X",
                    &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) != 6) {
        return false;
    }
    for (int i = 0; i < 6; ++i) {
        out[i] = static_cast<uint8_t>(b[i]);
    }
    return true;
}

static void store_device_id(hap::platform::Storage& storage, const uint8_t mac[6]) {
    char buf[18];
    std::snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    const std::string id(buf);
    storage.set("accessory_id", std::vector<uint8_t>(id.begin(), id.end()));
    ESP_LOGI(TAG, "HAP Device ID %s (static random, matches BLE address)", buf);
}

static bool is_static_random_addr(const uint8_t id[6]) {
    // BLE static random: bits 47 and 46 of the 48-bit address are 1.
    // Those bits live in the most-significant (first displayed) octet.
    return (id[0] & 0xC0) == 0xC0;
}

static bool mac_equals(const uint8_t a[6], const uint8_t b[6]) {
    return std::memcmp(a, b, 6) == 0;
}

static void generate_static_random_id(uint8_t out[6]) {
    esp_fill_random(out, 6);
    out[0] = static_cast<uint8_t>((out[0] & 0x3F) | 0xC0);
}

static bool read_factory_mac(esp_mac_type_t type, uint8_t out[6]) {
    return esp_read_mac(out, type) == ESP_OK;
}

bool hap_align_ble_identity(hap::platform::Storage& storage) {
    uint8_t factory_bt[6] = {};
    uint8_t factory_wifi[6] = {};
    const bool have_bt = read_factory_mac(ESP_MAC_BT, factory_bt);
    const bool have_wifi = read_factory_mac(ESP_MAC_WIFI_STA, factory_wifi);

    uint8_t device_id[6] = {};
    bool have_stored = false;
    auto stored = storage.get("accessory_id");
    if (stored && !stored->empty()) {
        const std::string id(stored->begin(), stored->end());
        have_stored = parse_device_id(id, device_id);
        if (!have_stored) {
            ESP_LOGW(TAG, "Ignoring invalid accessory_id '%s'", id.c_str());
        }
    }

    const bool usable = have_stored && is_static_random_addr(device_id) &&
                        !(have_bt && mac_equals(device_id, factory_bt)) &&
                        !(have_wifi && mac_equals(device_id, factory_wifi));
    if (usable) {
        ESP_LOGI(TAG,
                 "HAP Device ID %02X:%02X:%02X:%02X:%02X:%02X (static random)",
                 device_id[0], device_id[1], device_id[2],
                 device_id[3], device_id[4], device_id[5]);
        return true;
    }

    if (have_stored) {
        ESP_LOGW(TAG,
                 "Device ID %02X:%02X:%02X:%02X:%02X:%02X is a public MAC or "
                 "not static-random; iPhone cannot reconnect after pairing",
                 device_id[0], device_id[1], device_id[2],
                 device_id[3], device_id[4], device_id[5]);
        hap_clear_controller_pairings(storage);
    }

    generate_static_random_id(device_id);
    store_device_id(storage, device_id);
    return false;
}
