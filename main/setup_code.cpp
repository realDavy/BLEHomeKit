#include "setup_code.hpp"

#include "esp_log.h"
#include "esp_mac.h"

#include <array>
#include <cstdio>
#include <cstring>

static const char* TAG = "setup_code";

static bool read_factory_mac(std::array<uint8_t, 6>& mac) {
    if (esp_efuse_mac_get_default(mac.data()) == ESP_OK) {
        return true;
    }
    if (esp_read_mac(mac.data(), ESP_MAC_WIFI_STA) == ESP_OK) {
        return true;
    }
    return esp_read_mac(mac.data(), ESP_MAC_BT) == ESP_OK;
}

static bool is_invalid_setup_digits(uint32_t digits) {
    switch (digits) {
    case 0:
    case 11111111u:
    case 22222222u:
    case 33333333u:
    case 44444444u:
    case 55555555u:
    case 66666666u:
    case 77777777u:
    case 88888888u:
    case 99999999u:
    case 12345678u:
    case 87654321u:
        return true;
    default:
        return false;
    }
}

static uint32_t mac_to_digits(const std::array<uint8_t, 6>& mac) {
    uint32_t h = 2166136261u;
    for (uint8_t b : mac) {
        h ^= b;
        h *= 16777619u;
    }
    uint32_t digits = h % 100000000u;
    for (int i = 0; i < 16 && is_invalid_setup_digits(digits); ++i) {
        digits = (digits + 7919u) % 100000000u;
    }
    return digits;
}

static std::string format_setup_code(uint32_t digits) {
    char buf[12];
    std::snprintf(buf, sizeof(buf), "%03u-%02u-%03u",
                  static_cast<unsigned>(digits / 100000u),
                  static_cast<unsigned>((digits / 1000u) % 100u),
                  static_cast<unsigned>(digits % 1000u));
    return buf;
}

std::string hap_setup_code_from_mac() {
    std::array<uint8_t, 6> mac{};
    if (!read_factory_mac(mac)) {
        ESP_LOGW(TAG, "MAC unavailable, using fallback setup code");
        return "418-27-693";
    }
    const std::string code = format_setup_code(mac_to_digits(mac));
    ESP_LOGI(TAG, "MAC %02X:%02X:%02X:%02X:%02X:%02X -> setup code %s",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], code.c_str());
    return code;
}

std::string hap_serial_from_mac() {
    std::array<uint8_t, 6> mac{};
    if (!read_factory_mac(mac)) {
        return "C3LCDKIT01";
    }
    char buf[13];
    std::snprintf(buf, sizeof(buf), "%02X%02X%02X%02X%02X%02X",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return buf;
}
