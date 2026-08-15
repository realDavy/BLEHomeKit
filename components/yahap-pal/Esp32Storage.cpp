#include "Esp32Storage.hpp"
#include <esp_log.h>
#include <esp_random.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <iomanip>
#include <mbedtls/sha256.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <sstream>
#include <string>

static const char *TAG = "Esp32Platform";
static const char *NVS_NAMESPACE = "hap_storage";

Esp32Storage::Esp32Storage() {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
}

// Ensure key fits in NVS (15 char limit)
static std::string sanitize_key(std::string_view key) {
  if (key.length() <= 15) {
    return std::string(key);
  }
  // Hash long keys: SHA256 -> Hex (first 7 bytes = 14 chars)
  uint8_t hash[32];
  mbedtls_sha256(reinterpret_cast<const uint8_t *>(key.data()), key.size(),
                 hash, 0);

  std::stringstream ss;
  ss << std::hex << std::setfill('0');
  for (int i = 0; i < 7; ++i) {
    ss << std::setw(2) << static_cast<int>(hash[i]);
  }
  return ss.str();
}

void Esp32Storage::set(std::string_view key, std::span<const uint8_t> value) {
  nvs_handle_t handle;
  std::string k = sanitize_key(key);

  esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(err));
    return;
  }

  err = nvs_set_blob(handle, k.c_str(), value.data(), value.size());
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error setting blob '%s': %s", k.c_str(),
             esp_err_to_name(err));
  }

  nvs_commit(handle);
  nvs_close(handle);
}

std::optional<std::vector<uint8_t>> Esp32Storage::get(std::string_view key) {
  nvs_handle_t handle;
  std::string k = sanitize_key(key);

  esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
  if (err != ESP_OK) {
    return std::nullopt;
  }

  size_t required_size = 0;
  err = nvs_get_blob(handle, k.c_str(), NULL, &required_size);
  if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGE(TAG, "Error getting blob size '%s': %s", k.c_str(),
             esp_err_to_name(err));
    nvs_close(handle);
    return std::nullopt;
  }

  if (required_size == 0) {
    nvs_close(handle);
    return std::nullopt;
  }

  std::vector<uint8_t> vec(required_size);
  err = nvs_get_blob(handle, k.c_str(), vec.data(), &required_size);
  nvs_close(handle);

  if (err != ESP_OK) {
    return std::nullopt;
  }

  return vec;
}

void Esp32Storage::remove(std::string_view key) {
  nvs_handle_t handle;
  std::string k = sanitize_key(key);

  if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK) {
    nvs_erase_key(handle, k.c_str());
    nvs_commit(handle);
    nvs_close(handle);
  }
}

bool Esp32Storage::has(std::string_view key) {
  nvs_handle_t handle;
  std::string k = sanitize_key(key);

  if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK)
    return false;

  size_t required_size = 0;
  esp_err_t err = nvs_get_blob(handle, k.c_str(), NULL, &required_size);
  nvs_close(handle);

  return (err == ESP_OK);
}
