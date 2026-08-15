#include "Esp32Platform.hpp"
#include <esp_log.h>
#include <esp_random.h>
#include <esp_timer.h>
#include <string>

uint64_t Esp32System::millis() { return esp_timer_get_time() / 1000; }

void Esp32System::random_bytes(std::span<uint8_t> buffer) {
  esp_fill_random(buffer.data(), buffer.size());
}

void Esp32System::log(LogLevel level, std::string_view message) {
  // Basic mapping
  esp_log_level_t esp_level = ESP_LOG_INFO;
  switch (level) {
  case LogLevel::Debug:
    esp_level = ESP_LOG_DEBUG;
    break;
  case LogLevel::Info:
    esp_level = ESP_LOG_INFO;
    break;
  case LogLevel::Warning:
    esp_level = ESP_LOG_WARN;
    break;
  case LogLevel::Error:
    esp_level = ESP_LOG_ERROR;
    break;
  }

  std::string msg(message);
  ESP_LOG_LEVEL(esp_level, "HAP", "%s", msg.c_str());
}
