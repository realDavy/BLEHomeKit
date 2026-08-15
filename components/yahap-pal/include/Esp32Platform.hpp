#pragma once

#include "hap/platform/System.hpp"

class Esp32System : public hap::platform::System {
public:
  uint64_t millis() override;
  void random_bytes(std::span<uint8_t> buffer) override;
  void log(LogLevel level, std::string_view message) override;
};
