#pragma once

#include "hap/platform/Storage.hpp"

class Esp32Storage : public hap::platform::Storage {
public:
  Esp32Storage();
  ~Esp32Storage() override = default;

  void set(std::string_view key, std::span<const uint8_t> value) override;
  std::optional<std::vector<uint8_t>> get(std::string_view key) override;
  void remove(std::string_view key) override;
  bool has(std::string_view key) override;
};
