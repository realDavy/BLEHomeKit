#pragma once

#include "hap/platform/Ble.hpp"
#include "hap/platform/Storage.hpp"
#include <esp_timer.h>
#include <host/ble_hs.h>
#include <host/util/util.h>
#include <vector>

class Esp32Ble : public hap::platform::Ble {
public:
  Esp32Ble(hap::platform::Storage *storage);
  ~Esp32Ble() override = default;

  void start_advertising(const Advertisement &data,
                         uint32_t interval_ms) override;
  void stop_advertising() override;
  void register_service(const ServiceDefinition &service) override;
  bool send_indication(uint16_t connection_id,
                       const std::string &characteristic_uuid,
                       std::span<const uint8_t> data) override;
  void disconnect(uint16_t connection_id) override;
  void set_disconnect_callback(DisconnectCallback callback) override;
  void start_timed_advertising(const Advertisement &data,
                               uint32_t fast_interval_ms,
                               uint32_t fast_duration_ms,
                               uint32_t normal_interval_ms) override;
  void start_encrypted_advertising(const EncryptedAdvertisement &data,
                                   uint32_t interval_ms,
                                   uint32_t duration_ms) override;
  void start() override;

private:
  static int ble_gap_event(struct ble_gap_event *event, void *arg);
  static int gatt_svr_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                                 struct ble_gatt_access_ctxt *ctxt, void *arg);
  static int gatt_svr_dsc_access(uint16_t conn_handle, uint16_t attr_handle,
                                 struct ble_gatt_access_ctxt *ctxt, void *arg);

  struct CharacteristicContext {
    std::string uuid;
    CharacteristicDefinition::ReadCallback on_read;
    CharacteristicDefinition::WriteCallback on_write;
    CharacteristicDefinition::SubscribeCallback on_subscribe;
    uint16_t val_handle;
  };

  struct DescriptorContext {
    std::string uuid;
    DescriptorDefinition::ReadCallback on_read;
    DescriptorDefinition::WriteCallback on_write;
  };

  std::vector<struct ble_gatt_svc_def *> nim_services;
  std::vector<std::vector<struct ble_gatt_chr_def>> nim_characteristics_storage;
  static std::vector<CharacteristicContext *> all_contexts;
  static std::vector<DescriptorContext *> all_descriptor_contexts;

  DisconnectCallback disconnect_callback_;
  hap::platform::Storage *storage_ = nullptr;

  // Timed advertising state (for HAP Spec 7.4.6.3 Disconnected Events)
  esp_timer_handle_t adv_timer_ = nullptr;
  Advertisement timed_adv_data_;
  uint32_t normal_interval_ms_ = 1000;
  static void adv_timer_callback(void *arg);

  // Encrypted advertising state (for HAP Spec 7.4.6.2 Broadcasted Events)
  esp_timer_handle_t enc_adv_timer_ = nullptr;
  bool encrypted_adv_active_ = false;
  static void enc_adv_timer_callback(void *arg);
};
