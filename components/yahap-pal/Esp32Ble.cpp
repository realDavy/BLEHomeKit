#include <sdkconfig.h>
#if CONFIG_BT_NIMBLE_ENABLED
#include <cstdlib>
#include <cstring>
#include <esp_err.h>
#include <esp_log.h>
#include <optional>
#include <string>
#include <vector>

#include "Esp32Ble.hpp"
#include "esp_log_buffer.h"
#include "esp_random.h"
#if CONFIG_IDF_TARGET_ESP32
#include <esp_nimble_hci.h>
#endif
#include <cstdlib>
#include <cstring>
#include <nimble/nimble_port.h>
#include <nimble/nimble_port_freertos.h>
#include <optional>
#include <services/gap/ble_svc_gap.h>
#include <services/gatt/ble_svc_gatt.h>
#include <string>

static const char *TAG = "Esp32Ble";

static hap::platform::Storage *g_storage = nullptr;

std::vector<Esp32Ble::CharacteristicContext *> Esp32Ble::all_contexts;
std::vector<Esp32Ble::DescriptorContext *> Esp32Ble::all_descriptor_contexts;
static bool nimble_synced = false;
static Esp32Ble *g_ble_instance = nullptr;
static uint8_t g_own_addr_type = BLE_OWN_ADDR_PUBLIC;
static std::optional<Esp32Ble::Advertisement> pending_adv;
static std::optional<Esp32Ble::Advertisement> last_adv;
static uint32_t pending_adv_interval = 0;
static uint32_t last_adv_interval = 20;

static void parse_uuid(const std::string &uuid_str, ble_uuid_any_t *uuid) {
  ESP_LOGD(TAG, "Parsing UUID: %s", uuid_str.c_str());

  std::string clean;
  for (char c : uuid_str) {
    if (c != '-')
      clean += c;
  }

  if (clean.length() == 32) {
    uuid->u.type = BLE_UUID_TYPE_128;
    for (int i = 0; i < 16; i++) {
      std::string byte_str = clean.substr((15 - i) * 2, 2);
      uuid->u128.value[i] = (uint8_t)strtoul(byte_str.c_str(), nullptr, 16);
    }
  } else if (clean.length() == 4) {
    uuid->u.type = BLE_UUID_TYPE_16;
    uuid->u16.value = (uint16_t)strtoul(clean.c_str(), nullptr, 16);
  } else {
    ESP_LOGE(TAG, "Invalid UUID length: %d", (int)clean.length());
  }
}

Esp32Ble::Esp32Ble(hap::platform::Storage *storage) : storage_(storage) {
#if CONFIG_IDF_TARGET_ESP32
  ESP_ERROR_CHECK(esp_nimble_hci_init());
#endif
  nimble_port_init();

  g_storage = storage;
  g_ble_instance = this;

  ble_hs_cfg.sync_cb = []() {
    ESP_LOGI(TAG, "NimBLE Synced");
    nimble_synced = true;

    // Use the factory public BLE MAC. A rotating random address plus a
    // broken IRK store (status=8) lets iPhone discover HAP ads but fail
    // to connect ("失去连接").
    int rc = ble_hs_id_infer_auto(0, &g_own_addr_type);
    if (rc != 0) {
      ESP_LOGW(TAG, "ble_hs_id_infer_auto rc=%d, using public", rc);
      g_own_addr_type = BLE_OWN_ADDR_PUBLIC;
    }
    uint8_t addr[6] = {};
    ble_hs_id_copy_addr(g_own_addr_type, addr, nullptr);
    ESP_LOGI(TAG, "BLE address type=%d %02X:%02X:%02X:%02X:%02X:%02X",
             g_own_addr_type, addr[5], addr[4], addr[3], addr[2], addr[1],
             addr[0]);

    if (pending_adv && g_ble_instance) {
      g_ble_instance->start_advertising(*pending_adv, pending_adv_interval);
      pending_adv.reset();
    }
  };

  ble_hs_cfg.gatts_register_cb = [](struct ble_gatt_register_ctxt *ctxt,
                                    void *arg) {
    char buf[BLE_UUID_STR_LEN];
    switch (ctxt->op) {
    case BLE_GATT_REGISTER_OP_SVC:
      ESP_LOGD(TAG, "Reg Service: %s, handle=%d",
               ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf), ctxt->svc.handle);
      break;
    case BLE_GATT_REGISTER_OP_CHR:
      ESP_LOGD(TAG, "Reg Char: %s, val_handle=%d",
               ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
               ctxt->chr.val_handle);
      break;
    case BLE_GATT_REGISTER_OP_DSC:
      ESP_LOGD(TAG, "Reg Desc: %s, handle=%d",
               ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf), ctxt->dsc.handle);
      break;
    }
  };
  ble_hs_cfg.reset_cb = [](int reason) {
    ESP_LOGI(TAG, "NimBLE Reset: %d", reason);
  };

  // HAP-BLE pairs over GATT. OS-level SMP bonding fights iPhone Home
  // and shows up as "Connection Lost".
  ble_hs_cfg.sm_io_cap = BLE_HS_IO_NO_INPUT_OUTPUT;
  ble_hs_cfg.sm_bonding = 0;
  ble_hs_cfg.sm_mitm = 0;
  ble_hs_cfg.sm_sc = 0;
}

void Esp32Ble::start() {
  nimble_port_freertos_init([](void *arg) {
    nimble_port_run();
    nimble_port_freertos_deinit();
  });
}

void Esp32Ble::start_advertising(const Advertisement &data,
                                 uint32_t interval_ms) {
  if (!nimble_synced) {
    ESP_LOGI(TAG, "Stack not synced, queueing advertisement");
    pending_adv = data;
    pending_adv_interval = interval_ms;
    return;
  }

  ble_gap_adv_stop();

  struct ble_gap_adv_params adv_params;
  struct ble_hs_adv_fields adv_fields;
  struct ble_hs_adv_fields rsp_fields;
  int rc;

  memset(&adv_fields, 0, sizeof adv_fields);
  memset(&rsp_fields, 0, sizeof rsp_fields);

  adv_fields.flags = data.flags;

  std::vector<uint8_t> mfg_payload;
  if (!data.manufacturer_data.empty() || data.company_id != 0) {
    mfg_payload.reserve(2 + data.manufacturer_data.size());
    mfg_payload.push_back(data.company_id & 0xFF); // LE
    mfg_payload.push_back((data.company_id >> 8) & 0xFF);
    mfg_payload.insert(mfg_payload.end(), data.manufacturer_data.begin(),
                       data.manufacturer_data.end());

    adv_fields.mfg_data = mfg_payload.data();
    adv_fields.mfg_data_len = mfg_payload.size();
  }

  if (data.local_name.has_value()) {
    rsp_fields.name = (uint8_t *)data.local_name.value().c_str();
    rsp_fields.name_len = data.local_name.value().size();
    rsp_fields.name_is_complete = 1;
  }

  // HAP-BLE accessories also advertise the 16-bit service UUID 0xFE59.
  static const ble_uuid16_t hap_uuid = BLE_UUID16_INIT(0xFE59);
  adv_fields.uuids16 = &hap_uuid;
  adv_fields.num_uuids16 = 1;
  adv_fields.uuids16_is_complete = 1;

  rc = ble_gap_adv_set_fields(&adv_fields);
  if (rc != 0) {
    ESP_LOGW(TAG, "adv fields with FE59 failed rc=%d, retry without UUID", rc);
    adv_fields.uuids16 = nullptr;
    adv_fields.num_uuids16 = 0;
    rc = ble_gap_adv_set_fields(&adv_fields);
  }
  if (rc != 0) {
    ESP_LOGE(TAG, "error setting adv fields; rc=%d", rc);
    return;
  }

  rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
  if (rc != 0) {
    ESP_LOGE(TAG, "error setting rsp fields; rc=%d", rc);
    return;
  }

  if (interval_ms > 20) {
    interval_ms = 20;
  }

  memset(&adv_params, 0, sizeof adv_params);
  adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
  adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
  adv_params.channel_map = 0x07;
  adv_params.itvl_min = BLE_GAP_ADV_ITVL_MS(interval_ms);
  adv_params.itvl_max = BLE_GAP_ADV_ITVL_MS(interval_ms);

  rc = ble_gap_adv_start(g_own_addr_type, NULL, BLE_HS_FOREVER, &adv_params,
                         ble_gap_event, this);
  if (rc != 0) {
    ESP_LOGE(TAG, "error enabling advertisement; rc=%d", rc);
  } else {
    ESP_LOGI(TAG, "Advertising started");
    last_adv = data;
    last_adv_interval = interval_ms;
  }
}

void Esp32Ble::stop_advertising() { ble_gap_adv_stop(); }

void Esp32Ble::disconnect(uint16_t connection_id) {
  ble_gap_terminate(connection_id, BLE_ERR_REM_USER_CONN_TERM);
}

void Esp32Ble::set_disconnect_callback(DisconnectCallback callback) {
  disconnect_callback_ = callback;
}

void Esp32Ble::adv_timer_callback(void *arg) {
  auto *self = static_cast<Esp32Ble *>(arg);
  ESP_LOGI(TAG, "Timed advertising: switching to normal interval (%lu ms)",
           (unsigned long)self->normal_interval_ms_);

  self->start_advertising(self->timed_adv_data_, self->normal_interval_ms_);
}

void Esp32Ble::start_timed_advertising(const Advertisement &data,
                                       uint32_t fast_interval_ms,
                                       uint32_t fast_duration_ms,
                                       uint32_t normal_interval_ms) {
  ESP_LOGI(
      TAG,
      "Starting timed advertising: fast=%lums for %lums, then normal=%lums",
      (unsigned long)fast_interval_ms, (unsigned long)fast_duration_ms,
      (unsigned long)normal_interval_ms);

  timed_adv_data_ = data;
  normal_interval_ms_ = normal_interval_ms;

  if (adv_timer_ != nullptr) {
    esp_timer_stop(adv_timer_);
    esp_timer_delete(adv_timer_);
    adv_timer_ = nullptr;
  }

  start_advertising(data, fast_interval_ms);

  esp_timer_create_args_t timer_args = {
      .callback = adv_timer_callback,
      .arg = this,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "adv_timer",
      .skip_unhandled_events = true,
  };

  esp_err_t err = esp_timer_create(&timer_args, &adv_timer_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create advertising timer: %s",
             esp_err_to_name(err));
    return;
  }

  err =
      esp_timer_start_once(adv_timer_, fast_duration_ms * 1000); // microseconds
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start advertising timer: %s",
             esp_err_to_name(err));
    esp_timer_delete(adv_timer_);
    adv_timer_ = nullptr;
  }
}

void Esp32Ble::enc_adv_timer_callback(void *arg) {
  auto *self = static_cast<Esp32Ble *>(arg);
  ESP_LOGI(TAG, "Encrypted advertising duration complete, falling back to "
                "regular advertising");

  self->encrypted_adv_active_ = false;

  if (last_adv.has_value()) {
    self->start_advertising(*last_adv, last_adv_interval);
  }
}

void Esp32Ble::start_encrypted_advertising(const EncryptedAdvertisement &data,
                                           uint32_t interval_ms,
                                           uint32_t duration_ms) {
  if (!nimble_synced) {
    ESP_LOGW(TAG, "Stack not synced, cannot start encrypted advertising");
    return;
  }

  ESP_LOGI(
      TAG,
      "Starting encrypted advertising: interval=%lums duration=%lums GSN=%u",
      (unsigned long)interval_ms, (unsigned long)duration_ms, data.gsn);

  ble_gap_adv_stop();

  if (enc_adv_timer_ != nullptr) {
    esp_timer_stop(enc_adv_timer_);
    esp_timer_delete(enc_adv_timer_);
    enc_adv_timer_ = nullptr;
  }

  encrypted_adv_active_ = true;

  // Build HAP encrypted advertisement format per HAP Spec 7.4.2.2.2 Table 7-46:
  // - Type: 0x11 (HAP Encrypted Notification)
  // - STL: 0x36 (SubType=1 for encrypted notification, Length=22 bytes) per
  // Table 7-48
  // - Advertising ID: 6 bytes
  // - Encrypted Payload: 16 bytes (12 ciphertext + 4 truncated auth tag)
  std::vector<uint8_t> mfg_payload;
  mfg_payload.push_back(0x11); // Type: HAP Encrypted Notification
  mfg_payload.push_back(0x36); // STL: SubType=1 (encrypted), Length=22 (0x16)
  // Advertising ID (6 bytes)
  mfg_payload.insert(mfg_payload.end(), data.advertising_id.begin(),
                     data.advertising_id.end());
  // Encrypted payload (16 bytes: 12 ciphertext + 4 auth tag)
  mfg_payload.insert(mfg_payload.end(), data.encrypted_payload.begin(),
                     data.encrypted_payload.end());

  ESP_LOGI(TAG, "Encrypted adv payload: %d bytes", (int)mfg_payload.size());
  ESP_LOG_BUFFER_HEX_LEVEL(TAG, mfg_payload.data(), mfg_payload.size(),
                           ESP_LOG_DEBUG);

  std::vector<uint8_t> full_mfg_data;
  full_mfg_data.push_back(0x4C);
  full_mfg_data.push_back(0x00);
  full_mfg_data.insert(full_mfg_data.end(), mfg_payload.begin(),
                       mfg_payload.end());

  struct ble_gap_adv_params adv_params;
  struct ble_hs_adv_fields adv_fields;

  memset(&adv_params, 0, sizeof(adv_params));
  memset(&adv_fields, 0, sizeof(adv_fields));

  adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
  adv_fields.mfg_data = full_mfg_data.data();
  adv_fields.mfg_data_len = full_mfg_data.size();

  int rc = ble_gap_adv_set_fields(&adv_fields);
  if (rc != 0) {
    ESP_LOGE(TAG, "Error setting encrypted adv fields: rc=%d", rc);
    encrypted_adv_active_ = false;
    return;
  }

  adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
  adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
  adv_params.itvl_min = BLE_GAP_ADV_ITVL_MS(interval_ms);
  adv_params.itvl_max = BLE_GAP_ADV_ITVL_MS(interval_ms);

  rc = ble_gap_adv_start(g_own_addr_type, NULL, BLE_HS_FOREVER, &adv_params,
                         ble_gap_event, this);
  if (rc != 0) {
    ESP_LOGE(TAG, "Error starting encrypted advertising: rc=%d", rc);
    encrypted_adv_active_ = false;
    return;
  }

  ESP_LOGI(TAG, "Encrypted advertising started");

  esp_timer_create_args_t timer_args = {
      .callback = enc_adv_timer_callback,
      .arg = this,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "enc_adv_timer",
      .skip_unhandled_events = true,
  };

  esp_err_t err = esp_timer_create(&timer_args, &enc_adv_timer_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create encrypted adv timer: %s",
             esp_err_to_name(err));
    return;
  }

  err = esp_timer_start_once(enc_adv_timer_, duration_ms * 1000);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start encrypted adv timer: %s",
             esp_err_to_name(err));
    esp_timer_delete(enc_adv_timer_);
    enc_adv_timer_ = nullptr;
  }
}

bool Esp32Ble::send_indication(uint16_t connection_id,
                               const std::string &characteristic_uuid,
                               std::span<const uint8_t> data) {
  uint16_t attr_handle = 0;

  for (auto *ctx : all_contexts) {
    if (ctx->uuid == characteristic_uuid) {
      attr_handle = ctx->val_handle;
      break;
    }
  }

  if (attr_handle != 0) {
    struct os_mbuf *om = ble_hs_mbuf_from_flat(data.data(), data.size());
    return !ble_gatts_indicate_custom(connection_id, attr_handle, om);
  } else {
    ESP_LOGW(TAG, "Characteristic %s not found for indication",
             characteristic_uuid.c_str());
    return false;
  }
}

int Esp32Ble::gatt_svr_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                                  struct ble_gatt_access_ctxt *ctxt,
                                  void *arg) {
  CharacteristicContext *ctx = static_cast<CharacteristicContext *>(arg);
  if (!ctx)
    return BLE_ATT_ERR_UNLIKELY;

  if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
    ESP_LOGD(TAG, "GATT Read: %s", ctx->uuid.c_str());
    if (ctx->on_read) {
      auto data = ctx->on_read(conn_handle);
      ESP_LOGD(TAG, "  -> Returning %d bytes", (int)data.size());
      int rc = os_mbuf_append(ctxt->om, data.data(), data.size());
      return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
  } else if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
    if (ctx->on_write) {
      std::vector<uint8_t> data;
      data.resize(OS_MBUF_PKTLEN(ctxt->om));
      int rc = os_mbuf_copydata(ctxt->om, 0, data.size(), data.data());

      ESP_LOGD(TAG, "GATT Write: %s, len=%d", ctx->uuid.c_str(),
               (int)data.size());
      if (data.size() < 20) {
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, data.data(), data.size(), ESP_LOG_DEBUG);
      }

      if (rc == 0) {
        ctx->on_write(conn_handle, data, false);
        return 0;
      }
    }
  }

  return 0;
}

int Esp32Ble::gatt_svr_dsc_access(uint16_t conn_handle, uint16_t attr_handle,
                                  struct ble_gatt_access_ctxt *ctxt,
                                  void *arg) {
  DescriptorContext *ctx = static_cast<DescriptorContext *>(arg);
  if (!ctx)
    return BLE_ATT_ERR_UNLIKELY;

  if (ctxt->op == BLE_GATT_ACCESS_OP_READ_DSC) {
    ESP_LOGD(TAG, "GATT Desc Read: %s", ctx->uuid.c_str());
    if (ctx->on_read) {
      auto data = ctx->on_read(conn_handle);
      int rc = os_mbuf_append(ctxt->om, data.data(), data.size());
      return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
  } else if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_DSC) {
    if (ctx->on_write) {
      std::vector<uint8_t> data;
      data.resize(OS_MBUF_PKTLEN(ctxt->om));
      int rc = os_mbuf_copydata(ctxt->om, 0, data.size(), data.data());
      if (rc == 0) {
        ctx->on_write(conn_handle, data);
        return 0;
      }
    }
  }
  return 0;
}

void Esp32Ble::register_service(const ServiceDefinition &service) {
  ESP_LOGD(TAG, "register_service called for UUID: %s", service.uuid.c_str());
  auto svcs = new struct ble_gatt_svc_def[2];
  memset(svcs, 0, sizeof(struct ble_gatt_svc_def) * 2);

  auto &svc_def = svcs[0];
  svc_def.type = service.is_primary ? BLE_GATT_SVC_TYPE_PRIMARY
                                    : BLE_GATT_SVC_TYPE_SECONDARY;

  auto uuid_svc = new ble_uuid_any_t;
  parse_uuid(service.uuid, uuid_svc);
  svc_def.uuid = &uuid_svc->u;

  size_t char_count = service.characteristics.size();
  auto chars_def = new struct ble_gatt_chr_def[char_count + 1];
  memset(chars_def, 0, sizeof(struct ble_gatt_chr_def) * (char_count + 1));

  for (size_t i = 0; i < char_count; ++i) {
    const auto &c = service.characteristics[i];

    auto ctx = new CharacteristicContext();
    ctx->uuid = c.uuid;
    ctx->on_read = c.on_read;
    ctx->on_write = c.on_write;
    ctx->on_subscribe = c.on_subscribe;

    auto uuid_chr = new ble_uuid_any_t;
    parse_uuid(c.uuid, uuid_chr);
    chars_def[i].uuid = &uuid_chr->u;

    chars_def[i].flags = 0;
    if (c.properties.read)
      chars_def[i].flags |= BLE_GATT_CHR_F_READ;
    if (c.properties.write)
      chars_def[i].flags |= BLE_GATT_CHR_F_WRITE;
    if (c.properties.write_without_response)
      chars_def[i].flags |= BLE_GATT_CHR_F_WRITE_NO_RSP;
    if (c.properties.notify)
      chars_def[i].flags |= BLE_GATT_CHR_F_NOTIFY;
    if (c.properties.indicate)
      chars_def[i].flags |= BLE_GATT_CHR_F_INDICATE;

    chars_def[i].access_cb = gatt_svr_chr_access;
    chars_def[i].arg = ctx;

    chars_def[i].val_handle = &ctx->val_handle;

    all_contexts.push_back(ctx);

    size_t desc_count = c.descriptors.size();
    if (desc_count > 0) {
      auto descs_def = new struct ble_gatt_dsc_def[desc_count + 1];
      memset(descs_def, 0, sizeof(struct ble_gatt_dsc_def) * (desc_count + 1));

      for (size_t j = 0; j < desc_count; ++j) {
        const auto &d = c.descriptors[j];
        auto d_ctx = new DescriptorContext();
        d_ctx->uuid = d.uuid;
        d_ctx->on_read = d.on_read;
        d_ctx->on_write = d.on_write;

        auto uuid_dsc = new ble_uuid_any_t;
        parse_uuid(d.uuid, uuid_dsc);
        descs_def[j].uuid = &uuid_dsc->u;

        descs_def[j].att_flags = 0;
        if (d.properties.read)
          descs_def[j].att_flags |= BLE_ATT_F_READ;
        if (d.properties.write)
          descs_def[j].att_flags |= BLE_ATT_F_WRITE;

        descs_def[j].access_cb = gatt_svr_dsc_access;
        descs_def[j].arg = d_ctx;

        all_descriptor_contexts.push_back(d_ctx);
      }
      chars_def[i].descriptors = descs_def;
    }
  }

  svc_def.characteristics = chars_def;

  nim_services.push_back(svcs);

  int rc = ble_gatts_count_cfg(svcs);
  if (rc != 0)
    ESP_LOGE(TAG, "ble_gatts_count_cfg failed: %d", rc);

  rc = ble_gatts_add_svcs(svcs);
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_gatts_add_svcs failed: %d", rc);
  } else {
    ESP_LOGD(TAG, "ble_gatts_add_svcs success");
  }
}

int Esp32Ble::ble_gap_event(struct ble_gap_event *event, void *arg) {
  (void)arg;

  ESP_LOGI(TAG, "BLE GAP Event: type=%d", event->type);

  switch (event->type) {
  case BLE_GAP_EVENT_CONNECT:
    {
      auto self = static_cast<Esp32Ble *>(arg);
      if (event->connect.status != 0) {
        ESP_LOGE(TAG, "Connect failed status=%d, restarting advertising",
                 event->connect.status);
        if (self && last_adv.has_value()) {
          self->start_advertising(*last_adv, 20);
        }
        break;
      }
      ESP_LOGI(TAG, "Connected handle=%d", event->connect.conn_handle);
      struct ble_gap_upd_params params = {};
      params.itvl_min = 16;
      params.itvl_max = 48;
      params.latency = 0;
      params.supervision_timeout = 1600;
      params.min_ce_len = 0;
      params.max_ce_len = 0;
      int rc = ble_gap_update_params(event->connect.conn_handle, &params);
      ESP_LOGI(TAG, "Connection params update rc=%d (16s supervision)", rc);
      if (self && self->adv_timer_ != nullptr) {
        esp_timer_stop(self->adv_timer_);
        esp_timer_delete(self->adv_timer_);
        self->adv_timer_ = nullptr;
      }
      if (self && self->encrypted_adv_active_) {
        ESP_LOGI(
            TAG,
            "Connection during encrypted advertising - aborting broadcast");
        self->encrypted_adv_active_ = false;
        if (self->enc_adv_timer_ != nullptr) {
          esp_timer_stop(self->enc_adv_timer_);
          esp_timer_delete(self->enc_adv_timer_);
          self->enc_adv_timer_ = nullptr;
        }
      }
    }
    break;
  case BLE_GAP_EVENT_DISCONNECT:
    ESP_LOGI(TAG, "Disconnected, reason=0x%x", event->disconnect.reason);
    {
      auto self = static_cast<Esp32Ble *>(arg);
      if (self && self->disconnect_callback_) {
        self->disconnect_callback_(event->disconnect.conn.conn_handle);
      }
    }
    break;
  case BLE_GAP_EVENT_SUBSCRIBE:
    ESP_LOGI(TAG, "Subscribe: conn=%d attr=%d reason=%d notify=%d indicate=%d",
             event->subscribe.conn_handle, event->subscribe.attr_handle,
             event->subscribe.reason, event->subscribe.cur_notify,
             event->subscribe.cur_indicate);
    for (auto *ctx : all_contexts) {
      if (ctx->val_handle == event->subscribe.attr_handle) {
        if (ctx->on_subscribe) {
          // HAP uses indications (cur_indicate), but also support notifications
          bool subscribed = (event->subscribe.cur_notify > 0) ||
                            (event->subscribe.cur_indicate > 0);
          ctx->on_subscribe(event->subscribe.conn_handle, subscribed);
        }
        break;
      }
    }
    break;
  case BLE_GAP_EVENT_MTU:
    ESP_LOGI(TAG, "MTU Update: conn=%d mtu=%d", event->mtu.conn_handle,
             event->mtu.value);
    break;
  case BLE_GAP_EVENT_CONN_UPDATE:
    ESP_LOGI(TAG, "Connection Update: conn=%d", event->conn_update.conn_handle);
    break;
  case BLE_GAP_EVENT_ENC_CHANGE:
    ESP_LOGI(TAG, "Encryption Change: conn=%d status=%d",
             event->enc_change.conn_handle, event->enc_change.status);
    break;
  case BLE_GAP_EVENT_PASSKEY_ACTION:
    ESP_LOGI(TAG, "Passkey Action: conn=%d action=%d",
             event->passkey.conn_handle, event->passkey.params.action);
    return BLE_HS_ENOTSUP;
  case BLE_GAP_EVENT_REPEAT_PAIRING:
    ESP_LOGW(TAG, "Repeat Pairing: conn=%d — delete stale bond and retry",
             event->repeat_pairing.conn_handle);
    return BLE_GAP_REPEAT_PAIRING_RETRY;
  default:
    ESP_LOGI(TAG, "Unhandled GAP event: %d", event->type);
    break;
  }
  return 0;
}
#else
#warning                                                                       \
    "NimBLE is not enabled. Make sure to enable it in the IDF menuconfig if you want to use the BLE component."
#endif
