#include <sdkconfig.h>
#if CONFIG_BT_NIMBLE_ENABLED
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <optional>
#include <span>
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
#include <host/ble_gap.h>
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
static uint16_t s_ble_conns = 0;
static bool s_force_adv_restart = false;
static bool s_hold_adv_for_gatt = false;
static bool s_rnd_addr_ready = false;
static esp_timer_handle_t s_adv_ensure_timer = nullptr;
static int s_adv_ensure_pass = 0;
// ble_gap_adv_start() can return before ble_gap_adv_active() is true.
// The 100 ms ensure-timer then force-restarted the same payload and tore
// down the instance Home was about to use.
static int64_t s_last_adv_start_us = 0;
static constexpr int64_t kAdvStartSettleUs = 250000;

struct QueuedIndicate {
  uint16_t conn_id = 0;
  uint16_t attr_handle = 0;
  uint8_t retries = 0;
  std::vector<uint8_t> data;
};

static bool s_indicate_busy = false;
static int s_indicate_start_depth = 0;
static std::vector<QueuedIndicate> s_indicate_q;
static esp_timer_handle_t s_indicate_watchdog = nullptr;
static esp_timer_handle_t s_indicate_retry = nullptr;

static void flush_indicate_queue();
static void schedule_indicate_retry();
static void enqueue_indicate(uint16_t conn_id, uint16_t attr_handle,
                             std::span<const uint8_t> data);

static void indicate_watchdog_stop() {
  if (s_indicate_watchdog != nullptr) {
    esp_timer_stop(s_indicate_watchdog);
  }
}

static void indicate_watchdog_cb(void * /*arg*/) {
  ESP_LOGW(TAG, "Indicate confirmation timed out; flushing queue");
  flush_indicate_queue();
}

static void indicate_watchdog_arm() {
  if (s_indicate_watchdog == nullptr) {
    const esp_timer_create_args_t args = {
        .callback = indicate_watchdog_cb,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "ind_wd",
        .skip_unhandled_events = true,
    };
    if (esp_timer_create(&args, &s_indicate_watchdog) != ESP_OK) {
      s_indicate_watchdog = nullptr;
      return;
    }
  }
  esp_timer_stop(s_indicate_watchdog);
  esp_timer_start_once(s_indicate_watchdog, 800 * 1000);
}

static void clear_indicate_queue() {
  indicate_watchdog_stop();
  if (s_indicate_retry != nullptr) {
    esp_timer_stop(s_indicate_retry);
  }
  s_indicate_busy = false;
  s_indicate_q.clear();
}

static int start_indicate(uint16_t conn_id, uint16_t attr_handle,
                          std::span<const uint8_t> data) {
  static const uint8_t kEmpty = 0;
  const uint8_t *ptr = data.empty() ? &kEmpty : data.data();
  struct os_mbuf *om = ble_hs_mbuf_from_flat(ptr, data.size());
  if (om == nullptr) {
    ESP_LOGW(TAG, "indicate mbuf alloc failed attr=%u", attr_handle);
    return BLE_HS_ENOMEM;
  }
  // NimBLE posts BLE_GAP_EVENT_NOTIFY_TX status=0 from inside this call
  // when the controller accepts the PDU. That is not the peer confirm.
  ++s_indicate_start_depth;
  const int rc = ble_gatts_indicate_custom(conn_id, attr_handle, om);
  --s_indicate_start_depth;
  if (rc != 0) {
    ESP_LOGW(TAG, "indicate failed conn=%u attr=%u rc=%d", conn_id, attr_handle,
             rc);
    return rc;
  }
  s_indicate_busy = true;
  indicate_watchdog_arm();
  return 0;
}

static void enqueue_indicate(uint16_t conn_id, uint16_t attr_handle,
                             std::span<const uint8_t> data) {
  for (auto &queued : s_indicate_q) {
    if (queued.attr_handle == attr_handle) {
      queued.conn_id = conn_id;
      queued.data.assign(data.begin(), data.end());
      return;
    }
  }
  s_indicate_q.push_back(QueuedIndicate{
      conn_id, attr_handle, 0,
      std::vector<uint8_t>(data.begin(), data.end())});
}

static void indicate_retry_cb(void * /*arg*/) {
  if (!s_indicate_busy) {
    flush_indicate_queue();
  }
}

static void schedule_indicate_retry() {
  if (s_indicate_retry == nullptr) {
    const esp_timer_create_args_t args = {
        .callback = indicate_retry_cb,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "ind_retry",
        .skip_unhandled_events = true,
    };
    if (esp_timer_create(&args, &s_indicate_retry) != ESP_OK) {
      s_indicate_retry = nullptr;
      return;
    }
  }
  esp_timer_stop(s_indicate_retry);
  esp_timer_start_once(s_indicate_retry, 50 * 1000);
}

static void flush_indicate_queue() {
  indicate_watchdog_stop();
  s_indicate_busy = false;
  while (!s_indicate_q.empty()) {
    QueuedIndicate next = std::move(s_indicate_q.front());
    s_indicate_q.erase(s_indicate_q.begin());
    const int rc = start_indicate(next.conn_id, next.attr_handle, next.data);
    if (rc == 0) {
      return;
    }
    if (next.retries < 5 &&
        (rc == BLE_HS_ENOENT || rc == BLE_HS_EAGAIN || rc == BLE_HS_ENOMEM)) {
      next.retries++;
      s_indicate_q.insert(s_indicate_q.begin(), std::move(next));
      schedule_indicate_retry();
      return;
    }
    ESP_LOGW(TAG, "Dropping indicate attr=%u after retries rc=%d",
             next.attr_handle, rc);
  }
}

// HAP Spec 7.1: accessories use a static random address. The 48-bit
// address must match the HAP Device ID so iPhone can reconnect after
// Pair-Setup (public + random-rotating both show up as 未响应).
static void use_hap_static_random_addr() {
  if (s_rnd_addr_ready && g_own_addr_type == BLE_OWN_ADDR_RANDOM) {
    return;
  }
  uint8_t rnd[6] = {};
  bool have_id = false;
  if (g_storage) {
    auto stored = g_storage->get("accessory_id");
    if (stored && !stored->empty()) {
      const std::string id(stored->begin(), stored->end());
      unsigned b[6] = {};
      if (std::sscanf(id.c_str(), "%02X:%02X:%02X:%02X:%02X:%02X", &b[0], &b[1],
                      &b[2], &b[3], &b[4], &b[5]) == 6) {
        // NimBLE stores addresses LSB-first. Do not OR 0xC0 here: the
        // stored Device ID is already a static random address, and
        // changing bits would make AdvA != HAP Device ID (未响应).
        for (int i = 0; i < 6; ++i) {
          rnd[i] = static_cast<uint8_t>(b[5 - i]);
        }
        have_id = true;
      }
    }
  }
  if (!have_id) {
    ESP_LOGW(TAG, "No HAP Device ID; generating a static random address");
    esp_fill_random(rnd, sizeof(rnd));
    rnd[5] = static_cast<uint8_t>((rnd[5] & 0x3f) | 0xc0);
  }
  const int rc = ble_hs_id_set_rnd(rnd);
  if (rc != 0) {
    ESP_LOGW(TAG, "ble_hs_id_set_rnd rc=%d, using public address", rc);
    g_own_addr_type = BLE_OWN_ADDR_PUBLIC;
    return;
  }
  g_own_addr_type = BLE_OWN_ADDR_RANDOM;
  s_rnd_addr_ready = true;
}

static void force_start_last_adv(const char *why);

static bool adv_start_in_flight() {
  if (ble_gap_adv_active() || s_last_adv_start_us == 0) {
    return false;
  }
  return (esp_timer_get_time() - s_last_adv_start_us) < kAdvStartSettleUs;
}

static void adv_ensure_timer_cb(void *arg) {
  (void)arg;
  if (!last_adv.has_value() || g_ble_instance == nullptr) {
    return;
  }
  if (s_ble_conns > 0 || s_hold_adv_for_gatt) {
    ESP_LOGI(TAG, "Skip ensure-timer; link is up (pass %d)", s_adv_ensure_pass);
    return;
  }
  if (ble_gap_adv_active()) {
    ESP_LOGI(TAG, "Advertising confirmed after disconnect (pass %d)",
             s_adv_ensure_pass);
  } else if (adv_start_in_flight()) {
    ESP_LOGI(TAG, "Advertising start in flight after disconnect (pass %d)",
             s_adv_ensure_pass);
  } else {
    ESP_LOGW(TAG, "Advertising not active after disconnect; starting (pass %d)",
             s_adv_ensure_pass);
    force_start_last_adv("ensure-timer");
  }
  if (s_adv_ensure_pass < 1) {
    ++s_adv_ensure_pass;
    if (s_adv_ensure_timer != nullptr) {
      esp_timer_start_once(s_adv_ensure_timer, 200000);
    }
  }
}

static void schedule_adv_ensure() {
  if (s_adv_ensure_timer == nullptr) {
    esp_timer_create_args_t args = {
        .callback = adv_ensure_timer_cb,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "adv_ensure",
        .skip_unhandled_events = true,
    };
    if (esp_timer_create(&args, &s_adv_ensure_timer) != ESP_OK) {
      ESP_LOGW(TAG, "Failed to create advertising ensure timer");
      return;
    }
  }
  s_adv_ensure_pass = 0;
  esp_timer_stop(s_adv_ensure_timer);
  // Start after NimBLE finishes tearing down the connection. Starting
  // from the disconnect callback itself is often undone by the stack.
  esp_timer_start_once(s_adv_ensure_timer, 100000);
}

static void force_start_last_adv(const char *why) {
  if (s_hold_adv_for_gatt) {
    ESP_LOGI(TAG, "Defer advertising (%s): wait for Pair-Verify", why);
    return;
  }
  if (g_ble_instance == nullptr || !last_adv.has_value()) {
    ESP_LOGW(TAG, "Cannot restart advertising (%s): no last payload", why);
    return;
  }
  ESP_LOGI(TAG, "Force advertising restart (%s)", why);
  s_force_adv_restart = true;
  const uint32_t interval = last_adv_interval != 0 ? last_adv_interval : 20;
  g_ble_instance->start_advertising(*last_adv, interval);
}

static void restart_adv_for_new_link(const char *why) {
  if (s_hold_adv_for_gatt) {
    ESP_LOGI(TAG, "Defer advertising (%s): wait for Pair-Verify", why);
    return;
  }
  if (s_ble_conns >= CONFIG_BT_NIMBLE_MAX_CONNECTIONS) {
    ESP_LOGI(TAG, "Skip advertising (%s): at max links %u", why,
             static_cast<unsigned>(s_ble_conns));
    return;
  }
  force_start_last_adv(why);
}

static void log_adv_identity(const hap::platform::Ble::Advertisement &data) {
  uint8_t adva[6] = {};
  ble_hs_id_copy_addr(g_own_addr_type, adva, nullptr);
  ESP_LOGI(TAG, "AdvA type=%d %02X:%02X:%02X:%02X:%02X:%02X", g_own_addr_type,
           adva[5], adva[4], adva[3], adva[2], adva[1], adva[0]);
  if (data.manufacturer_data.size() >= 9 && data.manufacturer_data[0] == 0x06) {
    const uint8_t *id = data.manufacturer_data.data() + 3;
    ESP_LOGI(TAG, "HAP Device ID %02X:%02X:%02X:%02X:%02X:%02X SF=%u", id[0],
             id[1], id[2], id[3], id[4], id[5], data.manufacturer_data[2]);
    if (!(adva[5] == id[0] && adva[4] == id[1] && adva[3] == id[2] &&
          adva[2] == id[3] && adva[1] == id[4] && adva[0] == id[5])) {
      ESP_LOGW(TAG, "AdvA != Device ID; Home will show 未响应 after pairing");
    }
  }
}

// Update manufacturer data / name without ble_gap_adv_stop. Home scans for
// SF=0 during Add Accessory while the iPhone is still connected; a stop/start
// gap there looks like 未响应.
static int apply_advertising_fields(const hap::platform::Ble::Advertisement &data) {
  struct ble_hs_adv_fields adv_fields;
  struct ble_hs_adv_fields rsp_fields;
  memset(&adv_fields, 0, sizeof adv_fields);
  memset(&rsp_fields, 0, sizeof rsp_fields);

  adv_fields.flags = data.flags;

  std::vector<uint8_t> mfg_payload;
  if (!data.manufacturer_data.empty() || data.company_id != 0) {
    mfg_payload.reserve(2 + data.manufacturer_data.size());
    mfg_payload.push_back(data.company_id & 0xFF);
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

  static const ble_uuid16_t hap_uuid = BLE_UUID16_INIT(0xFE59);
  adv_fields.uuids16 = &hap_uuid;
  adv_fields.num_uuids16 = 1;
  adv_fields.uuids16_is_complete = 1;
  rsp_fields.uuids16 = &hap_uuid;
  rsp_fields.num_uuids16 = 1;
  rsp_fields.uuids16_is_complete = 1;

  int rc = ble_gap_adv_set_fields(&adv_fields);
  if (rc != 0) {
    ESP_LOGW(TAG, "adv fields with FE59 failed rc=%d, retry without UUID", rc);
    adv_fields.uuids16 = nullptr;
    adv_fields.num_uuids16 = 0;
    rc = ble_gap_adv_set_fields(&adv_fields);
  } else {
    ESP_LOGI(TAG, "Advertisement includes 0xFE59");
  }
  if (rc != 0) {
    ESP_LOGE(TAG, "error setting adv fields; rc=%d", rc);
    return rc;
  }

  rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
  if (rc != 0) {
    ESP_LOGW(TAG, "scan rsp with FE59 failed rc=%d, retry name only", rc);
    rsp_fields.uuids16 = nullptr;
    rsp_fields.num_uuids16 = 0;
    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
  }
  if (rc != 0) {
    ESP_LOGE(TAG, "error setting rsp fields; rc=%d", rc);
  }
  return rc;
}

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

    use_hap_static_random_addr();
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

uint16_t Esp32Ble::active_connections() const { return s_ble_conns; }

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

  if (interval_ms > 20) {
    interval_ms = 20;
  }

  // HAP asked to advertise (Pair-Verify SF=0, disconnected GSN). The
  // radio is free for GAP again.
  s_hold_adv_for_gatt = false;

  const bool same_payload =
      last_adv.has_value() && last_adv_interval == interval_ms &&
      last_adv->manufacturer_data == data.manufacturer_data &&
      last_adv->local_name == data.local_name && last_adv->flags == data.flags &&
      last_adv->company_id == data.company_id;

  const bool force = s_force_adv_restart;
  s_force_adv_restart = false;
  const bool adv_active = ble_gap_adv_active();

  // Same GSN already on the air (or a start just issued). A stop/start
  // here is what made Home miss GSN=31 for ~7 s in the sync-rev=3 logs.
  if (!force && same_payload && (adv_active || adv_start_in_flight())) {
    ESP_LOGI(TAG, "Advertising already current, skip restart");
    return;
  }

  // Flip SF / GSN without stopping only while a controller is connected.
  // iPhone scans for SF=0 during Add Accessory on the existing instance.
  // After hangup, iOS ignores in-place manufacturer-data updates — a
  // disconnected GSN change must be a new advertising instance.
  if (!force && s_ble_conns > 0 && adv_active &&
      last_adv_interval == interval_ms) {
    const int rc = apply_advertising_fields(data);
    if (rc == 0) {
      last_adv = data;
      last_adv_interval = interval_ms;
      ESP_LOGI(TAG, "Advertising payload updated in place");
      log_adv_identity(data);
      return;
    }
    ESP_LOGW(TAG, "In-place adv update failed rc=%d, restarting", rc);
  }

  if (!force && s_ble_conns == 0 && !same_payload && adv_active) {
    ESP_LOGI(TAG, "Advertising restart (disconnected GSN/payload change)");
  }

  ble_gap_adv_stop();
  use_hap_static_random_addr();

  const int fields_rc = apply_advertising_fields(data);
  if (fields_rc != 0) {
    s_last_adv_start_us = 0;
    return;
  }

  struct ble_gap_adv_params adv_params;
  memset(&adv_params, 0, sizeof adv_params);
  adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
  adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
  adv_params.channel_map = 0x07;
  adv_params.itvl_min = BLE_GAP_ADV_ITVL_MS(interval_ms);
  adv_params.itvl_max = BLE_GAP_ADV_ITVL_MS(interval_ms);

  const int rc = ble_gap_adv_start(g_own_addr_type, NULL, BLE_HS_FOREVER,
                                   &adv_params, ble_gap_event, this);
  if (rc != 0) {
    ESP_LOGE(TAG, "error enabling advertisement; rc=%d", rc);
    s_last_adv_start_us = 0;
  } else {
    ESP_LOGI(TAG, "Advertising started");
    last_adv = data;
    last_adv_interval = interval_ms;
    s_last_adv_start_us = esp_timer_get_time();
    log_adv_identity(data);
  }
}

void Esp32Ble::stop_advertising() {
  ble_gap_adv_stop();
  s_last_adv_start_us = 0;
}

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

  if (attr_handle == 0) {
    ESP_LOGW(TAG, "Characteristic %s not found for indication",
             characteristic_uuid.c_str());
    return false;
  }

  // NimBLE allows one GATT procedure at a time. Rapid encoder / Home writes
  // used to start overlapping indicates and iPhone dropped the link (0x213).
  if (s_indicate_busy) {
    enqueue_indicate(connection_id, attr_handle, data);
    return true;
  }
  const int rc = start_indicate(connection_id, attr_handle, data);
  if (rc == 0) {
    return true;
  }
  enqueue_indicate(connection_id, attr_handle, data);
  schedule_indicate_retry();
  return true;
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
      if (s_ble_conns < 0xFFFF) {
        ++s_ble_conns;
      }
      ESP_LOGI(TAG, "Connected handle=%d links=%u", event->connect.conn_handle,
               static_cast<unsigned>(s_ble_conns));
      struct ble_gap_conn_desc desc;
      if (ble_gap_conn_find(event->connect.conn_handle, &desc) == 0) {
        ESP_LOGI(TAG,
                 "Peer %02X:%02X:%02X:%02X:%02X:%02X type=%d",
                 desc.peer_ota_addr.val[5], desc.peer_ota_addr.val[4],
                 desc.peer_ota_addr.val[3], desc.peer_ota_addr.val[2],
                 desc.peer_ota_addr.val[1], desc.peer_ota_addr.val[0],
                 desc.peer_ota_addr.type);
      }
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
      // Keep the controller on GATT until Pair-Verify. Restarting
      // advertising ~60 ms after CONNECT (on CONN_UPDATE) made Home
      // subscribe to Service Changed and time out at ~2.5 s with no
      // Pair-Verify and no characteristic CCCDs.
      s_hold_adv_for_gatt = true;
      if (s_ble_conns >= CONFIG_BT_NIMBLE_MAX_CONNECTIONS) {
        ESP_LOGI(TAG, "At max BLE links (%u); advertising after a drop",
                 static_cast<unsigned>(s_ble_conns));
      } else {
        ESP_LOGI(TAG, "Hold advertising until Pair-Verify");
      }
    }
    break;
  case BLE_GAP_EVENT_NOTIFY_TX:
    if (!event->notify_tx.indication) {
      break;
    }
    // NimBLE delivers status=0 from inside ble_gatts_indicate_custom when
    // the controller accepts the PDU. Flushing then started the next
    // indicate before the peer confirmed. Wait for the later event.
    if (s_indicate_start_depth > 0) {
      ESP_LOGD(TAG, "Indicate queued conn=%d attr=%d status=%d",
               event->notify_tx.conn_handle, event->notify_tx.attr_handle,
               event->notify_tx.status);
      break;
    }
    ESP_LOGI(TAG, "Indicate confirmed conn=%d attr=%d status=%d",
             event->notify_tx.conn_handle, event->notify_tx.attr_handle,
             event->notify_tx.status);
    flush_indicate_queue();
    break;
  case BLE_GAP_EVENT_ADV_COMPLETE:
    ESP_LOGW(TAG, "Advertising complete reason=%d; restarting",
             event->adv_complete.reason);
    restart_adv_for_new_link("adv-complete");
    break;
  case BLE_GAP_EVENT_DISCONNECT:
    if (s_ble_conns > 0) {
      --s_ble_conns;
    }
    ESP_LOGI(TAG, "Disconnected, reason=0x%x links=%u",
             event->disconnect.reason, static_cast<unsigned>(s_ble_conns));
    {
      s_hold_adv_for_gatt = false;
      clear_indicate_queue();
      auto self = static_cast<Esp32Ble *>(arg);
      // Do not start advertising here. NimBLE is still cleaning up the
      // link and will drop a start issued from this callback. Stop now
      // and start again from the 100 ms timer.
      ble_gap_adv_stop();
      s_last_adv_start_us = 0;
      ESP_LOGI(TAG, "Advertising stopped on disconnect; will restart shortly");
      schedule_adv_ensure();
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
  case BLE_GAP_EVENT_CONN_UPDATE_REQ:
    ESP_LOGI(TAG, "Connection Update Request: conn=%d (accept)",
             event->conn_update_req.conn_handle);
    return 0;
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
