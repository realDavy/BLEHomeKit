#include <sdkconfig.h>
#if CONFIG_PAL_NETWORK_ENABLED
#include "Esp32Network.hpp"
#include "esp_wifi_he.h"
#include <arpa/inet.h>
#include <cstring>
#include <esp_log.h>
#include <esp_netif.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
#include <lwip/sockets.h>
#include <string>

static const char *TAG = "Esp32Network";

static EventGroupHandle_t s_wifi_event_group = nullptr;
static const int WIFI_CONNECTED_BIT = BIT0;
static const int WIFI_FAIL_BIT = BIT1;
static int s_retry_num = 0;
static const int MAX_RETRY = 10;
#if CONFIG_ITWT_TRIGGER_ENABLE
uint8_t trigger_enabled = 1;
#else
uint8_t trigger_enabled = 0;
#endif

#if CONFIG_ITWT_ANNOUNCED
uint8_t flow_type_announced = 1;
#else
uint8_t flow_type_announced = 0;
#endif

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {
    if (s_retry_num < MAX_RETRY) {
      esp_wifi_connect();
      s_retry_num++;
      ESP_LOGI(TAG, "Retrying WiFi connection (%d/%d)...", s_retry_num,
               MAX_RETRY);
    } else {
      xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
    }
    ESP_LOGI(TAG, "WiFi disconnected");
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    s_retry_num = 0;
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    /* setup a trigger-based announce individual TWT agreement. */
    wifi_phy_mode_t phymode;
    wifi_config_t sta_cfg = {};
    esp_wifi_get_config(WIFI_IF_STA, &sta_cfg);
    esp_wifi_sta_get_negotiated_phymode(&phymode);
    if (phymode == WIFI_PHY_MODE_HE20) {
      esp_err_t err = ESP_OK;
      wifi_itwt_setup_config_t setup_config = {
          .setup_cmd = TWT_REQUEST,
          .trigger = trigger_enabled,
          .flow_type = static_cast<uint16_t>(flow_type_announced ? 0 : 1),
          .flow_id = 0,
          .wake_invl_expn = CONFIG_ITWT_WAKE_INVL_EXPN,
          .wake_duration_unit = CONFIG_ITWT_WAKE_DURATION_UNIT,
          .min_wake_dura = CONFIG_ITWT_MIN_WAKE_DURA,
          .wake_invl_mant = CONFIG_ITWT_WAKE_INVL_MANT,
          .twt_id = CONFIG_ITWT_ID,
          .timeout_time_ms = CONFIG_ITWT_SETUP_TIMEOUT_TIME_MS,
      };
      err = esp_wifi_sta_itwt_setup(&setup_config);
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "itwt setup failed, err:0x%x", err);
      }
    } else {
      ESP_LOGE(TAG, "Must be in 11ax mode to support itwt");
    }
  }
}

static void itwt_setup_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
  wifi_event_sta_itwt_setup_t *setup =
      (wifi_event_sta_itwt_setup_t *)event_data;
  if (setup->status == 1) {
    ESP_LOGI(TAG,
             "<WIFI_EVENT_ITWT_SETUP>twt_id:%d, flow_id:%d, %s, %s, "
             "wake_dura:%d, wake_dura_unit:%d, wake_invl_e:%d, wake_invl_m:%d",
             setup->config.twt_id, setup->config.flow_id,
             setup->config.trigger ? "trigger-enabled" : "non-trigger-enabled",
             setup->config.flow_type ? "unannounced" : "announced",
             setup->config.min_wake_dura, setup->config.wake_duration_unit,
             setup->config.wake_invl_expn, setup->config.wake_invl_mant);
    ESP_LOGI(TAG,
             "<WIFI_EVENT_ITWT_SETUP>target wake time:%lld, wake duration:%d "
             "us, service period:%d us",
             setup->target_wake_time,
             setup->config.min_wake_dura
                 << (setup->config.wake_duration_unit == 1 ? 10 : 8),
             setup->config.wake_invl_mant << setup->config.wake_invl_expn);
  } else {
    if (setup->status == ESP_ERR_WIFI_TWT_SETUP_TIMEOUT) {
      ESP_LOGE(TAG,
               "<WIFI_EVENT_ITWT_SETUP>twt_id:%d, timeout of receiving twt "
               "setup response frame",
               setup->config.twt_id);
    } else if (setup->status == ESP_ERR_WIFI_TWT_SETUP_TXFAIL) {
      ESP_LOGE(TAG,
               "<WIFI_EVENT_ITWT_SETUP>twt_id:%d, twt setup frame tx failed, "
               "reason: %d",
               setup->config.twt_id, setup->reason);
    } else if (setup->status == ESP_ERR_WIFI_TWT_SETUP_REJECT) {
      ESP_LOGE(TAG,
               "<WIFI_EVENT_ITWT_SETUP>twt_id:%d, twt setup request was "
               "rejected, setup cmd: %d",
               setup->config.twt_id, setup->config.setup_cmd);
    } else {
      ESP_LOGE(TAG,
               "<WIFI_EVENT_ITWT_SETUP>twt_id:%d, twt setup failed, status: %d",
               setup->config.twt_id, setup->status);
    }
  }
}

static void itwt_teardown_handler(void *arg, esp_event_base_t event_base,
                                  int32_t event_id, void *event_data) {
  wifi_event_sta_itwt_teardown_t *teardown =
      (wifi_event_sta_itwt_teardown_t *)event_data;
  if (teardown->status == ITWT_TEARDOWN_FAIL) {
    ESP_LOGE(
        TAG,
        "<WIFI_EVENT_ITWT_TEARDOWN>flow_id %d%s, twt teardown frame tx failed",
        teardown->flow_id, (teardown->flow_id == 8) ? "(all twt)" : "");
  } else {
    ESP_LOGI(TAG, "<WIFI_EVENT_ITWT_TEARDOWN>flow_id %d%s", teardown->flow_id,
             (teardown->flow_id == 8) ? "(all twt)" : "");
  }
}

static void itwt_suspend_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data) {
  wifi_event_sta_itwt_suspend_t *suspend =
      (wifi_event_sta_itwt_suspend_t *)event_data;
  ESP_LOGI(
      TAG,
      "<WIFI_EVENT_ITWT_SUSPEND>status:%d, flow_id_bitmap:0x%x, "
      "actual_suspend_time_ms:[%lu %lu %lu %lu %lu %lu %lu %lu]",
      suspend->status, suspend->flow_id_bitmap,
      suspend->actual_suspend_time_ms[0], suspend->actual_suspend_time_ms[1],
      suspend->actual_suspend_time_ms[2], suspend->actual_suspend_time_ms[3],
      suspend->actual_suspend_time_ms[4], suspend->actual_suspend_time_ms[5],
      suspend->actual_suspend_time_ms[6], suspend->actual_suspend_time_ms[7]);
}

static const char *itwt_probe_status_to_str(wifi_itwt_probe_status_t status) {
  switch (status) {
  case ITWT_PROBE_FAIL:
    return "itwt probe fail";
  case ITWT_PROBE_SUCCESS:
    return "itwt probe success";
  case ITWT_PROBE_TIMEOUT:
    return "itwt probe timeout";
  case ITWT_PROBE_STA_DISCONNECTED:
    return "Sta disconnected";
  default:
    return "Unknown status";
  }
}

static void itwt_probe_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
  wifi_event_sta_itwt_probe_t *probe =
      (wifi_event_sta_itwt_probe_t *)event_data;
  ESP_LOGI(TAG, "<WIFI_EVENT_ITWT_PROBE>status:%s, reason:0x%x",
           itwt_probe_status_to_str(probe->status), probe->reason);
}

bool Esp32Network::wifi_init(const char *ssid, const char *password) {
  ESP_LOGI(TAG, "Initializing WiFi...");

  s_wifi_event_group = xEventGroupCreate();

  ESP_ERROR_CHECK(esp_netif_init());
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  esp_event_handler_instance_t instance_any_id;
  esp_event_handler_instance_t instance_got_ip;
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, nullptr,
      &instance_any_id));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, nullptr,
      &instance_got_ip));

  /* itwt */
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, WIFI_EVENT_ITWT_SETUP, &itwt_setup_handler, NULL, NULL));
  ESP_ERROR_CHECK(
      esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_ITWT_TEARDOWN,
                                          &itwt_teardown_handler, NULL, NULL));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, WIFI_EVENT_ITWT_SUSPEND, &itwt_suspend_handler, NULL, NULL));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, WIFI_EVENT_ITWT_PROBE, &itwt_probe_handler, NULL, NULL));

  wifi_config_t wifi_config = {};
  strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
  strncpy((char *)wifi_config.sta.password, password,
          sizeof(wifi_config.sta.password) - 1);

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  wifi_twt_config_t wifi_twt_config = {
      .post_wakeup_event = true,
      .twt_enable_keep_alive = true,
  };
  ESP_ERROR_CHECK(esp_wifi_sta_twt_config(&wifi_twt_config));
  esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW20);
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11N | WIFI_PROTOCOL_11AX);
  esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "WiFi init finished, connecting to %s...", ssid);

  // Wait for connection
  EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                         WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                         pdFALSE, pdFALSE, portMAX_DELAY);

  if (bits & WIFI_CONNECTED_BIT) {
    ESP_LOGI(TAG, "Connected to WiFi SSID: %s", ssid);
    return true;
  } else if (bits & WIFI_FAIL_BIT) {
    ESP_LOGE(TAG, "Failed to connect to SSID: %s", ssid);
    return false;
  }

  ESP_LOGE(TAG, "Unexpected WiFi event");
  return false;
}

Esp32Network::Esp32Network() {}

Esp32Network::~Esp32Network() {
  running_ = false;

  if (server_fd_ >= 0) {
    close(server_fd_);
    server_fd_ = -1;
  }

  // Close all client connections
  std::lock_guard<std::mutex> lock(connections_mutex_);
  for (auto &[id, fd] : connections_) {
    close(fd);
  }
  connections_.clear();

  if (mdns_initialized_) {
    mdns_free();
  }
}

void Esp32Network::mdns_register(const MdnsService &service) {
  ESP_LOGI(TAG, "mDNS Register: %.*s._hap._tcp Port: %d",
           (int)service.name.size(), service.name.data(), service.port);

  current_service_ = service;
  port_ = service.port;

  if (mdns_initialized_) {
    return;
  }
  esp_err_t err;
  std::string hostname(service.name);
#ifdef CONFIG_PAL_ESPHOME
  mdns_initialized_ = true;
#else
  err = mdns_init();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "mDNS init failed: %s", esp_err_to_name(err));
    return;
  }
  mdns_initialized_ = true;

  mdns_hostname_set(hostname.c_str());
#endif
  mdns_service_instance_name_set("_hap", "_tcp", hostname.c_str());
  err = mdns_service_add(hostname.c_str(), "_hap", "_tcp", service.port,
                         nullptr, 0);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "mDNS service add failed: %s", esp_err_to_name(err));
    return;
  }

  // Add TXT records
  for (const auto &kv : service.txt_records) {
    mdns_service_txt_item_set("_hap", "_tcp", kv.first.c_str(),
                              kv.second.c_str());
  }

  ESP_LOGI(TAG, "mDNS service registered");
}

void Esp32Network::mdns_update_txt_record(const MdnsService &service) {
  ESP_LOGI(TAG, "mDNS Update TXT: %.*s", (int)service.name.size(),
           service.name.data());

  current_service_ = service;

  if (!mdns_initialized_) {
    ESP_LOGE(TAG, "mDNS not initialized");
    return;
  }

  // Build TXT items array
  std::vector<mdns_txt_item_t> txt_items;
  std::vector<std::string> keys;
  std::vector<std::string> values;

  // Keep strings alive
  for (const auto &kv : service.txt_records) {
    keys.push_back(kv.first);
    values.push_back(kv.second);
  }

  for (size_t i = 0; i < keys.size(); i++) {
    mdns_txt_item_t item;
    item.key = keys[i].c_str();
    item.value = values[i].c_str();
    txt_items.push_back(item);
  }

  esp_err_t err =
      mdns_service_txt_set("_hap", "_tcp", txt_items.data(), txt_items.size());
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "mDNS TXT update failed: %s", esp_err_to_name(err));
    return;
  }

  ESP_LOGI(TAG, "mDNS TXT records updated");
}

void Esp32Network::tcp_listen(uint16_t port, ReceiveCallback callback,
                              DisconnectCallback disconnect) {
  receive_callback_ = callback;
  disconnect_callback_ = disconnect;
  port_ = port;
  running_ = true;

  xTaskCreate(tcp_server_task, "tcp_server", 8192, this, 5, nullptr);
}

void Esp32Network::tcp_server_task(void *arg) {
  Esp32Network *self = static_cast<Esp32Network *>(arg);

  self->server_fd_ = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
  if (self->server_fd_ < 0) {
    ESP_LOGE(TAG, "Failed to create socket: errno %d", errno);
    vTaskDelete(nullptr);
    return;
  }

  int opt = 1;
  setsockopt(self->server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  sockaddr_in addr = {};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(self->port_);

  if (bind(self->server_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    ESP_LOGE(TAG, "Socket bind failed: errno %d", errno);
    close(self->server_fd_);
    self->server_fd_ = -1;
    vTaskDelete(nullptr);
    return;
  }

  if (listen(self->server_fd_, 4) < 0) {
    ESP_LOGE(TAG, "Socket listen failed: errno %d", errno);
    close(self->server_fd_);
    self->server_fd_ = -1;
    vTaskDelete(nullptr);
    return;
  }

  ESP_LOGI(TAG, "TCP server listening on port %d", self->port_);

  while (self->running_) {
    sockaddr_in client_addr;
    socklen_t addr_len = sizeof(client_addr);
    int client_fd =
        accept(self->server_fd_, (struct sockaddr *)&client_addr, &addr_len);

    if (client_fd < 0) {
      if (self->running_) {
        ESP_LOGE(TAG, "Accept failed: errno %d", errno);
      }
      break;
    }

    ESP_LOGI(TAG, "New client connected from %s",
             inet_ntoa(client_addr.sin_addr));

    uint32_t conn_id;
    {
      std::lock_guard<std::mutex> lock(self->connections_mutex_);
      conn_id = self->next_connection_id_++;
      self->connections_[conn_id] = client_fd;
    }

    // Create a task to handle this client
    struct ClientTaskParams {
      Esp32Network *self;
      int fd;
      uint32_t id;
    };

    auto *params = new ClientTaskParams{self, client_fd, conn_id};
    xTaskCreate(
        [](void *arg) {
          auto *p = static_cast<ClientTaskParams *>(arg);
          p->self->handle_client(p->fd, p->id);
          delete p;
          vTaskDelete(nullptr);
        },
        "tcp_client", 8192, params, 4, nullptr);
  }

  close(self->server_fd_);
  self->server_fd_ = -1;
  vTaskDelete(nullptr);
}

void Esp32Network::handle_client(int client_fd, uint32_t connection_id) {
  uint8_t buffer[1024];

  while (running_) {
    ssize_t len = recv(client_fd, buffer, sizeof(buffer), 0);
    if (len > 0) {
      if (receive_callback_) {
        receive_callback_(connection_id, std::span<const uint8_t>(buffer, len));
      }
    } else if (len == 0) {
      // Client disconnected
      ESP_LOGI(TAG, "Client %u disconnected", connection_id);
      break;
    } else {
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
        ESP_LOGE(TAG, "Recv error on connection %u: errno %d", connection_id,
                 errno);
        break;
      }
    }
  }

  {
    std::lock_guard<std::mutex> lock(connections_mutex_);
    connections_.erase(connection_id);
  }
  close(client_fd);

  if (disconnect_callback_) {
    disconnect_callback_(connection_id);
  }
}

void Esp32Network::tcp_send(ConnectionId id, std::span<const uint8_t> data) {
  int fd = -1;
  {
    std::lock_guard<std::mutex> lock(connections_mutex_);
    auto it = connections_.find(id);
    if (it != connections_.end()) {
      fd = it->second;
    }
  }

  if (fd >= 0) {
    send(fd, data.data(), data.size(), 0);
  }
}

void Esp32Network::tcp_disconnect(ConnectionId id) {
  int fd = -1;
  {
    std::lock_guard<std::mutex> lock(connections_mutex_);
    auto it = connections_.find(id);
    if (it != connections_.end()) {
      fd = it->second;
      connections_.erase(it);
    }
  }

  if (fd >= 0) {
    close(fd);
  }
}
#endif
