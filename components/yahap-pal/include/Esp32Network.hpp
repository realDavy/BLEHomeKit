#pragma once
#include <sdkconfig.h>
#if CONFIG_PAL_NETWORK_ENABLED
#include "hap/platform/Network.hpp"
#include <atomic>
#include <map>
#include <mdns.h>
#include <mutex>

class Esp32Network : public hap::platform::Network {
public:
  Esp32Network();
  ~Esp32Network() override;

  void mdns_register(const MdnsService &service) override;
  void mdns_update_txt_record(const MdnsService &service) override;
  void tcp_listen(uint16_t port, ReceiveCallback callback,
                  DisconnectCallback disconnect) override;
  void tcp_send(ConnectionId id, std::span<const uint8_t> data) override;
  void tcp_disconnect(ConnectionId id) override;

  /**
   * @brief Initialize WiFi in station mode with static credentials.
   *
   * This should be called before starting the AccessoryServer.
   * Blocks until connected or connection fails.
   *
   * @param ssid WiFi network SSID
   * @param password WiFi network password
   * @return true if connection successful
   */
  static bool wifi_init(const char *ssid, const char *password);

private:
  static void tcp_server_task(void *arg);
  void handle_client(int client_fd, uint32_t connection_id);

  ReceiveCallback receive_callback_;
  DisconnectCallback disconnect_callback_;

  std::atomic<bool> running_{false};
  int server_fd_ = -1;
  uint16_t port_ = 0;

  std::map<uint32_t, int> connections_;
  std::mutex connections_mutex_;
  uint32_t next_connection_id_ = 1;

  MdnsService current_service_;
  bool mdns_initialized_ = false;
};
#endif
