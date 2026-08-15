#pragma once

#include <span>
#include <cstdint>
#include <functional>
#include <vector>
#include <string>
#include <optional>
#include <array>

namespace hap::platform {

/**
 * @brief Abstract interface for BLE operations.
 * This interface decouples the HAP library from the specific BLE stack (e.g., BlueZ, ESP32 Bluedroid/NimBLE).
 */
struct Ble {
    virtual ~Ble() = default;

    /**
     * @brief Configuration for BLE advertising intervals.
     * 
     * Per HAP Spec 7.4.6.3: Fast advertising (20ms) is used for 3 seconds
     * after state changes, then reverts to normal interval.
     */
    struct BleIntervalConfig {
        uint32_t normal_interval_ms = 500;    ///< Normal advertising interval
        uint32_t fast_interval_ms = 20;       ///< Fast advertising interval (HAP Spec 7.4.6.3)
        uint32_t fast_duration_ms = 3000;     ///< Duration for fast advertising
    } interval_config;

    struct Advertisement {
        static Advertisement create_hap(
            uint8_t status_flags, 
            uint8_t device_id[6], 
            uint16_t category_id, 
            uint16_t global_state_number,
            uint8_t config_number,
            uint8_t pair_setup_hash[4]
        );

        uint16_t company_id = 0x004C;
        std::vector<uint8_t> manufacturer_data;
        uint8_t flags = 0x06;
        std::optional<std::string> local_name;
    };

    virtual void start() = 0;

    /** Number of live BLE links (before HAP Pair-Verify). */
    virtual uint16_t active_connections() const { return 0; }

    /**
     * @brief Start BLE advertising with the given data.
     * @param data The advertisement payload and configuration.
     * @param interval_ms Advertising interval in milliseconds (default 20ms recommended for HAP).
     */
    virtual void start_advertising(const Advertisement& data, uint32_t interval_ms = 20) = 0;

    /**
     * @brief Stop BLE advertising.
     */
    virtual void stop_advertising() = 0;

    struct DescriptorDefinition {
        std::string uuid;
        struct Properties {
            bool read = false;
            bool write = false;
        } properties;
        
        using ReadCallback = std::function<std::vector<uint8_t>(uint16_t connection_id)>;
        ReadCallback on_read;

        using WriteCallback = std::function<void(uint16_t connection_id, std::span<const uint8_t> data)>;
        WriteCallback on_write;
    };

    struct CharacteristicDefinition {
        std::string uuid; // 128-bit UUID string
        
        struct Properties {
            bool read = false;
            bool write = false;
            bool write_without_response = false;
            bool notify = false;
            bool indicate = false;
        } properties;

        using WriteCallback = std::function<void(uint16_t connection_id, std::span<const uint8_t> data, bool response_needed)>;
        WriteCallback on_write;

        using ReadCallback = std::function<std::vector<uint8_t>(uint16_t connection_id)>;
        ReadCallback on_read;

        using SubscribeCallback = std::function<void(uint16_t connection_id, bool enabled)>;
        SubscribeCallback on_subscribe;

        std::vector<DescriptorDefinition> descriptors;
    };

    struct ServiceDefinition {
        std::string uuid;
        bool is_primary = true;
        std::vector<CharacteristicDefinition> characteristics;
        std::vector<ServiceDefinition> included_services;
    };

    /**
     * @brief Register a GATT service.
     * This should be called before starting advertising.
     * @param service The service definition.
     */
    virtual void register_service(const ServiceDefinition& service) = 0;

    /**
     * @brief Disconnect a specific client.
     */
    virtual void disconnect(uint16_t connection_id) = 0;

    /**
     * @brief Callback type for disconnect events.
     * @param connection_id The ID of the disconnected connection.
     */
    using DisconnectCallback = std::function<void(uint16_t connection_id)>;

    /**
     * @brief Set a callback to be invoked when a device disconnects.
     * @param callback The callback function.
     */
    virtual void set_disconnect_callback(DisconnectCallback callback) = 0;

    /**
     * @brief Send a GATT indication to a connected client.
     * 
     * Per HAP Spec 7.4.6.1 Connected Events: When a characteristic value changes,
     * a zero-length indication is sent. The controller then reads the value.
     * 
     * @param connection_id The ID of the connection to send to.
     * @param characteristic_uuid The UUID of the characteristic.
     * @param data The data to send (typically empty for HAP Connected Events).
     * @return true if indication was queued successfully.
     */
    virtual bool send_indication(uint16_t connection_id, const std::string& characteristic_uuid, 
                                  std::span<const uint8_t> data) = 0;

    /**
     * @brief Encrypted advertisement data for HAP BLE Broadcasted Events.
     * 
     * Per HAP Spec 7.4.6.2 and 7.4.7.3: Broadcasted Events use encrypted
     * advertisement payloads secured with ChaCha20-Poly1305.
     */
    struct EncryptedAdvertisement {
        std::array<uint8_t, 6> advertising_id;     ///< Device advertising identifier
        std::vector<uint8_t> encrypted_payload;     ///< 12-byte encrypted data + 4-byte truncated auth tag
        uint16_t gsn;                               ///< Current Global State Number
    };

    /**
     * @brief Start encrypted advertisement for HAP Broadcasted Events.
     * 
     * Per HAP Spec 7.4.6.2: When a characteristic configured for broadcast
     * notification changes while disconnected, the accessory broadcasts an
     * encrypted advertisement containing the value.
     * 
     * @param data The encrypted advertisement payload.
     * @param interval_ms Advertising interval: 20ms (0x01), 1280ms (0x02), or 2560ms (0x03).
     * @param duration_ms Duration to advertise (typically 3000ms per spec).
     */
    virtual void start_encrypted_advertising(const EncryptedAdvertisement& data,
                                              uint32_t interval_ms = 20,
                                              uint32_t duration_ms = 3000) {
        // Default: no-op. Platform implementations may override.
        (void)data; (void)interval_ms; (void)duration_ms;
    }

    /**
     * @brief Check if any client is currently connected.
     * @return true if at least one client is connected.
     */
    virtual bool has_connections() const { return false; }

    /**
     * @brief Negotiated ATT MTU for a connection (default 23).
     * Used to fragment HAP-BLE PDU responses so they fit a GATT Read.
     */
    virtual uint16_t att_mtu(uint16_t connection_id) const {
        (void)connection_id;
        return 23;
    }

    /**
     * @brief Start timed advertising with automatic fallback to normal interval.
     * 
     * Per HAP Spec 7.4.6.3 Disconnected Events: After updating GSN, the accessory
     * must use 20ms advertising for at least 3 seconds, then revert to normal.
     * 
     * @param data The advertisement payload.
     * @param fast_interval_ms Fast interval to use initially (typically 20ms).
     * @param fast_duration_ms Duration for fast advertising (typically 3000ms).
     * @param normal_interval_ms Normal interval to revert to after fast period.
     */
    virtual void start_timed_advertising(const Advertisement& data,
                                          uint32_t fast_interval_ms = 20,
                                          uint32_t fast_duration_ms = 3000,
                                          uint32_t normal_interval_ms = 1000) {
        // Default implementation: just use fast interval (platforms override for timer support)
        start_advertising(data, fast_interval_ms);
        (void)fast_duration_ms;
        (void)normal_interval_ms;
    }
};

} // namespace hap::platform
