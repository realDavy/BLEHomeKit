#pragma once

#include "hap/transport/ble/HapPdu.hpp"
#include "hap/transport/ConnectionContext.hpp"
#include "hap/platform/System.hpp"
#include <cstdint>
#include <map>
#include <memory>
#include <vector>

namespace hap::transport::ble {

/**
 * @brief Transaction state for PDU processing.
 */
struct TransactionState {
    std::vector<uint8_t> buffer;
    PDUOpcode opcode = PDUOpcode::CharacteristicRead;
    uint16_t transaction_id = 0;
    bool active = false;
    std::string target_uuid;                // UUID of the characteristic being written
    uint8_t ttl = 0;                        // Timed Write TTL
    std::vector<uint8_t> response_buffer;   // Buffer for GATT Read response
    size_t response_read_offset = 0;        // Bytes of response_buffer already returned
    uint64_t last_activity_ms = 0;          // Timestamp of last HAP transaction
    uint64_t procedure_start_ms = 0;        // Timestamp when procedure started
    uint64_t connection_established_ms = 0; // Timestamp when connection established
    uint64_t last_write_ms = 0;             // Timestamp of last write
    uint16_t expected_body_length = 0;      // Expected body length from PDU header
    bool gsn_incremented = false;           // Per spec: GSN increments only once per connection
    std::vector<uint8_t> timed_write_body;  // Body data pending for ExecuteWrite
    uint16_t timed_write_iid = 0;           // IID for pending timed write
};

/**
 * @brief Session data for a BLE connection.
 */
struct BleSession {
    uint16_t connection_id = 0;
    std::unique_ptr<ConnectionContext> context;
    TransactionState transaction;
    
    BleSession() = default;
    BleSession(uint16_t conn_id) : connection_id(conn_id) {}
    
    BleSession(const BleSession&) = delete;
    BleSession& operator=(const BleSession&) = delete;
    
    BleSession(BleSession&&) = default;
    BleSession& operator=(BleSession&&) = default;
};

/**
 * @brief Manages BLE connection sessions and their state.
 * 
 * Handles:
 * - Session creation/destruction
 * - Transaction state per connection
 * - Timeout checking (idle, procedure, initial)
 * - Subscription tracking
 */
class BleSessionManager {
public:
    explicit BleSessionManager(platform::System* system);
    ~BleSessionManager() = default;
    
    BleSessionManager(const BleSessionManager&) = delete;
    BleSessionManager& operator=(const BleSessionManager&) = delete;
    
    /**
     * @brief Get or create a session for a connection.
     */
    BleSession& get_or_create(uint16_t connection_id);
    
    /**
     * @brief Check if a session exists for a connection.
     */
    [[nodiscard]] bool has_session(uint16_t connection_id) const;
    
    /**
     * @brief Get an existing session (returns nullptr if not found).
     */
    [[nodiscard]] BleSession* get_session(uint16_t connection_id);
    [[nodiscard]] const BleSession* get_session(uint16_t connection_id) const;
    
    /**
     * @brief Remove a session.
     */
    void remove(uint16_t connection_id);
    
    /**
     * @brief Remove all sessions.
     */
    void clear();
    
    /**
     * @brief Check for session timeouts and return connections to terminate.
     * 
     * Per HAP Spec:
     * - 30-second idle timeout (7.2.5)
     * - 10-second HAP procedure timeout (7.3.1)
     * - 10-second initial procedure timeout (7.5 Req #40)
     */
    [[nodiscard]] std::vector<uint16_t> check_timeouts();
    
    /**
     * @brief Get all active connection IDs.
     */
    [[nodiscard]] std::vector<uint16_t> get_connection_ids() const;
    
    /**
     * @brief Get the number of active sessions.
     */
    [[nodiscard]] size_t session_count() const { return sessions_.size(); }
    
    // Subscription management
    
    /**
     * @brief Add a subscription for a connection.
     */
    void add_subscription(const std::string& uuid, uint16_t connection_id);
    
    /**
     * @brief Remove a subscription for a connection.
     */
    void remove_subscription(const std::string& uuid, uint16_t connection_id);
    
    /**
     * @brief Get all subscribers for a UUID.
     */
    [[nodiscard]] const std::vector<uint16_t>& get_subscribers(const std::string& uuid) const;
    
    /**
     * @brief Check if a UUID has any subscribers.
     */
    [[nodiscard]] bool has_subscribers(const std::string& uuid) const;

private:
    platform::System* system_;
    std::map<uint16_t, BleSession> sessions_;
    std::map<std::string, std::vector<uint16_t>> subscriptions_;
    
    static constexpr uint64_t kIdleTimeoutMs = 30000;      // 30 seconds
    static constexpr uint64_t kProcedureTimeoutMs = 10000; // 10 seconds
    static constexpr uint64_t kInitialTimeoutMs = 10000;   // 10 seconds
    
    static const std::vector<uint16_t> kEmptySubscribers;
};

} // namespace hap::transport::ble
