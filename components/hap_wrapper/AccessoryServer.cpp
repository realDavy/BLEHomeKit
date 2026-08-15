// Local overlay of hap/src/AccessoryServer.cpp:
// Home "Remove Accessory" must not regenerate Device ID / LTSK. The BLE
// static random address is set at boot from the stored ID; changing the ID
// without changing the radio address makes the accessory un-addable.
#include "hap/AccessoryServer.hpp"
#include "hap/common/TaskScheduler.hpp"
#include "hap/transport/Router.hpp"
#include "hap/transport/BleTransport.hpp"
#include "hap/transport/ConnectionContext.hpp"
#include "hap/transport/PairingEndpoints.hpp"
#include "hap/transport/AccessoryEndpoints.hpp"
#include "hap/core/HAPStatus.hpp"
#include <map>
#include <nlohmann/json.hpp>
#include <iomanip>
#include <sstream>

namespace hap {

class AccessoryServer::Impl {
public:
    std::unique_ptr<transport::Router> router;
    std::unique_ptr<transport::PairingEndpoints> pairing_endpoints;
    std::unique_ptr<transport::BleTransport> ble_transport;
    std::unique_ptr<transport::AccessoryEndpoints> accessory_endpoints;
    std::map<uint32_t, std::unique_ptr<transport::ConnectionContext>> connections;
    std::map<uint32_t, transport::HTTPParser> parsers;
};

AccessoryServer::AccessoryServer(Config config) : config_(std::move(config)), impl_(std::make_unique<Impl>()) {
    // Auto-generate accessory ID if not provided
    if (config_.accessory_id.empty()) {
        auto stored_id = config_.storage->get("accessory_id");
        if (stored_id && !stored_id->empty()) {
            config_.accessory_id = std::string(stored_id->begin(), stored_id->end());
            config_.system->log(platform::System::LogLevel::Info, 
                "[AccessoryServer] Loaded stored accessory ID: " + config_.accessory_id);
        } else {
            // Generate random 6 bytes and format as MAC address
            uint8_t random_id[6];
            config_.system->random_bytes(std::span<uint8_t>(random_id, 6));
            
            std::ostringstream oss;
            oss << std::hex << std::uppercase << std::setfill('0');
            for (int i = 0; i < 6; ++i) {
                if (i > 0) oss << ":";
                oss << std::setw(2) << static_cast<int>(random_id[i]);
            }
            config_.accessory_id = oss.str();
            
            // Store for future boots
            std::vector<uint8_t> id_bytes(config_.accessory_id.begin(), config_.accessory_id.end());
            config_.storage->set("accessory_id", id_bytes);
            
            config_.system->log(platform::System::LogLevel::Info, 
                "[AccessoryServer] Generated new accessory ID: " + config_.accessory_id);
        }
    }
    
    // Initialize endpoints
    transport::PairingEndpoints::Config pairing_config;
    pairing_config.crypto = config_.crypto;
    pairing_config.storage = config_.storage;
    pairing_config.system = config_.system;
    pairing_config.accessory_id = config_.accessory_id;
    pairing_config.setup_code = config_.setup_code;
    pairing_config.on_pairings_changed = [this](const std::string& pairing_id, const std::array<uint8_t, 32>& ltpk, bool is_add) {
        config_.system->log(platform::System::LogLevel::Info, 
            "[AccessoryServer] Pairing " + std::string(is_add ? "added" : "removed") + ": " + pairing_id);
        
        // Last controller removed (Home → Remove Accessory): clear pairings
        // but keep Device ID / LTSK so BLE address still matches advertisements.
        auto pairing_list_data = config_.storage->get("pairing_list");
        bool is_paired = false;
        if (pairing_list_data && pairing_list_data->size() > 2) {
            const std::string list(pairing_list_data->begin(), pairing_list_data->end());
            is_paired = list != "[]";
        }
        
        if (!is_add && !is_paired) {
            reset_pairing_state();
        }
        
        update_mdns();
        // Pair-Setup writes pairing_list before Pair-Verify. A missing
        // pair_verified flag used to make this update advertise SF=0 and
        // Home then failed Add Accessory on the post-M6 rescan.
        if (is_add) {
            auto verified = config_.storage->get("pair_verified");
            if (!verified || verified->empty() || (*verified)[0] != '1') {
                config_.storage->set("pair_verified", std::vector<uint8_t>{'0'});
            }
        }
        if (impl_->ble_transport) {
            impl_->ble_transport->update_advertising();
        }
        
        // Invoke user callback if set
        if (config_.on_pairings_changed) {
            PairingEvent event;
            event.type = is_add ? PairingEventType::Added : PairingEventType::Removed;
            event.pairing_id = pairing_id;
            event.ltpk = ltpk;
            config_.on_pairings_changed(event);
        }
    };
    impl_->pairing_endpoints = std::make_unique<transport::PairingEndpoints>(pairing_config);
    
    // Initialize IIDManager for persistent IID allocation
    iid_manager_ = std::make_unique<core::IIDManager>(config_.storage, config_.system);
    database_.set_iid_manager(iid_manager_.get());
    
    // Initialize task scheduler first (needed by BleTransport)
    scheduler_ = std::make_unique<common::TaskScheduler>(config_.system);
    
    if (config_.ble) {
        transport::BleTransport::Config ble_config;
        ble_config.ble = config_.ble;
        ble_config.crypto = config_.crypto;
        ble_config.database = &database_;
        ble_config.pairing_endpoints = impl_->pairing_endpoints.get();
        ble_config.system = config_.system;
        ble_config.storage = config_.storage;
        ble_config.scheduler = scheduler_.get();
        ble_config.accessory_id = config_.accessory_id;
        ble_config.device_name = config_.device_name;
        ble_config.category_id = static_cast<uint16_t>(config_.category_id);
        ble_config.iid_manager = iid_manager_.get();
        impl_->ble_transport = std::make_unique<transport::BleTransport>(ble_config);
        
        // Schedule periodic BLE session timeout checks (every 1 second)
        scheduler_->schedule_periodic(1000, [this]() {
            impl_->ble_transport->check_session_timeouts();
        });
    }
    
    impl_->accessory_endpoints = std::make_unique<transport::AccessoryEndpoints>(&database_);
    
    // Initialize router
    impl_->router = std::make_unique<transport::Router>();
    setup_routes();
    
    // Set up deferred callback execution for characteristic value changes
    // This avoids stack overflow on platforms with limited callback stack (e.g., ESP32 GATT)
    core::Characteristic::set_dispatcher([this](std::function<void()> work) {
        scheduler_->schedule_once(0, std::move(work));
    });
}

AccessoryServer::~AccessoryServer() = default;

static std::string method_to_string(transport::Method method) {
    switch (method) {
        case transport::Method::GET: return "GET";
        case transport::Method::POST: return "POST";
        case transport::Method::PUT: return "PUT";
        case transport::Method::DELETE: return "DELETE";
        case transport::Method::OPTIONS: return "OPTIONS";
        default: return "UNKNOWN";
    }
}

void AccessoryServer::add_accessory(std::shared_ptr<core::Accessory> accessory) {
    database_.add_accessory(accessory);
    
    // Register event callbacks
    uint64_t aid = accessory->aid();
    for (const auto& service : accessory->services()) {
        for (const auto& characteristic : service->characteristics()) {
            if (core::has_permission(characteristic->permissions(), core::Permission::Notify)) {
                auto ch_ptr = characteristic.get();
                characteristic->set_event_callback([this, aid, ch_ptr](const core::Value& value, const core::EventSource& source) {
                    uint64_t iid = ch_ptr->iid();
                    uint32_t exclude_id = UINT32_MAX;
                    if (source.type == core::EventSource::Type::Connection) {
                        exclude_id = source.id;
                    }
                    
                    if(source.type != core::EventSource::Type::Connection) {
                      broadcast_event(aid, iid, value, exclude_id);
                    }
                });
            }
        }
    }
}

void AccessoryServer::setup_routes() {
    using namespace transport;
    
    // Pairing endpoints (no pairing required)
    impl_->router->add_route(Method::POST, "/pair-setup", 
        [this](const Request& req, ConnectionContext& ctx) {            
            return impl_->pairing_endpoints->handle_pair_setup(req, ctx);
        }, false);
    
    impl_->router->add_route(Method::POST, "/pair-verify",
        [this](const Request& req, ConnectionContext& ctx) {
            return impl_->pairing_endpoints->handle_pair_verify(req, ctx);
        }, false);
    
    impl_->router->add_route(Method::POST, "/pairings",
        [this](const Request& req, ConnectionContext& ctx) {
            return impl_->pairing_endpoints->handle_pairings(req, ctx);
        }, true);
        
    impl_->router->add_route(Method::POST, "/prepare",
        [this](const Request& req, ConnectionContext& ctx) {
            return impl_->accessory_endpoints->handle_prepare(req, ctx);
        }, true);
        
    impl_->router->add_route(Method::POST, "/identify",
        [this](const Request& req, ConnectionContext& ctx) {
            (void)req;
            (void)ctx;
            
            // HAP Spec 6.7.7: /identify is only valid if accessory is unpaired
            auto pairing_list_data = config_.storage->get("pairing_list");
            bool is_paired = pairing_list_data && pairing_list_data->size() > 2;
            
            if (is_paired) {
                // Return 400 Bad Request with HAP status -70401 (InsufficientPrivileges)
                nlohmann::json error_response;
                error_response["status"] = core::to_int(core::HAPStatus::InsufficientPrivileges);
                transport::Response resp{transport::Status::BadRequest};
                resp.set_header("Content-Type", "application/hap+json");
                resp.set_body(error_response.dump());
                return resp;
            }
            
            if (config_.on_identify) {
                config_.on_identify();
            }
            return transport::Response{transport::Status::NoContent};
        }, false); // Allow unencrypted access
    
    // Accessory endpoints (require verified pairing)
    impl_->router->add_route(Method::GET, "/accessories",
        [this](const Request& req, ConnectionContext& ctx) {
            return impl_->accessory_endpoints->handle_get_accessories(req, ctx);
        }, true);
    
    impl_->router->add_route(Method::GET, "/characteristics",
        [this](const Request& req, ConnectionContext& ctx) {
            return impl_->accessory_endpoints->handle_get_characteristics(req, ctx);
        }, true);
    
    impl_->router->add_route(Method::PUT, "/characteristics",
        [this](const Request& req, ConnectionContext& ctx) {
            return impl_->accessory_endpoints->handle_put_characteristics(req, ctx);
        }, true);
}

void AccessoryServer::start() {
    config_.system->log(platform::System::LogLevel::Info, "HAP Server starting...");
    
    // Check if database structure changed and increment CN if needed
    check_and_update_config_number();
    
    // Start TCP listener
    auto receive_cb = [this](uint32_t conn_id, std::span<const uint8_t> data) {
        on_tcp_receive(conn_id, data);
    };
    
    auto disconnect_cb = [this](uint32_t conn_id) {
        on_tcp_disconnect(conn_id);
    };
    
    if (config_.network) {
        config_.network->tcp_listen(config_.port, receive_cb, disconnect_cb);
        
        // Register mDNS service
        update_mdns();
    }
    
    if (impl_->ble_transport) {
        impl_->ble_transport->start();
    }
}

void AccessoryServer::update_mdns() {
    if (!config_.network) return;
    platform::Network::MdnsService mdns_service;
    mdns_service.name = config_.device_name;
    mdns_service.type = "_hap._tcp";
    mdns_service.port = config_.port;
    
    // Check pairing status
    auto pairing_list_data = config_.storage->get("pairing_list");
    bool is_paired = false;
    if (pairing_list_data) {
        if (pairing_list_data->size() > 2) {
            is_paired = true;
        }
    }
    
    std::string sf_val = is_paired ? "0" : "1";
    
    // Check config number
    auto config_num_data = config_.storage->get("config_number");
    std::string config_num = "1";
    if (config_num_data && !config_num_data->empty()) {
        config_num = std::string(config_num_data->begin(), config_num_data->end());
    } else {
        // Initialize to 1
        std::vector<uint8_t> initial_cn = {'1'};
        config_.storage->set("config_number", initial_cn);
    }
    
    // Add required TXT records
    mdns_service.txt_records.push_back({"c#", config_num}); // Configuration number
    mdns_service.txt_records.push_back({"id", config_.accessory_id}); // Device ID
    mdns_service.txt_records.push_back({"md", config_.device_name}); // Model name
    mdns_service.txt_records.push_back({"pv", "1.1"}); // Protocol version
    mdns_service.txt_records.push_back({"s#", "1"}); // State number
    mdns_service.txt_records.push_back({"sf", sf_val}); // Status flags (1 = discoverable/unpaired, 0 = paired)
    mdns_service.txt_records.push_back({"ci", std::to_string(static_cast<uint16_t>(config_.category_id))}); // Category identifier
    mdns_service.txt_records.push_back({"ff", "0"}); // Feature flags
    
    // Use update instead of re-register if already registered
    if (mdns_registered_) {
        config_.network->mdns_update_txt_record(mdns_service);
    } else {
        config_.network->mdns_register(mdns_service);
        mdns_registered_ = true;
    }
}

void AccessoryServer::stop() {
    config_.system->log(platform::System::LogLevel::Info, "HAP Server stopping...");
    
    if (impl_->ble_transport) {
        impl_->ble_transport->stop();
    }
    
    impl_->connections.clear();
    impl_->parsers.clear();
}

void AccessoryServer::reset_pairing_state() {
    // Keep accessory_id, LTSK/LTPK, and setup_id. HAP-BLE advertisements
    // carry the Device ID, which must stay equal to the static random
    // address set at boot. Regenerating the ID on Home unpair made the
    // accessory advertise a new ID on the old address (无法添加 / 未响应).
    auto list_data = config_.storage->get("pairing_list");
    if (list_data && !list_data->empty()) {
        auto list_json = nlohmann::json::parse(list_data->begin(), list_data->end(), nullptr, false);
        if (list_json.is_array()) {
            for (const auto& id_json : list_json) {
                if (id_json.is_string()) {
                    config_.storage->remove("pairing_" + id_json.get<std::string>());
                }
            }
        }
    }
    config_.storage->remove("pairing_list");
    config_.storage->remove("gsn");
    config_.storage->remove("pair_verified");

    config_.system->log(platform::System::LogLevel::Info,
        "[AccessoryServer] Controller pairings cleared; Device ID kept (" +
        config_.accessory_id + ")");

    if (impl_->pairing_endpoints) {
        impl_->pairing_endpoints->reset();
    }

    pending_connection_cleanup_ = true;
}

void AccessoryServer::factory_reset() {
    config_.system->log(platform::System::LogLevel::Warning, 
        "[AccessoryServer] Factory reset initiated");
    
    reset_pairing_state();
    
    // Update advertising/mDNS
    update_mdns();
    if (impl_->ble_transport) {
        impl_->ble_transport->update_advertising();
    }
    
    // Invoke user callback if set (device is now unpaired)
    if (config_.on_pairings_changed) {
        PairingEvent event;
        event.type = PairingEventType::AllRemoved;
        event.pairing_id = "";  // All pairings removed
        event.ltpk = {};        // No specific LTPK
        config_.on_pairings_changed(event);
    }
    
    config_.system->log(platform::System::LogLevel::Info, 
        "[AccessoryServer] Factory reset complete - accessory is now unpaired");
}

void AccessoryServer::tick() {
    if (scheduler_) {
        scheduler_->tick();
    }
}

void AccessoryServer::on_tcp_receive(uint32_t connection_id, std::span<const uint8_t> data) {
    config_.system->log(platform::System::LogLevel::Debug, 
        "[AccessoryServer] Received " + std::to_string(data.size()) + " bytes from connection " + std::to_string(connection_id));
    
    // Get or create connection context
    auto& ctx = impl_->connections[connection_id];
    if (!ctx) {
        config_.system->log(platform::System::LogLevel::Info, 
            "[AccessoryServer] New connection #" + std::to_string(connection_id));
        ctx = std::make_unique<transport::ConnectionContext>(config_.crypto, config_.system, connection_id);
    }
    
    // Get or create HTTP parser
    auto& parser = impl_->parsers[connection_id];
    
    // Decrypt if connection is encrypted
    std::vector<uint8_t> plaintext_data;
    if (ctx->is_encrypted() && ctx->get_secure_session()) {
        if(!ctx->rx_encrypted()) {
            ctx->set_rx_encrypted(true);
        }
        config_.system->log(platform::System::LogLevel::Debug, 
            "[AccessoryServer] Decrypting frame for connection #" + std::to_string(connection_id));
        auto decrypted = ctx->get_secure_session()->decrypt_frame(data);
        if (!decrypted) {
            config_.system->log(platform::System::LogLevel::Warning, 
                "[AccessoryServer] Decryption failed or incomplete frame for connection #" + std::to_string(connection_id));
            return;
        }
        plaintext_data = *decrypted;
        config_.system->log(platform::System::LogLevel::Debug, 
            "[AccessoryServer] Decrypted " + std::to_string(plaintext_data.size()) + " bytes");
    } else {
        plaintext_data.assign(data.begin(), data.end());
    }
    
    // Feed to HTTP parser
    if (parser.feed(plaintext_data)) {
        auto request = parser.take_request();
        parser.reset();
        
        config_.system->log(platform::System::LogLevel::Debug,
            "[AccessoryServer] HTTP Request: " + method_to_string(request.method) + " " + request.path);
        
        // Log headers
        for (const auto& [key, value] : request.headers) {
            config_.system->log(platform::System::LogLevel::Debug, 
                "[AccessoryServer] Header: " + key + ": " + value);
        }
        
        // Log body
        if (!request.body.empty()) {
            std::string content_type = request.get_header("Content-Type");
            if (content_type == "application/pairing+tlv8") {
                std::ostringstream oss;
                for (uint8_t b : request.body) {
                    oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(b);
                }
                config_.system->log(platform::System::LogLevel::Debug, 
                    "[AccessoryServer] Body (TLV8): " + oss.str());
            } else {
                std::string body_str(request.body.begin(), request.body.end());
                config_.system->log(platform::System::LogLevel::Debug, 
                    "[AccessoryServer] Body: " + body_str);
            }
        }
        
        // Dispatch to router
        auto response = impl_->router->dispatch(request, *ctx);
        
        transport::Response final_response;
        if (response) {
            final_response = *response;
            config_.system->log(platform::System::LogLevel::Debug, 
                "[AccessoryServer] HTTP Response: " + std::to_string(static_cast<int>(final_response.status)));
            
            // Log headers
            for (const auto& [key, value] : final_response.headers) {
                config_.system->log(platform::System::LogLevel::Debug, 
                    "[AccessoryServer] Response Header: " + key + ": " + value);
            }
            
            // Log body
            if (!final_response.body.empty()) {
                auto it = final_response.headers.find("Content-Type");
                std::string content_type = (it != final_response.headers.end()) ? it->second : "";
                
                if (content_type == "application/pairing+tlv8") {
                    std::ostringstream oss;
                    for (uint8_t b : final_response.body) {
                        oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(b);
                    }
                    config_.system->log(platform::System::LogLevel::Debug, 
                        "[AccessoryServer] Response Body (TLV8): " + oss.str());
                } else {
                    std::string body_str(final_response.body.begin(), final_response.body.end());
                    config_.system->log(platform::System::LogLevel::Debug, 
                        "[AccessoryServer] Response Body: " + body_str);
                }
            }
        } else {
            config_.system->log(platform::System::LogLevel::Warning, 
                "[AccessoryServer] No route found for: " + request.path);
            // HAP Spec 6.7.1.4: 4xx responses must include HAP status code
            nlohmann::json error_response;
            error_response["status"] = core::to_int(core::HAPStatus::ResourceDoesNotExist);
            final_response = transport::Response{transport::Status::NotFound};
            final_response.set_header("Content-Type", "application/hap+json");
            final_response.set_body(error_response.dump());
        }
        
        // Build HTTP response
        auto response_bytes = transport::HTTPBuilder::build(final_response);
        
        // Encrypt if connection is encrypted
        if (ctx->is_encrypted() && ctx->rx_encrypted()) {
            config_.system->log(platform::System::LogLevel::Debug, 
                "[AccessoryServer] Encrypting response (" + std::to_string(response_bytes.size()) + " bytes)");
            
            size_t offset = 0;
            while (offset < response_bytes.size()) {
                size_t chunk_size = std::min(response_bytes.size() - offset, (size_t)1024);
                std::span<const uint8_t> chunk(response_bytes.data() + offset, chunk_size);
                auto encrypted = ctx->get_secure_session()->encrypt_frame(chunk);
                config_.network->tcp_send(connection_id, encrypted);
                offset += chunk_size;
            }
        } else {
            config_.system->log(platform::System::LogLevel::Debug, 
                "[AccessoryServer] Sending plaintext response (" + std::to_string(response_bytes.size()) + " bytes)");
            config_.network->tcp_send(connection_id, response_bytes);
        }
        
        if (ctx->should_close()) {
            config_.system->log(platform::System::LogLevel::Info, 
                "[AccessoryServer] Closing connection #" + std::to_string(connection_id) + " as requested");
            config_.network->tcp_disconnect(connection_id);
        }
    }
    
    if (pending_connection_cleanup_) {
        pending_connection_cleanup_ = false;
        impl_->connections.clear();
        impl_->parsers.clear();
        config_.system->log(platform::System::LogLevel::Debug,
            "[AccessoryServer] Deferred connection cleanup completed");
    }
}

void AccessoryServer::on_tcp_disconnect(uint32_t connection_id) {
    config_.system->log(platform::System::LogLevel::Info, 
        "[AccessoryServer] Connection #" + std::to_string(connection_id) + " disconnected");
    impl_->connections.erase(connection_id);
    impl_->parsers.erase(connection_id);
}

void AccessoryServer::broadcast_event(uint64_t aid, uint64_t iid, const core::Value& value, uint32_t exclude_conn_id) {
    if (impl_->ble_transport) {
        impl_->ble_transport->notify_value_changed(aid, iid, value, exclude_conn_id);
    }
    if(!config_.network) {
        return;
    }
    nlohmann::json body_json;
    nlohmann::json characteristics = nlohmann::json::array();
    nlohmann::json char_json;
    char_json["aid"] = aid;
    char_json["iid"] = iid;

    std::visit([&char_json](auto&& arg) {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (std::is_same_v<T, bool>) {
            char_json["value"] = arg;
        } else if constexpr (std::is_same_v<T, uint8_t> || std::is_same_v<T, uint16_t> || 
                             std::is_same_v<T, uint32_t> || std::is_same_v<T, uint64_t> || 
                             std::is_same_v<T, int32_t> || std::is_same_v<T, float>) {
            char_json["value"] = arg;
        } else if constexpr (std::is_same_v<T, std::string>) {
            char_json["value"] = arg;
        } else {
            char_json["value"] = nullptr;
        }
    }, value);

    characteristics.push_back(char_json);
    body_json["characteristics"] = characteristics;
    std::string body = body_json.dump();

    std::ostringstream ss;
    ss << "EVENT/1.0 200 OK\r\n";
    ss << "Content-Type: application/hap+json\r\n";
    ss << "Content-Length: " << body.length() << "\r\n";
    ss << "\r\n";
    ss << body;
    
    std::string response_str = ss.str();
    std::vector<uint8_t> response_bytes(response_str.begin(), response_str.end());

    for (auto& [conn_id, ctx] : impl_->connections) {
        if (conn_id == exclude_conn_id) continue;
        
        if (ctx->is_encrypted() && ctx->has_subscription(aid, iid)) {
            config_.system->log(platform::System::LogLevel::Debug, 
                "[AccessoryServer] Sending event to connection #" + std::to_string(conn_id));
            
            size_t offset = 0;
            while (offset < response_bytes.size()) {
                size_t chunk_size = std::min(response_bytes.size() - offset, (size_t)1024);
                std::span<const uint8_t> chunk(response_bytes.data() + offset, chunk_size);
                auto encrypted = ctx->get_secure_session()->encrypt_frame(chunk);
                config_.network->tcp_send(conn_id, encrypted);
                offset += chunk_size;
            }
        }
    }
}

void AccessoryServer::check_and_update_config_number() {
    // Build database structure hash
    std::ostringstream hash_input;
    
    for (const auto& acc : database_.accessories()) {
        hash_input << "A" << acc->aid() << ":";
        for (const auto& svc : acc->services()) {
            hash_input << "S" << std::hex << svc->type() << ":";
            for (const auto& ch : svc->characteristics()) {
                hash_input << "C" << std::hex << ch->type() << ",";
            }
        }
    }
    
    std::string current_hash_input = hash_input.str();
    
    // Simple DJB2 hash
    uint32_t hash = 5381;
    for (char c : current_hash_input) {
        hash = ((hash << 5) + hash) + static_cast<uint8_t>(c);
    }
    
    std::string current_hash = std::to_string(hash);
    
    // Check for structure change using IIDManager if available, else direct storage
    bool structure_changed = false;
    if (iid_manager_) {
        structure_changed = iid_manager_->has_structure_changed(current_hash);
    } else {
        auto stored_hash_data = config_.storage->get("db_hash");
        std::string stored_hash;
        if (stored_hash_data && !stored_hash_data->empty()) {
            stored_hash = std::string(stored_hash_data->begin(), stored_hash_data->end());
        }
        structure_changed = (stored_hash != current_hash);
    }
    
    if (structure_changed) {
        config_.system->log(platform::System::LogLevel::Info, 
            "[AccessoryServer] Database structure changed, incrementing Configuration Number");
        
        auto cn_data = config_.storage->get("config_number");
        uint16_t cn = 0;
        if (cn_data && !cn_data->empty()) {
            cn = static_cast<uint16_t>(std::stoi(std::string(cn_data->begin(), cn_data->end())));
        }
        
        cn = (cn >= 65535) ? 1 : cn + 1;
        
        std::string cn_str = std::to_string(cn);
        config_.storage->set("config_number", std::vector<uint8_t>(cn_str.begin(), cn_str.end()));
        
        // Update stored hash
        if (iid_manager_) {
            iid_manager_->update_stored_hash(current_hash);
        } else {
            config_.storage->set("db_hash", std::vector<uint8_t>(current_hash.begin(), current_hash.end()));
        }

        config_.system->log(platform::System::LogLevel::Info, 
            "[AccessoryServer] Configuration Number updated to: " + cn_str);
    } else {
        config_.system->log(platform::System::LogLevel::Debug, 
            "[AccessoryServer] Database structure unchanged, CN remains the same");
    }
}

} // namespace hap
