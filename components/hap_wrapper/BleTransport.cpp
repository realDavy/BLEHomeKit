// Local overlay of hap/src/transport/BleTransport.cpp:
// fragment HAP-BLE GATT reads to ATT MTU, do not apply the 10s
// procedure timeout while waiting for the next pair-setup write,
// keep SF=1 until Pair-Verify, do not register an empty 0xFE59 GATT service,
// coalesce disconnected-event GSN bumps, avoid 500 ms advertising after drop,
// and disconnect after Home RemovePairing so advertising returns to SF=1.
#include "hap/transport/BleTransport.hpp"
#include "hap/transport/ConnectionContext.hpp"
#include "hap/core/CharacteristicFinder.hpp"
#include "hap/core/HAPStatus.hpp"
#include "hap/platform/Storage.hpp"
#include "hap/platform/System.hpp"
#include <cstdint>
#include <random>
#include <algorithm>
#include <cctype>
#include <cstring>
#include <optional>
#include <string>
#include <vector>
#include "hap/core/TLV8.hpp"
#include "hap/transport/ble/BleTlvBuilder.hpp"
#include "hap/core/CharacteristicSerializer.hpp"

static bool s_adv_dirty = false;
static bool s_gsn_bumped_while_disconnected = false;
static uint32_t s_exclude_conn_id = UINT32_MAX;
static uint16_t s_disconnect_after_read = 0;
static constexpr const char* kPairVerifiedKey = "pair_verified";

static bool hap_list_means_paired(hap::platform::Storage* storage) {
    if (!storage) {
        return false;
    }
    auto pairing_list = storage->get("pairing_list");
    if (!pairing_list || pairing_list->size() <= 2) {
        return false;
    }
    std::string s(pairing_list->begin(), pairing_list->end());
    return !s.empty() && s.front() == '[' && s.back() == ']' && s != "[]";
}

// Home's Add Accessory flow disconnects after Pair-Setup and scans again.
// Advertising SF=0 at that moment makes iOS treat the accessory as already
// in a Home and fail with "Discovery failed". Keep SF=1 until Pair-Verify.
static bool hap_should_advertise_paired(hap::platform::Storage* storage) {
    if (!hap_list_means_paired(storage)) {
        return false;
    }
    auto verified = storage->get(kPairVerifiedKey);
    if (!verified || verified->empty()) {
        // Pairings written before this flag existed already finished Add Accessory.
        return true;
    }
    return (*verified)[0] == '1';
}

static void hap_note_pair_setup_saved(hap::platform::Storage* storage, hap::platform::System* system) {
    if (!hap_list_means_paired(storage)) {
        return;
    }
    auto verified = storage->get(kPairVerifiedKey);
    if (verified && !verified->empty() && (*verified)[0] == '0') {
        return;
    }
    storage->set(kPairVerifiedKey, std::vector<uint8_t>{'0'});
    if (system) {
        system->log(hap::platform::System::LogLevel::Info,
            "[BleTransport] Pair-Setup saved controller; keep SF=1 until Pair Verify");
    }
}

static void hap_note_unpaired_if_empty(hap::platform::Storage* storage, hap::platform::System* system) {
    if (!storage || hap_list_means_paired(storage)) {
        return;
    }
    storage->remove(kPairVerifiedKey);
    if (system) {
        system->log(hap::platform::System::LogLevel::Info,
            "[BleTransport] No controllers left; next advertisement uses SF=1");
    }
}

static void hap_after_pairings(hap::platform::Storage* storage, hap::platform::System* system,
                              hap::transport::ConnectionContext& ctx, uint16_t connection_id) {
    hap_note_unpaired_if_empty(storage, system);
    if (!ctx.should_close()) {
        return;
    }
    s_disconnect_after_read = connection_id;
    if (system) {
        system->log(hap::platform::System::LogLevel::Info,
            "[BleTransport] RemovePairing: will disconnect after response");
    }
}

static void hap_note_pair_verify_done(hap::platform::Storage* storage, hap::platform::System* system,
                                     bool encrypted) {
    if (!encrypted || !storage) {
        return;
    }
    auto verified = storage->get(kPairVerifiedKey);
    if (verified && !verified->empty() && (*verified)[0] == '1') {
        return;
    }
    storage->set(kPairVerifiedKey, std::vector<uint8_t>{'1'});
    if (system) {
        system->log(hap::platform::System::LogLevel::Info,
            "[BleTransport] Pair-Verify succeeded; next advertisement uses SF=0");
    }
}

static std::string to_hex_string(const uint8_t* data, size_t len) {
    std::string s;
    char buf[3];
    for (size_t i = 0; i < len; ++i) {
        snprintf(buf, sizeof(buf), "%02X", data[i]);
        s += buf;
    }
    return s;
}

namespace hap::transport {

// Import types from the extracted ble:: namespace components
using ble::PDUOpcode;
using ble::TransactionState;
using ble::HAPBLEPDUTLVType;
using ble::BleTlvBuilder;

BleTransport::BleTransport(Config config) : config_(std::move(config)),
    session_manager_(std::make_unique<ble::BleSessionManager>(config_.system)) {
    if (!config_.ble) {
        if(config_.system) config_.system->log(platform::System::LogLevel::Warning, "[BleTransport] No BLE platform interface provided");
    }
}

BleTransport::~BleTransport() {
    stop();
}

void BleTransport::start() {
    if (!config_.ble) return;

    config_.system->log(platform::System::LogLevel::Info, "[BleTransport] Starting...");

    config_.ble->set_disconnect_callback([this](uint16_t connection_id) {
        config_.system->log(platform::System::LogLevel::Info, 
            "[BleTransport] Device disconnected, connection_id=" + std::to_string(connection_id));
        
        session_manager_->remove(connection_id);
        
        config_.system->log(platform::System::LogLevel::Info, 
            "[BleTransport] Connection state cleaned up, refreshing advertising");
        s_adv_dirty = false;
        s_gsn_bumped_while_disconnected = false;
        if (s_disconnect_after_read == connection_id) {
            s_disconnect_after_read = 0;
        }
        update_advertising();
    });

    register_accessory_info_service();
    
    setup_protocol_info_service();
    
    setup_hap_service();

    // 0xFE59 belongs in advertisements only. An empty GATT service with that
    // UUID makes Home treat HAP as having no characteristics ("Discovery failed").
    
    register_user_services();
    
    update_advertising();
    
    config_.ble->start();
}

void BleTransport::stop() {
    if (config_.ble) {
        config_.ble->stop_advertising();
    }
}

void BleTransport::setup_hap_service() {
    // Use IIDManager for stable IIDs with dedicated keys for HAP pairing service
    auto get_iid = [this](const std::string& key) -> uint16_t {
        if (config_.iid_manager) {
            return config_.iid_manager->get_or_assign(key);
        }
        // Fallback: use hash of key as stable-ish IID
        uint16_t iid = 1;
        for (char c : key) iid = ((iid << 5) + iid) ^ c;
        return (iid & 0x7FFF) + 1;  // Ensure positive, non-zero
    };
    
    uint16_t svc_iid = get_iid("BLE:S:0055");  // HAP Pairing Service
    platform::Ble::ServiceDefinition hap_service;
    hap_service.uuid = kHapPairingServiceUUID;
    hap_service.is_primary = true;

    auto add_iid_descriptor = [](platform::Ble::CharacteristicDefinition& def, uint16_t iid) {
        platform::Ble::DescriptorDefinition desc;
        desc.uuid = kCharacteristicInstanceIdDescUUID;
        desc.properties.read = true;
        desc.on_read = [iid](uint16_t) {
            std::vector<uint8_t> val;
            val.push_back(iid & 0xFF);
            val.push_back((iid >> 8) & 0xFF);
            return val;
        };
        def.descriptors.push_back(std::move(desc));
    };

    {
        platform::Ble::CharacteristicDefinition def;
        def.uuid = kServiceInstanceIdCharUUID;
        def.properties = { .read = true };
        def.on_read = [svc_iid](uint16_t) {
            std::vector<uint8_t> val;
            val.push_back(svc_iid & 0xFF);
            val.push_back((svc_iid >> 8) & 0xFF);
            return val;
        };
        hap_service.characteristics.push_back(std::move(def));
    }

    {
        uint16_t char_iid = get_iid("BLE:C:004C:0055");  // Pair Setup
        platform::Ble::CharacteristicDefinition def;
        def.uuid = "0000004C-0000-1000-8000-0026BB765291";
        def.properties = { .read = true, .write = true };
        def.on_write = [this](uint16_t conn, std::span<const uint8_t> data, bool) {
            config_.system->log(platform::System::LogLevel::Info, "[BleTransport] Pair Setup Write: " + std::to_string(data.size()) + " bytes");
            handle_hap_write(conn, "0000004C-0000-1000-8000-0026BB765291", data);
        };
        def.on_read = [this](uint16_t conn) {
            return handle_hap_read(conn);
        };
        add_iid_descriptor(def, char_iid);
        
        CharacteristicMetadata meta;
        meta.instance_id = char_iid;
        meta.service_id = svc_iid;
        meta.char_type = 0x4C;  // Pair Setup
        meta.service_type = 0x55;  // Pairing Service
        meta.properties = 0x0003;  // Read | Write (NOT Paired Read/Write!)
        pairing_char_metadata_[char_iid] = meta;
        
        config_.system->log(platform::System::LogLevel::Info, "[BleTransport] Registered Pair Setup (4C) IID=" + std::to_string(char_iid));
        hap_service.characteristics.push_back(std::move(def));
    }

    {
        uint16_t char_iid = get_iid("BLE:C:004E:0055");  // Pair Verify
        platform::Ble::CharacteristicDefinition def;
        def.uuid = "0000004E-0000-1000-8000-0026BB765291";
        def.properties.read = true;
        def.properties.write = true;
        def.properties.indicate = false;
        def.properties.notify = false;
        def.on_write = [this](uint16_t conn, std::span<const uint8_t> data, bool) {
            handle_hap_write(conn, "0000004E-0000-1000-8000-0026BB765291", data);
        };
        def.on_read = [this](uint16_t conn) {
            return handle_hap_read(conn);
        };
        add_iid_descriptor(def, char_iid);
        
        CharacteristicMetadata meta;
        meta.instance_id = char_iid;
        meta.service_id = svc_iid;
        meta.service_type = 0x55;  // Pairing Service
        meta.char_type = 0x4E;  // Pair Verify
        meta.properties = 0x0003;  // Read | Write (NOT Paired Read/Write!)
        pairing_char_metadata_[char_iid] = meta;
        
        hap_service.characteristics.push_back(std::move(def));
    }

    {
        config_.system->log(platform::System::LogLevel::Info, "[BleTransport] Adding Pairing Features...");
        uint16_t char_iid = get_iid("BLE:C:004F:0055");  // Pairing Features
        platform::Ble::CharacteristicDefinition def;
        def.uuid = "0000004F-0000-1000-8000-0026BB765291";
        def.properties.read = true;
        def.properties.write = true;
        def.properties.indicate = false;
        def.properties.notify = false;
        def.on_write = [this](uint16_t conn, std::span<const uint8_t> data, bool) {
            handle_hap_write(conn, "0000004F-0000-1000-8000-0026BB765291", data);
        };
        def.on_read = [this](uint16_t conn) {
            return handle_hap_read(conn);
        };
        add_iid_descriptor(def, char_iid);
        
        CharacteristicMetadata meta;
        meta.instance_id = char_iid;
        meta.service_id = svc_iid;
        meta.char_type = 0x4F;  // Pairing Features
        meta.service_type = 0x55;  // Pairing Service
        meta.properties = 0x0001;  // Read only (NOT Paired Read!)
        meta.user_description = "Pairing Features";
        pairing_char_metadata_[char_iid] = meta;
        
        hap_service.characteristics.push_back(std::move(def));
    }

    {
        config_.system->log(platform::System::LogLevel::Info, "[BleTransport] Adding Pairing Pairings...");
        uint16_t char_iid = get_iid("BLE:C:0050:0055");  // Pairing Pairings
        platform::Ble::CharacteristicDefinition def;
        def.uuid = "00000050-0000-1000-8000-0026BB765291";
        def.properties.read = true;
        def.properties.write = true;
        def.properties.indicate = false;
        def.properties.notify = false;
        
        def.on_write = [this](uint16_t conn, std::span<const uint8_t> data, bool) {
            handle_hap_write(conn, "00000050-0000-1000-8000-0026BB765291", data);
        };
        
        def.on_read = [this](uint16_t conn) {
            return handle_hap_read(conn);
        };
        add_iid_descriptor(def, char_iid);
        
        CharacteristicMetadata meta;
        meta.instance_id = char_iid;
        meta.service_id = svc_iid;
        meta.char_type = 0x50;  // Pairing Pairings
        meta.service_type = 0x55;  // Pairing Service
        meta.properties = 0x0030;  // Paired Read (0x10) | Paired Write (0x20)
        pairing_char_metadata_[char_iid] = meta;
        
        hap_service.characteristics.push_back(std::move(def));
    }

    config_.system->log(platform::System::LogLevel::Info, "[BleTransport] Registering Service...");
    config_.ble->register_service(hap_service);
    
    // Save IIDManager state if available
    if (config_.iid_manager) {
        config_.iid_manager->save();
    }
}

void BleTransport::setup_protocol_info_service() {
    // Use IIDManager for stable IIDs with dedicated keys for Protocol Info service
    auto get_iid = [this](const std::string& key) -> uint16_t {
        if (config_.iid_manager) {
            return config_.iid_manager->get_or_assign(key);
        }
        // Fallback: use hash of key as stable-ish IID
        uint16_t iid = 1;
        for (char c : key) iid = ((iid << 5) + iid) ^ c;
        return (iid & 0x7FFF) + 1;  // Ensure positive, non-zero
    };
    
    uint16_t svc_iid = get_iid("BLE:S:00A2");  // Protocol Information Service
    platform::Ble::ServiceDefinition proto_service;
    proto_service.uuid = kHapProtocolInformationServiceUUID;
    proto_service.is_primary = true;

    auto add_iid_descriptor = [](platform::Ble::CharacteristicDefinition& def, uint16_t iid) {
        platform::Ble::DescriptorDefinition desc;
        desc.uuid = kCharacteristicInstanceIdDescUUID;
        desc.properties.read = true;
        desc.on_read = [iid](uint16_t) {
            std::vector<uint8_t> val;
            val.push_back(iid & 0xFF);
            val.push_back((iid >> 8) & 0xFF);
            return val;
        };
        def.descriptors.push_back(std::move(desc));
    };

    {
        platform::Ble::CharacteristicDefinition def;
        def.uuid = kServiceInstanceIdCharUUID;
        def.properties = { .read = true };
        def.on_read = [svc_iid](uint16_t) {
            std::vector<uint8_t> val;
            val.push_back(svc_iid & 0xFF);
            val.push_back((svc_iid >> 8) & 0xFF);
            return val;
        };
        proto_service.characteristics.push_back(std::move(def));
    }

    {
        uint16_t char_iid = get_iid("BLE:C:00A5:00A2");  // Service Signature
        platform::Ble::CharacteristicDefinition def;
        def.uuid = kServiceSignatureCharUUID;
        def.properties = { .read = true, .write = true };
        def.on_write = [this](uint16_t conn, std::span<const uint8_t> data, bool) {
            handle_hap_write(conn, kServiceSignatureCharUUID, data);
        };
        def.on_read = [this](uint16_t conn) {
             return handle_hap_read(conn);
         };
        add_iid_descriptor(def, char_iid);
        
        CharacteristicMetadata meta;
        meta.instance_id = char_iid;
        meta.service_id = svc_iid;
        meta.char_type = 0xA5;  // Service Signature
        meta.service_type = 0xA2;  // Protocol Information Service
        meta.properties = 0x0010;  // Paired Read only
        pairing_char_metadata_[char_iid] = meta;
        
        proto_service.characteristics.push_back(std::move(def));
    }

    {
        uint16_t char_iid = get_iid("BLE:C:0037:00A2");  // Version
        platform::Ble::CharacteristicDefinition def;
        def.uuid = "00000037-0000-1000-8000-0026BB765291";
        def.properties = { .read = true, .write = true };
        def.on_write = [this](uint16_t conn, std::span<const uint8_t> data, bool) {
            handle_hap_write(conn, "00000037-0000-1000-8000-0026BB765291", data);
        };
        def.on_read = [this](uint16_t conn) {
            return handle_hap_read(conn);
        };
        add_iid_descriptor(def, char_iid);
        
        CharacteristicMetadata meta;
        meta.instance_id = char_iid;
        meta.service_id = svc_iid;
        meta.char_type = 0x37;  // Version
        meta.service_type = 0xA2;  // Protocol Information Service
        meta.properties = 0x0010;  // Paired Read only
        pairing_char_metadata_[char_iid] = meta;
        
        proto_service.characteristics.push_back(std::move(def));
    }

    config_.ble->register_service(proto_service);
    
    // Save IIDManager state if available
    if (config_.iid_manager) {
        config_.iid_manager->save();
    }
}

void BleTransport::update_advertising() {
    if (session_manager_ && session_manager_->session_count() > 0) {
        s_adv_dirty = true;
        config_.system->log(platform::System::LogLevel::Info,
            "[BleTransport] Defer advertising update until disconnect (avoid nimble stack overflow)");
        return;
    }

    config_.system->log(platform::System::LogLevel::Info, "[BleTransport] update_advertising entry");
    
    auto setup_id_bytes = config_.storage->get("setup_id");
    std::string setup_id;
    if (setup_id_bytes && setup_id_bytes->size() == 4) {
        setup_id = std::string(setup_id_bytes->begin(), setup_id_bytes->end());
        config_.system->log(platform::System::LogLevel::Debug, "[BleTransport] Using existing Setup ID: " + setup_id);
    } else {
        const char charset[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
        std::mt19937 rng(std::random_device{}());
        std::uniform_int_distribution<> dist(0, 35);
        for (int i = 0; i < 4; ++i) {
            setup_id += charset[dist(rng)];
        }
        config_.storage->set("setup_id", std::vector<uint8_t>(setup_id.begin(), setup_id.end()));
        config_.system->log(platform::System::LogLevel::Info, "[BleTransport] Generated new Setup ID: " + setup_id);
    }
    
    std::string input = setup_id + config_.accessory_id;
    std::vector<uint8_t> hash_output(64);
    config_.system->log(platform::System::LogLevel::Debug, "[BleTransport] Calculating Setup Hash for: " + input);
    
    if (config_.crypto == nullptr) {
         config_.system->log(platform::System::LogLevel::Error, "[BleTransport] No crypto provider!");
         return;
    }

    config_.crypto->sha512(
        std::span<const uint8_t>(reinterpret_cast<const uint8_t*>(input.data()), input.size()), 
        std::span<uint8_t, 64>(hash_output.data(), 64)
    );
    
    uint8_t setup_hash[4];
    std::copy_n(hash_output.begin(), 4, setup_hash);
    
    uint8_t status_flags = hap_should_advertise_paired(config_.storage) ? 0x00 : 0x01;
    
    uint8_t device_id[6] = {0};
    int scanned = sscanf(config_.accessory_id.c_str(), "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", 
        &device_id[0], &device_id[1], &device_id[2], &device_id[3], &device_id[4], &device_id[5]);
    if (scanned != 6) {
         config_.system->log(platform::System::LogLevel::Warning, "[BleTransport] Invalid Device ID format: " + config_.accessory_id);
    }

    uint16_t gsn = 1;
    auto gsn_bytes = config_.storage->get("gsn");
    if (gsn_bytes && gsn_bytes->size() == 2) {
        gsn = static_cast<uint16_t>((*gsn_bytes)[0]) | (static_cast<uint16_t>((*gsn_bytes)[1]) << 8);
        if (gsn == 0) gsn = 1;
    } else {
        std::vector<uint8_t> gsn_data = {0x01, 0x00};
        config_.storage->set("gsn", gsn_data);
        config_.system->log(platform::System::LogLevel::Info, "[BleTransport] Initialized GSN to 1");
    }

    uint8_t config_number = 1;
    auto cn_bytes = config_.storage->get("config_number");
    if (cn_bytes && !cn_bytes->empty()) {
        std::string cn_str(cn_bytes->begin(), cn_bytes->end());
        config_number = static_cast<uint8_t>(std::stoi(cn_str));
    }

    auto adv = platform::Ble::Advertisement::create_hap(
        status_flags,
        device_id,
        config_.category_id,
        gsn,
        config_number,
        setup_hash
    );
    
    adv.local_name = config_.device_name;

    std::string adv_hex;
    for (auto byte : adv.manufacturer_data) {
        char buf[4];
        snprintf(buf, sizeof(buf), "%02X ", byte);
        adv_hex += buf;
    }
    config_.system->log(platform::System::LogLevel::Info, 
        "[BleTransport] Advertisement Data (" + std::to_string(adv.manufacturer_data.size()) + " bytes): " + adv_hex);
    config_.system->log(platform::System::LogLevel::Info,
        "[BleTransport] SF=" + std::to_string(status_flags) + 
        " ACID=" + std::to_string(config_.category_id) +
        " GSN=" + std::to_string(gsn) +
        " CN=" + std::to_string(config_number) +
        (status_flags ? " (pairable)" : " (paired)"));

    config_.ble->start_advertising(adv, 20);
}

void BleTransport::set_accessory_id(const std::string& new_id) {
    config_.accessory_id = new_id;
    config_.system->log(platform::System::LogLevel::Info, 
        "[BleTransport] Accessory ID updated to: " + new_id);
}

void BleTransport::notify_value_changed(uint64_t aid, uint64_t iid, const core::Value& value, uint32_t exclude_conn_id) {
    handle_characteristic_change(aid, iid, value, exclude_conn_id);
}

void BleTransport::increment_gsn() {
    // Per Spec 7.4.6: GSN increments on characteristic changes
    // Range: 1-65535, wraps to 1 on overflow
    auto gsn_bytes = config_.storage->get("gsn");
    uint16_t gsn = 1;
    
    if (gsn_bytes && gsn_bytes->size() == 2) {
        gsn = static_cast<uint16_t>((*gsn_bytes)[0]) | (static_cast<uint16_t>((*gsn_bytes)[1]) << 8);
    }
    
    gsn++;
    if (gsn == 0) {
        gsn = 1;
    }
    
    std::vector<uint8_t> gsn_data = {
        static_cast<uint8_t>(gsn & 0xFF),
        static_cast<uint8_t>((gsn >> 8) & 0xFF)
    };
    config_.storage->set("gsn", gsn_data);
    
    config_.system->log(platform::System::LogLevel::Debug, 
        "[BleTransport] GSN incremented to " + std::to_string(gsn));
    
    update_advertising();
}

void BleTransport::check_session_timeouts() {
    auto timed_out = session_manager_->check_timeouts();
    
    for (uint16_t conn_id : timed_out) {
        session_manager_->remove(conn_id);
        config_.ble->disconnect(conn_id);
        config_.system->log(platform::System::LogLevel::Info,
            "[BleTransport] Terminated connection " + std::to_string(conn_id) + " due to timeout");
    }
}

void BleTransport::handle_hap_write_with_id(uint16_t connection_id, std::string uuid, std::span<const uint8_t> data) {
    config_.system->log(platform::System::LogLevel::Debug, "[BleTransport] Write to Char UUID: " + uuid);
    handle_hap_write(connection_id, uuid, data);
}

void BleTransport::handle_hap_write(uint16_t connection_id, const std::string& uuid, std::span<const uint8_t> data) {
    if (data.empty()) return;
    
    config_.system->log(platform::System::LogLevel::Debug, "[BleTransport] Write PDU Fragment (" + std::to_string(data.size()) + " bytes): " + to_hex_string(data.data(), data.size()));

    bool session_is_secured = false;
    auto* session = session_manager_->get_session(connection_id);
    if (session && session->context) {
        session_is_secured = session->context->is_encrypted();
    }
    
    bool requires_encryption = true;
    {
        if (uuid.size() >= 8) {
            std::string short_uuid_str = uuid.substr(4, 4);
            unsigned int short_uuid = 0;
            if (sscanf(short_uuid_str.c_str(), "%x", &short_uuid) == 1) {
                if (short_uuid == 0x4C || short_uuid == 0x4E || short_uuid == 0x4F) {
                    requires_encryption = false;
                }
            }
        }
    }

    std::vector<uint8_t> decrypted_data;
    std::span<const uint8_t> working_data = data;
    
    if (session_is_secured && requires_encryption) {
        auto& session_ref = session_manager_->get_or_create(connection_id);
        if (!session_ref.context) {
            config_.system->log(platform::System::LogLevel::Error,
                "[BleTransport] Context missing for secured session - disconnecting");
            config_.ble->disconnect(connection_id);
            session_manager_->remove(connection_id);
            return;
        }
        auto decrypted = session_ref.context->get_secure_session()->decrypt_ble_pdu(data);
        if (!decrypted) {
            config_.system->log(platform::System::LogLevel::Error, 
                "[BleTransport] Decryption failed for connection " + std::to_string(connection_id) + " - disconnecting");
            // Per HAP Spec 6.5.2.2: Close connection on decryption failure
            config_.ble->disconnect(connection_id);
            session_manager_->remove(connection_id);
            return;
        }
        decrypted_data = std::move(*decrypted);
        working_data = decrypted_data;
        config_.system->log(platform::System::LogLevel::Debug, 
            "[BleTransport] Decrypted PDU (" + std::to_string(decrypted_data.size()) + " bytes): " + 
            to_hex_string(decrypted_data.data(), decrypted_data.size()));
    }
    
    if (working_data.empty()) return;
    uint8_t control_field = working_data[0];
    bool continuation = (control_field & 0x80) != 0;
    PDUOpcode opcode = PDUOpcode::CharacteristicWrite;
    uint16_t tid = 0;
    
    if (!continuation) {
        if (working_data.size() < 3) {
            config_.system->log(platform::System::LogLevel::Error, "[BleTransport] PDU too short");
            return;
        }
        opcode = static_cast<PDUOpcode>(working_data[1]);
        tid = working_data[2];
        
        config_.system->log(platform::System::LogLevel::Info, "[BleTransport] New Transaction TID=" + std::to_string(tid) + " Opcode=" + std::to_string((int)opcode));
        
        auto& state = session_manager_->get_or_create(connection_id).transaction;
        state.opcode = opcode;
        state.transaction_id = tid;
        state.target_uuid = uuid;
        state.buffer.clear();
        state.response_buffer.clear();
        state.response_read_offset = 0;
        state.active = true;
        state.last_activity_ms = config_.system->millis();
        state.procedure_start_ms = config_.system->millis();
        state.last_write_ms = config_.system->millis();
        
        if (state.connection_established_ms == 0) {
            state.connection_established_ms = config_.system->millis();
        }
        
        state.buffer.insert(state.buffer.end(), working_data.begin(), working_data.end());
        process_transaction(connection_id, state);
    } else {
        auto& state = session_manager_->get_or_create(connection_id).transaction;
        if (!state.active) {
             config_.system->log(platform::System::LogLevel::Warning, "[BleTransport] Orphaned continuation fragment!");
             return;
        }
        
        // Spec 7.3.3.1: TID is present in every fragment.
        // Packet: CF (1) | TID (1) | Data...
        if (working_data.size() < 2) return;
        uint16_t cont_tid = working_data[1];
        if (cont_tid != state.transaction_id) {
            config_.system->log(platform::System::LogLevel::Error, "[BleTransport] TID mismatch in continuation! Expected " + std::to_string(state.transaction_id) + " got " + std::to_string(cont_tid));
            return;
        }

        // Continuation Body starts at index 2 (CF, TID)
        state.buffer.insert(state.buffer.end(), working_data.begin() + 2, working_data.end());
        process_transaction(connection_id, state);
    }
}

std::vector<uint8_t> BleTransport::handle_hap_read(uint16_t connection_id) {
    auto* session = session_manager_->get_session(connection_id);
    if (!session) {
        return {};
    }
    auto& state = session->transaction;

    if (state.last_write_ms > 0) {
        uint64_t current_time = config_.system->millis();
        uint64_t time_since_write = current_time - state.last_write_ms;

        if (time_since_write > 10000) { // 10 seconds
            config_.system->log(platform::System::LogLevel::Warning,
                "[BleTransport] Rejecting GATT Read - >10s since write (Req #12)");
            return {};
        }
    }

    const auto& buf = state.response_buffer;
    if (buf.empty() || state.response_read_offset >= buf.size()) {
        return {};
    }

    uint16_t mtu = config_.ble ? config_.ble->att_mtu(connection_id) : 23;
    if (mtu < 23) {
        mtu = 23;
    }
    // ATT Read response payload is MTU - 1 (opcode) - 2 (handle) = MTU - 3.
    const size_t max_payload = static_cast<size_t>(mtu - 3);

    std::vector<uint8_t> fragment;
    if (state.response_read_offset == 0) {
        const size_t n = std::min(buf.size(), max_payload);
        fragment.assign(buf.begin(), buf.begin() + static_cast<std::ptrdiff_t>(n));
        state.response_read_offset = n;
    } else {
        // HAP Spec 7.3.3: continuation is CF | TID | next PDU bytes.
        const uint8_t tid = (buf.size() >= 2) ? buf[1] : 0;
        const size_t remain = buf.size() - state.response_read_offset;
        const size_t chunk = std::min(remain, max_payload > 2 ? max_payload - 2 : 0);
        fragment.reserve(2 + chunk);
        fragment.push_back(0x82);
        fragment.push_back(tid);
        fragment.insert(fragment.end(),
                        buf.begin() + static_cast<std::ptrdiff_t>(state.response_read_offset),
                        buf.begin() + static_cast<std::ptrdiff_t>(state.response_read_offset + chunk));
        state.response_read_offset += chunk;
    }

    state.last_activity_ms = config_.system->millis();
    config_.system->log(platform::System::LogLevel::Info,
        "[BleTransport] Handling GATT Read. Returning " +
        std::to_string(fragment.size()) + "/" + std::to_string(buf.size()) +
        " bytes (offset=" + std::to_string(state.response_read_offset) +
        " mtu=" + std::to_string(mtu) + ")");

    if (state.response_read_offset >= buf.size() &&
        s_disconnect_after_read == connection_id && config_.ble) {
        s_disconnect_after_read = 0;
        const uint16_t conn = connection_id;
        auto disconnect = [this, conn]() {
            config_.system->log(platform::System::LogLevel::Info,
                "[BleTransport] Disconnecting after RemovePairing");
            config_.ble->disconnect(conn);
        };
        if (config_.scheduler) {
            config_.scheduler->schedule_once(200, std::move(disconnect));
        } else {
            disconnect();
        }
    }
    return fragment;
}

void BleTransport::process_transaction(uint16_t connection_id, TransactionState& state) {
    // Header is 5 bytes: CF(1) | Opcode(1) | TID(1) | IID(2)
    if (state.buffer.size() < 5) return; 
    
    PDUOpcode opcode = state.opcode;
    
    // For Write operations (opcode 0x02, 0x04), the header is 7 bytes:
    // CF(1) | Opcode(1) | TID(1) | IID(2) | BodyLen(2) | Body...
    if (ble::HapPdu::opcode_has_body(opcode)) {
        
        if (state.buffer.size() < 7) return;
        
        uint16_t body_length = state.buffer[5] | (static_cast<uint16_t>(state.buffer[6]) << 8);
        state.expected_body_length = body_length;
        
        size_t expected_total = 7 + body_length;
        
        if (state.buffer.size() < expected_total) {
            config_.system->log(platform::System::LogLevel::Debug, 
                "[BleTransport] Waiting for more fragments: have " + 
                std::to_string(state.buffer.size()) + " bytes, need " + 
                std::to_string(expected_total) + " bytes");
            return;
        }
        
        config_.system->log(platform::System::LogLevel::Debug, 
            "[BleTransport] All fragments received: " + 
            std::to_string(state.buffer.size()) + " bytes (body=" + 
            std::to_string(body_length) + ")");
    }
    
    uint16_t tid = state.buffer[2];
    uint16_t iid = state.buffer[3] | (state.buffer[4] << 8);

    config_.system->log(platform::System::LogLevel::Info, "[BleTransport] Processing Opcode " + std::to_string((int)opcode) + " TID=" + std::to_string(tid));
    
    std::unique_ptr<core::CharacteristicFinder> finder;
    if (config_.database) {
        finder = std::make_unique<core::CharacteristicFinder>(*config_.database);
    }
    
    auto find_service = [&](uint16_t target_iid) -> std::shared_ptr<core::Service> {
        return finder ? finder->service_by_iid(target_iid) : nullptr;
    };
    
    auto find_char_in_db = [&](uint16_t target_iid) -> std::shared_ptr<core::Characteristic> {
        return finder ? finder->by_iid(target_iid) : nullptr;
    };
    
    // Helper to get full characteristic info including AID
    auto find_char_info = [&](uint16_t target_iid) -> core::CharacteristicFinder::CharacteristicInfo {
        if (finder) {
            return finder->find_info(target_iid);
        }
        return {};
    };

    if (opcode == PDUOpcode::ServiceSignatureRead) {
        state.active = false;
        state.procedure_start_ms = 0;
        
        std::vector<uint8_t> sig_response;
        bool is_primary = false;
        bool found = false;
        std::vector<uint16_t> linked_services;

        for (const auto& [cid, meta] : pairing_char_metadata_) {
            if (meta.service_id == iid) {
                is_primary = true;
                found = true;
                break;
            }
        }

        if (!found) {
            auto svc = find_service(iid);
            if (svc) {
                is_primary = svc->is_primary();
                found = true;
                for (uint64_t linked_iid : svc->linked_services()) {
                    linked_services.push_back(static_cast<uint16_t>(linked_iid));
                }
            }
        }

        // Per Spec 7.3.4.13: If invalid Service IID, return props=0 and linked=0 length
        if (!found) {
            config_.system->log(platform::System::LogLevel::Warning, 
                "[BleTransport] Service Signature Read IID=" + std::to_string(iid) + " Not Found - returning empty");
            // Return empty response with props=0
            BleTlvBuilder builder;
            builder.add_uint16(HAPBLEPDUTLVType::ServiceProperties, 0);
            auto tlv = builder.build();
            sig_response.insert(sig_response.end(), tlv.begin(), tlv.end());
            
            sig_response.push_back(static_cast<uint8_t>(HAPBLEPDUTLVType::LinkedServices));
            sig_response.push_back(0x00);
        } else {
            uint16_t props = is_primary ? 0x0001 : 0x0000;
            
            BleTlvBuilder builder;
            builder.add_uint16(HAPBLEPDUTLVType::ServiceProperties, props);
            auto tlv = builder.build();
            sig_response.insert(sig_response.end(), tlv.begin(), tlv.end());
            
            sig_response.push_back(static_cast<uint8_t>(HAPBLEPDUTLVType::LinkedServices));
            sig_response.push_back(static_cast<uint8_t>(linked_services.size() * 2));
            for (uint16_t linked_iid : linked_services) {
                sig_response.push_back(linked_iid & 0xFF);
                sig_response.push_back((linked_iid >> 8) & 0xFF);
            }
            
            config_.system->log(platform::System::LogLevel::Info, 
                "[BleTransport] Service Signature Read IID=" + std::to_string(iid) + 
                " Primary=" + std::to_string(is_primary));
        }
        
        send_response(connection_id, state.transaction_id, state.target_uuid, 0x00, sig_response);
        return;
    }
    
    if (opcode == PDUOpcode::CharacteristicSignatureRead) {
        state.active = false; 
        
        std::vector<uint8_t> sig_response = process_signature_read(connection_id, iid);
        
        uint8_t status = 0x00;
        if (sig_response.empty()) {
             config_.system->log(platform::System::LogLevel::Warning, "[BleTransport] Char Signature Read IID=" + std::to_string(iid) + " Not Found");
             status = 0x05; // Invalid Request (Attribute Not Found)
        } else {
             config_.system->log(platform::System::LogLevel::Info, "[BleTransport] Char Signature Read IID=" + std::to_string(iid) + " Len=" + std::to_string(sig_response.size()));
             config_.system->log(platform::System::LogLevel::Debug, "[BleTransport] Signature Response: " + to_hex_string(sig_response.data(), sig_response.size()));
        }
        
        send_response(connection_id, state.transaction_id, state.target_uuid, status, sig_response);
        return;
    }

    // For operations with Body (Write, Timed Write, Configuration)
    // Header is 7 bytes: CF(1) | Opcode(1) | TID(1) | IID(2) | BodyLen(2)
    // Body starts at offset 7
    // For Read (0x03), header is only 5 bytes with no body
    
    size_t body_offset = ble::HapPdu::body_offset(opcode);
    
    std::span<const uint8_t> body;
    if (state.buffer.size() > body_offset) {
        body = std::span<const uint8_t>(state.buffer.data() + body_offset, state.buffer.size() - body_offset);
    }
    
    if (opcode == PDUOpcode::CharacteristicRead) {
        
        std::vector<uint8_t> value_bytes;
        uint8_t status = 0x00;
        
        auto meta_it = pairing_char_metadata_.find(iid);
        if (meta_it != pairing_char_metadata_.end()) {
            if (meta_it->second.char_type == 0x4F) { // Pairing Features
                value_bytes = {0x01, 0x01, 0x00};
                config_.system->log(platform::System::LogLevel::Info, "[BleTransport] Pairing Features Read: returning 0x00");
            } else if (meta_it->second.char_type == 0x37) { // Version
                // HAP Spec 7.4.4.5.2: Version characteristic returns protocol version string
                // Format: "major.minor.revision" e.g., "1.1.0"
                // Per HAP spec, current version for BLE is 1.1.0
                std::string version = "1.1.0";
                value_bytes.push_back(0x01); // TLV Type: HAP-Param-Value
                value_bytes.push_back(static_cast<uint8_t>(version.size())); // Length
                value_bytes.insert(value_bytes.end(), version.begin(), version.end()); // Value
                config_.system->log(platform::System::LogLevel::Info, "[BleTransport] Version Read: returning " + version);
            } else {
                status = 0x05; // Invalid Request
            }
        } 
        else {
            auto ch = find_char_in_db(iid);
            if (ch) {
                auto read_result = ch->get_value();
                if (std::holds_alternative<core::HAPStatus>(read_result)) {
                    // Read callback returned error
                    status = 0x02; // HAP Error (map HAPStatus to BLE status)
                    config_.system->log(platform::System::LogLevel::Warning, 
                        "[BleTransport] Read IID=" + std::to_string(iid) + " callback returned error");
                } else {
                    auto raw_value = core::CharacteristicSerializer::to_bytes(std::get<core::Value>(read_result));
                    value_bytes.push_back(0x01); // Type: HAP-Param-Value
                    value_bytes.push_back(static_cast<uint8_t>(raw_value.size()));
                    value_bytes.insert(value_bytes.end(), raw_value.begin(), raw_value.end());
                }
            } else {
                status = 0x05; // Invalid Request (Attribute Not Found)
                config_.system->log(platform::System::LogLevel::Warning, "[BleTransport] Read IID=" + std::to_string(iid) + " Not Found");
            }
        }
        
        config_.system->log(platform::System::LogLevel::Info,
            "[BleTransport] Characteristic Read IID=" + std::to_string(iid) +
            " status=" + std::to_string(status) +
            " len=" + std::to_string(value_bytes.size()));
        send_response(connection_id, state.transaction_id, state.target_uuid, status, value_bytes);
    }
    else if (opcode == PDUOpcode::CharacteristicWrite) {
        uint8_t status = 0x00;
        std::vector<uint8_t> response_body;

        auto meta_it = pairing_char_metadata_.find(iid);
        if (meta_it != pairing_char_metadata_.end()) {
             uint8_t type = meta_it->second.char_type;
             
             auto& session = session_manager_->get_or_create(connection_id);
             if (!session.context) {
                session.context = std::make_unique<ConnectionContext>(config_.crypto, config_.system, connection_id);
             }
             auto& ctx = *session.context;
             
             // HAP-BLE Pair Setup/Verify are "Write-with-Response" (Spec 7.3.5.5)
             // The body is a LIST of TLVs:
             // - kTLVType_ReturnResponse (0x09): (Empty implies request response)
             // - kTLVType_Value (0x01): The actual SRP payload (M1, M3, etc)
             
             std::vector<uint8_t> inner_body;
             bool return_response_requested = false;

             // Only parse as BLE TLVs for Pairing Services (Setup 4C, Verify 4E, Pairings 50)
             if (type == 0x4C || type == 0x4E || type == 0x50) {
                 auto ble_tlvs = core::TLV8::parse(std::vector<uint8_t>(body.begin(), body.end()));
                 
                 // Check for Return-Response (0x09)
                 if (core::TLV8::find(ble_tlvs, (uint8_t)HAPBLEPDUTLVType::ReturnResponse)) {
                     return_response_requested = true;
                 }
                 
                 // Extract Value (0x01)
                 auto val = core::TLV8::find(ble_tlvs, 0x01); // 0x01 = Param-Value
                 if (val) {
                     inner_body.assign(val->begin(), val->end());
                 } else {
                     config_.system->log(platform::System::LogLevel::Warning, "[BleTransport] Warning: Pairing Write missing Value TLV wrapper. Using raw body.");
                     inner_body.assign(body.begin(), body.end());
                 }
             } else {
                inner_body.assign(body.begin(), body.end());
             }

             Request req;
             req.body = std::move(inner_body);
             req.method = Method::POST;
             
             Response resp;
             if (type == 0x4C) { // Pair Setup
                req.path = "/pair-setup";
                resp = config_.pairing_endpoints->handle_pair_setup(req, ctx);
                hap_note_pair_setup_saved(config_.storage, config_.system);
             } else if (type == 0x4E) { // Pair Verify
                req.path = "/pair-verify";
                resp = config_.pairing_endpoints->handle_pair_verify(req, ctx);
                hap_note_pair_verify_done(config_.storage, config_.system, ctx.is_encrypted());
             } else if (type == 0x50) { // Pairings
                req.path = "/pairings";
                resp = config_.pairing_endpoints->handle_pairings(req, ctx);
                hap_after_pairings(config_.storage, config_.system, ctx, connection_id);
             } else if (type == 0xA5) { // Service Signature
                config_.system->log(platform::System::LogLevel::Info, "[BleTransport] Software Auth Write - Skipping (Success)");
                resp = Response{Status::OK}; 
             }
             
             status = (resp.status == Status::OK) ? 0x00 : 0x02;

             // If Return-Response was requested, we MUST wrap the response body in Result TLVs
             // TlV 0x01 (Value) -> Response Body
             if (return_response_requested && !resp.body.empty()) {
                 std::vector<core::TLV> resp_tlvs;
                 resp_tlvs.emplace_back(0x01, resp.body); // 0x01 = Param-Value
                 response_body = core::TLV8::encode(resp_tlvs);
             } else {
                 response_body = resp.body;
             }
        }
        else {
             auto ch = find_char_in_db(iid);
             if (ch) {
                 auto body_tlvs = core::TLV8::parse(std::vector<uint8_t>(body.begin(), body.end()));
                 
                 bool return_response_requested = core::TLV8::find(body_tlvs, (uint8_t)HAPBLEPDUTLVType::ReturnResponse).has_value();
                 
                 auto value_tlv = core::TLV8::find(body_tlvs, 0x01);
                 if (value_tlv && !value_tlv->empty()) {
                     core::Value new_value;
                     switch (ch->format()) {
                         case core::Format::Bool:
                             new_value = static_cast<bool>((*value_tlv)[0] != 0);
                             break;
                         case core::Format::UInt8:
                             new_value = (*value_tlv)[0];
                             break;
                         case core::Format::UInt16:
                             if (value_tlv->size() >= 2) {
                                 new_value = static_cast<uint16_t>((*value_tlv)[0] | ((*value_tlv)[1] << 8));
                             }
                             break;
                         case core::Format::UInt32:
                             if (value_tlv->size() >= 4) {
                                 new_value = static_cast<uint32_t>(
                                     (*value_tlv)[0] | ((*value_tlv)[1] << 8) |
                                     ((*value_tlv)[2] << 16) | ((*value_tlv)[3] << 24));
                             }
                             break;
                         case core::Format::Int:
                             if (value_tlv->size() >= 4) {
                                 new_value = static_cast<int32_t>(
                                     (*value_tlv)[0] | ((*value_tlv)[1] << 8) |
                                     ((*value_tlv)[2] << 16) | ((*value_tlv)[3] << 24));
                             }
                             break;
                         case core::Format::Float:
                            if (value_tlv->size() >= 4) {
                                uint32_t raw = (*value_tlv)[0] | ((*value_tlv)[1] << 8) |
                                            ((*value_tlv)[2] << 16) | ((*value_tlv)[3] << 24);
                                float f;
                                std::memcpy(&f, &raw, sizeof(f));
                                new_value = f;
                            }
                            break;
                         case core::Format::String:
                            new_value = std::string(value_tlv->begin(), value_tlv->end());
                            break;
                         case core::Format::TLV8:
                         case core::Format::Data:
                            new_value = std::vector<uint8_t>(value_tlv->begin(), value_tlv->end());
                            break;
                         default:
                            break;
                     }
                     
                     ch->set_value(new_value, core::EventSource::from_connection(connection_id));
                     config_.system->log(platform::System::LogLevel::Info, 
                         "[BleTransport] Write IID=" + std::to_string(iid) + " success");
                     
                     // Per HAP Spec 7.4.1.8: GSN increments on first characteristic change per connection
                     if (!state.gsn_incremented) {
                         state.gsn_incremented = true;
                         increment_gsn();
                     }
                     
                     // HAP Spec 7.3.5.5: Write-with-Response - return value if requested
                     if (return_response_requested && 
                         core::has_permission(ch->permissions(), core::Permission::WriteResponse)) {
                         auto response_opt = ch->handle_write_response(new_value);
                         core::Value val_to_send;
                         
                         if (response_opt.has_value()) {
                             auto& response = *response_opt;
                             if (std::holds_alternative<core::HAPStatus>(response)) {
                                 // WriteResponse callback returned error - still send status
                                 status = 0x02; // HAP Error
                                 config_.system->log(platform::System::LogLevel::Warning,
                                     "[BleTransport] WriteResponse IID=" + std::to_string(iid) + " callback returned error");
                             } else {
                                 val_to_send = std::get<core::Value>(response);
                             }
                         } else {
                             // No callback, use current value
                             auto read_result = ch->get_value();
                             if (std::holds_alternative<core::Value>(read_result)) {
                                 val_to_send = std::get<core::Value>(read_result);
                             }
                         }
                         
                         if (status == 0x00) {
                             auto raw_bytes = core::CharacteristicSerializer::to_bytes(val_to_send);
                             std::vector<core::TLV> resp_tlvs;
                             resp_tlvs.emplace_back(0x01, raw_bytes); // HAP-Param-Value
                             response_body = core::TLV8::encode(resp_tlvs);
                             
                             config_.system->log(platform::System::LogLevel::Info, 
                                 "[BleTransport] Write-Response IID=" + std::to_string(iid) + 
                                 " returning " + std::to_string(response_body.size()) + " bytes");
                         }
                     }
                     
                     // Get actual AID for this characteristic
                     auto char_info = find_char_info(iid);
                     handle_characteristic_change(char_info.accessory_id, iid, new_value, connection_id);
                 } else {
                     config_.system->log(platform::System::LogLevel::Warning, 
                         "[BleTransport] Write IID=" + std::to_string(iid) + " - no value TLV found");
                     status = 0x06; // Invalid Request
                 }
             } else {
                 status = 0x05; // Not Found
             }
        }
        
        send_response(connection_id, state.transaction_id, state.target_uuid, status, response_body);
    }
    else if (opcode == PDUOpcode::CharacteristicTimedWrite) {
        state.timed_write_body.assign(body.begin(), body.end());
        state.timed_write_iid = iid;
        state.target_uuid = state.target_uuid;
        
        config_.system->log(platform::System::LogLevel::Info,
            "[BleTransport] Timed Write stored for IID=" + std::to_string(iid) + 
            " Body size=" + std::to_string(body.size()));
        
        send_response(connection_id, state.transaction_id, state.target_uuid, 0x00, {});
    }
    else if (opcode == PDUOpcode::CharacteristicExecuteWrite) {
        uint8_t status = 0x00;
        std::vector<uint8_t> response_body;
        
        if (state.timed_write_body.empty()) {
            config_.system->log(platform::System::LogLevel::Warning,
                "[BleTransport] Execute Write with no pending timed write");
            status = 0x06; // Invalid Request
        } else {
            config_.system->log(platform::System::LogLevel::Info,
                "[BleTransport] Executing pending timed write for IID=" + std::to_string(state.timed_write_iid));
            
            auto meta_it = pairing_char_metadata_.find(state.timed_write_iid);
            if (meta_it != pairing_char_metadata_.end()) {
                uint8_t type = meta_it->second.char_type;
                
                auto& session = session_manager_->get_or_create(connection_id);
                if (!session.context) {
                    session.context = std::make_unique<ConnectionContext>(config_.crypto, config_.system, connection_id);
                }
                auto& ctx = *session.context;
                
                std::vector<uint8_t> inner_body;
                bool return_response_requested = false;
                
                // Parse BLE TLVs for Pairing characteristics
                if (type == 0x4C || type == 0x4E || type == 0x50) {
                    auto ble_tlvs = core::TLV8::parse(state.timed_write_body);
                    
                    if (core::TLV8::find(ble_tlvs, (uint8_t)HAPBLEPDUTLVType::ReturnResponse)) {
                        return_response_requested = true;
                    }
                    
                    auto val = core::TLV8::find(ble_tlvs, 0x01);
                    if (val) {
                        inner_body.assign(val->begin(), val->end());
                    } else {
                        inner_body = state.timed_write_body;
                    }
                } else {
                    inner_body = state.timed_write_body;
                }
                
                Request req;
                req.body = std::move(inner_body);
                req.method = Method::POST;
                
                Response resp;
                if (type == 0x4C) {
                    req.path = "/pair-setup";
                    resp = config_.pairing_endpoints->handle_pair_setup(req, ctx);
                    hap_note_pair_setup_saved(config_.storage, config_.system);
                } else if (type == 0x4E) {
                    req.path = "/pair-verify";
                    resp = config_.pairing_endpoints->handle_pair_verify(req, ctx);
                    hap_note_pair_verify_done(config_.storage, config_.system, ctx.is_encrypted());
                } else if (type == 0x50) {
                    req.path = "/pairings";
                    resp = config_.pairing_endpoints->handle_pairings(req, ctx);
                    hap_after_pairings(config_.storage, config_.system, ctx, connection_id);
                }
                
                status = (resp.status == Status::OK) ? 0x00 : 0x02;
                
                if (return_response_requested && !resp.body.empty()) {
                    std::vector<core::TLV> resp_tlvs;
                    resp_tlvs.emplace_back(0x01, resp.body);
                    response_body = core::TLV8::encode(resp_tlvs);
                } else {
                    response_body = resp.body;
                }
            } else {
                auto ch = find_char_in_db(state.timed_write_iid);
                if (ch) {
                    auto body_tlvs = core::TLV8::parse(state.timed_write_body);
                    auto value_tlv = core::TLV8::find(body_tlvs, 0x01);
                    if (value_tlv && !value_tlv->empty()) {
                        core::Value new_value;
                        switch (ch->format()) {
                            case core::Format::Bool:
                                new_value = static_cast<bool>((*value_tlv)[0] != 0);
                                break;
                            case core::Format::UInt8:
                                new_value = (*value_tlv)[0];
                                break;
                            case core::Format::UInt16:
                                if (value_tlv->size() >= 2) {
                                    new_value = static_cast<uint16_t>((*value_tlv)[0] | ((*value_tlv)[1] << 8));
                                }
                                break;
                            case core::Format::UInt32:
                                if (value_tlv->size() >= 4) {
                                    new_value = static_cast<uint32_t>(
                                        (*value_tlv)[0] | ((*value_tlv)[1] << 8) |
                                        ((*value_tlv)[2] << 16) | ((*value_tlv)[3] << 24));
                                }
                                break;
                            case core::Format::Int:
                                if (value_tlv->size() >= 4) {
                                    new_value = static_cast<int32_t>(
                                        (*value_tlv)[0] | ((*value_tlv)[1] << 8) |
                                        ((*value_tlv)[2] << 16) | ((*value_tlv)[3] << 24));
                                }
                                break;
                            case core::Format::Float:
                                if (value_tlv->size() >= 4) {
                                    uint32_t raw = (*value_tlv)[0] | ((*value_tlv)[1] << 8) |
                                                ((*value_tlv)[2] << 16) | ((*value_tlv)[3] << 24);
                                    float f;
                                    std::memcpy(&f, &raw, sizeof(f));
                                    new_value = f;
                                }
                                break;
                            case core::Format::String:
                                new_value = std::string(value_tlv->begin(), value_tlv->end());
                                break;
                            default:
                                break;
                        }
                        
                        // Pass EventSource so the originating connection is excluded from notifications
                        ch->set_value(new_value, core::EventSource::from_connection(connection_id));
                        config_.system->log(platform::System::LogLevel::Info, 
                            "[BleTransport] Execute Timed Write IID=" + std::to_string(state.timed_write_iid) + " success");
                        
                        // Per HAP Spec 7.4.1.8: GSN increments on first characteristic change per connection
                        if (!state.gsn_incremented) {
                            state.gsn_incremented = true;
                            increment_gsn();
                        }
                        
                    } else {
                        config_.system->log(platform::System::LogLevel::Warning, 
                            "[BleTransport] Execute Timed Write IID=" + std::to_string(state.timed_write_iid) + " - no value TLV found");
                        status = 0x06; // Invalid Request
                    }
                } else {
                    config_.system->log(platform::System::LogLevel::Warning, 
                        "[BleTransport] Execute Timed Write IID=" + std::to_string(state.timed_write_iid) + " - characteristic not found");
                    status = 0x05; // Not Found
                }
            }
            
            state.timed_write_body.clear();
            state.timed_write_iid = 0;
        }
        
        send_response(connection_id, state.transaction_id, state.target_uuid, status, response_body);
    }
    else if (opcode == PDUOpcode::CharacteristicConfiguration) {
        // HAP-Characteristic-Configuration-Request/Response
        // Per Spec 7.3.4.14-15, Table 7-28, 7-29, 7-30
        
        std::vector<uint8_t> response_body;
        uint8_t status = 0x00;
        
        auto config_tlvs = core::TLV8::parse(std::vector<uint8_t>(body.begin(), body.end()));
        
        uint16_t properties = 0x0000; // Broadcast disabled by default
        uint8_t broadcast_interval = 0x01; // 20ms default (Table 7-30)
        
        auto props_tlv = core::TLV8::find(config_tlvs, 0x01);
        if (props_tlv && props_tlv->size() == 2) {
            properties = static_cast<uint16_t>((*props_tlv)[0]) | 
                        (static_cast<uint16_t>((*props_tlv)[1]) << 8);
        }
        
        auto interval_tlv = core::TLV8::find(config_tlvs, 0x02);
        if (interval_tlv && interval_tlv->size() == 1) {
            broadcast_interval = (*interval_tlv)[0];
            if (broadcast_interval > 0x03) {
                broadcast_interval = 0x01;
            }
        }
        
        std::vector<core::TLV> resp_tlvs;
        
        std::vector<uint8_t> props_data = {
            static_cast<uint8_t>(properties & 0xFF),
            static_cast<uint8_t>((properties >> 8) & 0xFF)
        };
        resp_tlvs.emplace_back(0x01, props_data);
        
        resp_tlvs.emplace_back(0x02, std::vector<uint8_t>{broadcast_interval});
        
        response_body = core::TLV8::encode(resp_tlvs);
        
        // Store broadcast configuration for this characteristic
        // Per HAP Spec 7.4.6.2: Properties bit 0x0001 enables broadcast notification
        bool broadcast_enabled = (properties & 0x0001) != 0;
        broadcast_configs_[iid] = BroadcastConfig{iid, broadcast_interval, broadcast_enabled};
        
        config_.system->log(platform::System::LogLevel::Info,
            "[BleTransport] Characteristic Configuration IID=" + std::to_string(iid) +
            " Props=" + std::to_string(properties) + " Interval=" + std::to_string(broadcast_interval) +
            " BroadcastEnabled=" + std::to_string(broadcast_enabled));
        
        send_response(connection_id, state.transaction_id, state.target_uuid, status, response_body);
    }
    else if (opcode == PDUOpcode::ProtocolConfiguration) {
        // HAP-Protocol-Configuration-Request/Response
        // Per Spec 7.3.4.16-17, Table 7-32, 7-34
        
        std::vector<uint8_t> response_body;
        uint8_t status = 0x00;
        
        auto config_tlvs = core::TLV8::parse(std::vector<uint8_t>(body.begin(), body.end()));
        
        auto gen_key_tlv = core::TLV8::find(config_tlvs, 0x01);
        auto get_all_tlv = core::TLV8::find(config_tlvs, 0x02);
        
        std::vector<core::TLV> resp_tlvs;
        
        if (gen_key_tlv) {
            // Generate broadcast encryption key per HAP Spec 7.4.7.3:
            // BroadcastEncryptionKey = HKDF-SHA-512(
            //     IKM = Current Session shared secret,
            //     Salt = Controller's Ed25519 long-term public key,
            //     Info = "Broadcast-Encryption-Key",
            //     L = 32 bytes
            // )
            config_.system->log(platform::System::LogLevel::Info,
                "[BleTransport] Protocol Config: Generate Broadcast Encryption Key requested");
            
            auto* session = session_manager_->get_session(connection_id);
            if (!session || !session->context || !session->context->is_encrypted()) {
                config_.system->log(platform::System::LogLevel::Error,
                    "[BleTransport] Protocol Config: Cannot generate key - no secure session");
                status = 0x06; // Invalid Request
            } else {
                auto& ctx = *session->context;
                
                std::string pairing_key = "pairing_" + ctx.controller_id();
                auto controller_ltpk = config_.storage->get(pairing_key);
                
                if (!controller_ltpk || controller_ltpk->size() != 32) {
                    config_.system->log(platform::System::LogLevel::Error,
                        "[BleTransport] Protocol Config: Controller LTPK not found for: " + ctx.controller_id());
                    status = 0x06; // Invalid Request
                } else {
                    // Derive broadcast encryption key using HKDF-SHA-512
                    std::array<uint8_t, 32> broadcast_key;
                    config_.crypto->hkdf_sha512(
                        ctx.session_shared_secret(),                                         // IKM: Session shared secret
                        std::span<const uint8_t>(controller_ltpk->data(), 32),              // Salt: Controller LTPK
                        std::span(reinterpret_cast<const uint8_t*>("Broadcast-Encryption-Key"), 24), // Info
                        broadcast_key                                                        // Output: 32 bytes
                    );
                    
                    std::copy(broadcast_key.begin(), broadcast_key.end(), broadcast_key_.begin());
                    broadcast_key_valid_ = true;
                    broadcast_key_gsn_start_ = get_current_gsn();
                    
                    resp_tlvs.emplace_back(0x04, std::vector<uint8_t>(broadcast_key.begin(), broadcast_key.end()));
                    
                    config_.system->log(platform::System::LogLevel::Info,
                        "[BleTransport] Protocol Config: Generated and stored Broadcast Encryption Key (GSN start=" + 
                        std::to_string(broadcast_key_gsn_start_) + ")");
                }
            }
        }
        
        if (get_all_tlv || true) {
            uint16_t gsn = 1;
            auto gsn_bytes = config_.storage->get("gsn");
            if (gsn_bytes && gsn_bytes->size() == 2) {
                gsn = static_cast<uint16_t>((*gsn_bytes)[0]) | 
                     (static_cast<uint16_t>((*gsn_bytes)[1]) << 8);
            }
            
            // State Number TLV (0x01)
            std::vector<uint8_t> state_data = {
                static_cast<uint8_t>(gsn & 0xFF),
                static_cast<uint8_t>((gsn >> 8) & 0xFF)
            };
            resp_tlvs.emplace_back(0x01, state_data);
            
            // Configuration Number TLV (0x02)
            resp_tlvs.emplace_back(0x02, std::vector<uint8_t>{
                config_.config_number
            });
            
            // Accessory Advertising Identifier TLV (0x03) - 6 bytes
            // Use Device ID as advertising identifier
            uint8_t device_id[6] = {0};
            sscanf(config_.accessory_id.c_str(), "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
                &device_id[0], &device_id[1], &device_id[2], 
                &device_id[3], &device_id[4], &device_id[5]);
            resp_tlvs.emplace_back(0x03, std::vector<uint8_t>(device_id, device_id + 6));
        }
        
        response_body = core::TLV8::encode(resp_tlvs);
        
        config_.system->log(platform::System::LogLevel::Info,
            "[BleTransport] Protocol Configuration completed");
        
        send_response(connection_id, state.transaction_id, state.target_uuid, status, response_body);
    }
    else if (opcode == PDUOpcode::ServiceSignatureRead) {
        send_response(connection_id, state.transaction_id, state.target_uuid, 0x00, {});
    }
    else {
        send_response(connection_id, state.transaction_id, state.target_uuid, 0x01, {});
    }
}

void BleTransport::send_response(uint16_t conn_id, uint16_t tid, const std::string& uuid, uint8_t status, std::span<const uint8_t> body) {
    std::vector<uint8_t> packet = ble::HapPdu::build_response(tid, status, body);
    
    bool session_is_secured = false;
    auto* session_ptr = session_manager_->get_session(conn_id);
    if (session_ptr && session_ptr->context) {
        session_is_secured = session_ptr->context->is_encrypted();
    }
    
    bool requires_encryption = true;
    if (uuid.size() >= 8) {
        std::string short_uuid_str = uuid.substr(4, 4);
        unsigned int short_uuid = 0;
        if (sscanf(short_uuid_str.c_str(), "%x", &short_uuid) == 1) {
            if (short_uuid == 0x4C || short_uuid == 0x4E || short_uuid == 0x4F) {
                requires_encryption = false;
            }
        }
    }
    
    if (session_is_secured && requires_encryption) {
        auto& ctx = *session_ptr->context;
        auto encrypted = ctx.get_secure_session()->encrypt_ble_pdu(packet);
        if (encrypted.empty()) {
            config_.system->log(platform::System::LogLevel::Error, 
                "[BleTransport] Response encryption failed for connection " + std::to_string(conn_id));
            session_manager_->get_or_create(conn_id).transaction.response_buffer = packet;
        } else {
            config_.system->log(platform::System::LogLevel::Debug, 
                "[BleTransport] Encrypted response (" + std::to_string(encrypted.size()) + " bytes)");
            session_manager_->get_or_create(conn_id).transaction.response_buffer = std::move(encrypted);
        }
    } else {
        session_manager_->get_or_create(conn_id).transaction.response_buffer = packet;
    }

    auto& queued = session_manager_->get_or_create(conn_id).transaction;
    queued.response_read_offset = 0;
    // Response is queued for GATT reads. Clear the 10s procedure timer so
    // pair-setup can wait for the next write (user entering the setup code).
    queued.active = false;

    // HAP-BLE Spec 7.3.5.1/7.3.5.5: The response is returned in the GATT Read Response.
}

bool BleTransport::process_characteristic_write(uint16_t connection_id, uint16_t tid, const std::string& uuid, std::span<const uint8_t> body) {
    auto& session = session_manager_->get_or_create(connection_id);
    if (!session.context) {
        session.context = std::make_unique<ConnectionContext>(config_.crypto, config_.system, connection_id);
    }
    
    if (uuid == "0000004C-0000-1000-8000-0026BB765291") { // Pair Setup
        Request req;
        req.body.assign(body.begin(), body.end());
        req.method = Method::POST; 
        req.path = "/pair-setup";
        
        auto resp = config_.pairing_endpoints->handle_pair_setup(req, *session.context);
        hap_note_pair_setup_saved(config_.storage, config_.system);
        
        uint8_t status = (resp.status == Status::OK) ? 0x00 : 0x05; 
        send_response(connection_id, tid, uuid, status, resp.body);
        return true; 
    }
    else if (uuid == "0000004E-0000-1000-8000-0026BB765291") { // Pair Verify
        Request req;
        req.body.assign(body.begin(), body.end());
        req.method = Method::POST;
        req.path = "/pair-verify";
        
        auto resp = config_.pairing_endpoints->handle_pair_verify(req, *session.context);
        hap_note_pair_verify_done(config_.storage, config_.system, session.context->is_encrypted());
        uint8_t status = (resp.status == Status::OK) ? 0x00 : 0x05;
        send_response(connection_id, tid, uuid, status, resp.body);
        return true;
    }
    else if (uuid == "00000050-0000-1000-8000-0026BB765291") { // Pairing Mappings
        if (!session.context) return false;
        
        Request req;
        req.body.assign(body.begin(), body.end());
        req.method = Method::POST;
        req.path = "/pairings";
        
        auto resp = config_.pairing_endpoints->handle_pairings(req, *session.context);
        hap_after_pairings(config_.storage, config_.system, *session.context, connection_id);
        uint8_t status = (resp.status == Status::OK) ? 0x00 : 0x05;
        send_response(connection_id, tid, uuid, status, resp.body);
        return true;
    }
    
    return false;
}
std::vector<uint8_t> BleTransport::process_signature_read(uint16_t connection_id, uint16_t char_iid) {
    (void)connection_id;
    std::vector<uint8_t> response;
    
    if (pairing_char_metadata_.count(char_iid)) {
        const auto& meta = pairing_char_metadata_[char_iid];
        
        {
            BleTlvBuilder builder;
            builder.add_hap_uuid128(HAPBLEPDUTLVType::CharacteristicType, meta.char_type);
            auto tlv = builder.build();
            response.insert(response.end(), tlv.begin(), tlv.end());
        }
        
        {
            BleTlvBuilder builder;
            builder.add_uint16(HAPBLEPDUTLVType::ServiceInstanceID, meta.service_id);
            auto tlv = builder.build();
            response.insert(response.end(), tlv.begin(), tlv.end());
        }
        
        {
            BleTlvBuilder builder;
            builder.add_hap_uuid128(HAPBLEPDUTLVType::ServiceType, meta.service_type);
            auto tlv = builder.build();
            response.insert(response.end(), tlv.begin(), tlv.end());
        }
        
        {
            BleTlvBuilder builder;
            builder.add_uint16(HAPBLEPDUTLVType::CharacteristicProperties, meta.properties);
            auto tlv = builder.build();
            response.insert(response.end(), tlv.begin(), tlv.end());
        }
        
        if (!meta.user_description.empty()) {
            response.push_back((uint8_t)HAPBLEPDUTLVType::GATTUserDescription);
            response.push_back(static_cast<uint8_t>(meta.user_description.size()));
            response.insert(response.end(), meta.user_description.begin(), meta.user_description.end());
        }
        
        {
             uint8_t gatt_format = (meta.char_type == 0x4F) ? 0x04 : 0x1B;
             BleTlvBuilder builder;
             builder.add_gatt_format(gatt_format);
             auto tlv = builder.build();
             response.insert(response.end(), tlv.begin(), tlv.end());
        }

        config_.system->log(platform::System::LogLevel::Info, 
            "[BleTransport] Generated signature for special char IID=" + std::to_string(char_iid));
        
        return response;
    }
    
    if (config_.database) {
        for (const auto& acc : config_.database->accessories()) {
            for (const auto& svc : acc->services()) {
                for (const auto& ch : svc->characteristics()) {
                     if (ch->iid() == char_iid) {
                            {
                             uint16_t short_uuid = ch->type() & 0xFFFF;
                             BleTlvBuilder builder;
                             builder.add_hap_uuid128(HAPBLEPDUTLVType::CharacteristicType, short_uuid);
                             auto tlv = builder.build();
                             response.insert(response.end(), tlv.begin(), tlv.end());
                         }

                         {
                             BleTlvBuilder builder;
                             builder.add_uint16(HAPBLEPDUTLVType::ServiceInstanceID, svc->iid());
                             auto tlv = builder.build();
                             response.insert(response.end(), tlv.begin(), tlv.end());
                         }
                         
                         {
                             uint16_t svc_short_uuid = svc->type() & 0xFFFF;
                             BleTlvBuilder builder;
                             builder.add_hap_uuid128(HAPBLEPDUTLVType::ServiceType, svc_short_uuid);
                             auto tlv = builder.build();
                             response.insert(response.end(), tlv.begin(), tlv.end());
                         }
                         
                         {
                             uint16_t props = 0;
                             auto perms = ch->permissions();
                             for (auto p : perms) {
                                 if (p == core::Permission::PairedRead) props |= 0x0010;
                                 else if (p == core::Permission::PairedWrite) props |= 0x0020;
                                 else if (p == core::Permission::Notify) props |= (0x0080 | 0x0100); 
                                 else if (p == core::Permission::TimedWrite) props |= 0x0008;
                                 else if (p == core::Permission::Hidden) props |= 0x0040;
                                 else if (p == core::Permission::AdditionalAuthorization) props |= 0x0004;
                                 else if (p == core::Permission::Broadcast) props |= 0x0200;
                             }
                             
                             BleTlvBuilder builder;
                             builder.add_uint16(HAPBLEPDUTLVType::CharacteristicProperties, props);
                             auto tlv = builder.build();
                             response.insert(response.end(), tlv.begin(), tlv.end());
                         }

                         {
                             uint8_t gatt_format = core::CharacteristicSerializer::gatt_format_byte(ch->format());
                             BleTlvBuilder builder;
                             builder.add_gatt_format(gatt_format);
                             auto tlv = builder.build();
                             response.insert(response.end(), tlv.begin(), tlv.end());
                         }
                         
                         return response;
                     }
                }
            }
        }
    }

    return response;
}

std::vector<uint8_t> BleTransport::process_characteristic_read(uint16_t connection_id, std::span<const uint8_t> body) { (void)connection_id; (void)body; return {}; }

void BleTransport::register_accessory_info_service() {
    register_services_by_type(0x3E);
}

void BleTransport::register_user_services() {
    register_services_by_type(0);
}

void BleTransport::register_services_by_type(uint16_t filter_type) {
    if (!config_.database) return;
    auto type_to_uuid_str = [](uint64_t type) {
        char buffer[37];
        snprintf(buffer, sizeof(buffer), "0000%04X-0000-1000-8000-0026BB765291", (unsigned int)(type & 0xFFFF));
        return std::string(buffer);
    };

    for (const auto& acc : config_.database->accessories()) {
        for (const auto& svc : acc->services()) {
            uint16_t svc_type = svc->type() & 0xFFFF;
            
            if (filter_type == 0 && svc_type == 0x3E) continue;
            if (filter_type != 0 && svc_type != filter_type) continue;
            
            // Use IID already assigned by AttributeDatabase (via IIDManager)
            uint16_t svc_iid = static_cast<uint16_t>(svc->iid());

            platform::Ble::ServiceDefinition def;
            def.uuid = type_to_uuid_str(svc->type());
            // HAP Spec 7.4.1: All HAP services must be primary GATT services.
            def.is_primary = true;
            
            {
                platform::Ble::CharacteristicDefinition svc_iid_char;
                svc_iid_char.uuid = kServiceInstanceIdCharUUID;
                svc_iid_char.properties.read = true;
                svc_iid_char.properties.write = false;
                svc_iid_char.properties.indicate = false;
                svc_iid_char.properties.notify = false;
                svc_iid_char.on_read = [svc_iid](uint16_t) {
                    std::vector<uint8_t> val;
                    val.push_back(svc_iid & 0xFF);
                    val.push_back((svc_iid >> 8) & 0xFF);
                    return val;
                };
                def.characteristics.push_back(std::move(svc_iid_char));
            }
            
            auto add_iid_descriptor = [](platform::Ble::CharacteristicDefinition& d, uint16_t iid) {
                platform::Ble::DescriptorDefinition desc;
                desc.uuid = kCharacteristicInstanceIdDescUUID;
                desc.properties.read = true;
                desc.on_read = [iid](uint16_t) {
                    std::vector<uint8_t> val;
                    val.push_back(iid & 0xFF);
                    val.push_back((iid >> 8) & 0xFF);
                    return val;
                };
                d.descriptors.push_back(std::move(desc));
            };

            for (const auto& ch : svc->characteristics()) {
                // Use IID already assigned by AttributeDatabase (via IIDManager)
                uint16_t char_iid = static_cast<uint16_t>(ch->iid());

                std::string char_uuid = type_to_uuid_str(ch->type());
                platform::Ble::CharacteristicDefinition cdef;
                cdef.uuid = char_uuid;
                
                instance_map_[{acc->aid(), ch->iid()}] = char_uuid;
                
                auto perms = ch->permissions();
                cdef.properties.read = true;
                cdef.properties.write = true;
                bool has_notify = core::has_permission(perms, core::Permission::Notify);
                cdef.properties.notify = false;
                cdef.properties.indicate = has_notify;

                add_iid_descriptor(cdef, char_iid);

                if (ch->description().has_value()) {
                    platform::Ble::DescriptorDefinition desc;
                    desc.uuid = "2901"; 
                    desc.properties.read = true;
                    std::string d = ch->description().value();
                    desc.on_read = [d](uint16_t) {
                        return std::vector<uint8_t>(d.begin(), d.end());
                    };
                    cdef.descriptors.push_back(std::move(desc));
                }
                
                cdef.on_read = [this](uint16_t conn_id) {
                    return handle_hap_read(conn_id);
                };
                
                cdef.on_write = [this, uuid=char_uuid](uint16_t conn_id, std::span<const uint8_t> data, bool response) {
                    (void)response;
                    handle_hap_write_with_id(conn_id, uuid, data);
                };
                
                cdef.on_subscribe = [this, uuid=char_uuid](uint16_t conn_id, bool enabled) {
                     if (enabled) {
                         session_manager_->add_subscription(uuid, conn_id);
                     } else {
                         session_manager_->remove_subscription(uuid, conn_id);
                     }
                };

                def.characteristics.push_back(std::move(cdef));
            }
            config_.ble->register_service(def);
        }
    }
}

uint16_t BleTransport::get_current_gsn() {
    auto gsn_bytes = config_.storage->get("gsn");
    uint16_t gsn = 1;
    if (gsn_bytes && gsn_bytes->size() == 2) {
        gsn = static_cast<uint16_t>((*gsn_bytes)[0]) | (static_cast<uint16_t>((*gsn_bytes)[1]) << 8);
    }
    return gsn;
}

bool BleTransport::is_broadcast_key_valid() {
    if (!broadcast_key_valid_) return false;
    
    // Per HAP Spec 7.4.7.4: Key expires after 32767 GSN increments
    uint16_t current_gsn = get_current_gsn();
    uint16_t gsn_diff = 0;
    
    // Handle GSN wraparound (1-65535, wraps to 1)
    if (current_gsn >= broadcast_key_gsn_start_) {
        gsn_diff = current_gsn - broadcast_key_gsn_start_;
    } else {
        // GSN wrapped around
        gsn_diff = (65535 - broadcast_key_gsn_start_) + current_gsn;
    }
    
    if (gsn_diff >= 32767) {
        config_.system->log(platform::System::LogLevel::Warning,
            "[BleTransport] Broadcast encryption key expired (GSN diff=" + std::to_string(gsn_diff) + ")");
        broadcast_key_valid_ = false;
        return false;
    }
    
    return true;
}

void BleTransport::handle_characteristic_change(uint64_t aid, uint64_t iid, 
                                                 const core::Value& value, 
                                                 uint32_t exclude_conn_id) {
    config_.system->log(platform::System::LogLevel::Debug,
        "[BleTransport] Characteristic change: AID=" + std::to_string(aid) + 
        " IID=" + std::to_string(iid));
    
    // Find the characteristic to check its event properties
    auto ch = config_.database ? config_.database->find_characteristic(aid, iid) : nullptr;
    if (!ch) {
        config_.system->log(platform::System::LogLevel::Warning,
            "[BleTransport] Cannot find characteristic for event: IID=" + std::to_string(iid));
        return;
    }
    
    // Check HAP characteristic properties for event support
    // Per HAP Spec Table 7-50:
    // - 0x0080: Notifies Events in Connected State
    // - 0x0100: Notifies Events in Disconnected State
    // - 0x0200: Supports Broadcast Notify
    bool supports_connected = false;
    bool supports_disconnected = false;
    bool supports_broadcast = false;
    
    for (const auto& perm : ch->permissions()) {
        if (perm == core::Permission::Notify) {
            supports_connected = true;
            supports_disconnected = true;
        }
        if (perm == core::Permission::Broadcast) {
            supports_broadcast = true;
        }
    }
    
    bool broadcast_enabled = false;
    if (broadcast_configs_.count(static_cast<uint16_t>(iid))) {
        auto& bc = broadcast_configs_[static_cast<uint16_t>(iid)];
        broadcast_enabled = bc.enabled;
    }
    
    auto it = instance_map_.find({aid, iid});
    if (it == instance_map_.end()) {
        config_.system->log(platform::System::LogLevel::Warning,
            "[BleTransport] No UUID mapping for IID=" + std::to_string(iid));
        return;
    }
    std::string uuid = it->second;
    
    bool has_connected_subscribers = false;
    if (session_manager_->has_subscribers(uuid)) {
        for (uint16_t conn_id : session_manager_->get_subscribers(uuid)) {
            if (conn_id != exclude_conn_id) {
                has_connected_subscribers = true;
                break;
            }
        }
    }
    
    is_connected_ = session_manager_->session_count() > 0;
    
    if (is_connected_ && has_connected_subscribers && supports_connected) {
        s_exclude_conn_id = exclude_conn_id;
        config_.system->log(platform::System::LogLevel::Info,
            "[BleTransport] Sending Connected Event for IID=" + std::to_string(iid));
        send_connected_event(static_cast<uint16_t>(iid));
        s_exclude_conn_id = UINT32_MAX;
    }
    else if (!is_connected_ && supports_broadcast && broadcast_enabled && is_broadcast_key_valid()) {
        config_.system->log(platform::System::LogLevel::Info,
            "[BleTransport] Sending Broadcasted Event for IID=" + std::to_string(iid));
        send_broadcasted_event(static_cast<uint16_t>(iid), value);
    }
    else if (!is_connected_ && supports_disconnected) {
        send_disconnected_event(static_cast<uint16_t>(iid));
    }
    else {
        config_.system->log(platform::System::LogLevel::Debug,
            "[BleTransport] No event sent for IID=" + std::to_string(iid) + 
            " (connected=" + std::to_string(is_connected_) +
            ", has_subs=" + std::to_string(has_connected_subscribers) +
            ", supports_connected=" + std::to_string(supports_connected) + ")");
    }
}

void BleTransport::send_connected_event(uint16_t iid) {
    // Per HAP Spec 7.4.6.1 Connected Events:
    // Send a ZERO-LENGTH indication to controllers that registered for indications.
    
    std::string uuid;
    for (const auto& [key, val] : instance_map_) {
        if (key.second == iid) {
            uuid = val;
            break;
        }
    }
    
    if (uuid.empty()) {
        config_.system->log(platform::System::LogLevel::Warning,
            "[BleTransport] Cannot send Connected Event - no UUID for IID=" + std::to_string(iid));
        return;
    }
    
    if (!session_manager_->has_subscribers(uuid)) {
        config_.system->log(platform::System::LogLevel::Debug,
            "[BleTransport] No subscribers for Connected Event IID=" + std::to_string(iid));
        return;
    }
    
    std::vector<uint8_t> empty_indication;
    
    for (uint16_t conn_id : session_manager_->get_subscribers(uuid)) {
        if (conn_id == s_exclude_conn_id) {
            continue;
        }
        config_.system->log(platform::System::LogLevel::Debug,
            "[BleTransport] Sending zero-length indication to conn=" + std::to_string(conn_id) + 
            " for IID=" + std::to_string(iid));
        
        config_.ble->send_indication(conn_id, uuid, empty_indication);
    }
}

void BleTransport::send_broadcasted_event(uint16_t iid, const core::Value& value) {
    // Per HAP Spec 7.4.6.2 Broadcasted Events:
    // When disconnected and broadcast is configured, send encrypted advertisement
    // containing the characteristic value.
    
    if (!is_broadcast_key_valid()) {
        config_.system->log(platform::System::LogLevel::Warning,
            "[BleTransport] Cannot send Broadcasted Event - no valid broadcast key");
        send_disconnected_event(iid);
        return;
    }
    
    std::vector<uint8_t> encrypted_payload = build_encrypted_advertisement_payload(iid, value);
    if (encrypted_payload.empty()) {
        config_.system->log(platform::System::LogLevel::Error,
            "[BleTransport] Failed to build encrypted advertisement payload");
        send_disconnected_event(iid);
        return;
    }
    
    increment_gsn();
    
    std::array<uint8_t, 6> adv_id = {};
    sscanf(config_.accessory_id.c_str(), "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
        &adv_id[0], &adv_id[1], &adv_id[2], 
        &adv_id[3], &adv_id[4], &adv_id[5]);
    
    uint32_t interval_ms = 20;  // Default 20ms
    if (broadcast_configs_.count(iid)) {
        switch (broadcast_configs_[iid].interval) {
            case 0x01: interval_ms = 20; break;
            case 0x02: interval_ms = 1280; break;
            case 0x03: interval_ms = 2560; break;
            default: interval_ms = 20; break;
        }
    }
    
    platform::Ble::EncryptedAdvertisement enc_adv;
    enc_adv.advertising_id = adv_id;
    enc_adv.encrypted_payload = std::move(encrypted_payload);
    enc_adv.gsn = get_current_gsn();
    
    config_.system->log(platform::System::LogLevel::Info,
        "[BleTransport] Starting encrypted advertisement for IID=" + std::to_string(iid) +
        " interval=" + std::to_string(interval_ms) + "ms duration=3000ms");
    
    config_.ble->start_encrypted_advertising(enc_adv, interval_ms, 3000);
}

void BleTransport::send_disconnected_event(uint16_t iid) {
    // Per HAP Spec 7.4.6.3 Disconnected Events:
    // Increment GSN once per disconnected period, then fast-advertise.
    // Extra characteristic changes must not stop/start advertising again
    // or iPhone cannot reconnect (Home shows 未响应).
    if (s_gsn_bumped_while_disconnected) {
        config_.system->log(platform::System::LogLevel::Debug,
            "[BleTransport] Disconnected Event IID=" + std::to_string(iid) +
            " skipped (GSN already incremented this disconnect period)");
        return;
    }
    s_gsn_bumped_while_disconnected = true;

    config_.system->log(platform::System::LogLevel::Info,
        "[BleTransport] Disconnected Event for IID=" + std::to_string(iid));
    
    increment_gsn();
    
    std::string setup_id;
    auto setup_id_bytes = config_.storage->get("setup_id");
    if (setup_id_bytes && setup_id_bytes->size() == 4) {
        setup_id = std::string(setup_id_bytes->begin(), setup_id_bytes->end());
    } else {
        setup_id = "X-HZ";
    }
    
    std::string input = setup_id + config_.accessory_id;
    std::array<uint8_t, 64> hash_output = {};
    config_.crypto->sha512(
        std::span<const uint8_t>(reinterpret_cast<const uint8_t*>(input.data()), input.size()),
        std::span<uint8_t, 64>(hash_output.data(), 64)
    );
    uint8_t setup_hash[4];
    std::copy_n(hash_output.begin(), 4, setup_hash);
    
    uint8_t status_flags = hap_should_advertise_paired(config_.storage) ? 0x00 : 0x01;
    
    uint8_t device_id[6] = {0};
    sscanf(config_.accessory_id.c_str(), "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
        &device_id[0], &device_id[1], &device_id[2],
        &device_id[3], &device_id[4], &device_id[5]);
    
    uint16_t gsn = get_current_gsn();
    
    uint8_t config_number = 1;
    auto cn_bytes = config_.storage->get("config_number");
    if (cn_bytes && !cn_bytes->empty()) {
        std::string cn_str(cn_bytes->begin(), cn_bytes->end());
        config_number = static_cast<uint8_t>(std::stoi(cn_str));
    }
    
    auto adv = platform::Ble::Advertisement::create_hap(
        status_flags, device_id, config_.category_id, gsn, config_number, setup_hash);
    adv.local_name = config_.device_name;
    
    // Keep 20 ms advertising after a GSN bump. Switching to 500 ms after 3 s
    // made iPhone miss the accessory (Home 未响应) while the knob was still
    // being turned.
    config_.system->log(platform::System::LogLevel::Info,
        "[BleTransport] Disconnected Event advertising at 20ms (GSN updated)");
    config_.ble->start_advertising(adv, 20);
    
    (void)iid;
}

std::vector<uint8_t> BleTransport::build_encrypted_advertisement_payload(uint16_t iid, const core::Value& value) {
    // Per HAP Spec 7.4.7.3 Broadcast Encryption:
    // Payload: 12 bytes = GSN(2) + IID(2) + Value(8, with padding)
    // Nonce: GSN padded to 12 bytes with zeros
    // AAD: 6-byte advertising identifier
    // AuthTag: First 4 bytes of 16-byte ChaCha20-Poly1305 tag
    
    if (!is_broadcast_key_valid()) {
        return {};
    }
    
    uint16_t gsn = get_current_gsn();
    
    std::vector<uint8_t> plaintext(12, 0);
    plaintext[0] = gsn & 0xFF;
    plaintext[1] = (gsn >> 8) & 0xFF;
    plaintext[2] = iid & 0xFF;
    plaintext[3] = (iid >> 8) & 0xFF;
    
    std::vector<uint8_t> value_bytes = core::CharacteristicSerializer::to_bytes(value);
    for (size_t i = 0; i < 8 && i < value_bytes.size(); ++i) {
        plaintext[4 + i] = value_bytes[i];
    }
    
    std::array<uint8_t, 12> nonce = {};
    nonce[0] = gsn & 0xFF;
    nonce[1] = (gsn >> 8) & 0xFF;
    
    std::array<uint8_t, 6> aad = {};
    sscanf(config_.accessory_id.c_str(), "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
        &aad[0], &aad[1], &aad[2], &aad[3], &aad[4], &aad[5]);
    
    std::vector<uint8_t> ciphertext(12);
    std::array<uint8_t, 16> full_tag = {};
    
    std::array<uint8_t, 32> key_copy;
    std::copy(broadcast_key_.begin(), broadcast_key_.end(), key_copy.begin());
    
    bool success = config_.crypto->chacha20_poly1305_encrypt_and_tag(
        key_copy,
        nonce,
        std::span<const uint8_t>(aad.data(), 6),
        plaintext,
        ciphertext,
        full_tag
    );
    
    if (!success) {
        config_.system->log(platform::System::LogLevel::Error,
            "[BleTransport] Broadcast encryption failed");
        return {};
    }
    
    std::vector<uint8_t> result;
    result.reserve(16);
    result.insert(result.end(), ciphertext.begin(), ciphertext.end());
    result.insert(result.end(), full_tag.begin(), full_tag.begin() + 4);
    
    return result;
}

} // namespace hap::transport

