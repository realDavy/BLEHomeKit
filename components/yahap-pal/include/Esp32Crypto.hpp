#pragma once

#include "hap/platform/CryptoSRP.hpp"
#include <string_view>

class Esp32Crypto : public hap::platform::CryptoSRP {
public:
    Esp32Crypto();
    ~Esp32Crypto() override = default;

    void sha512(std::span<const uint8_t> data, std::span<uint8_t, 64> output) override;

    void hkdf_sha512(
        std::span<const uint8_t> key,
        std::span<const uint8_t> salt,
        std::span<const uint8_t> info,
        std::span<uint8_t> output) override;

    void ed25519_generate_keypair(std::span<uint8_t, 32> public_key, std::span<uint8_t, 64> private_key) override;
    void ed25519_sign(
        std::span<const uint8_t, 64> private_key,
        std::span<const uint8_t> message,
        std::span<uint8_t, 64> signature) override;
    bool ed25519_verify(
        std::span<const uint8_t, 32> public_key,
        std::span<const uint8_t> message,
        std::span<const uint8_t, 64> signature) override;

    void x25519_generate_keypair(std::span<uint8_t, 32> public_key, std::span<uint8_t, 32> private_key) override;
    void x25519_shared_secret(
        std::span<const uint8_t, 32> private_key,
        std::span<const uint8_t, 32> peer_public_key,
        std::span<uint8_t, 32> shared_secret) override;

    bool chacha20_poly1305_encrypt_and_tag(
        std::span<const uint8_t, 32> key,
        std::span<const uint8_t, 12> nonce,
        std::span<const uint8_t> aad,
        std::span<const uint8_t> plaintext,
        std::span<uint8_t> ciphertext,
        std::span<uint8_t, 16> tag) override;

    bool chacha20_poly1305_decrypt_and_verify(
        std::span<const uint8_t, 32> key,
        std::span<const uint8_t, 12> nonce,
        std::span<const uint8_t> aad,
        std::span<const uint8_t> ciphertext,
        std::span<const uint8_t, 16> tag,
        std::span<uint8_t> plaintext) override;

    std::unique_ptr<hap::platform::SRPSession> srp_new_verifier(
        std::string_view username,
        std::string_view password) override;
        
    std::array<uint8_t, 16> srp_get_salt(hap::platform::SRPSession* session) override;
    std::vector<uint8_t> srp_get_public_key(hap::platform::SRPSession* session) override;
    bool srp_set_client_public_key(hap::platform::SRPSession* session, std::span<const uint8_t> client_public) override;
    bool srp_verify_client_proof(hap::platform::SRPSession* session, std::span<const uint8_t> proof) override;
    std::vector<uint8_t> srp_get_server_proof(hap::platform::SRPSession* session) override;
    std::vector<uint8_t> srp_get_session_key(hap::platform::SRPSession* session) override;
};
