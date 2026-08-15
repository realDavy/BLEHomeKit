#include "Esp32Crypto.hpp"
#include <cstring>
#include <esp_err.h>
#include <esp_log.h>
#include <mbedtls/bignum.h>
#if CONFIG_PAL_CRYPTO_HKDF
#include <mbedtls/hkdf.h>
#else
#error "HKDF is not enabled. HAP requires HKDF for Pairing."
#endif
#include <mbedtls/sha512.h>
#include <sodium.h>
#include <vector>

static const char *TAG = "Esp32Crypto";

// RFC 5054 3072-bit Group N
static const char *SRP_N_HEX =
    "FFFFFFFFFFFFFFFFC90FDAA22168C234C4C6628B80DC1CD1"
    "29024E088A67CC74020BBEA63B139B22514A08798E3404DD"
    "EF9519B3CD3A431B302B0A6DF25F14374FE1356D6D51C245"
    "E485B576625E7EC6F44C42E9A637ED6B0BFF5CB6F406B7ED"
    "EE386BFB5A899FA5AE9F24117C4B1FE649286651ECE45B3D"
    "C2007CB8A163BF0598DA48361C55D39A69163FA8FD24CF5F"
    "83655D23DCA3AD961C62F356208552BB9ED529077096966D"
    "670C354E4ABC9804F1746C08CA18217C32905E462E36CE3B"
    "E39E772C180E86039B2783A2EC07A28FB5C55DF06F4C52C9"
    "DE2BCBF6955817183995497CEA956AE515D2261898FA0510"
    "15728E5A8AAAC42DAD33170D04507A33A85521ABDF1CBA64"
    "ECFB850458DBEF0A8AEA71575D060C7DB3970F85A6E1E4C7"
    "ABF5AE8CDB0933D71E8C94E04A25619DCEE3D2261AD2EE6B"
    "F12FFA06D98A0864D87602733EC86A64521F2B18177B200C"
    "BBE117577A615D6C770988C0BAD946E208E24FA074E5AB31"
    "43DB5BFCE0FD108E4B82D120A93AD2CAFFFFFFFFFFFFFFFF";

static const char *SRP_g_HEX = "05";

struct Esp32SRPSession : public hap::platform::SRPSession {
  Esp32SRPSession(std::array<uint8_t, 16> s, std::vector<uint8_t> v,
                  std::string u, std::string p)
      : hap::platform::SRPSession(s, std::move(v), std::move(u), std::move(p)) {
  }

  std::vector<uint8_t> b;
  std::vector<uint8_t> B;
  std::vector<uint8_t> S;
  std::vector<uint8_t> K;
};

Esp32Crypto::Esp32Crypto() {}

void Esp32Crypto::sha512(std::span<const uint8_t> data,
                         std::span<uint8_t, 64> output) {
  mbedtls_sha512(data.data(), data.size(), output.data(), 0);
}

void Esp32Crypto::hkdf_sha512(std::span<const uint8_t> key,
                              std::span<const uint8_t> salt,
                              std::span<const uint8_t> info,
                              std::span<uint8_t> output) {
  mbedtls_hkdf(mbedtls_md_info_from_type(MBEDTLS_MD_SHA512), salt.data(),
               salt.size(), key.data(), key.size(), info.data(), info.size(),
               output.data(), output.size());
}

void Esp32Crypto::ed25519_generate_keypair(std::span<uint8_t, 32> public_key,
                                           std::span<uint8_t, 64> private_key) {
  crypto_sign_ed25519_keypair(public_key.data(), private_key.data());
}

void Esp32Crypto::ed25519_sign(std::span<const uint8_t, 64> private_key,
                               std::span<const uint8_t> message,
                               std::span<uint8_t, 64> signature) {
  unsigned long long sig_len;
  crypto_sign_ed25519_detached(signature.data(), &sig_len, message.data(),
                               message.size(), private_key.data());
}

bool Esp32Crypto::ed25519_verify(std::span<const uint8_t, 32> public_key,
                                 std::span<const uint8_t> message,
                                 std::span<const uint8_t, 64> signature) {
  return crypto_sign_ed25519_verify_detached(signature.data(), message.data(),
                                             message.size(),
                                             public_key.data()) == 0;
}

void Esp32Crypto::x25519_generate_keypair(std::span<uint8_t, 32> public_key,
                                          std::span<uint8_t, 32> private_key) {
  randombytes_buf(private_key.data(), 32);
  crypto_box_keypair(public_key.data(), private_key.data());
}

void Esp32Crypto::x25519_shared_secret(
    std::span<const uint8_t, 32> private_key,
    std::span<const uint8_t, 32> peer_public_key,
    std::span<uint8_t, 32> shared_secret) {
  int ret = crypto_scalarmult(shared_secret.data(), private_key.data(),
                              peer_public_key.data());
  (void)ret;
}

bool Esp32Crypto::chacha20_poly1305_encrypt_and_tag(
    std::span<const uint8_t, 32> key, std::span<const uint8_t, 12> nonce,
    std::span<const uint8_t> aad, std::span<const uint8_t> plaintext,
    std::span<uint8_t> ciphertext, std::span<uint8_t, 16> tag) {
  unsigned long long ciphertext_len;
  int ret = crypto_aead_chacha20poly1305_ietf_encrypt_detached(
      ciphertext.data(), tag.data(), &ciphertext_len, plaintext.data(),
      plaintext.size(), aad.data(), aad.size(), NULL, nonce.data(), key.data());
  return ret == 0;
}

bool Esp32Crypto::chacha20_poly1305_decrypt_and_verify(
    std::span<const uint8_t, 32> key, std::span<const uint8_t, 12> nonce,
    std::span<const uint8_t> aad, std::span<const uint8_t> ciphertext,
    std::span<const uint8_t, 16> tag, std::span<uint8_t> plaintext) {
  int ret = crypto_aead_chacha20poly1305_ietf_decrypt_detached(
      plaintext.data(), NULL, ciphertext.data(), ciphertext.size(), tag.data(),
      aad.data(), aad.size(), nonce.data(), key.data());
  return ret == 0;
}

// --- SRP Utilities ---

static void mpi_to_bytes(const mbedtls_mpi *x, std::vector<uint8_t> &out) {
  size_t len = mbedtls_mpi_size(x);
  out.resize(len);
  mbedtls_mpi_write_binary(x, out.data(), len);
}

static void mpi_to_bytes_pad(const mbedtls_mpi *x, std::vector<uint8_t> &out,
                             size_t pad_len) {
  out.resize(pad_len);
  size_t len = mbedtls_mpi_size(x);
  if (len > pad_len)
    len = pad_len;
  mbedtls_mpi_write_binary(x, out.data() + (pad_len - len), len);
}

static void hash_update(mbedtls_sha512_context *ctx,
                        std::span<const uint8_t> data) {
  mbedtls_sha512_update(ctx, data.data(), data.size());
}

static size_t count_leading_zeros(const uint8_t *data, size_t len) {
  size_t z = 0;
  while (z < len && data[z] == 0) {
    z++;
  }
  return z;
}
std::unique_ptr<hap::platform::SRPSession>
Esp32Crypto::srp_new_verifier(std::string_view username,
                              std::string_view password) {
  std::array<uint8_t, 16> s;
  randombytes_buf(s.data(), s.size());

  // x = H(s | H(I | ":" | P))
  std::vector<uint8_t> inner_hash(64);
  mbedtls_sha512_context ctx;
  mbedtls_sha512_init(&ctx);
  mbedtls_sha512_starts(&ctx, 0);
  mbedtls_sha512_update(&ctx, (const uint8_t *)username.data(),
                        username.size());
  mbedtls_sha512_update(&ctx, (const uint8_t *)":", 1);
  mbedtls_sha512_update(&ctx, (const uint8_t *)password.data(),
                        password.size());
  mbedtls_sha512_finish(&ctx, inner_hash.data());

  std::vector<uint8_t> x_hash(64);
  mbedtls_sha512_starts(&ctx, 0);
  mbedtls_sha512_update(&ctx, s.data(), s.size());
  mbedtls_sha512_update(&ctx, inner_hash.data(), inner_hash.size());
  mbedtls_sha512_finish(&ctx, x_hash.data());
  mbedtls_sha512_free(&ctx);

  // v = g^x mod N
  mbedtls_mpi N, g, x, v;
  mbedtls_mpi_init(&N);
  mbedtls_mpi_init(&g);
  mbedtls_mpi_init(&x);
  mbedtls_mpi_init(&v);

  int ret = mbedtls_mpi_read_string(&N, 16, SRP_N_HEX);
  if (ret != 0) {
    ESP_LOGE(TAG, "srp_new_verifier: Failed to parse SRP_N_HEX: %d", ret);
    mbedtls_mpi_free(&N);
    mbedtls_mpi_free(&g);
    mbedtls_mpi_free(&x);
    mbedtls_mpi_free(&v);
    return nullptr;
  }
  ret = mbedtls_mpi_read_string(&g, 16, SRP_g_HEX);
  if (ret != 0) {
    ESP_LOGE(TAG, "srp_new_verifier: Failed to parse SRP_g_HEX: %d", ret);
    mbedtls_mpi_free(&N);
    mbedtls_mpi_free(&g);
    mbedtls_mpi_free(&x);
    mbedtls_mpi_free(&v);
    return nullptr;
  }
  mbedtls_mpi_read_binary(&x, x_hash.data(), x_hash.size());

  ret = mbedtls_mpi_exp_mod(&v, &g, &x, &N, NULL);
  if (ret != 0) {
    ESP_LOGE(TAG, "srp_new_verifier: mbedtls_mpi_exp_mod failed: %d", ret);
    mbedtls_mpi_free(&N);
    mbedtls_mpi_free(&g);
    mbedtls_mpi_free(&x);
    mbedtls_mpi_free(&v);
    return nullptr;
  }

  std::vector<uint8_t> v_bytes;
  mpi_to_bytes(&v, v_bytes);
  ESP_LOGI(TAG, "SRP verifier generated: %zu bytes", v_bytes.size());

  mbedtls_mpi_free(&N);
  mbedtls_mpi_free(&g);
  mbedtls_mpi_free(&x);
  mbedtls_mpi_free(&v);

  return std::make_unique<Esp32SRPSession>(s, v_bytes, std::string(username),
                                           std::string(password));
}

std::array<uint8_t, 16>
Esp32Crypto::srp_get_salt(hap::platform::SRPSession *session) {
  return session->salt;
}

std::vector<uint8_t>
Esp32Crypto::srp_get_public_key(hap::platform::SRPSession *session) {
  auto ess = static_cast<Esp32SRPSession *>(session);

  ess->b.resize(32);
  randombytes_buf(ess->b.data(), ess->b.size());

  mbedtls_mpi N, g, v, b, B, k_mpi, tmp;
  mbedtls_mpi_init(&N);
  mbedtls_mpi_init(&g);
  mbedtls_mpi_init(&v);
  mbedtls_mpi_init(&b);
  mbedtls_mpi_init(&B);
  mbedtls_mpi_init(&k_mpi);
  mbedtls_mpi_init(&tmp);

  int ret = mbedtls_mpi_read_string(&N, 16, SRP_N_HEX);
  if (ret != 0) {
    ESP_LOGE(TAG, "Failed to parse SRP_N_HEX: %d", ret);
    return {};
  }
  ret = mbedtls_mpi_read_string(&g, 16, SRP_g_HEX);
  if (ret != 0) {
    ESP_LOGE(TAG, "Failed to parse SRP_g_HEX: %d", ret);
    return {};
  }
  mbedtls_mpi_read_binary(&v, ess->verifier.data(), ess->verifier.size());
  mbedtls_mpi_read_binary(&b, ess->b.data(), ess->b.size());

  // k = H(N | PAD(g))
  // RFC 5054 says PAD(g). N is 3072 bits (384 bytes).
  std::vector<uint8_t> N_bytes, g_bytes_padded;
  mpi_to_bytes(&N, N_bytes);
  mpi_to_bytes_pad(&g, g_bytes_padded, N_bytes.size()); // Pad g to N size

  std::vector<uint8_t> k_hash(64);
  mbedtls_sha512_context ctx;
  mbedtls_sha512_init(&ctx);
  mbedtls_sha512_starts(&ctx, 0);
  hash_update(&ctx, N_bytes);
  hash_update(&ctx, g_bytes_padded);
  mbedtls_sha512_finish(&ctx, k_hash.data());
  mbedtls_sha512_free(&ctx);

  mbedtls_mpi_read_binary(&k_mpi, k_hash.data(), k_hash.size());

  // B = (k*v + g^b) mod N
  // tmp = g^b mod N
  mbedtls_mpi_exp_mod(&tmp, &g, &b, &N, NULL);
  // B = k*v
  mbedtls_mpi_mul_mpi(&B, &k_mpi, &v);
  // B = B mod N (ADK does this before addition)
  mbedtls_mpi_mod_mpi(&B, &B, &N);
  // B = B + tmp
  mbedtls_mpi_add_mpi(&B, &B, &tmp);
  // B = B mod N
  mbedtls_mpi_mod_mpi(&B, &B, &N);

  mpi_to_bytes(&B, ess->B);
  ESP_LOGI(TAG, "SRP public key B generated: %zu bytes", ess->B.size());

  mbedtls_mpi_free(&N);
  mbedtls_mpi_free(&g);
  mbedtls_mpi_free(&v);
  mbedtls_mpi_free(&b);
  mbedtls_mpi_free(&B);
  mbedtls_mpi_free(&k_mpi);
  mbedtls_mpi_free(&tmp);

  session->server_public_key = ess->B;
  return ess->B;
}

bool Esp32Crypto::srp_set_client_public_key(
    hap::platform::SRPSession *session,
    std::span<const uint8_t> client_public) {
  session->client_public_key.assign(client_public.begin(), client_public.end());
  return true;
}

bool Esp32Crypto::srp_verify_client_proof(hap::platform::SRPSession *session,
                                          std::span<const uint8_t> proof) {
  auto ess = static_cast<Esp32SRPSession *>(session);

  mbedtls_mpi N, A, B, v, b, S, u_mpi;
  mbedtls_mpi_init(&N);
  mbedtls_mpi_init(&A);
  mbedtls_mpi_init(&B);
  mbedtls_mpi_init(&v);
  mbedtls_mpi_init(&b);
  mbedtls_mpi_init(&S);
  mbedtls_mpi_init(&u_mpi);

  mbedtls_mpi_read_string(&N, 16, SRP_N_HEX);
  mbedtls_mpi_read_binary(&A, session->client_public_key.data(),
                          session->client_public_key.size());
  mbedtls_mpi_read_binary(&B, ess->B.data(), ess->B.size());
  mbedtls_mpi_read_binary(&v, ess->verifier.data(), ess->verifier.size());
  mbedtls_mpi_read_binary(&b, ess->b.data(), ess->b.size());

  // u = H(PAD(A) | PAD(B))
  std::vector<uint8_t> A_pad, B_pad;
  size_t param_len = mbedtls_mpi_size(&N);
  mpi_to_bytes_pad(&A, A_pad, param_len);
  mpi_to_bytes_pad(&B, B_pad, param_len);

  std::vector<uint8_t> u_hash(64);
  mbedtls_sha512_context ctx;
  mbedtls_sha512_init(&ctx);
  mbedtls_sha512_starts(&ctx, 0);
  hash_update(&ctx, A_pad);
  hash_update(&ctx, B_pad);
  mbedtls_sha512_finish(&ctx, u_hash.data());

  mbedtls_mpi_read_binary(&u_mpi, u_hash.data(), u_hash.size());

  // S = (A * v^u)^b mod N
  // tmp = v^u mod N
  mbedtls_mpi tmp;
  mbedtls_mpi_init(&tmp);
  mbedtls_mpi_exp_mod(&tmp, &v, &u_mpi, &N, NULL);
  // tmp = A * tmp
  mbedtls_mpi_mul_mpi(&tmp, &A, &tmp);
  // S = tmp^b mod N
  mbedtls_mpi_exp_mod(&S, &tmp, &b, &N, NULL);
  mbedtls_mpi_free(&tmp);

  mpi_to_bytes(&S, ess->S);

  // K = H(S) where S has leading zeros stripped (HAP spec)
  size_t z_S = count_leading_zeros(ess->S.data(), ess->S.size());
  ess->K.resize(64);
  mbedtls_sha512_starts(&ctx, 0);
  mbedtls_sha512_update(&ctx, ess->S.data() + z_S, ess->S.size() - z_S);
  mbedtls_sha512_finish(&ctx, ess->K.data());

  // M1 = H(H(N) xor H(g) | H(I) | s | A | B | K)
  // HAP Spec: A and B are padded to 384 bytes (SRP_PUBLIC_KEY_BYTES)
  // but leading zeros are stripped before hashing (HAP SRP flag)
  std::vector<uint8_t> hN(64), hg(64), hI(64);
  std::vector<uint8_t> N_bytes, g_bytes;
  mbedtls_mpi g_mpi;
  mbedtls_mpi_init(&g_mpi);
  mbedtls_mpi_read_string(&g_mpi, 16, SRP_g_HEX);
  mpi_to_bytes(&N, N_bytes);
  mpi_to_bytes(&g_mpi, g_bytes);

  mbedtls_sha512_starts(&ctx, 0);
  hash_update(&ctx, N_bytes);
  mbedtls_sha512_finish(&ctx, hN.data());
  mbedtls_sha512_starts(&ctx, 0);
  hash_update(&ctx, g_bytes);
  mbedtls_sha512_finish(&ctx, hg.data());
  mbedtls_sha512_starts(&ctx, 0);
  mbedtls_sha512_update(&ctx, (const uint8_t *)session->username.data(),
                        session->username.size());
  mbedtls_sha512_finish(&ctx, hI.data());

  for (size_t i = 0; i < 64; i++)
    hN[i] ^= hg[i];

  std::vector<uint8_t> A_padded, B_padded;
  size_t param_size = 384;
  mpi_to_bytes_pad(&A, A_padded, param_size);
  mpi_to_bytes_pad(&B, B_padded, param_size);

  size_t z_A = count_leading_zeros(A_padded.data(), A_padded.size());
  size_t z_B = count_leading_zeros(B_padded.data(), B_padded.size());

  std::vector<uint8_t> M1_calc(64);
  mbedtls_sha512_starts(&ctx, 0);
  hash_update(&ctx, hN);
  hash_update(&ctx, hI);
  hash_update(&ctx, ess->salt);
  mbedtls_sha512_update(&ctx, A_padded.data() + z_A, A_padded.size() - z_A);
  mbedtls_sha512_update(&ctx, B_padded.data() + z_B, B_padded.size() - z_B);
  hash_update(&ctx, ess->K);
  mbedtls_sha512_finish(&ctx, M1_calc.data());

  if (proof.size() != M1_calc.size() ||
      std::memcmp(proof.data(), M1_calc.data(), proof.size()) != 0) {
    ESP_LOGE(TAG, "SRP Client Proof Failed");
    mbedtls_mpi_free(&N);
    mbedtls_mpi_free(&A);
    mbedtls_mpi_free(&B);
    mbedtls_mpi_free(&v);
    mbedtls_mpi_free(&b);
    mbedtls_mpi_free(&S);
    mbedtls_mpi_free(&u_mpi);
    mbedtls_mpi_free(&g_mpi);
    mbedtls_sha512_free(&ctx);
    return false;
  }

  // M2 = H(A | M1 | K)
  ess->M2.resize(64);
  mbedtls_sha512_starts(&ctx, 0);
  hash_update(&ctx, A_padded);
  hash_update(&ctx, M1_calc);
  hash_update(&ctx, ess->K);
  mbedtls_sha512_finish(&ctx, ess->M2.data());

  ESP_LOGI(TAG, "SRP Debug - M2 Summary:");
  ESP_LOGI(TAG, "Client Public Key (Received): %d bytes",
           (int)session->client_public_key.size());
  ESP_LOG_BUFFER_HEX(TAG, session->client_public_key.data(),
                     std::min((size_t)64, session->client_public_key.size()));
  ESP_LOGI(TAG, "A_padded: %d bytes", (int)A_padded.size());
  ESP_LOG_BUFFER_HEX(TAG, A_padded.data(),
                     std::min((size_t)64, A_padded.size()));
  ESP_LOGI(TAG, "M1_calc: %d bytes", (int)M1_calc.size());
  ESP_LOG_BUFFER_HEX(TAG, M1_calc.data(), M1_calc.size());
  ESP_LOGI(TAG, "K: %d bytes", (int)ess->K.size());
  ESP_LOG_BUFFER_HEX(TAG, ess->K.data(), ess->K.size());
  ESP_LOGI(TAG, "M2 (Result): %d bytes", (int)ess->M2.size());
  ESP_LOG_BUFFER_HEX(TAG, ess->M2.data(), ess->M2.size());

  mbedtls_mpi_free(&N);
  mbedtls_mpi_free(&A);
  mbedtls_mpi_free(&B);
  mbedtls_mpi_free(&v);
  mbedtls_mpi_free(&b);
  mbedtls_mpi_free(&S);
  mbedtls_mpi_free(&u_mpi);
  mbedtls_mpi_free(&g_mpi);
  mbedtls_sha512_free(&ctx);

  return true;
}

std::vector<uint8_t>
Esp32Crypto::srp_get_server_proof(hap::platform::SRPSession *session) {
  return session->M2;
}

std::vector<uint8_t>
Esp32Crypto::srp_get_session_key(hap::platform::SRPSession *session) {
  auto ess = static_cast<Esp32SRPSession *>(session);
  return ess->K;
}
