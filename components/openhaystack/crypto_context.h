#pragma once

#include <cstddef>
#include <cstdint>

#define MBEDTLS_ALLOW_PRIVATE_ACCESS
#include <mbedtls/bignum.h>
#include <mbedtls/ecp.h>

namespace esphome {
namespace openhaystack {

class CryptoContext {
 public:
  CryptoContext();
  ~CryptoContext();
  CryptoContext(const CryptoContext &) = delete;
  CryptoContext &operator=(const CryptoContext &) = delete;
  CryptoContext(CryptoContext &&) = delete;
  CryptoContext &operator=(CryptoContext &&) = delete;

  bool derive_public_key(const uint8_t *private_key,
                         size_t private_key_len,
                         uint8_t *advertising_key,
                         size_t advertising_key_len);
  bool calculate_derived_private_key(const uint8_t *shared_data,
                                     size_t shared_data_len,
                                     const uint8_t *master_private_key,
                                     size_t master_private_key_len,
                                     uint8_t *out_private_key,
                                     size_t out_private_key_len);
  static bool compute_master_key_digest(const uint8_t *master_private_key,
                                        size_t master_private_key_len,
                                        const uint8_t *master_symmetric_key,
                                        size_t master_symmetric_key_len,
                                        uint8_t *digest,
                                        size_t digest_len);
  static bool kdf(const uint8_t *input,
                  size_t input_length,
                  const char *label,
                  uint8_t *output,
                  size_t output_length);

 private:
  static int esp_random_callback_(void *, unsigned char *output, size_t output_len);
  bool ensure_initialized_();
  void wipe_point_();

  bool ready_ = false;
  mbedtls_ecp_group group_;
  mbedtls_ecp_point point_;
  mbedtls_mpi private_mpi_;
};

}  // namespace openhaystack
}  // namespace esphome
