#include "crypto_context.h"

#include <algorithm>
#include <cstring>
#include <esp_random.h>
#include <mbedtls/sha256.h>
#include "esphome/core/log.h"

namespace esphome {
namespace openhaystack {

static const char *const TAG_CRYPTO = "openhaystack.crypto";

CryptoContext::CryptoContext() {
  mbedtls_ecp_group_init(&this->group_);
  mbedtls_ecp_point_init(&this->point_);
  mbedtls_mpi_init(&this->private_mpi_);
}

CryptoContext::~CryptoContext() {
  mbedtls_ecp_group_free(&this->group_);
  mbedtls_ecp_point_free(&this->point_);
  mbedtls_mpi_free(&this->private_mpi_);
}

int CryptoContext::esp_random_callback_(void *, unsigned char *output, size_t output_len) {
  esp_fill_random(output, output_len);
  return 0;
}

bool CryptoContext::ensure_initialized_() {
  if (this->ready_)
    return true;

  int ret = mbedtls_ecp_group_load(&this->group_, MBEDTLS_ECP_DP_SECP224R1);
  if (ret != 0) {
    ESP_LOGE(TAG_CRYPTO, "mbedtls_ecp_group_load failed: %d", ret);
    mbedtls_ecp_group_free(&this->group_);
    mbedtls_ecp_group_init(&this->group_);
    return false;
  }

  this->ready_ = true;
  return true;
}

void CryptoContext::wipe_point_() {
  (void) mbedtls_mpi_lset(&this->point_.MBEDTLS_PRIVATE(X), 0);
  (void) mbedtls_mpi_lset(&this->point_.MBEDTLS_PRIVATE(Y), 0);
  (void) mbedtls_mpi_lset(&this->point_.MBEDTLS_PRIVATE(Z), 0);
}

bool CryptoContext::derive_public_key(const uint8_t *private_key,
                                      size_t private_key_len,
                                      uint8_t *advertising_key,
                                      size_t advertising_key_len) {
  if (private_key == nullptr || advertising_key == nullptr)
    return false;

  if (!this->ensure_initialized_()) {
    std::memset(advertising_key, 0, advertising_key_len);
    return false;
  }

  bool ok = true;
  int ret;

  do {
    ret = mbedtls_mpi_read_binary(&this->private_mpi_, private_key, private_key_len);
    if (ret != 0) {
      ESP_LOGE(TAG_CRYPTO, "mbedtls_mpi_read_binary failed: %d", ret);
      ok = false;
      break;
    }

    ret = mbedtls_ecp_check_privkey(&this->group_, &this->private_mpi_);
    if (ret != 0) {
      ESP_LOGE(TAG_CRYPTO, "mbedtls_ecp_check_privkey failed: %d", ret);
      ok = false;
      break;
    }

    ret = mbedtls_ecp_mul(&this->group_, &this->point_, &this->private_mpi_, &this->group_.G,
                          &CryptoContext::esp_random_callback_, nullptr);
    if (ret != 0) {
      ESP_LOGE(TAG_CRYPTO, "mbedtls_ecp_mul failed: %d", ret);
      ok = false;
      break;
    }

    ret = mbedtls_mpi_write_binary(&this->point_.MBEDTLS_PRIVATE(X), advertising_key, advertising_key_len);
    if (ret != 0) {
      ESP_LOGE(TAG_CRYPTO, "mbedtls_mpi_write_binary failed: %d", ret);
      ok = false;
      break;
    }
  } while (false);

  (void) mbedtls_mpi_lset(&this->private_mpi_, 0);
  this->wipe_point_();

  if (!ok)
    std::memset(advertising_key, 0, advertising_key_len);

  return ok;
}

bool CryptoContext::calculate_derived_private_key(const uint8_t *shared_data,
                                                  size_t shared_data_len,
                                                  const uint8_t *master_private_key,
                                                  size_t master_private_key_len,
                                                  uint8_t *out_private_key,
                                                  size_t out_private_key_len) {
  if (shared_data == nullptr || master_private_key == nullptr || out_private_key == nullptr)
    return false;

  if (shared_data_len == 0 || master_private_key_len == 0 || out_private_key_len == 0)
    return false;

  if (!this->ensure_initialized_()) {
    std::memset(out_private_key, 0, out_private_key_len);
    return false;
  }

  if ((shared_data_len % 2U) != 0U) {
    return false;
  }

  mbedtls_mpi order;
  mbedtls_mpi order_minus_one;
  mbedtls_mpi ui;
  mbedtls_mpi vi;
  mbedtls_mpi d0;
  mbedtls_mpi di;
  mbedtls_mpi tmp;

  mbedtls_mpi_init(&order);
  mbedtls_mpi_init(&order_minus_one);
  mbedtls_mpi_init(&ui);
  mbedtls_mpi_init(&vi);
  mbedtls_mpi_init(&d0);
  mbedtls_mpi_init(&di);
  mbedtls_mpi_init(&tmp);

  bool ok = true;
  int ret;

  do {
    ret = mbedtls_mpi_copy(&order, &this->group_.N);
    if (ret != 0) {
      ESP_LOGE(TAG_CRYPTO, "mbedtls_mpi_copy(order) failed: %d", ret);
      ok = false;
      break;
    }

    ret = mbedtls_mpi_copy(&order_minus_one, &this->group_.N);
    if (ret != 0) {
      ESP_LOGE(TAG_CRYPTO, "mbedtls_mpi_copy(order_minus_one) failed: %d", ret);
      ok = false;
      break;
    }

    ret = mbedtls_mpi_sub_int(&order_minus_one, &order_minus_one, 1);
    if (ret != 0) {
      ESP_LOGE(TAG_CRYPTO, "mbedtls_mpi_sub_int failed: %d", ret);
      ok = false;
      break;
    }

    size_t half = shared_data_len / 2;
    ret = mbedtls_mpi_read_binary(&ui, shared_data, half);
    if (ret != 0) {
      ESP_LOGE(TAG_CRYPTO, "mbedtls_mpi_read_binary(u_i) failed: %d", ret);
      ok = false;
      break;
    }
    ret = mbedtls_mpi_mod_mpi(&ui, &ui, &order_minus_one);
    if (ret != 0) {
      ESP_LOGE(TAG_CRYPTO, "mbedtls_mpi_mod_mpi(u_i) failed: %d", ret);
      ok = false;
      break;
    }
    ret = mbedtls_mpi_add_int(&ui, &ui, 1);
    if (ret != 0) {
      ESP_LOGE(TAG_CRYPTO, "mbedtls_mpi_add_int(u_i) failed: %d", ret);
      ok = false;
      break;
    }

    ret = mbedtls_mpi_read_binary(&vi, shared_data + half, half);
    if (ret != 0) {
      ESP_LOGE(TAG_CRYPTO, "mbedtls_mpi_read_binary(v_i) failed: %d", ret);
      ok = false;
      break;
    }
    ret = mbedtls_mpi_mod_mpi(&vi, &vi, &order_minus_one);
    if (ret != 0) {
      ESP_LOGE(TAG_CRYPTO, "mbedtls_mpi_mod_mpi(v_i) failed: %d", ret);
      ok = false;
      break;
    }
    ret = mbedtls_mpi_add_int(&vi, &vi, 1);
    if (ret != 0) {
      ESP_LOGE(TAG_CRYPTO, "mbedtls_mpi_add_int(v_i) failed: %d", ret);
      ok = false;
      break;
    }

    ret = mbedtls_mpi_read_binary(&d0, master_private_key, master_private_key_len);
    if (ret != 0) {
      ESP_LOGE(TAG_CRYPTO, "mbedtls_mpi_read_binary(d0) failed: %d", ret);
      ok = false;
      break;
    }

    ret = mbedtls_mpi_mul_mpi(&tmp, &d0, &ui);
    if (ret != 0) {
      ESP_LOGE(TAG_CRYPTO, "mbedtls_mpi_mul_mpi failed: %d", ret);
      ok = false;
      break;
    }
    ret = mbedtls_mpi_add_mpi(&di, &tmp, &vi);
    if (ret != 0) {
      ESP_LOGE(TAG_CRYPTO, "mbedtls_mpi_add_mpi failed: %d", ret);
      ok = false;
      break;
    }

    ret = mbedtls_mpi_mod_mpi(&di, &di, &order);
    if (ret != 0) {
      ESP_LOGE(TAG_CRYPTO, "mbedtls_mpi_mod_mpi(d_i) failed: %d", ret);
      ok = false;
      break;
    }

    if (mbedtls_mpi_cmp_int(&di, 0) == 0) {
      ret = mbedtls_mpi_add_int(&di, &di, 1);
      if (ret != 0) {
        ESP_LOGE(TAG_CRYPTO, "mbedtls_mpi_add_int(d_i) failed: %d", ret);
        ok = false;
        break;
      }
    }

    ret = mbedtls_mpi_write_binary(&di, out_private_key, out_private_key_len);
    if (ret != 0) {
      ESP_LOGE(TAG_CRYPTO, "mbedtls_mpi_write_binary(d_i) failed: %d", ret);
      ok = false;
      break;
    }
  } while (false);

  if (!ok)
    std::memset(out_private_key, 0, out_private_key_len);

  (void) mbedtls_mpi_lset(&tmp, 0);
  (void) mbedtls_mpi_lset(&di, 0);
  (void) mbedtls_mpi_lset(&d0, 0);
  (void) mbedtls_mpi_lset(&vi, 0);
  (void) mbedtls_mpi_lset(&ui, 0);

  mbedtls_mpi_free(&tmp);
  mbedtls_mpi_free(&di);
  mbedtls_mpi_free(&d0);
  mbedtls_mpi_free(&vi);
  mbedtls_mpi_free(&ui);
  mbedtls_mpi_free(&order_minus_one);
  mbedtls_mpi_free(&order);

  return ok;
}

bool CryptoContext::compute_master_key_digest(const uint8_t *master_private_key,
                                              size_t master_private_key_len,
                                              const uint8_t *master_symmetric_key,
                                              size_t master_symmetric_key_len,
                                              uint8_t *digest,
                                              size_t digest_len) {
  if (master_private_key == nullptr || master_symmetric_key == nullptr || digest == nullptr)
    return false;

  if (digest_len < 16U)
    return false;

  unsigned char full_digest[32] = {0};
  mbedtls_sha256_context sha;
  mbedtls_sha256_init(&sha);

  bool ok = true;
  do {
    if (mbedtls_sha256_starts(&sha, 0) != 0) {
      ok = false;
      break;
    }
    if (mbedtls_sha256_update(&sha, master_private_key, master_private_key_len) != 0) {
      ok = false;
      break;
    }
    if (mbedtls_sha256_update(&sha, master_symmetric_key, master_symmetric_key_len) != 0) {
      ok = false;
      break;
    }
    if (mbedtls_sha256_finish(&sha, full_digest) != 0) {
      ok = false;
      break;
    }
  } while (false);

  mbedtls_sha256_free(&sha);

  if (!ok) {
    std::fill(full_digest, full_digest + sizeof(full_digest), 0);
    return false;
  }

  size_t copy_len = std::min(digest_len, sizeof(full_digest));
  std::memcpy(digest, full_digest, copy_len);
  std::fill(full_digest, full_digest + sizeof(full_digest), 0);
  return true;
}

bool CryptoContext::kdf(const uint8_t *input,
                        size_t input_length,
                        const char *label,
                        uint8_t *output,
                        size_t output_length) {
  if (input == nullptr || output == nullptr || label == nullptr)
    return false;

  size_t label_length = std::strlen(label);
  if (label_length == 0)
    return false;

  uint32_t counter = 1;
  size_t produced = 0;

  while (produced < output_length) {
    unsigned char digest[32] = {0};
    mbedtls_sha256_context sha;
    mbedtls_sha256_init(&sha);

    mbedtls_sha256_starts(&sha, 0);
    mbedtls_sha256_update(&sha, input, input_length);

    unsigned char counter_bytes[4] = {
        static_cast<uint8_t>((counter >> 24) & 0xFF),
        static_cast<uint8_t>((counter >> 16) & 0xFF),
        static_cast<uint8_t>((counter >> 8) & 0xFF),
        static_cast<uint8_t>(counter & 0xFF),
    };
    mbedtls_sha256_update(&sha, counter_bytes, sizeof(counter_bytes));
    mbedtls_sha256_update(&sha, reinterpret_cast<const unsigned char *>(label), label_length);

    mbedtls_sha256_finish(&sha, digest);
    mbedtls_sha256_free(&sha);

    size_t copy_len = std::min(static_cast<size_t>(sizeof(digest)), output_length - produced);
    std::memcpy(output + produced, digest, copy_len);
    produced += copy_len;
    counter++;
    std::fill(digest, digest + sizeof(digest), 0);
  }

  return true;
}

}  // namespace openhaystack
}  // namespace esphome
