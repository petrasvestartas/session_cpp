#pragma once

#ifdef _WIN32
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif
#endif

#include <array>
#include <cstdint>
#include <cstring>
#include <random>
#include <string>

namespace UUIDv4 {

// Lookup table for fast hex conversion (avoiding ostringstream)
static constexpr char hex_chars[16] = {
    '0', '1', '2', '3', '4', '5', '6', '7',
    '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'
};

class UUID {
public:
  UUID() { std::fill(data.begin(), data.end(), static_cast<uint8_t>(0)); }

  UUID(const UUID &other) : data(other.data) {}

  UUID &operator=(const UUID &other) {
    if (this != &other) {
      data = other.data;
    }
    return *this;
  }

  // Generate a random UUID v4 - optimized to use 64-bit generation
  static UUID generate() {
    // Use thread_local to initialize RNG only once per thread
    static thread_local std::mt19937_64 gen(std::random_device{}());

    UUID uuid;
    // Generate 128 bits using two 64-bit random numbers (much faster than 16 x 8-bit)
    uint64_t r1 = gen();
    uint64_t r2 = gen();
    std::memcpy(uuid.data.data(), &r1, 8);
    std::memcpy(uuid.data.data() + 8, &r2, 8);

    // Set version (4) and variant bits according to RFC 4122
    uuid.data[6] = (uuid.data[6] & 0x0F) | 0x40; // Version 4
    uuid.data[8] = (uuid.data[8] & 0x3F) | 0x80; // Variant 10

    return uuid;
  }

  // Parse UUID from string
  static UUID fromStrFactory(const std::string &str) {
    return fromStrFactory(str.c_str());
  }

  static UUID fromStrFactory(const char *str) {
    UUID uuid;

    int byte_idx = 0;
    for (int i = 0; str[i] != '\0' && byte_idx < 16; ++i) {
      if (str[i] == '-')
        continue;

      // Parse two hex characters
      char hex_pair[3] = {str[i], str[i + 1], '\0'};
      if (str[i + 1] == '\0')
        break; // Incomplete pair

      unsigned int byte_val;
      if (std::sscanf(hex_pair, "%x", &byte_val) == 1) {
        uuid.data[byte_idx++] = static_cast<uint8_t>(byte_val);
        ++i; // Skip the second character
      }
    }

    return uuid;
  }

  // Convert to string representation - optimized with lookup table (no ostringstream)
  std::string str() const {
    // UUID string format: xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx (36 chars)
    std::string result(36, '\0');
    int pos = 0;

    for (int i = 0; i < 16; ++i) {
      if (i == 4 || i == 6 || i == 8 || i == 10) {
        result[pos++] = '-';
      }
      result[pos++] = hex_chars[(data[i] >> 4) & 0x0F];
      result[pos++] = hex_chars[data[i] & 0x0F];
    }

    return result;
  }

  void str(std::string &s) const { s = str(); }

  void str(char *res) const {
    // Direct write to char buffer - even faster
    int pos = 0;
    for (int i = 0; i < 16; ++i) {
      if (i == 4 || i == 6 || i == 8 || i == 10) {
        res[pos++] = '-';
      }
      res[pos++] = hex_chars[(data[i] >> 4) & 0x0F];
      res[pos++] = hex_chars[data[i] & 0x0F];
    }
    res[pos] = '\0';
  }

  // Get raw bytes
  std::string bytes() const {
    return std::string(reinterpret_cast<const char *>(data.data()), 16);
  }

  void bytes(std::string &out) const { out = bytes(); }

  void bytes(char *bytes_out) const { std::memcpy(bytes_out, data.data(), 16); }

  // Comparison operators
  friend bool operator==(const UUID &lhs, const UUID &rhs) {
    return lhs.data == rhs.data;
  }

  friend bool operator!=(const UUID &lhs, const UUID &rhs) {
    return !(lhs == rhs);
  }

  friend bool operator<(const UUID &lhs, const UUID &rhs) {
    return lhs.data < rhs.data;
  }

  friend bool operator>(const UUID &lhs, const UUID &rhs) { return rhs < lhs; }

  friend bool operator<=(const UUID &lhs, const UUID &rhs) {
    return !(lhs > rhs);
  }

  friend bool operator>=(const UUID &lhs, const UUID &rhs) {
    return !(lhs < rhs);
  }

  // Stream operators
  friend std::ostream &operator<<(std::ostream &stream, const UUID &uuid) {
    return stream << uuid.str();
  }

  friend std::istream &operator>>(std::istream &stream, UUID &uuid) {
    std::string s;
    stream >> s;
    uuid = fromStrFactory(s);
    return stream;
  }

  // Hash function
  size_t hash() const {
    size_t h1 =
        std::hash<uint64_t>{}(*reinterpret_cast<const uint64_t *>(data.data()));
    size_t h2 = std::hash<uint64_t>{}(
        *reinterpret_cast<const uint64_t *>(data.data() + 8));
    return h1 ^ (h2 << 1);
  }

private:
  std::array<uint8_t, 16> data;
};

// UUID Generator class for compatibility
template <typename RNG> class UUIDGenerator {
public:
  UUIDGenerator() {}
  UUIDGenerator(uint64_t seed) : gen(seed) {}

  UUID getUUID() { return UUID::generate(); }

private:
  RNG gen;
};

} // namespace UUIDv4

// Convenience function for simple UUID generation
inline std::string guid() { return UUIDv4::UUID::generate().str(); }

// Hash specialization for std::unordered_map support
namespace std {
template <> struct hash<UUIDv4::UUID> {
  size_t operator()(const UUIDv4::UUID &uuid) const { return uuid.hash(); }
};
} // namespace std
