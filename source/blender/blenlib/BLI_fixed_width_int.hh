/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <immintrin.h>
#include <iostream>

#include <gmpxx.h>

#include "BLI_string_ref.hh"
#include "BLI_utildefines.h"

namespace blender::fixed_width_int {

struct MyUint16 {
  uint8_t low;
  uint8_t high;

  MyUint16(uint16_t a) : low(a), high(a >> 8) {}
};

inline std::ostream &operator<<(std::ostream &stream, const MyUint16 &a)
{
  const uint16_t result = (uint16_t(a.high) << 8) + a.low;
  return stream << result;
}

inline MyUint16 operator+(const MyUint16 &a, const MyUint16 &b)
{
  MyUint16 result{0};
  result.low = a.low + b.low;
  const bool has_overflow = (result.low < a.low || result.low < b.low);
  result.high = uint8_t(has_overflow) + a.high + b.high;
  return result;
}

inline MyUint16 operator-(const MyUint16 &a, const MyUint16 &b)
{
  MyUint16 result{0};
  result.low = a.low - b.low;
  const bool has_underflow = result.low > a.low;
  result.high = a.high - b.high - uint8_t(has_underflow);
  return result;
}

inline MyUint16 operator*(const MyUint16 &a, const MyUint16 &b)
{
  MyUint16 r1{0};
  r1.low = a.low * b.low;
  const uint8_t overflow1 = (uint16_t(a.low) * uint16_t(b.low)) >> 8;
  r1.high = a.low * b.high + overflow1;
  MyUint16 r2{0};
  r2.low = 0;
  r2.high = a.high * b.low;
  return r1 + r2;
}

struct UInt128_32 {
  uint32_t v1;
  uint32_t v2;
  uint32_t v3;
  uint32_t v4;

  UInt128_32() = default;

  UInt128_32(const StringRefNull str, const int base = 10)
  {
    mpz_t a;
    mpz_init(a);
    mpz_set_str(a, str.c_str(), base);
    for (uint32_t *v : {&v1, &v2, &v3, &v4}) {
      *v = mpz_get_ui(a);
      mpz_div_2exp(a, a, 32);
    }
    mpz_clear(a);
  }

  void print() const
  {
    mpz_t a;
    mpz_init(a);
    for (const uint32_t v : {v4, v3, v2, v1}) {
      mpz_mul_2exp(a, a, 32);
      mpz_add_ui(a, a, v);
    }
    mpz_out_str(stdout, 10, a);
    mpz_clear(a);
    std::cout << "\n";
  }
};

using UInt128 = UInt128_32;

template<typename T, typename T2, int S> inline void generic_add(T *dst, const T *a, const T *b)
{
  constexpr int shift = 8 * sizeof(T);
  const T2 r0 = T2(a[0]) + T2(b[0]);
  dst[0] = T(r0);
  T2 carry = r0 >> shift;
  for (int i = 1; i < S - 1; i++) {
    const T2 ri = T2(a[i]) + T2(b[i]) + carry;
    dst[i] = T(ri);
    carry = ri >> shift;
  }
  dst[S - 1] = T(T2(a[S - 1]) + T2(b[S - 1]) + carry);
}

inline UInt128_32 operator+(const UInt128_32 &a, const UInt128_32 &b)
{
  UInt128_32 result;
  generic_add<uint32_t, uint64_t, 4>(&result.v1, &a.v1, &b.v1);
  return result;

  const uint64_t r1 = uint64_t(a.v1) + uint64_t(b.v1);
  const uint64_t r2 = uint64_t(a.v2) + uint64_t(b.v2) + (r1 >> 32);
  const uint64_t r3 = uint64_t(a.v3) + uint64_t(b.v3) + (r2 >> 32);
  const uint64_t r4 = uint64_t(a.v4) + uint64_t(b.v4) + (r3 >> 32);

  result.v1 = uint32_t(r1);
  result.v2 = uint32_t(r2);
  result.v3 = uint32_t(r3);
  result.v4 = uint32_t(r4);
  return result;
}

inline UInt128_32 operator*(const UInt128_32 &a, const uint32_t b)
{
  const uint64_t r1 = a.v1 * b;
  const uint64_t r2 = a.v2 * b + (r1 >> 32);
  const uint64_t r3 = a.v3 * b + (r2 >> 32);
  const uint64_t r4 = a.v4 * b + (r3 >> 32);

  UInt128_32 result;
  result.v1 = uint32_t(r1);
  result.v2 = uint32_t(r2);
  result.v2 = uint32_t(r3);
  result.v4 = uint32_t(r4);
  return result;
}

template<typename T, typename T2, int S> inline void generic_mul(T *dst, const T *a, const T *b)
{
  constexpr int shift = 8 * sizeof(T);

  T2 r[S] = {};

  for (int i = 0; i < S; i++) {
    const T2 bi = T2(b[i]);
    T2 carry = 0;
    for (int j = 0; j < S - i; j++) {
      const T2 rji = T2(a[j]) * bi + carry;
      carry = rji >> shift;
      r[i + j] += T2(T(rji));
    }
  }

  T2 carry = 0;
  for (int i = 0; i < S; i++) {
    const T2 ri = r[i] + carry;
    carry = ri >> shift;
    dst[i] = T(ri);
  }
}

inline UInt128_32 operator*(const UInt128_32 &a, const UInt128_32 &b)
{
  UInt128_32 result;
  generic_mul<uint32_t, uint64_t, 4>(&result.v1, &a.v1, &b.v1);
  return result;

  const uint64_t r1_1 = uint64_t(a.v1) * uint64_t(b.v1);
  const uint64_t r1_2 = uint64_t(a.v2) * uint64_t(b.v1) + (r1_1 >> 32);
  const uint64_t r1_3 = uint64_t(a.v3) * uint64_t(b.v1) + (r1_2 >> 32);
  const uint64_t r1_4 = uint64_t(a.v4) * uint64_t(b.v1) + (r1_3 >> 32);

  const uint64_t r2_2 = uint64_t(a.v1) * uint64_t(b.v2);
  const uint64_t r2_3 = uint64_t(a.v2) * uint64_t(b.v2) + (r2_2 >> 32);
  const uint64_t r2_4 = uint64_t(a.v3) * uint64_t(b.v2) + (r2_3 >> 32);

  const uint64_t r3_3 = uint64_t(a.v1) * uint64_t(b.v3);
  const uint64_t r3_4 = uint64_t(a.v2) * uint64_t(b.v3) + (r3_3 >> 32);

  const uint64_t r4_4 = uint64_t(a.v1) * uint64_t(b.v4);

  const uint64_t r1 = uint64_t(uint32_t(r1_1));
  const uint64_t r2 = uint64_t(uint32_t(r1_2)) + uint64_t(uint32_t(r2_2)) + (r1 >> 32);
  const uint64_t r3 = uint64_t(uint32_t(r1_3)) + uint64_t(uint32_t(r2_3)) +
                      uint64_t(uint32_t(r3_3)) + (r2 >> 32);
  const uint64_t r4 = uint64_t(uint32_t(r1_4)) + uint64_t(uint32_t(r2_4)) +
                      uint64_t(uint32_t(r3_4)) + uint64_t(uint32_t(r4_4)) + (r3 >> 32);

  result.v1 = uint32_t(r1);
  result.v2 = uint32_t(r2);
  result.v3 = uint32_t(r3);
  result.v4 = uint32_t(r4);
  return result;
}

}  // namespace blender::fixed_width_int
