/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <immintrin.h>
#include <iostream>

#include <gmpxx.h>

#include "BLI_string_ref.hh"
#include "BLI_utildefines.h"

namespace blender::fixed_width_int {

template<typename T, int S> struct UIntF {
  static_assert(std::is_unsigned_v<T>);
  static_assert(S >= 1);

  std::array<T, S> v;

  UIntF() = default;

  UIntF(const char *str, const int base = 10)
  {
    mpz_t x;
    mpz_init(x);
    mpz_set_str(x, str, base);
    for (int i = 0; i < S; i++) {
      static_assert(sizeof(T) <= sizeof(decltype(mpz_get_ui(x))));
      this->v[i] = T(mpz_get_ui(x));
      mpz_div_2exp(x, x, 8 * sizeof(T));
    }
    mpz_clear(x);
  }

  void print(const int base = 10) const
  {
    mpz_t x;
    mpz_init(x);
    for (int i = S - 1; i >= 0; i--) {
      static_assert(sizeof(T) <= sizeof(decltype(mpz_get_ui(x))));
      mpz_mul_2exp(x, x, 8 * sizeof(T));
      mpz_add_ui(x, x, this->v[i]);
    }
    mpz_out_str(stdout, base, x);
    mpz_clear(x);
    std::cout << "\n";
  }
};

template<typename T, int S> struct IntF {
  static_assert(std::is_unsigned_v<T>);
  static_assert(S >= 1);

  std::array<T, S> v;

  IntF() = default;

  IntF(const char *str, const int base = 10)
  {
    if (str[0] == '-') {
      const UIntF<T, S> unsigned_value(str + 1, base);
      const UIntF<T, S> zero_value("0");
      const UIntF<T, S> result = zero_value - unsigned_value;
      this->v = result.v;
    }
    else {
      const UIntF<T, S> unsigned_value(str, base);
      this->v = unsigned_value.v;
    }
  }

  void print(const int base = 10) const
  {
    const bool is_negative = (this->v[S - 1] & (T(1) << (8 * sizeof(T) - 1))) != 0;
    if (is_negative) {
      std::cout << "-";
      UIntF<T, S> negated;
      negated.v = this->v;
      for (int i = 0; i < S; i++) {
        negated.v[i] = ~negated.v[i];
      }
      negated = negated + UIntF<T, S>("1");
      negated.print(base);
    }
    else {
      UIntF<T, S> value;
      value.v = this->v;
      value.print(base);
    }
  }
};

template<typename T>
using double_uint_type = std::conditional_t<
    std::is_same_v<T, uint8_t>,
    uint16_t,
    std::conditional_t<
        std::is_same_v<T, uint16_t>,
        uint32_t,
        std::conditional_t<std::is_same_v<T, uint32_t>,
                           uint64_t,
                           std::conditional_t<std::is_same_v<T, uint64_t>, __uint128_t, void>>>>;

using UInt128_8 = UIntF<uint8_t, 16>;
using UInt128_16 = UIntF<uint16_t, 8>;
using UInt128_32 = UIntF<uint32_t, 4>;
using UInt128_64 = UIntF<uint64_t, 2>;

using UInt256_8 = UIntF<uint8_t, 32>;
using UInt256_16 = UIntF<uint16_t, 16>;
using UInt256_32 = UIntF<uint32_t, 8>;
using UInt256_64 = UIntF<uint64_t, 4>;

using UInt128 = UInt128_64;
using UInt256 = UInt256_64;

using Int128_8 = IntF<uint8_t, 16>;
using Int128_16 = IntF<uint16_t, 8>;
using Int128_32 = IntF<uint32_t, 4>;
using Int128_64 = IntF<uint64_t, 2>;

using Int128 = Int128_8;

template<typename T, typename T2, int S> inline void generic_add(T *dst, const T *a, const T *b)
{
  constexpr int shift = 8 * sizeof(T);
  T2 carry = 0;
  for (int i = 0; i < S; i++) {
    const T2 ri = T2(a[i]) + T2(b[i]) + carry;
    dst[i] = T(ri);
    carry = ri >> shift;
  }
}

template<typename T, int S> inline void generic_sub(T *dst, const T *a, const T *b)
{
  T carry = 0;
  for (int i = 0; i < S; i++) {
    const T ri = a[i] - b[i] - carry;
    dst[i] = T(ri);
    carry = ri > a[i];
  }
}

template<typename T, typename T2, int S>
inline void generic_unsigned_mul(T *dst, const T *a, const T *b)
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

template<typename T, int Size, BLI_ENABLE_IF((!std::is_void_v<double_uint_type<T>>))>
inline UIntF<T, Size> operator+(const UIntF<T, Size> &a, const UIntF<T, Size> &b)
{
  UIntF<T, Size> result;
  generic_add<T, double_uint_type<T>, Size>(result.v.data(), a.v.data(), b.v.data());
  return result;
}

template<typename T, int Size, BLI_ENABLE_IF((!std::is_void_v<double_uint_type<T>>))>
inline IntF<T, Size> operator+(const IntF<T, Size> &a, const IntF<T, Size> &b)
{
  IntF<T, Size> result;
  generic_add<T, double_uint_type<T>, Size>(result.v.data(), a.v.data(), b.v.data());
  return result;
}

template<typename T, int Size>
inline UIntF<T, Size> operator-(const UIntF<T, Size> &a, const UIntF<T, Size> &b)
{
  UIntF<T, Size> result;
  generic_sub<T, Size>(result.v.data(), a.v.data(), b.v.data());
  return result;
}

template<typename T, int Size>
inline IntF<T, Size> operator-(const IntF<T, Size> &a, const IntF<T, Size> &b)
{
  IntF<T, Size> result;
  generic_sub<T, Size>(result.v.data(), a.v.data(), b.v.data());
  return result;
}

template<typename T, int Size, BLI_ENABLE_IF((!std::is_void_v<double_uint_type<T>>))>
inline UIntF<T, Size> operator*(const UIntF<T, Size> &a, const UIntF<T, Size> &b)
{
  UIntF<T, Size> result;
  generic_unsigned_mul<T, double_uint_type<T>, Size>(result.v.data(), a.v.data(), b.v.data());
  return result;
}

}  // namespace blender::fixed_width_int
