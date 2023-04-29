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

  explicit UIntF(const uint64_t value) : UIntF(std::to_string(value).c_str()) {}

  explicit UIntF(const char *str, const int base = 10)
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

  explicit IntF(const int64_t value) : IntF(std::to_string(value).c_str()) {}

  explicit IntF(const UIntF<T, S> &value) : v(value.v) {}

  explicit IntF(const char *str, const int base = 10)
  {
    if (str[0] == '-') {
      const UIntF<T, S> unsigned_value(str + 1, base);
      this->v = unsigned_value.v;
      *this = -*this;
    }
    else {
      const UIntF<T, S> unsigned_value(str, base);
      this->v = unsigned_value.v;
    }
  }

  explicit operator UIntF<T, S>() const
  {
    UIntF<T, S> result;
    result.v = this->v;
    return result;
  }

  void print(const int base = 10) const
  {
    if (is_negative(*this)) {
      std::cout << "-";
      (-*this).print(base);
    }
    else {
      UIntF<T, S>(*this).print(base);
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

template<typename T, int Size> bool is_negative(const IntF<T, Size> &a)
{
  return (a.v[Size - 1] & (T(1) << (sizeof(T) * 8 - 1))) != 0;
}

template<typename T, int Size, BLI_ENABLE_IF((!std::is_void_v<double_uint_type<T>>))>
inline IntF<T, Size> operator*(const IntF<T, Size> &a, const IntF<T, Size> &b)
{
  using UIntF = UIntF<T, Size>;
  using IntF = IntF<T, Size>;

  const bool is_negative_a = is_negative(a);
  const bool is_negative_b = is_negative(b);
  if (is_negative_a && is_negative_b) {
    return IntF(UIntF(-a) * UIntF(-b));
  }
  if (is_negative_a) {
    return -IntF(UIntF(-a) * UIntF(b));
  }
  if (is_negative_b) {
    return -IntF(UIntF(a) * UIntF(-b));
  }
  return IntF(UIntF(a) * UIntF(b));
}

template<typename T, int Size> inline IntF<T, Size> operator-(const IntF<T, Size> &a)
{
  IntF<T, Size> result;
  for (int i = 0; i < Size; i++) {
    result.v[i] = ~a.v[i];
  }
  return result + IntF<T, Size>(1);
}

}  // namespace blender::fixed_width_int
