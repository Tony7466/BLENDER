/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <immintrin.h>
#include <iostream>

#include <gmpxx.h>

#include "BLI_string_ref.hh"
#include "BLI_utildefines.h"

namespace blender::fixed_width_int {

template<typename T, int S> struct UIntF {
  T v[S];

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

using UInt128_32 = UIntF<uint32_t, 4>;

using UInt128 = UInt128_32;

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

inline UInt128_32 operator+(const UInt128_32 &a, const UInt128_32 &b)
{
  UInt128_32 result;
  generic_add<uint32_t, uint64_t, 4>(result.v, a.v, b.v);
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
  generic_mul<uint32_t, uint64_t, 4>(result.v, a.v, b.v);
  return result;
}

}  // namespace blender::fixed_width_int
