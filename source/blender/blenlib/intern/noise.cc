/* SPDX-FileCopyrightText: 2009-2010 Sony Pictures Imageworks Inc., et al.
 *                         All Rights Reserved. (BSD-3-Clause).
 * SPDX-FileCopyrightText: 2011 Blender Authors (GPL-2.0-or-later).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later AND BSD-3-Clause */

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "BLI_math_base.hh"
#include "BLI_math_base_safe.h"
#include "BLI_math_matrix_types.hh"
#include "BLI_math_numbers.hh"
#include "BLI_math_vector.hh"
#include "BLI_noise.hh"
#include "BLI_utildefines.h"

namespace blender::noise {

/* -------------------------------------------------------------------- */
/** \name Jenkins Lookup3 Hash Functions
 *
 * https://burtleburtle.net/bob/c/lookup3.c
 * \{ */

BLI_INLINE uint32_t hash_bit_rotate(uint32_t x, uint32_t k)
{
  return (x << k) | (x >> (32 - k));
}

BLI_INLINE void hash_bit_mix(uint32_t &a, uint32_t &b, uint32_t &c)
{
  a -= c;
  a ^= hash_bit_rotate(c, 4);
  c += b;
  b -= a;
  b ^= hash_bit_rotate(a, 6);
  a += c;
  c -= b;
  c ^= hash_bit_rotate(b, 8);
  b += a;
  a -= c;
  a ^= hash_bit_rotate(c, 16);
  c += b;
  b -= a;
  b ^= hash_bit_rotate(a, 19);
  a += c;
  c -= b;
  c ^= hash_bit_rotate(b, 4);
  b += a;
}

BLI_INLINE void hash_bit_final(uint32_t &a, uint32_t &b, uint32_t &c)
{
  c ^= b;
  c -= hash_bit_rotate(b, 14);
  a ^= c;
  a -= hash_bit_rotate(c, 11);
  b ^= a;
  b -= hash_bit_rotate(a, 25);
  c ^= b;
  c -= hash_bit_rotate(b, 16);
  a ^= c;
  a -= hash_bit_rotate(c, 4);
  b ^= a;
  b -= hash_bit_rotate(a, 14);
  c ^= b;
  c -= hash_bit_rotate(b, 24);
}

uint32_t hash(uint32_t kx)
{
  uint32_t a, b, c;
  a = b = c = 0xdeadbeef + (1 << 2) + 13;

  a += kx;
  hash_bit_final(a, b, c);

  return c;
}

uint32_t hash(uint32_t kx, uint32_t ky)
{
  uint32_t a, b, c;
  a = b = c = 0xdeadbeef + (2 << 2) + 13;

  b += ky;
  a += kx;
  hash_bit_final(a, b, c);

  return c;
}

uint32_t hash(uint32_t kx, uint32_t ky, uint32_t kz)
{
  uint32_t a, b, c;
  a = b = c = 0xdeadbeef + (3 << 2) + 13;

  c += kz;
  b += ky;
  a += kx;
  hash_bit_final(a, b, c);

  return c;
}

uint32_t hash(uint32_t kx, uint32_t ky, uint32_t kz, uint32_t kw)
{
  uint32_t a, b, c;
  a = b = c = 0xdeadbeef + (4 << 2) + 13;

  a += kx;
  b += ky;
  c += kz;
  hash_bit_mix(a, b, c);

  a += kw;
  hash_bit_final(a, b, c);

  return c;
}

BLI_INLINE uint32_t float_as_uint(float f)
{
  union {
    uint32_t i;
    float f;
  } u;
  u.f = f;
  return u.i;
}

uint32_t hash_float(float kx)
{
  return hash(float_as_uint(kx));
}

uint32_t hash_float(float2 k)
{
  return hash(float_as_uint(k.x), float_as_uint(k.y));
}

uint32_t hash_float(float3 k)
{
  return hash(float_as_uint(k.x), float_as_uint(k.y), float_as_uint(k.z));
}

uint32_t hash_float(float4 k)
{
  return hash(float_as_uint(k.x), float_as_uint(k.y), float_as_uint(k.z), float_as_uint(k.w));
}

uint32_t hash_float(const float4x4 &k)
{
  return hash(hash_float(k.x), hash_float(k.y), hash_float(k.z), hash_float(k.w));
}

/* Hashing a number of uint32_t into a float in the range [0, 1]. */

BLI_INLINE float uint_to_float_01(uint32_t k)
{
  return float(k) / float(0xFFFFFFFFu);
}

float hash_to_float(uint32_t kx)
{
  return uint_to_float_01(hash(kx));
}

float hash_to_float(uint32_t kx, uint32_t ky)
{
  return uint_to_float_01(hash(kx, ky));
}

float hash_to_float(uint32_t kx, uint32_t ky, uint32_t kz)
{
  return uint_to_float_01(hash(kx, ky, kz));
}

float hash_to_float(uint32_t kx, uint32_t ky, uint32_t kz, uint32_t kw)
{
  return uint_to_float_01(hash(kx, ky, kz, kw));
}

/* Hashing a number of floats into a float in the range [0, 1]. */

float hash_float_to_float(float k)
{
  return uint_to_float_01(hash_float(k));
}

float hash_float_to_float(float2 k)
{
  return uint_to_float_01(hash_float(k));
}

float hash_float_to_float(float3 k)
{
  return uint_to_float_01(hash_float(k));
}

float hash_float_to_float(float4 k)
{
  return uint_to_float_01(hash_float(k));
}

float2 hash_float_to_float2(float2 k)
{
  return float2(hash_float_to_float(k), hash_float_to_float(float3(k.x, k.y, 1.0)));
}

float2 hash_float_to_float2(float3 k)
{
  return float2(hash_float_to_float(float3(k.x, k.y, k.z)),
                hash_float_to_float(float3(k.z, k.x, k.y)));
}

float2 hash_float_to_float2(float4 k)
{
  return float2(hash_float_to_float(float4(k.x, k.y, k.z, k.w)),
                hash_float_to_float(float4(k.z, k.x, k.w, k.y)));
}

float3 hash_float_to_float3(float k)
{
  return float3(hash_float_to_float(k),
                hash_float_to_float(float2(k, 1.0)),
                hash_float_to_float(float2(k, 2.0)));
}

float3 hash_float_to_float3(float2 k)
{
  return float3(hash_float_to_float(k),
                hash_float_to_float(float3(k.x, k.y, 1.0)),
                hash_float_to_float(float3(k.x, k.y, 2.0)));
}

float3 hash_float_to_float3(float3 k)
{
  return float3(hash_float_to_float(k),
                hash_float_to_float(float4(k.x, k.y, k.z, 1.0)),
                hash_float_to_float(float4(k.x, k.y, k.z, 2.0)));
}

float3 hash_float_to_float3(float4 k)
{
  return float3(hash_float_to_float(k),
                hash_float_to_float(float4(k.z, k.x, k.w, k.y)),
                hash_float_to_float(float4(k.w, k.z, k.y, k.x)));
}

float4 hash_float_to_float4(float4 k)
{
  return float4(hash_float_to_float(k),
                hash_float_to_float(float4(k.w, k.x, k.y, k.z)),
                hash_float_to_float(float4(k.z, k.w, k.x, k.y)),
                hash_float_to_float(float4(k.y, k.z, k.w, k.x)));
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Perlin Noise
 *
 * Perlin, Ken. "Improving noise." Proceedings of the 29th annual conference on Computer graphics
 * and interactive techniques. 2002.
 *
 * This implementation is functionally identical to the implementations in EEVEE, OSL, and SVM. So
 * any changes should be applied in all relevant implementations.
 * \{ */

/* Linear Interpolation. */
template<typename T> T static mix(T v0, T v1, float x)
{
  return (1 - x) * v0 + x * v1;
}

/* Bilinear Interpolation:
 *
 * v2          v3
 *  @ + + + + @       y
 *  +         +       ^
 *  +         +       |
 *  +         +       |
 *  @ + + + + @       @------> x
 * v0          v1
 */
BLI_INLINE float mix(float v0, float v1, float v2, float v3, float x, float y)
{
  float x1 = 1.0 - x;
  return (1.0 - y) * (v0 * x1 + v1 * x) + y * (v2 * x1 + v3 * x);
}

/* Trilinear Interpolation:
 *
 *   v6               v7
 *     @ + + + + + + @
 *     +\            +\
 *     + \           + \
 *     +  \          +  \
 *     +   \ v4      +   \ v5
 *     +    @ + + + +++ + @          z
 *     +    +        +    +      y   ^
 *  v2 @ + +++ + + + @ v3 +       \  |
 *      \   +         \   +        \ |
 *       \  +          \  +         \|
 *        \ +           \ +          +---------> x
 *         \+            \+
 *          @ + + + + + + @
 *        v0               v1
 */
BLI_INLINE float mix(float v0,
                     float v1,
                     float v2,
                     float v3,
                     float v4,
                     float v5,
                     float v6,
                     float v7,
                     float x,
                     float y,
                     float z)
{
  float x1 = 1.0 - x;
  float y1 = 1.0 - y;
  float z1 = 1.0 - z;
  return z1 * (y1 * (v0 * x1 + v1 * x) + y * (v2 * x1 + v3 * x)) +
         z * (y1 * (v4 * x1 + v5 * x) + y * (v6 * x1 + v7 * x));
}

/* Quadrilinear Interpolation. */
BLI_INLINE float mix(float v0,
                     float v1,
                     float v2,
                     float v3,
                     float v4,
                     float v5,
                     float v6,
                     float v7,
                     float v8,
                     float v9,
                     float v10,
                     float v11,
                     float v12,
                     float v13,
                     float v14,
                     float v15,
                     float x,
                     float y,
                     float z,
                     float w)
{
  return mix(mix(v0, v1, v2, v3, v4, v5, v6, v7, x, y, z),
             mix(v8, v9, v10, v11, v12, v13, v14, v15, x, y, z),
             w);
}

BLI_INLINE float fade(float t)
{
  return t * t * t * (t * (t * 6.0 - 15.0) + 10.0);
}

BLI_INLINE float negate_if(float value, uint32_t condition)
{
  return (condition != 0u) ? -value : value;
}

BLI_INLINE float noise_grad(uint32_t hash, float x)
{
  uint32_t h = hash & 15u;
  float g = 1u + (h & 7u);
  return negate_if(g, h & 8u) * x;
}

BLI_INLINE float noise_grad(uint32_t hash, float x, float y)
{
  uint32_t h = hash & 7u;
  float u = h < 4u ? x : y;
  float v = 2.0 * (h < 4u ? y : x);
  return negate_if(u, h & 1u) + negate_if(v, h & 2u);
}

BLI_INLINE float noise_grad(uint32_t hash, float x, float y, float z)
{
  uint32_t h = hash & 15u;
  float u = h < 8u ? x : y;
  float vt = ELEM(h, 12u, 14u) ? x : z;
  float v = h < 4u ? y : vt;
  return negate_if(u, h & 1u) + negate_if(v, h & 2u);
}

BLI_INLINE float noise_grad(uint32_t hash, float x, float y, float z, float w)
{
  uint32_t h = hash & 31u;
  float u = h < 24u ? x : y;
  float v = h < 16u ? y : z;
  float s = h < 8u ? z : w;
  return negate_if(u, h & 1u) + negate_if(v, h & 2u) + negate_if(s, h & 4u);
}

BLI_INLINE float floor_fraction(float x, int &i)
{
  float x_floor = math::floor(x);
  i = int(x_floor);
  return x - x_floor;
}

BLI_INLINE float perlin_noise(float position)
{
  int X;

  float fx = floor_fraction(position, X);

  float u = fade(fx);

  float r = mix(noise_grad(hash(X), fx), noise_grad(hash(X + 1), fx - 1.0), u);

  return r;
}

BLI_INLINE float perlin_noise(float2 position)
{
  int X, Y;

  float fx = floor_fraction(position.x, X);
  float fy = floor_fraction(position.y, Y);

  float u = fade(fx);
  float v = fade(fy);

  float r = mix(noise_grad(hash(X, Y), fx, fy),
                noise_grad(hash(X + 1, Y), fx - 1.0, fy),
                noise_grad(hash(X, Y + 1), fx, fy - 1.0),
                noise_grad(hash(X + 1, Y + 1), fx - 1.0, fy - 1.0),
                u,
                v);

  return r;
}

BLI_INLINE float perlin_noise(float3 position)
{
  int X, Y, Z;

  float fx = floor_fraction(position.x, X);
  float fy = floor_fraction(position.y, Y);
  float fz = floor_fraction(position.z, Z);

  float u = fade(fx);
  float v = fade(fy);
  float w = fade(fz);

  float r = mix(noise_grad(hash(X, Y, Z), fx, fy, fz),
                noise_grad(hash(X + 1, Y, Z), fx - 1, fy, fz),
                noise_grad(hash(X, Y + 1, Z), fx, fy - 1, fz),
                noise_grad(hash(X + 1, Y + 1, Z), fx - 1, fy - 1, fz),
                noise_grad(hash(X, Y, Z + 1), fx, fy, fz - 1),
                noise_grad(hash(X + 1, Y, Z + 1), fx - 1, fy, fz - 1),
                noise_grad(hash(X, Y + 1, Z + 1), fx, fy - 1, fz - 1),
                noise_grad(hash(X + 1, Y + 1, Z + 1), fx - 1, fy - 1, fz - 1),
                u,
                v,
                w);

  return r;
}

BLI_INLINE float perlin_noise(float4 position)
{
  int X, Y, Z, W;

  float fx = floor_fraction(position.x, X);
  float fy = floor_fraction(position.y, Y);
  float fz = floor_fraction(position.z, Z);
  float fw = floor_fraction(position.w, W);

  float u = fade(fx);
  float v = fade(fy);
  float t = fade(fz);
  float s = fade(fw);

  float r = mix(
      noise_grad(hash(X, Y, Z, W), fx, fy, fz, fw),
      noise_grad(hash(X + 1, Y, Z, W), fx - 1.0, fy, fz, fw),
      noise_grad(hash(X, Y + 1, Z, W), fx, fy - 1.0, fz, fw),
      noise_grad(hash(X + 1, Y + 1, Z, W), fx - 1.0, fy - 1.0, fz, fw),
      noise_grad(hash(X, Y, Z + 1, W), fx, fy, fz - 1.0, fw),
      noise_grad(hash(X + 1, Y, Z + 1, W), fx - 1.0, fy, fz - 1.0, fw),
      noise_grad(hash(X, Y + 1, Z + 1, W), fx, fy - 1.0, fz - 1.0, fw),
      noise_grad(hash(X + 1, Y + 1, Z + 1, W), fx - 1.0, fy - 1.0, fz - 1.0, fw),
      noise_grad(hash(X, Y, Z, W + 1), fx, fy, fz, fw - 1.0),
      noise_grad(hash(X + 1, Y, Z, W + 1), fx - 1.0, fy, fz, fw - 1.0),
      noise_grad(hash(X, Y + 1, Z, W + 1), fx, fy - 1.0, fz, fw - 1.0),
      noise_grad(hash(X + 1, Y + 1, Z, W + 1), fx - 1.0, fy - 1.0, fz, fw - 1.0),
      noise_grad(hash(X, Y, Z + 1, W + 1), fx, fy, fz - 1.0, fw - 1.0),
      noise_grad(hash(X + 1, Y, Z + 1, W + 1), fx - 1.0, fy, fz - 1.0, fw - 1.0),
      noise_grad(hash(X, Y + 1, Z + 1, W + 1), fx, fy - 1.0, fz - 1.0, fw - 1.0),
      noise_grad(hash(X + 1, Y + 1, Z + 1, W + 1), fx - 1.0, fy - 1.0, fz - 1.0, fw - 1.0),
      u,
      v,
      t,
      s);

  return r;
}

/* Signed versions of perlin noise in the range [-1, 1]. The scale values were computed
 * experimentally by the OSL developers to remap the noise output to the correct range. */

float perlin_signed(float position)
{
  float precision_correction = 0.5f * float(math::abs(position) >= 1000000.0f);
  /* Repeat Perlin noise texture every 100000.0 on each axis to prevent floating point
   * representation issues. */
  position = math::mod(position, 100000.0f) + precision_correction;

  return perlin_noise(position) * 0.2500f;
}

float perlin_signed(float2 position)
{
  float2 precision_correction = 0.5f * float2(float(math::abs(position.x) >= 1000000.0f),
                                              float(math::abs(position.y) >= 1000000.0f));
  /* Repeat Perlin noise texture every 100000.0f on each axis to prevent floating point
   * representation issues. This causes discontinuities every 100000.0f, however at such scales
   * this usually shouldn't be noticeable. */
  position = math::mod(position, 100000.0f) + precision_correction;

  return perlin_noise(position) * 0.6616f;
}

float perlin_signed(float3 position)
{
  float3 precision_correction = 0.5f * float3(float(math::abs(position.x) >= 1000000.0f),
                                              float(math::abs(position.y) >= 1000000.0f),
                                              float(math::abs(position.z) >= 1000000.0f));
  /* Repeat Perlin noise texture every 100000.0f on each axis to prevent floating point
   * representation issues. This causes discontinuities every 100000.0f, however at such scales
   * this usually shouldn't be noticeable. */
  position = math::mod(position, 100000.0f) + precision_correction;

  return perlin_noise(position) * 0.9820f;
}

float perlin_signed(float4 position)
{
  float4 precision_correction = 0.5f * float4(float(math::abs(position.x) >= 1000000.0f),
                                              float(math::abs(position.y) >= 1000000.0f),
                                              float(math::abs(position.z) >= 1000000.0f),
                                              float(math::abs(position.w) >= 1000000.0f));
  /* Repeat Perlin noise texture every 100000.0f on each axis to prevent floating point
   * representation issues. This causes discontinuities every 100000.0f, however at such scales
   * this usually shouldn't be noticeable. */
  position = math::mod(position, 100000.0f) + precision_correction;

  return perlin_noise(position) * 0.8344f;
}

/* Positive versions of perlin noise in the range [0, 1]. */

float perlin(float position)
{
  return perlin_signed(position) / 2.0f + 0.5f;
}

float perlin(float2 position)
{
  return perlin_signed(position) / 2.0f + 0.5f;
}

float perlin(float3 position)
{
  return perlin_signed(position) / 2.0f + 0.5f;
}

float perlin(float4 position)
{
  return perlin_signed(position) / 2.0f + 0.5f;
}

/* Fractal perlin noise. */

/* fBM = Fractal Brownian Motion */
template<typename T>
float perlin_fbm(
    T p, const float detail, const float roughness, const float lacunarity, const bool normalize)
{
  float fscale = 1.0f;
  float amp = 1.0f;
  float maxamp = 0.0f;
  float sum = 0.0f;

  for (int i = 0; i <= int(detail); i++) {
    float t = perlin_signed(fscale * p);
    sum += t * amp;
    maxamp += amp;
    amp *= roughness;
    fscale *= lacunarity;
  }
  float rmd = detail - std::floor(detail);
  if (rmd != 0.0f) {
    float t = perlin_signed(fscale * p);
    float sum2 = sum + t * amp;
    return normalize ? mix(0.5f * sum / maxamp + 0.5f, 0.5f * sum2 / (maxamp + amp) + 0.5f, rmd) :
                       mix(sum, sum2, rmd);
  }
  return normalize ? 0.5f * sum / maxamp + 0.5f : sum;
}

/* Explicit instantiation for Wave Texture. */
template float perlin_fbm<float3>(float3 p,
                                  const float detail,
                                  const float roughness,
                                  const float lacunarity,
                                  const bool normalize);

template<typename T>
float perlin_multi_fractal(T p, const float detail, const float roughness, const float lacunarity)
{
  float value = 1.0f;
  float pwr = 1.0f;

  for (int i = 0; i <= int(detail); i++) {
    value *= (pwr * perlin_signed(p) + 1.0f);
    pwr *= roughness;
    p *= lacunarity;
  }

  const float rmd = detail - floorf(detail);
  if (rmd != 0.0f) {
    value *= (rmd * pwr * perlin_signed(p) + 1.0f); /* correct? */
  }

  return value;
}

template<typename T>
float perlin_hetero_terrain(
    T p, const float detail, const float roughness, const float lacunarity, const float offset)
{
  float pwr = roughness;

  /* First unscaled octave of function; later octaves are scaled. */
  float value = offset + perlin_signed(p);
  p *= lacunarity;

  for (int i = 1; i <= int(detail); i++) {
    float increment = (perlin_signed(p) + offset) * pwr * value;
    value += increment;
    pwr *= roughness;
    p *= lacunarity;
  }

  const float rmd = detail - floorf(detail);
  if (rmd != 0.0f) {
    float increment = (perlin_signed(p) + offset) * pwr * value;
    value += rmd * increment;
  }

  return value;
}

template<typename T>
float perlin_hybrid_multi_fractal(T p,
                                  const float detail,
                                  const float roughness,
                                  const float lacunarity,
                                  const float offset,
                                  const float gain)
{
  float pwr = 1.0f;
  float value = 0.0f;
  float weight = 1.0f;

  for (int i = 0; (weight > 0.001f) && (i <= int(detail)); i++) {
    if (weight > 1.0f) {
      weight = 1.0f;
    }

    float signal = (perlin_signed(p) + offset) * pwr;
    pwr *= roughness;
    value += weight * signal;
    weight *= gain * signal;
    p *= lacunarity;
  }

  const float rmd = detail - floorf(detail);
  if ((rmd != 0.0f) && (weight > 0.001f)) {
    if (weight > 1.0f) {
      weight = 1.0f;
    }
    float signal = (perlin_signed(p) + offset) * pwr;
    value += rmd * weight * signal;
  }

  return value;
}

template<typename T>
float perlin_ridged_multi_fractal(T p,
                                  const float detail,
                                  const float roughness,
                                  const float lacunarity,
                                  const float offset,
                                  const float gain)
{
  float pwr = roughness;

  float signal = offset - std::abs(perlin_signed(p));
  signal *= signal;
  float value = signal;
  float weight = 1.0f;

  for (int i = 1; i <= int(detail); i++) {
    p *= lacunarity;
    weight = std::clamp(signal * gain, 0.0f, 1.0f);
    signal = offset - std::abs(perlin_signed(p));
    signal *= signal;
    signal *= weight;
    value += signal * pwr;
    pwr *= roughness;
  }

  return value;
}

enum {
  NOISE_SHD_PERLIN_MULTIFRACTAL = 0,
  NOISE_SHD_PERLIN_FBM = 1,
  NOISE_SHD_PERLIN_HYBRID_MULTIFRACTAL = 2,
  NOISE_SHD_PERLIN_RIDGED_MULTIFRACTAL = 3,
  NOISE_SHD_PERLIN_HETERO_TERRAIN = 4,
};

template<typename T>
float perlin_select(T p,
                    float detail,
                    float roughness,
                    float lacunarity,
                    float offset,
                    float gain,
                    int type,
                    bool normalize)
{
  switch (type) {
    case NOISE_SHD_PERLIN_MULTIFRACTAL: {
      return perlin_multi_fractal<T>(p, detail, roughness, lacunarity);
    }
    case NOISE_SHD_PERLIN_FBM: {
      return perlin_fbm<T>(p, detail, roughness, lacunarity, normalize);
    }
    case NOISE_SHD_PERLIN_HYBRID_MULTIFRACTAL: {
      return perlin_hybrid_multi_fractal<T>(p, detail, roughness, lacunarity, offset, gain);
    }
    case NOISE_SHD_PERLIN_RIDGED_MULTIFRACTAL: {
      return perlin_ridged_multi_fractal<T>(p, detail, roughness, lacunarity, offset, gain);
    }
    case NOISE_SHD_PERLIN_HETERO_TERRAIN: {
      return perlin_hetero_terrain<T>(p, detail, roughness, lacunarity, offset);
    }
    default: {
      return 0.0;
    }
  }
}

/* The following offset functions generate random offsets to be added to
 * positions to act as a seed since the noise functions don't have seed values.
 * The offset's components are in the range [100, 200], not too high to cause
 * bad precision and not too small to be noticeable. We use float seed because
 * OSL only supports float hashes and we need to maintain compatibility with it.
 */

BLI_INLINE float random_float_offset(float seed)
{
  return 100.0f + hash_float_to_float(seed) * 100.0f;
}

BLI_INLINE float2 random_float2_offset(float seed)
{
  return float2(100.0f + hash_float_to_float(float2(seed, 0.0f)) * 100.0f,
                100.0f + hash_float_to_float(float2(seed, 1.0f)) * 100.0f);
}

BLI_INLINE float3 random_float3_offset(float seed)
{
  return float3(100.0f + hash_float_to_float(float2(seed, 0.0f)) * 100.0f,
                100.0f + hash_float_to_float(float2(seed, 1.0f)) * 100.0f,
                100.0f + hash_float_to_float(float2(seed, 2.0f)) * 100.0f);
}

BLI_INLINE float4 random_float4_offset(float seed)
{
  return float4(100.0f + hash_float_to_float(float2(seed, 0.0f)) * 100.0f,
                100.0f + hash_float_to_float(float2(seed, 1.0f)) * 100.0f,
                100.0f + hash_float_to_float(float2(seed, 2.0f)) * 100.0f,
                100.0f + hash_float_to_float(float2(seed, 3.0f)) * 100.0f);
}

/* Perlin noises to be added to the position to distort other noises. */

BLI_INLINE float perlin_distortion(float position, float strength)
{
  return perlin_signed(position + random_float_offset(0.0)) * strength;
}

BLI_INLINE float2 perlin_distortion(float2 position, float strength)
{
  return float2(perlin_signed(position + random_float2_offset(0.0f)) * strength,
                perlin_signed(position + random_float2_offset(1.0f)) * strength);
}

BLI_INLINE float3 perlin_distortion(float3 position, float strength)
{
  return float3(perlin_signed(position + random_float3_offset(0.0f)) * strength,
                perlin_signed(position + random_float3_offset(1.0f)) * strength,
                perlin_signed(position + random_float3_offset(2.0f)) * strength);
}

BLI_INLINE float4 perlin_distortion(float4 position, float strength)
{
  return float4(perlin_signed(position + random_float4_offset(0.0f)) * strength,
                perlin_signed(position + random_float4_offset(1.0f)) * strength,
                perlin_signed(position + random_float4_offset(2.0f)) * strength,
                perlin_signed(position + random_float4_offset(3.0f)) * strength);
}

/* Distorted fractal perlin noise. */

template<typename T>
float perlin_fractal_distorted(T position,
                               float detail,
                               float roughness,
                               float lacunarity,
                               float offset,
                               float gain,
                               float distortion,
                               int type,
                               bool normalize)
{
  position += perlin_distortion(position, distortion);
  return perlin_select<T>(position, detail, roughness, lacunarity, offset, gain, type, normalize);
}

template float perlin_fractal_distorted<float>(float position,
                                               float detail,
                                               float roughness,
                                               float lacunarity,
                                               float offset,
                                               float gain,
                                               float distortion,
                                               int type,
                                               bool normalize);
template float perlin_fractal_distorted<float2>(float2 position,
                                                float detail,
                                                float roughness,
                                                float lacunarity,
                                                float offset,
                                                float gain,
                                                float distortion,
                                                int type,
                                                bool normalize);
template float perlin_fractal_distorted<float3>(float3 position,
                                                float detail,
                                                float roughness,
                                                float lacunarity,
                                                float offset,
                                                float gain,
                                                float distortion,
                                                int type,
                                                bool normalize);
template float perlin_fractal_distorted<float4>(float4 position,
                                                float detail,
                                                float roughness,
                                                float lacunarity,
                                                float offset,
                                                float gain,
                                                float distortion,
                                                int type,
                                                bool normalize);

/* Distorted fractal perlin noise that outputs a float3. The arbitrary seeds are for
 * compatibility with shading functions. */

float3 perlin_float3_fractal_distorted(float position,
                                       float detail,
                                       float roughness,
                                       float lacunarity,
                                       float offset,
                                       float gain,
                                       float distortion,
                                       int type,
                                       bool normalize)
{
  position += perlin_distortion(position, distortion);
  return float3(
      perlin_select<float>(position, detail, roughness, lacunarity, offset, gain, type, normalize),
      perlin_select<float>(position + random_float_offset(1.0f),
                           detail,
                           roughness,
                           lacunarity,
                           offset,
                           gain,
                           type,
                           normalize),
      perlin_select<float>(position + random_float_offset(2.0f),
                           detail,
                           roughness,
                           lacunarity,
                           offset,
                           gain,
                           type,
                           normalize));
}

float3 perlin_float3_fractal_distorted(float2 position,
                                       float detail,
                                       float roughness,
                                       float lacunarity,
                                       float offset,
                                       float gain,
                                       float distortion,
                                       int type,
                                       bool normalize)
{
  position += perlin_distortion(position, distortion);
  return float3(perlin_select<float2>(
                    position, detail, roughness, lacunarity, offset, gain, type, normalize),
                perlin_select<float2>(position + random_float2_offset(2.0f),
                                      detail,
                                      roughness,
                                      lacunarity,
                                      offset,
                                      gain,
                                      type,
                                      normalize),
                perlin_select<float2>(position + random_float2_offset(3.0f),
                                      detail,
                                      roughness,
                                      lacunarity,
                                      offset,
                                      gain,
                                      type,
                                      normalize));
}

float3 perlin_float3_fractal_distorted(float3 position,
                                       float detail,
                                       float roughness,
                                       float lacunarity,
                                       float offset,
                                       float gain,
                                       float distortion,
                                       int type,
                                       bool normalize)
{
  position += perlin_distortion(position, distortion);
  return float3(perlin_select<float3>(
                    position, detail, roughness, lacunarity, offset, gain, type, normalize),
                perlin_select<float3>(position + random_float3_offset(3.0f),
                                      detail,
                                      roughness,
                                      lacunarity,
                                      offset,
                                      gain,
                                      type,
                                      normalize),
                perlin_select<float3>(position + random_float3_offset(4.0f),
                                      detail,
                                      roughness,
                                      lacunarity,
                                      offset,
                                      gain,
                                      type,
                                      normalize));
}

float3 perlin_float3_fractal_distorted(float4 position,
                                       float detail,
                                       float roughness,
                                       float lacunarity,
                                       float offset,
                                       float gain,
                                       float distortion,
                                       int type,
                                       bool normalize)
{
  position += perlin_distortion(position, distortion);
  return float3(perlin_select<float4>(
                    position, detail, roughness, lacunarity, offset, gain, type, normalize),
                perlin_select<float4>(position + random_float4_offset(4.0f),
                                      detail,
                                      roughness,
                                      lacunarity,
                                      offset,
                                      gain,
                                      type,
                                      normalize),
                perlin_select<float4>(position + random_float4_offset(5.0f),
                                      detail,
                                      roughness,
                                      lacunarity,
                                      offset,
                                      gain,
                                      type,
                                      normalize));
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Voronoi Noise
 *
 * \note Ported from Cycles code.
 *
 * Original code is under the MIT License, Copyright (c) 2013 Inigo Quilez.
 *
 * Smooth Voronoi:
 *
 * - https://wiki.blender.org/wiki/User:OmarSquircleArt/GSoC2019/Documentation/Smooth_Voronoi
 *
 * Distance To Edge based on:
 *
 * - https://www.iquilezles.org/www/articles/voronoilines/voronoilines.htm
 * - https://www.shadertoy.com/view/ldl3W8
 *
 * With optimization to change -2..2 scan window to -1..1 for better performance,
 * as explained in https://www.shadertoy.com/view/llG3zy.
 * \{ */

/* Ensure to align with DNA. */

enum {
  NOISE_SHD_VORONOI_EUCLIDEAN = 0,
  NOISE_SHD_VORONOI_MANHATTAN = 1,
  NOISE_SHD_VORONOI_CHEBYCHEV = 2,
  NOISE_SHD_VORONOI_MINKOWSKI = 3,
};

enum {
  NOISE_SHD_VORONOI_F1 = 0,
  NOISE_SHD_VORONOI_F2 = 1,
  NOISE_SHD_VORONOI_SMOOTH_F1 = 2,
  NOISE_SHD_VORONOI_DISTANCE_TO_EDGE = 3,
  NOISE_SHD_VORONOI_N_SPHERE_RADIUS = 4,
};

/* ***** Distances ***** */

float voronoi_distance(const float a, const float b)
{
  return std::abs(b - a);
}

float voronoi_distance(const float2 a, const float2 b, const VoronoiParams &params)
{
  switch (params.metric) {
    case NOISE_SHD_VORONOI_EUCLIDEAN:
      return math::distance(a, b);
    case NOISE_SHD_VORONOI_MANHATTAN:
      return std::abs(a.x - b.x) + std::abs(a.y - b.y);
    case NOISE_SHD_VORONOI_CHEBYCHEV:
      return std::max(std::abs(a.x - b.x), std::abs(a.y - b.y));
    case NOISE_SHD_VORONOI_MINKOWSKI:
      return std::pow(std::pow(std::abs(a.x - b.x), params.exponent) +
                          std::pow(std::abs(a.y - b.y), params.exponent),
                      1.0f / params.exponent);
    default:
      BLI_assert_unreachable();
      break;
  }
  return 0.0f;
}

float voronoi_distance(const float3 a, const float3 b, const VoronoiParams &params)
{
  switch (params.metric) {
    case NOISE_SHD_VORONOI_EUCLIDEAN:
      return math::distance(a, b);
    case NOISE_SHD_VORONOI_MANHATTAN:
      return std::abs(a.x - b.x) + std::abs(a.y - b.y) + std::abs(a.z - b.z);
    case NOISE_SHD_VORONOI_CHEBYCHEV:
      return std::max(std::abs(a.x - b.x), std::max(std::abs(a.y - b.y), std::abs(a.z - b.z)));
    case NOISE_SHD_VORONOI_MINKOWSKI:
      return std::pow(std::pow(std::abs(a.x - b.x), params.exponent) +
                          std::pow(std::abs(a.y - b.y), params.exponent) +
                          std::pow(std::abs(a.z - b.z), params.exponent),
                      1.0f / params.exponent);
    default:
      BLI_assert_unreachable();
      break;
  }
  return 0.0f;
}

float voronoi_distance(const float4 a, const float4 b, const VoronoiParams &params)
{
  switch (params.metric) {
    case NOISE_SHD_VORONOI_EUCLIDEAN:
      return math::distance(a, b);
    case NOISE_SHD_VORONOI_MANHATTAN:
      return std::abs(a.x - b.x) + std::abs(a.y - b.y) + std::abs(a.z - b.z) + std::abs(a.w - b.w);
    case NOISE_SHD_VORONOI_CHEBYCHEV:
      return std::max(
          std::abs(a.x - b.x),
          std::max(std::abs(a.y - b.y), std::max(std::abs(a.z - b.z), std::abs(a.w - b.w))));
    case NOISE_SHD_VORONOI_MINKOWSKI:
      return std::pow(std::pow(std::abs(a.x - b.x), params.exponent) +
                          std::pow(std::abs(a.y - b.y), params.exponent) +
                          std::pow(std::abs(a.z - b.z), params.exponent) +
                          std::pow(std::abs(a.w - b.w), params.exponent),
                      1.0f / params.exponent);
    default:
      BLI_assert_unreachable();
      break;
  }
  return 0.0f;
}

/* **** 1D Voronoi **** */

float4 voronoi_position(const float coord)
{
  return {0.0f, 0.0f, 0.0f, coord};
}

VoronoiOutput voronoi_f1(const VoronoiParams &params, const float coord)
{
  float cellPosition = floorf(coord);
  float localPosition = coord - cellPosition;

  float minDistance = FLT_MAX;
  float targetOffset = 0.0f;
  float targetPosition = 0.0f;
  for (int i = -1; i <= 1; i++) {
    float cellOffset = i;
    float pointPosition = cellOffset +
                          hash_float_to_float(cellPosition + cellOffset) * params.randomness;
    float distanceToPoint = voronoi_distance(pointPosition, localPosition);
    if (distanceToPoint < minDistance) {
      targetOffset = cellOffset;
      minDistance = distanceToPoint;
      targetPosition = pointPosition;
    }
  }

  VoronoiOutput octave;
  octave.distance = minDistance;
  octave.color = hash_float_to_float3(cellPosition + targetOffset);
  octave.position = voronoi_position(targetPosition + cellPosition);
  return octave;
}

VoronoiOutput voronoi_smooth_f1(const VoronoiParams &params,
                                const float coord,
                                const bool calc_color)
{
  float cellPosition = floorf(coord);
  float localPosition = coord - cellPosition;

  float smoothDistance = 0.0f;
  float smoothPosition = 0.0f;
  float3 smoothColor = {0.0f, 0.0f, 0.0f};
  float h = -1.0f;
  for (int i = -2; i <= 2; i++) {
    float cellOffset = i;
    float pointPosition = cellOffset +
                          hash_float_to_float(cellPosition + cellOffset) * params.randomness;
    float distanceToPoint = voronoi_distance(pointPosition, localPosition);
    h = h == -1.0f ?
            1.0f :
            smoothstep(
                0.0f, 1.0f, 0.5f + 0.5f * (smoothDistance - distanceToPoint) / params.smoothness);
    float correctionFactor = params.smoothness * h * (1.0f - h);
    smoothDistance = mix(smoothDistance, distanceToPoint, h) - correctionFactor;
    correctionFactor /= 1.0f + 3.0f * params.smoothness;
    float3 cellColor = hash_float_to_float3(cellPosition + cellOffset);
    if (calc_color) {
      /* Only compute Color output if necessary, as it is very expensive. */
      smoothColor = mix(smoothColor, cellColor, h) - correctionFactor;
    }
    smoothPosition = mix(smoothPosition, pointPosition, h) - correctionFactor;
  }

  VoronoiOutput octave;
  octave.distance = smoothDistance;
  octave.color = smoothColor;
  octave.position = voronoi_position(cellPosition + smoothPosition);
  return octave;
}

VoronoiOutput voronoi_f2(const VoronoiParams &params, const float coord)
{
  float cellPosition = floorf(coord);
  float localPosition = coord - cellPosition;

  float distanceF1 = FLT_MAX;
  float distanceF2 = FLT_MAX;
  float offsetF1 = 0.0f;
  float positionF1 = 0.0f;
  float offsetF2 = 0.0f;
  float positionF2 = 0.0f;
  for (int i = -1; i <= 1; i++) {
    float cellOffset = i;
    float pointPosition = cellOffset +
                          hash_float_to_float(cellPosition + cellOffset) * params.randomness;
    float distanceToPoint = voronoi_distance(pointPosition, localPosition);
    if (distanceToPoint < distanceF1) {
      distanceF2 = distanceF1;
      distanceF1 = distanceToPoint;
      offsetF2 = offsetF1;
      offsetF1 = cellOffset;
      positionF2 = positionF1;
      positionF1 = pointPosition;
    }
    else if (distanceToPoint < distanceF2) {
      distanceF2 = distanceToPoint;
      offsetF2 = cellOffset;
      positionF2 = pointPosition;
    }
  }

  VoronoiOutput octave;
  octave.distance = distanceF2;
  octave.color = hash_float_to_float3(cellPosition + offsetF2);
  octave.position = voronoi_position(positionF2 + cellPosition);
  return octave;
}

float voronoi_distance_to_edge(const VoronoiParams &params, const float coord)
{
  float cellPosition = floorf(coord);
  float localPosition = coord - cellPosition;

  float midPointPosition = hash_float_to_float(cellPosition) * params.randomness;
  float leftPointPosition = -1.0f + hash_float_to_float(cellPosition - 1.0f) * params.randomness;
  float rightPointPosition = 1.0f + hash_float_to_float(cellPosition + 1.0f) * params.randomness;
  float distanceToMidLeft = fabsf((midPointPosition + leftPointPosition) / 2.0f - localPosition);
  float distanceToMidRight = fabsf((midPointPosition + rightPointPosition) / 2.0f - localPosition);

  return math::min(distanceToMidLeft, distanceToMidRight);
}

float voronoi_n_sphere_radius(const VoronoiParams &params, const float coord)
{
  float cellPosition = floorf(coord);
  float localPosition = coord - cellPosition;

  float closestPoint = 0.0f;
  float closestPointOffset = 0.0f;
  float minDistance = FLT_MAX;
  for (int i = -1; i <= 1; i++) {
    float cellOffset = i;
    float pointPosition = cellOffset +
                          hash_float_to_float(cellPosition + cellOffset) * params.randomness;
    float distanceToPoint = fabsf(pointPosition - localPosition);
    if (distanceToPoint < minDistance) {
      minDistance = distanceToPoint;
      closestPoint = pointPosition;
      closestPointOffset = cellOffset;
    }
  }

  minDistance = FLT_MAX;
  float closestPointToClosestPoint = 0.0f;
  for (int i = -1; i <= 1; i++) {
    if (i == 0) {
      continue;
    }
    float cellOffset = i + closestPointOffset;
    float pointPosition = cellOffset +
                          hash_float_to_float(cellPosition + cellOffset) * params.randomness;
    float distanceToPoint = fabsf(closestPoint - pointPosition);
    if (distanceToPoint < minDistance) {
      minDistance = distanceToPoint;
      closestPointToClosestPoint = pointPosition;
    }
  }

  return fabsf(closestPointToClosestPoint - closestPoint) / 2.0f;
}

/* **** 2D Voronoi **** */

float4 voronoi_position(const float2 coord)
{
  return {coord.x, coord.y, 0.0f, 0.0f};
}

VoronoiOutput voronoi_f1(const VoronoiParams &params, const float2 coord)
{
  float2 cellPosition = math::floor(coord);
  float2 localPosition = coord - cellPosition;

  float minDistance = FLT_MAX;
  float2 targetOffset = {0.0f, 0.0f};
  float2 targetPosition = {0.0f, 0.0f};
  for (int j = -1; j <= 1; j++) {
    for (int i = -1; i <= 1; i++) {
      float2 cellOffset(i, j);
      float2 pointPosition = cellOffset +
                             hash_float_to_float2(cellPosition + cellOffset) * params.randomness;
      float distanceToPoint = voronoi_distance(pointPosition, localPosition, params);
      if (distanceToPoint < minDistance) {
        targetOffset = cellOffset;
        minDistance = distanceToPoint;
        targetPosition = pointPosition;
      }
    }
  }

  VoronoiOutput octave;
  octave.distance = minDistance;
  octave.color = hash_float_to_float3(cellPosition + targetOffset);
  octave.position = voronoi_position(targetPosition + cellPosition);
  return octave;
}

VoronoiOutput voronoi_smooth_f1(const VoronoiParams &params,
                                const float2 coord,
                                const bool calc_color)
{
  float2 cellPosition = math::floor(coord);
  float2 localPosition = coord - cellPosition;

  float smoothDistance = 0.0f;
  float3 smoothColor = {0.0f, 0.0f, 0.0f};
  float2 smoothPosition = {0.0f, 0.0f};
  float h = -1.0f;
  for (int j = -2; j <= 2; j++) {
    for (int i = -2; i <= 2; i++) {
      float2 cellOffset(i, j);
      float2 pointPosition = cellOffset +
                             hash_float_to_float2(cellPosition + cellOffset) * params.randomness;
      float distanceToPoint = voronoi_distance(pointPosition, localPosition, params);
      h = h == -1.0f ?
              1.0f :
              smoothstep(0.0f,
                         1.0f,
                         0.5f + 0.5f * (smoothDistance - distanceToPoint) / params.smoothness);
      float correctionFactor = params.smoothness * h * (1.0f - h);
      smoothDistance = mix(smoothDistance, distanceToPoint, h) - correctionFactor;
      correctionFactor /= 1.0f + 3.0f * params.smoothness;
      if (calc_color) {
        /* Only compute Color output if necessary, as it is very expensive. */
        float3 cellColor = hash_float_to_float3(cellPosition + cellOffset);
        smoothColor = mix(smoothColor, cellColor, h) - correctionFactor;
      }
      smoothPosition = mix(smoothPosition, pointPosition, h) - correctionFactor;
    }
  }

  VoronoiOutput octave;
  octave.distance = smoothDistance;
  octave.color = smoothColor;
  octave.position = voronoi_position(cellPosition + smoothPosition);
  return octave;
}

VoronoiOutput voronoi_f2(const VoronoiParams &params, const float2 coord)
{
  float2 cellPosition = math::floor(coord);
  float2 localPosition = coord - cellPosition;

  float distanceF1 = FLT_MAX;
  float distanceF2 = FLT_MAX;
  float2 offsetF1 = {0.0f, 0.0f};
  float2 positionF1 = {0.0f, 0.0f};
  float2 offsetF2 = {0.0f, 0.0f};
  float2 positionF2 = {0.0f, 0.0f};
  for (int j = -1; j <= 1; j++) {
    for (int i = -1; i <= 1; i++) {
      float2 cellOffset(i, j);
      float2 pointPosition = cellOffset +
                             hash_float_to_float2(cellPosition + cellOffset) * params.randomness;
      float distanceToPoint = voronoi_distance(pointPosition, localPosition, params);
      if (distanceToPoint < distanceF1) {
        distanceF2 = distanceF1;
        distanceF1 = distanceToPoint;
        offsetF2 = offsetF1;
        offsetF1 = cellOffset;
        positionF2 = positionF1;
        positionF1 = pointPosition;
      }
      else if (distanceToPoint < distanceF2) {
        distanceF2 = distanceToPoint;
        offsetF2 = cellOffset;
        positionF2 = pointPosition;
      }
    }
  }

  VoronoiOutput octave;
  octave.distance = distanceF2;
  octave.color = hash_float_to_float3(cellPosition + offsetF2);
  octave.position = voronoi_position(positionF2 + cellPosition);
  return octave;
}

float voronoi_distance_to_edge(const VoronoiParams &params, const float2 coord)
{
  float2 cellPosition = math::floor(coord);
  float2 localPosition = coord - cellPosition;

  float2 vectorToClosest = {0.0f, 0.0f};
  float minDistance = FLT_MAX;
  for (int j = -1; j <= 1; j++) {
    for (int i = -1; i <= 1; i++) {
      float2 cellOffset(i, j);
      float2 vectorToPoint = cellOffset +
                             hash_float_to_float2(cellPosition + cellOffset) * params.randomness -
                             localPosition;
      float distanceToPoint = math::dot(vectorToPoint, vectorToPoint);
      if (distanceToPoint < minDistance) {
        minDistance = distanceToPoint;
        vectorToClosest = vectorToPoint;
      }
    }
  }

  minDistance = FLT_MAX;
  for (int j = -1; j <= 1; j++) {
    for (int i = -1; i <= 1; i++) {
      float2 cellOffset(i, j);
      float2 vectorToPoint = cellOffset +
                             hash_float_to_float2(cellPosition + cellOffset) * params.randomness -
                             localPosition;
      float2 perpendicularToEdge = vectorToPoint - vectorToClosest;
      if (math::dot(perpendicularToEdge, perpendicularToEdge) > 0.0001f) {
        float distanceToEdge = math::dot((vectorToClosest + vectorToPoint) / 2.0f,
                                         math::normalize(perpendicularToEdge));
        minDistance = math::min(minDistance, distanceToEdge);
      }
    }
  }

  return minDistance;
}

float voronoi_n_sphere_radius(const VoronoiParams &params, const float2 coord)
{
  float2 cellPosition = math::floor(coord);
  float2 localPosition = coord - cellPosition;

  float2 closestPoint = {0.0f, 0.0f};
  float2 closestPointOffset = {0.0f, 0.0f};
  float minDistance = FLT_MAX;
  for (int j = -1; j <= 1; j++) {
    for (int i = -1; i <= 1; i++) {
      float2 cellOffset(i, j);
      float2 pointPosition = cellOffset +
                             hash_float_to_float2(cellPosition + cellOffset) * params.randomness;
      float distanceToPoint = math::distance(pointPosition, localPosition);
      if (distanceToPoint < minDistance) {
        minDistance = distanceToPoint;
        closestPoint = pointPosition;
        closestPointOffset = cellOffset;
      }
    }
  }

  minDistance = FLT_MAX;
  float2 closestPointToClosestPoint = {0.0f, 0.0f};
  for (int j = -1; j <= 1; j++) {
    for (int i = -1; i <= 1; i++) {
      if (i == 0 && j == 0) {
        continue;
      }
      float2 cellOffset = float2(i, j) + closestPointOffset;
      float2 pointPosition = cellOffset +
                             hash_float_to_float2(cellPosition + cellOffset) * params.randomness;
      float distanceToPoint = math::distance(closestPoint, pointPosition);
      if (distanceToPoint < minDistance) {
        minDistance = distanceToPoint;
        closestPointToClosestPoint = pointPosition;
      }
    }
  }

  return math::distance(closestPointToClosestPoint, closestPoint) / 2.0f;
}

/* **** 3D Voronoi **** */

float4 voronoi_position(const float3 coord)
{
  return {coord.x, coord.y, coord.z, 0.0f};
}

VoronoiOutput voronoi_f1(const VoronoiParams &params, const float3 coord)
{
  float3 cellPosition = math::floor(coord);
  float3 localPosition = coord - cellPosition;

  float minDistance = FLT_MAX;
  float3 targetOffset = {0.0f, 0.0f, 0.0f};
  float3 targetPosition = {0.0f, 0.0f, 0.0f};
  for (int k = -1; k <= 1; k++) {
    for (int j = -1; j <= 1; j++) {
      for (int i = -1; i <= 1; i++) {
        float3 cellOffset(i, j, k);
        float3 pointPosition = cellOffset +
                               hash_float_to_float3(cellPosition + cellOffset) * params.randomness;
        float distanceToPoint = voronoi_distance(pointPosition, localPosition, params);
        if (distanceToPoint < minDistance) {
          targetOffset = cellOffset;
          minDistance = distanceToPoint;
          targetPosition = pointPosition;
        }
      }
    }
  }

  VoronoiOutput octave;
  octave.distance = minDistance;
  octave.color = hash_float_to_float3(cellPosition + targetOffset);
  octave.position = voronoi_position(targetPosition + cellPosition);
  return octave;
}

VoronoiOutput voronoi_smooth_f1(const VoronoiParams &params,
                                const float3 coord,
                                const bool calc_color)
{
  float3 cellPosition = math::floor(coord);
  float3 localPosition = coord - cellPosition;

  float smoothDistance = 0.0f;
  float3 smoothColor = {0.0f, 0.0f, 0.0f};
  float3 smoothPosition = {0.0f, 0.0f, 0.0f};
  float h = -1.0f;
  for (int k = -2; k <= 2; k++) {
    for (int j = -2; j <= 2; j++) {
      for (int i = -2; i <= 2; i++) {
        float3 cellOffset(i, j, k);
        float3 pointPosition = cellOffset +
                               hash_float_to_float3(cellPosition + cellOffset) * params.randomness;
        float distanceToPoint = voronoi_distance(pointPosition, localPosition, params);
        h = h == -1.0f ?
                1.0f :
                smoothstep(0.0f,
                           1.0f,
                           0.5f + 0.5f * (smoothDistance - distanceToPoint) / params.smoothness);
        float correctionFactor = params.smoothness * h * (1.0f - h);
        smoothDistance = mix(smoothDistance, distanceToPoint, h) - correctionFactor;
        correctionFactor /= 1.0f + 3.0f * params.smoothness;
        if (calc_color) {
          /* Only compute Color output if necessary, as it is very expensive. */
          float3 cellColor = hash_float_to_float3(cellPosition + cellOffset);
          smoothColor = mix(smoothColor, cellColor, h) - correctionFactor;
        }
        smoothPosition = mix(smoothPosition, pointPosition, h) - correctionFactor;
      }
    }
  }

  VoronoiOutput octave;
  octave.distance = smoothDistance;
  octave.color = smoothColor;
  octave.position = voronoi_position(cellPosition + smoothPosition);
  return octave;
}

VoronoiOutput voronoi_f2(const VoronoiParams &params, const float3 coord)
{
  float3 cellPosition = math::floor(coord);
  float3 localPosition = coord - cellPosition;

  float distanceF1 = FLT_MAX;
  float distanceF2 = FLT_MAX;
  float3 offsetF1 = {0.0f, 0.0f, 0.0f};
  float3 positionF1 = {0.0f, 0.0f, 0.0f};
  float3 offsetF2 = {0.0f, 0.0f, 0.0f};
  float3 positionF2 = {0.0f, 0.0f, 0.0f};
  for (int k = -1; k <= 1; k++) {
    for (int j = -1; j <= 1; j++) {
      for (int i = -1; i <= 1; i++) {
        float3 cellOffset(i, j, k);
        float3 pointPosition = cellOffset +
                               hash_float_to_float3(cellPosition + cellOffset) * params.randomness;
        float distanceToPoint = voronoi_distance(pointPosition, localPosition, params);
        if (distanceToPoint < distanceF1) {
          distanceF2 = distanceF1;
          distanceF1 = distanceToPoint;
          offsetF2 = offsetF1;
          offsetF1 = cellOffset;
          positionF2 = positionF1;
          positionF1 = pointPosition;
        }
        else if (distanceToPoint < distanceF2) {
          distanceF2 = distanceToPoint;
          offsetF2 = cellOffset;
          positionF2 = pointPosition;
        }
      }
    }
  }

  VoronoiOutput octave;
  octave.distance = distanceF2;
  octave.color = hash_float_to_float3(cellPosition + offsetF2);
  octave.position = voronoi_position(positionF2 + cellPosition);
  return octave;
}

float voronoi_distance_to_edge(const VoronoiParams &params, const float3 coord)
{
  float3 cellPosition = math::floor(coord);
  float3 localPosition = coord - cellPosition;

  float3 vectorToClosest = {0.0f, 0.0f, 0.0f};
  float minDistance = FLT_MAX;
  for (int k = -1; k <= 1; k++) {
    for (int j = -1; j <= 1; j++) {
      for (int i = -1; i <= 1; i++) {
        float3 cellOffset(i, j, k);
        float3 vectorToPoint = cellOffset +
                               hash_float_to_float3(cellPosition + cellOffset) *
                                   params.randomness -
                               localPosition;
        float distanceToPoint = math::dot(vectorToPoint, vectorToPoint);
        if (distanceToPoint < minDistance) {
          minDistance = distanceToPoint;
          vectorToClosest = vectorToPoint;
        }
      }
    }
  }

  minDistance = FLT_MAX;
  for (int k = -1; k <= 1; k++) {
    for (int j = -1; j <= 1; j++) {
      for (int i = -1; i <= 1; i++) {
        float3 cellOffset(i, j, k);
        float3 vectorToPoint = cellOffset +
                               hash_float_to_float3(cellPosition + cellOffset) *
                                   params.randomness -
                               localPosition;
        float3 perpendicularToEdge = vectorToPoint - vectorToClosest;
        if (math::dot(perpendicularToEdge, perpendicularToEdge) > 0.0001f) {
          float distanceToEdge = math::dot((vectorToClosest + vectorToPoint) / 2.0f,
                                           math::normalize(perpendicularToEdge));
          minDistance = math::min(minDistance, distanceToEdge);
        }
      }
    }
  }

  return minDistance;
}

float voronoi_n_sphere_radius(const VoronoiParams &params, const float3 coord)
{
  float3 cellPosition = math::floor(coord);
  float3 localPosition = coord - cellPosition;

  float3 closestPoint = {0.0f, 0.0f, 0.0f};
  float3 closestPointOffset = {0.0f, 0.0f, 0.0f};
  float minDistance = FLT_MAX;
  for (int k = -1; k <= 1; k++) {
    for (int j = -1; j <= 1; j++) {
      for (int i = -1; i <= 1; i++) {
        float3 cellOffset(i, j, k);
        float3 pointPosition = cellOffset +
                               hash_float_to_float3(cellPosition + cellOffset) * params.randomness;
        float distanceToPoint = math::distance(pointPosition, localPosition);
        if (distanceToPoint < minDistance) {
          minDistance = distanceToPoint;
          closestPoint = pointPosition;
          closestPointOffset = cellOffset;
        }
      }
    }
  }

  minDistance = FLT_MAX;
  float3 closestPointToClosestPoint = {0.0f, 0.0f, 0.0f};
  for (int k = -1; k <= 1; k++) {
    for (int j = -1; j <= 1; j++) {
      for (int i = -1; i <= 1; i++) {
        if (i == 0 && j == 0 && k == 0) {
          continue;
        }
        float3 cellOffset = float3(i, j, k) + closestPointOffset;
        float3 pointPosition = cellOffset +
                               hash_float_to_float3(cellPosition + cellOffset) * params.randomness;
        float distanceToPoint = math::distance(closestPoint, pointPosition);
        if (distanceToPoint < minDistance) {
          minDistance = distanceToPoint;
          closestPointToClosestPoint = pointPosition;
        }
      }
    }
  }

  return math::distance(closestPointToClosestPoint, closestPoint) / 2.0f;
}

/* **** 4D Voronoi **** */

float4 voronoi_position(const float4 coord)
{
  return coord;
}

VoronoiOutput voronoi_f1(const VoronoiParams &params, const float4 coord)
{
  float4 cellPosition = math::floor(coord);
  float4 localPosition = coord - cellPosition;

  float minDistance = FLT_MAX;
  float4 targetOffset = {0.0f, 0.0f, 0.0f, 0.0f};
  float4 targetPosition = {0.0f, 0.0f, 0.0f, 0.0f};
  for (int u = -1; u <= 1; u++) {
    for (int k = -1; k <= 1; k++) {
      for (int j = -1; j <= 1; j++) {
        for (int i = -1; i <= 1; i++) {
          float4 cellOffset(i, j, k, u);
          float4 pointPosition = cellOffset + hash_float_to_float4(cellPosition + cellOffset) *
                                                  params.randomness;
          float distanceToPoint = voronoi_distance(pointPosition, localPosition, params);
          if (distanceToPoint < minDistance) {
            targetOffset = cellOffset;
            minDistance = distanceToPoint;
            targetPosition = pointPosition;
          }
        }
      }
    }
  }

  VoronoiOutput octave;
  octave.distance = minDistance;
  octave.color = hash_float_to_float3(cellPosition + targetOffset);
  octave.position = voronoi_position(targetPosition + cellPosition);
  return octave;
}

VoronoiOutput voronoi_smooth_f1(const VoronoiParams &params,
                                const float4 coord,
                                const bool calc_color)
{
  float4 cellPosition = math::floor(coord);
  float4 localPosition = coord - cellPosition;

  float smoothDistance = 0.0f;
  float3 smoothColor = {0.0f, 0.0f, 0.0f};
  float4 smoothPosition = {0.0f, 0.0f, 0.0f, 0.0f};
  float h = -1.0f;
  for (int u = -2; u <= 2; u++) {
    for (int k = -2; k <= 2; k++) {
      for (int j = -2; j <= 2; j++) {
        for (int i = -2; i <= 2; i++) {
          float4 cellOffset(i, j, k, u);
          float4 pointPosition = cellOffset + hash_float_to_float4(cellPosition + cellOffset) *
                                                  params.randomness;
          float distanceToPoint = voronoi_distance(pointPosition, localPosition, params);
          h = h == -1.0f ?
                  1.0f :
                  smoothstep(0.0f,
                             1.0f,
                             0.5f + 0.5f * (smoothDistance - distanceToPoint) / params.smoothness);
          float correctionFactor = params.smoothness * h * (1.0f - h);
          smoothDistance = mix(smoothDistance, distanceToPoint, h) - correctionFactor;
          correctionFactor /= 1.0f + 3.0f * params.smoothness;
          if (calc_color) {
            /* Only compute Color output if necessary, as it is very expensive. */
            float3 cellColor = hash_float_to_float3(cellPosition + cellOffset);
            smoothColor = mix(smoothColor, cellColor, h) - correctionFactor;
          }
          smoothPosition = mix(smoothPosition, pointPosition, h) - correctionFactor;
        }
      }
    }
  }

  VoronoiOutput octave;
  octave.distance = smoothDistance;
  octave.color = smoothColor;
  octave.position = voronoi_position(cellPosition + smoothPosition);
  return octave;
}

VoronoiOutput voronoi_f2(const VoronoiParams &params, const float4 coord)
{
  float4 cellPosition = math::floor(coord);
  float4 localPosition = coord - cellPosition;

  float distanceF1 = FLT_MAX;
  float distanceF2 = FLT_MAX;
  float4 offsetF1 = {0.0f, 0.0f, 0.0f, 0.0f};
  float4 positionF1 = {0.0f, 0.0f, 0.0f, 0.0f};
  float4 offsetF2 = {0.0f, 0.0f, 0.0f, 0.0f};
  float4 positionF2 = {0.0f, 0.0f, 0.0f, 0.0f};
  for (int u = -1; u <= 1; u++) {
    for (int k = -1; k <= 1; k++) {
      for (int j = -1; j <= 1; j++) {
        for (int i = -1; i <= 1; i++) {
          float4 cellOffset(i, j, k, u);
          float4 pointPosition = cellOffset + hash_float_to_float4(cellPosition + cellOffset) *
                                                  params.randomness;
          float distanceToPoint = voronoi_distance(pointPosition, localPosition, params);
          if (distanceToPoint < distanceF1) {
            distanceF2 = distanceF1;
            distanceF1 = distanceToPoint;
            offsetF2 = offsetF1;
            offsetF1 = cellOffset;
            positionF2 = positionF1;
            positionF1 = pointPosition;
          }
          else if (distanceToPoint < distanceF2) {
            distanceF2 = distanceToPoint;
            offsetF2 = cellOffset;
            positionF2 = pointPosition;
          }
        }
      }
    }
  }

  VoronoiOutput octave;
  octave.distance = distanceF2;
  octave.color = hash_float_to_float3(cellPosition + offsetF2);
  octave.position = voronoi_position(positionF2 + cellPosition);
  return octave;
}

float voronoi_distance_to_edge(const VoronoiParams &params, const float4 coord)
{
  float4 cellPosition = math::floor(coord);
  float4 localPosition = coord - cellPosition;

  float4 vectorToClosest = {0.0f, 0.0f, 0.0f, 0.0f};
  float minDistance = FLT_MAX;
  for (int u = -1; u <= 1; u++) {
    for (int k = -1; k <= 1; k++) {
      for (int j = -1; j <= 1; j++) {
        for (int i = -1; i <= 1; i++) {
          float4 cellOffset(i, j, k, u);
          float4 vectorToPoint = cellOffset +
                                 hash_float_to_float4(cellPosition + cellOffset) *
                                     params.randomness -
                                 localPosition;
          float distanceToPoint = math::dot(vectorToPoint, vectorToPoint);
          if (distanceToPoint < minDistance) {
            minDistance = distanceToPoint;
            vectorToClosest = vectorToPoint;
          }
        }
      }
    }
  }

  minDistance = FLT_MAX;
  for (int u = -1; u <= 1; u++) {
    for (int k = -1; k <= 1; k++) {
      for (int j = -1; j <= 1; j++) {
        for (int i = -1; i <= 1; i++) {
          float4 cellOffset(i, j, k, u);
          float4 vectorToPoint = cellOffset +
                                 hash_float_to_float4(cellPosition + cellOffset) *
                                     params.randomness -
                                 localPosition;
          float4 perpendicularToEdge = vectorToPoint - vectorToClosest;
          if (math::dot(perpendicularToEdge, perpendicularToEdge) > 0.0001f) {
            float distanceToEdge = math::dot((vectorToClosest + vectorToPoint) / 2.0f,
                                             math::normalize(perpendicularToEdge));
            minDistance = math::min(minDistance, distanceToEdge);
          }
        }
      }
    }
  }

  return minDistance;
}

float voronoi_n_sphere_radius(const VoronoiParams &params, const float4 coord)
{
  float4 cellPosition = math::floor(coord);
  float4 localPosition = coord - cellPosition;

  float4 closestPoint = {0.0f, 0.0f, 0.0f, 0.0f};
  float4 closestPointOffset = {0.0f, 0.0f, 0.0f, 0.0f};
  float minDistance = FLT_MAX;
  for (int u = -1; u <= 1; u++) {
    for (int k = -1; k <= 1; k++) {
      for (int j = -1; j <= 1; j++) {
        for (int i = -1; i <= 1; i++) {
          float4 cellOffset(i, j, k, u);
          float4 pointPosition = cellOffset + hash_float_to_float4(cellPosition + cellOffset) *
                                                  params.randomness;
          float distanceToPoint = math::distance(pointPosition, localPosition);
          if (distanceToPoint < minDistance) {
            minDistance = distanceToPoint;
            closestPoint = pointPosition;
            closestPointOffset = cellOffset;
          }
        }
      }
    }
  }

  minDistance = FLT_MAX;
  float4 closestPointToClosestPoint = {0.0f, 0.0f, 0.0f, 0.0f};
  for (int u = -1; u <= 1; u++) {
    for (int k = -1; k <= 1; k++) {
      for (int j = -1; j <= 1; j++) {
        for (int i = -1; i <= 1; i++) {
          if (i == 0 && j == 0 && k == 0 && u == 0) {
            continue;
          }
          float4 cellOffset = float4(i, j, k, u) + closestPointOffset;
          float4 pointPosition = cellOffset + hash_float_to_float4(cellPosition + cellOffset) *
                                                  params.randomness;
          float distanceToPoint = math::distance(closestPoint, pointPosition);
          if (distanceToPoint < minDistance) {
            minDistance = distanceToPoint;
            closestPointToClosestPoint = pointPosition;
          }
        }
      }
    }
  }

  return math::distance(closestPointToClosestPoint, closestPoint) / 2.0f;
}

/* **** Fractal Voronoi **** */

/* The fractalization logic is the same as for fBM Noise, except that some additions are replaced
 * by lerps. */
template<typename T>
VoronoiOutput fractal_voronoi_x_fx(const VoronoiParams &params,
                                   const T coord,
                                   const bool calc_color /* Only used to optimize Smooth F1 */)
{
  float amplitude = 1.0f;
  float max_amplitude = 0.0f;
  float scale = 1.0f;

  VoronoiOutput output;
  const bool zero_input = params.detail == 0.0f || params.roughness == 0.0f;

  for (int i = 0; i <= ceilf(params.detail); ++i) {
    VoronoiOutput octave = (params.feature == NOISE_SHD_VORONOI_F2) ?
                               voronoi_f2(params, coord * scale) :
                           (params.feature == NOISE_SHD_VORONOI_SMOOTH_F1 &&
                            params.smoothness != 0.0f) ?
                               voronoi_smooth_f1(params, coord * scale, calc_color) :
                               voronoi_f1(params, coord * scale);

    if (zero_input) {
      max_amplitude = 1.0f;
      output = octave;
      break;
    }
    if (i <= params.detail) {
      max_amplitude += amplitude;
      output.distance += octave.distance * amplitude;
      output.color += octave.color * amplitude;
      output.position = mix(output.position, octave.position / scale, amplitude);
      scale *= params.lacunarity;
      amplitude *= params.roughness;
    }
    else {
      float remainder = params.detail - floorf(params.detail);
      if (remainder != 0.0f) {
        max_amplitude = mix(max_amplitude, max_amplitude + amplitude, remainder);
        output.distance = mix(
            output.distance, output.distance + octave.distance * amplitude, remainder);
        output.color = mix(output.color, output.color + octave.color * amplitude, remainder);
        output.position = mix(
            output.position, mix(output.position, octave.position / scale, amplitude), remainder);
      }
    }
  }

  if (params.normalize) {
    output.distance /= max_amplitude * params.max_distance;
    output.color /= max_amplitude;
  }

  output.position = (params.scale != 0.0f) ? output.position / params.scale :
                                             float4{0.0f, 0.0f, 0.0f, 0.0f};

  return output;
}

/* The fractalization logic is the same as for fBM Noise, except that some additions are replaced
 * by lerps. */
template<typename T>
float fractal_voronoi_distance_to_edge(const VoronoiParams &params, const T coord)
{
  float amplitude = 1.0f;
  float max_amplitude = params.max_distance;
  float scale = 1.0f;
  float distance = 8.0f;

  const bool zero_input = params.detail == 0.0f || params.roughness == 0.0f;

  for (int i = 0; i <= ceilf(params.detail); ++i) {
    const float octave_distance = voronoi_distance_to_edge(params, coord * scale);

    if (zero_input) {
      distance = octave_distance;
      break;
    }
    if (i <= params.detail) {
      max_amplitude = mix(max_amplitude, params.max_distance / scale, amplitude);
      distance = mix(distance, math::min(distance, octave_distance / scale), amplitude);
      scale *= params.lacunarity;
      amplitude *= params.roughness;
    }
    else {
      float remainder = params.detail - floorf(params.detail);
      if (remainder != 0.0f) {
        float lerp_amplitude = mix(max_amplitude, params.max_distance / scale, amplitude);
        max_amplitude = mix(max_amplitude, lerp_amplitude, remainder);
        float lerp_distance = mix(
            distance, math::min(distance, octave_distance / scale), amplitude);
        distance = mix(distance, math::min(distance, lerp_distance), remainder);
      }
    }
  }

  if (params.normalize) {
    distance /= max_amplitude;
  }

  return distance;
}

/* Explicit function template instantiation */

template VoronoiOutput fractal_voronoi_x_fx<float>(const VoronoiParams &params,
                                                   const float coord,
                                                   const bool calc_color);
template VoronoiOutput fractal_voronoi_x_fx<float2>(const VoronoiParams &params,
                                                    const float2 coord,
                                                    const bool calc_color);
template VoronoiOutput fractal_voronoi_x_fx<float3>(const VoronoiParams &params,
                                                    const float3 coord,
                                                    const bool calc_color);
template VoronoiOutput fractal_voronoi_x_fx<float4>(const VoronoiParams &params,
                                                    const float4 coord,
                                                    const bool calc_color);

template float fractal_voronoi_distance_to_edge<float>(const VoronoiParams &params,
                                                       const float coord);
template float fractal_voronoi_distance_to_edge<float2>(const VoronoiParams &params,
                                                        const float2 coord);
template float fractal_voronoi_distance_to_edge<float3>(const VoronoiParams &params,
                                                        const float3 coord);
template float fractal_voronoi_distance_to_edge<float4>(const VoronoiParams &params,
                                                        const float4 coord);
/** \} */

/* -------------------------------------------------------------------- */
/** \name Raiko Texture
 *
 * \note Ported from Cycles code.
 *
 * Original code is under the Apache-2.0 License, Copyright (c) 2024 Tenkai Raiko.
 * \{ */

enum {
  SHD_RAIKO_ADDITIVE = 0,
  SHD_RAIKO_CLOSEST = 1,
  SHD_RAIKO_SMOOTH_MINIMUM = 2,
};

/* Fast Cramer Coefficients of the 3x3 matrix M. */
struct Fcc_3x3 {
  /* Determinant of the 3x3 matrix M. */
  float M_det;
  /* Determinant of the 2x2 submatrices M_i_j. The first index i denotes the row, the second index
   * j the coloumn being removed from M. */
  float M_1_1_det;
  float M_2_1_det;
  float M_3_1_det;
};

/* Fast Cramer Coefficients of the 4x4 matrix M. */
struct Fcc_4x4 {
  /* Determinant of the 4x4 matrix M. */
  float M_det;
  /* Fast Cramer Coefficients of the 3x3 submatrices M_i_j. The first index i denotes the row, the
   * second index j the coloumn being removed from M. */
  Fcc_3x3 M_1_1_fcc;
  Fcc_3x3 M_2_1_fcc;
  Fcc_3x3 M_3_1_fcc;
  Fcc_3x3 M_4_1_fcc;
};

Fcc_3x3 calculate_Fcc_3x3(float3 a_1, float3 a_2, float3 a_3)
{
  Fcc_3x3 A_fcc;

  A_fcc.M_1_1_det = a_2.y * a_3.z - a_3.y * a_2.z;
  A_fcc.M_2_1_det = a_2.x * a_3.z - a_3.x * a_2.z;
  A_fcc.M_3_1_det = a_2.x * a_3.y - a_3.x * a_2.y;

  A_fcc.M_det = a_1.x * A_fcc.M_1_1_det - a_1.y * A_fcc.M_2_1_det + a_1.z * A_fcc.M_3_1_det;

  return A_fcc;
}

Fcc_4x4 calculate_Fcc_4x4(float4 a_1, float4 a_2, float4 a_3, float4 a_4)
{
  Fcc_4x4 A_fcc;

  A_fcc.M_1_1_fcc = calculate_Fcc_3x3(
      float3(a_2.y, a_2.z, a_2.w), float3(a_3.y, a_3.z, a_3.w), float3(a_4.y, a_4.z, a_4.w));
  A_fcc.M_2_1_fcc = calculate_Fcc_3x3(
      float3(a_2.x, a_2.z, a_2.w), float3(a_3.x, a_3.z, a_3.w), float3(a_4.x, a_4.z, a_4.w));
  A_fcc.M_3_1_fcc = calculate_Fcc_3x3(
      float3(a_2.x, a_2.y, a_2.w), float3(a_3.x, a_3.y, a_3.w), float3(a_4.x, a_4.y, a_4.w));
  A_fcc.M_4_1_fcc = calculate_Fcc_3x3(
      float3(a_2.x, a_2.y, a_2.z), float3(a_3.x, a_3.y, a_3.z), float3(a_4.x, a_4.y, a_4.z));

  A_fcc.M_det = a_1.x * A_fcc.M_1_1_fcc.M_det - a_1.y * A_fcc.M_2_1_fcc.M_det +
                a_1.z * A_fcc.M_3_1_fcc.M_det - a_1.w * A_fcc.M_4_1_fcc.M_det;

  return A_fcc;
}

/* Solves Ax=b for x using the Fast Cramer algorithm, with a_n being the nth coloumn vector of the
 * invertible 2x2 matrix A. fc_linear_system_solve_non_singular_4x4 doesn't check whether or not A
 * is invertible. Calling it on a singular matrix leads to division by 0. */
float2 fc_linear_system_solve_non_singular_2x2(float2 a_1, float2 a_2, float2 b, float M_det)
{
  /* Use Cramer's rule on both components instead of further recursion because it is faster. */
  return float2((b.x * a_2.y - a_2.x * b.y) / M_det, (a_1.x * b.y - b.x * a_1.y) / M_det);
}

/* Solves Ax=b for x using the Fast Cramer algorithm, with a_n being the nth coloumn vector of the
 * invertible 3x3 matrix A. fc_linear_system_solve_non_singular_4x4 doesn't check whether or not A
 * is invertible. Calling it on a singular matrix leads to division by 0. */
float3 fc_linear_system_solve_non_singular_3x3(
    float3 a_1, float3 a_2, float3 a_3, float3 b, Fcc_3x3 A_fcc)
{
  float solution_x = (b.x * A_fcc.M_1_1_det - b.y * A_fcc.M_2_1_det + b.z * A_fcc.M_3_1_det) /
                     A_fcc.M_det;

  if (A_fcc.M_1_1_det != 0.0f) {
    float2 solution_yz = fc_linear_system_solve_non_singular_2x2(
        float2(a_2.y, a_2.z),
        float2(a_3.y, a_3.z),
        float2(b.y - a_1.y * solution_x, b.z - a_1.z * solution_x),
        A_fcc.M_1_1_det);
    return float3(solution_x, solution_yz.x, solution_yz.y);
  }
  else if (A_fcc.M_2_1_det != 0.0f) {
    float2 solution_yz = fc_linear_system_solve_non_singular_2x2(
        float2(a_2.x, a_2.z),
        float2(a_3.x, a_3.z),
        float2(b.x - a_1.x * solution_x, b.z - a_1.z * solution_x),
        A_fcc.M_2_1_det);
    return float3(solution_x, solution_yz.x, solution_yz.y);
  }
  else {
    float2 solution_yz = fc_linear_system_solve_non_singular_2x2(
        float2(a_2.x, a_2.y),
        float2(a_3.x, a_3.y),
        float2(b.x - a_1.x * solution_x, b.y - a_1.y * solution_x),
        A_fcc.M_3_1_det);
    return float3(solution_x, solution_yz.x, solution_yz.y);
  }
}

/* Solves Ax=b for x using the Fast Cramer algorithm, with a_n being the nth coloumn vector of the
 * invertible 4x4 matrix A. fc_linear_system_solve_non_singular_4x4 doesn't check whether or not A
 * is invertible. Calling it on a singular matrix leads to division by 0. */
float4 fc_linear_system_solve_non_singular_4x4(
    float4 a_1, float4 a_2, float4 a_3, float4 a_4, float4 b, Fcc_4x4 A_fcc)
{
  float solution_x = (b.x * A_fcc.M_1_1_fcc.M_det - b.y * A_fcc.M_2_1_fcc.M_det +
                      b.z * A_fcc.M_3_1_fcc.M_det - b.w * A_fcc.M_4_1_fcc.M_det) /
                     A_fcc.M_det;

  if (A_fcc.M_1_1_fcc.M_det != 0.0f) {
    float3 solution_yzw = fc_linear_system_solve_non_singular_3x3(
        float3(a_2.y, a_2.z, a_2.w),
        float3(a_3.y, a_3.z, a_3.w),
        float3(a_4.y, a_4.z, a_4.w),
        float3(b.y - a_1.y * solution_x, b.z - a_1.z * solution_x, b.w - a_1.w * solution_x),
        A_fcc.M_1_1_fcc);
    return float4(solution_x, solution_yzw.x, solution_yzw.y, solution_yzw.z);
  }
  else if (A_fcc.M_2_1_fcc.M_det != 0.0f) {
    float3 solution_yzw = fc_linear_system_solve_non_singular_3x3(
        float3(a_2.x, a_2.z, a_2.w),
        float3(a_3.x, a_3.z, a_3.w),
        float3(a_4.x, a_4.z, a_4.w),
        float3(b.x - a_1.x * solution_x, b.z - a_1.z * solution_x, b.w - a_1.w * solution_x),
        A_fcc.M_2_1_fcc);
    return float4(solution_x, solution_yzw.x, solution_yzw.y, solution_yzw.z);
  }
  else if (A_fcc.M_3_1_fcc.M_det != 0.0f) {
    float3 solution_yzw = fc_linear_system_solve_non_singular_3x3(
        float3(a_2.x, a_2.y, a_2.w),
        float3(a_3.x, a_3.y, a_3.w),
        float3(a_4.x, a_4.y, a_4.w),
        float3(b.x - a_1.x * solution_x, b.y - a_1.y * solution_x, b.w - a_1.w * solution_x),
        A_fcc.M_3_1_fcc);
    return float4(solution_x, solution_yzw.x, solution_yzw.y, solution_yzw.z);
  }
  else {
    float3 solution_yzw = fc_linear_system_solve_non_singular_3x3(
        float3(a_2.x, a_2.y, a_2.z),
        float3(a_3.x, a_3.y, a_3.z),
        float3(a_4.x, a_4.y, a_4.z),
        float3(b.x - a_1.x * solution_x, b.y - a_1.y * solution_x, b.z - a_1.z * solution_x),
        A_fcc.M_4_1_fcc);
    return float4(solution_x, solution_yzw.x, solution_yzw.y, solution_yzw.z);
  }
}

float chebychev_norm(float coord)
{
  return fabsf(coord);
}

float chebychev_norm(float2 coord)
{
  return math::max(fabsf(coord.x), fabsf(coord.y));
}

float chebychev_norm(float3 coord)
{
  return math::max(fabsf(coord.x), math::max(fabsf(coord.y), fabsf(coord.z)));
}

float p_norm(float coord)
{
  return fabsf(coord);
}

float p_norm(float2 coord, float exponent)
{
  /* Use Chebychev norm instead of p-norm for high exponent values to avoid going out of the
   * floating point representable range. */
  return (exponent > 32.0f) ? chebychev_norm(coord) :
                              powf(powf(fabsf(coord.x), exponent) + powf(fabsf(coord.y), exponent),
                                   1.0f / exponent);
}

float p_norm(float3 coord, float exponent)
{
  /* Use Chebychev norm instead of p-norm for high exponent values to avoid going out of the
   * floating point representable range. */
  return (exponent > 32.0f) ?
             chebychev_norm(coord) :
             powf(powf(fabsf(coord.x), exponent) + powf(fabsf(coord.y), exponent) +
                      powf(fabsf(coord.z), exponent),
                  1.0f / exponent);
}

float euclidean_norm(float4 coord)
{
  return sqrtf(math::square(coord.x) + math::square(coord.y) + math::square(coord.z) +
               math::square(coord.w));
}

float calculate_l_angle_bisector_2d_full_roundness_irregular_elliptical(float r_gon_sides,
                                                                        float2 coord,
                                                                        float l_projection_2d)
{
  float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0f) * M_TAU_F;
  float ref_A_angle_bisector = M_PI_F / r_gon_sides;

  float last_angle_bisector_A_x_axis = M_PI_F - floorf(r_gon_sides) * ref_A_angle_bisector;
  float last_ref_A_x_axis = 2.0f * last_angle_bisector_A_x_axis;

  if ((x_axis_A_coord >= ref_A_angle_bisector) &&
      (x_axis_A_coord < M_TAU_F - last_ref_A_x_axis - ref_A_angle_bisector))
  {
    return l_projection_2d;
  }
  else {
    /* MSA == Mirrored Signed Angle. The values are mirrored around the last angle bisector
     * to avoid a case distinction. */
    float nearest_ref_MSA_coord = atan2(coord.y, coord.x);
    if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - ref_A_angle_bisector) &&
        (x_axis_A_coord < M_TAU_F - last_angle_bisector_A_x_axis))
    {
      nearest_ref_MSA_coord += last_ref_A_x_axis;
      nearest_ref_MSA_coord *= -1;
    }
    float l_basis_vector_1 = tan(ref_A_angle_bisector);
    /* When the fractional part of r_gon_sides is very small division by l_basis_vector_2 causes
     * precision issues. Change to double if necessary */
    float l_basis_vector_2 = sin(last_angle_bisector_A_x_axis) *
                             sqrtf(math::square(tan(ref_A_angle_bisector)) + 1.0f);
    float2 ellipse_center =
        float2(cos(ref_A_angle_bisector) / cos(ref_A_angle_bisector - ref_A_angle_bisector),
               sin(ref_A_angle_bisector) / cos(ref_A_angle_bisector - ref_A_angle_bisector)) -
        l_basis_vector_2 *
            float2(sin(last_angle_bisector_A_x_axis), cos(last_angle_bisector_A_x_axis));
    float2 transformed_direction_vector = float2(
        cos(last_angle_bisector_A_x_axis + nearest_ref_MSA_coord) /
            (l_basis_vector_1 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)),
        cos(ref_A_angle_bisector - nearest_ref_MSA_coord) /
            (l_basis_vector_2 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)));
    float2 transformed_origin = float2(
        (ellipse_center.y * sin(last_angle_bisector_A_x_axis) -
         ellipse_center.x * cos(last_angle_bisector_A_x_axis)) /
            (l_basis_vector_1 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)),
        -(ellipse_center.y * sin(ref_A_angle_bisector) +
          ellipse_center.x * cos(ref_A_angle_bisector)) /
            (l_basis_vector_2 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)));
    float l_coord_R_l_angle_bisector_2d =
        (-(transformed_direction_vector.x * transformed_origin.x +
           transformed_direction_vector.y * transformed_origin.y) +
         sqrtf(math::square(transformed_direction_vector.x * transformed_origin.x +
                            transformed_direction_vector.y * transformed_origin.y) -
               (math::square(transformed_direction_vector.x) +
                math::square(transformed_direction_vector.y)) *
                   (math::square(transformed_origin.x) + math::square(transformed_origin.y) -
                    1.0f))) /
        (math::square(transformed_direction_vector.x) +
         math::square(transformed_direction_vector.y));
    return l_projection_2d / l_coord_R_l_angle_bisector_2d;
  }
}

float calculate_l_angle_bisector_2d_irregular_elliptical(float r_gon_sides,
                                                         float r_gon_roundness,
                                                         float2 coord,
                                                         float l_projection_2d)
{
  float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0f) * M_TAU_F;
  float ref_A_angle_bisector = M_PI_F / r_gon_sides;
  float ref_A_next_ref = 2.0f * ref_A_angle_bisector;
  float segment_id = floorf(x_axis_A_coord / ref_A_next_ref);
  float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;
  float ref_A_bevel_start = ref_A_angle_bisector -
                            atan((1 - r_gon_roundness) * tan(ref_A_angle_bisector));

  float last_angle_bisector_A_x_axis = M_PI_F - floorf(r_gon_sides) * ref_A_angle_bisector;
  float last_ref_A_x_axis = 2.0f * last_angle_bisector_A_x_axis;
  float inner_last_bevel_start_A_x_axis = last_angle_bisector_A_x_axis -
                                          atan((1 - r_gon_roundness) *
                                               tan(last_angle_bisector_A_x_axis));

  if ((x_axis_A_coord >= ref_A_bevel_start) &&
      (x_axis_A_coord < M_TAU_F - last_ref_A_x_axis - ref_A_bevel_start))
  {
    if ((ref_A_coord >= ref_A_bevel_start) && (ref_A_coord < ref_A_next_ref - ref_A_bevel_start)) {
      return l_projection_2d * cos(ref_A_angle_bisector - ref_A_coord);
    }
    else {
      /* SA == Signed Angle in [-M_PI_F, M_PI_F]. Counterclockwise angles are positive, clockwise
       * angles are negative.*/
      float nearest_ref_SA_coord = ref_A_coord -
                                   float(ref_A_coord > ref_A_angle_bisector) * ref_A_next_ref;
      float l_circle_radius = sin(ref_A_bevel_start) / sin(ref_A_angle_bisector);
      float l_circle_center = sin(ref_A_angle_bisector - ref_A_bevel_start) /
                              sin(ref_A_angle_bisector);
      float l_coord_R_l_bevel_start =
          cos(nearest_ref_SA_coord) * l_circle_center +
          sqrtf(math::square(cos(nearest_ref_SA_coord) * l_circle_center) +
                math::square(l_circle_radius) - math::square(l_circle_center));
      return cos(ref_A_angle_bisector - ref_A_bevel_start) * l_projection_2d /
             l_coord_R_l_bevel_start;
    }
  }
  else {
    if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis + inner_last_bevel_start_A_x_axis) &&
        (x_axis_A_coord < M_TAU_F - inner_last_bevel_start_A_x_axis))
    {
      float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cos(ref_A_angle_bisector) /
                                                             cos(last_angle_bisector_A_x_axis);
      return l_projection_2d * cos(last_angle_bisector_A_x_axis - ref_A_coord) *
             l_angle_bisector_2d_R_l_last_angle_bisector_2d;
    }
    else {
      /* MSA == Mirrored Signed Angle. The values are mirrored around the last angle bisector
       * to avoid a case distinction. */
      float nearest_ref_MSA_coord = atan2(coord.y, coord.x);
      if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - ref_A_bevel_start) &&
          (x_axis_A_coord < M_TAU_F - last_angle_bisector_A_x_axis))
      {
        nearest_ref_MSA_coord += last_ref_A_x_axis;
        nearest_ref_MSA_coord *= -1;
      }
      float l_basis_vector_1 = r_gon_roundness * tan(ref_A_angle_bisector);
      float l_basis_vector_2 = r_gon_roundness * sin(last_angle_bisector_A_x_axis) *
                               sqrtf(math::square(tan(ref_A_angle_bisector)) + 1.0f);
      float2 ellipse_center =
          float2(cos(ref_A_bevel_start) / cos(ref_A_angle_bisector - ref_A_bevel_start),
                 sin(ref_A_bevel_start) / cos(ref_A_angle_bisector - ref_A_bevel_start)) -
          l_basis_vector_2 *
              float2(sin(last_angle_bisector_A_x_axis), cos(last_angle_bisector_A_x_axis));
      float2 transformed_direction_vector = float2(
          cos(last_angle_bisector_A_x_axis + nearest_ref_MSA_coord) /
              (l_basis_vector_1 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)),
          cos(ref_A_angle_bisector - nearest_ref_MSA_coord) /
              (l_basis_vector_2 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)));
      float2 transformed_origin = float2(
          (ellipse_center.y * sin(last_angle_bisector_A_x_axis) -
           ellipse_center.x * cos(last_angle_bisector_A_x_axis)) /
              (l_basis_vector_1 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)),
          -(ellipse_center.y * sin(ref_A_angle_bisector) +
            ellipse_center.x * cos(ref_A_angle_bisector)) /
              (l_basis_vector_2 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)));
      float l_coord_R_l_angle_bisector_2d =
          (-(transformed_direction_vector.x * transformed_origin.x +
             transformed_direction_vector.y * transformed_origin.y) +
           sqrtf(math::square(transformed_direction_vector.x * transformed_origin.x +
                              transformed_direction_vector.y * transformed_origin.y) -
                 (math::square(transformed_direction_vector.x) +
                  math::square(transformed_direction_vector.y)) *
                     (math::square(transformed_origin.x) + math::square(transformed_origin.y) -
                      1.0f))) /
          (math::square(transformed_direction_vector.x) +
           math::square(transformed_direction_vector.y));
      return l_projection_2d / l_coord_R_l_angle_bisector_2d;
    }
  }
}

float calculate_l_angle_bisector_2d_full_roundness_irregular_circular(float r_gon_sides,
                                                                      float2 coord,
                                                                      float l_projection_2d)
{
  float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0f) * M_TAU_F;
  float ref_A_angle_bisector = M_PI_F / r_gon_sides;
  float ref_A_next_ref = 2.0f * ref_A_angle_bisector;
  float segment_id = floorf(x_axis_A_coord / ref_A_next_ref);
  float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;

  float last_angle_bisector_A_x_axis = M_PI_F - floorf(r_gon_sides) * ref_A_angle_bisector;
  float last_ref_A_x_axis = 2.0f * last_angle_bisector_A_x_axis;
  float l_last_circle_radius = tan(last_angle_bisector_A_x_axis) /
                               tan(0.5f * (ref_A_angle_bisector + last_angle_bisector_A_x_axis));
  float2 last_circle_center = float2(cos(last_angle_bisector_A_x_axis) -
                                         l_last_circle_radius * cos(last_angle_bisector_A_x_axis),
                                     l_last_circle_radius * sin(last_angle_bisector_A_x_axis) -
                                         sin(last_angle_bisector_A_x_axis));
  float2 outer_last_bevel_start = last_circle_center +
                                  l_last_circle_radius *
                                      float2(cos(ref_A_angle_bisector), sin(ref_A_angle_bisector));
  float x_axis_A_outer_last_bevel_start = atan(outer_last_bevel_start.y /
                                               outer_last_bevel_start.x);

  if ((x_axis_A_coord >= x_axis_A_outer_last_bevel_start) &&
      (x_axis_A_coord < M_TAU_F - last_ref_A_x_axis - x_axis_A_outer_last_bevel_start))
  {
    if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - ref_A_angle_bisector) ||
        (x_axis_A_coord < ref_A_angle_bisector))
    {
      return l_projection_2d * cos(ref_A_angle_bisector - ref_A_coord);
    }
    else {
      return l_projection_2d;
    }
  }
  else {
    /* MSA == Mirrored Signed Angle. The values are mirrored around the last angle bisector
     * to avoid a case distinction. */
    float nearest_ref_MSA_coord = atan2(coord.y, coord.x);
    if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - x_axis_A_outer_last_bevel_start) &&
        (x_axis_A_coord < M_TAU_F - last_angle_bisector_A_x_axis))
    {
      nearest_ref_MSA_coord += last_ref_A_x_axis;
      nearest_ref_MSA_coord *= -1;
    }
    float l_coord_R_l_last_angle_bisector_2d =
        sin(nearest_ref_MSA_coord) * last_circle_center.y +
        cos(nearest_ref_MSA_coord) * last_circle_center.x +
        sqrtf(math::square(sin(nearest_ref_MSA_coord) * last_circle_center.y +
                           cos(nearest_ref_MSA_coord) * last_circle_center.x) +
              math::square(l_last_circle_radius) - math::square(last_circle_center.x) -
              math::square(last_circle_center.y));
    return (cos(ref_A_angle_bisector) * l_projection_2d) /
           (cos(last_angle_bisector_A_x_axis) * l_coord_R_l_last_angle_bisector_2d);
  }
}

float calculate_l_angle_bisector_2d_irregular_circular(float r_gon_sides,
                                                       float r_gon_roundness,
                                                       float2 coord,
                                                       float l_projection_2d)
{
  float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0f) * M_TAU_F;
  float ref_A_angle_bisector = M_PI_F / r_gon_sides;
  float ref_A_next_ref = 2.0f * ref_A_angle_bisector;
  float segment_id = floorf(x_axis_A_coord / ref_A_next_ref);
  float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;
  float ref_A_bevel_start = ref_A_angle_bisector -
                            atan((1 - r_gon_roundness) * tan(ref_A_angle_bisector));

  float last_angle_bisector_A_x_axis = M_PI_F - floorf(r_gon_sides) * ref_A_angle_bisector;
  float last_ref_A_x_axis = 2.0f * last_angle_bisector_A_x_axis;
  float inner_last_bevel_start_A_x_axis = last_angle_bisector_A_x_axis -
                                          atan((1 - r_gon_roundness) *
                                               tan(last_angle_bisector_A_x_axis));
  float l_last_circle_radius = r_gon_roundness * tan(last_angle_bisector_A_x_axis) /
                               tan(0.5f * (ref_A_angle_bisector + last_angle_bisector_A_x_axis));
  float2 last_circle_center = float2(
      (cos(inner_last_bevel_start_A_x_axis) /
       cos(last_angle_bisector_A_x_axis - inner_last_bevel_start_A_x_axis)) -
          l_last_circle_radius * cos(last_angle_bisector_A_x_axis),
      l_last_circle_radius * sin(last_angle_bisector_A_x_axis) -
          (sin(inner_last_bevel_start_A_x_axis) /
           cos(last_angle_bisector_A_x_axis - inner_last_bevel_start_A_x_axis)));
  float2 outer_last_bevel_start = last_circle_center +
                                  l_last_circle_radius *
                                      float2(cos(ref_A_angle_bisector), sin(ref_A_angle_bisector));
  float x_axis_A_outer_last_bevel_start = atan(outer_last_bevel_start.y /
                                               outer_last_bevel_start.x);

  if ((x_axis_A_coord >= x_axis_A_outer_last_bevel_start) &&
      (x_axis_A_coord < M_TAU_F - last_ref_A_x_axis - x_axis_A_outer_last_bevel_start))
  {
    if (((ref_A_coord >= ref_A_bevel_start) &&
         (ref_A_coord < ref_A_next_ref - ref_A_bevel_start)) ||
        (x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - ref_A_bevel_start) ||
        (x_axis_A_coord < ref_A_bevel_start))
    {
      return l_projection_2d * cos(ref_A_angle_bisector - ref_A_coord);
    }
    else {
      /* SA == Signed Angle in [-M_PI_F, M_PI_F]. Counterclockwise angles are positive, clockwise
       * angles are negative.*/
      float nearest_ref_SA_coord = ref_A_coord -
                                   float(ref_A_coord > ref_A_angle_bisector) * ref_A_next_ref;
      float l_circle_radius = sin(ref_A_bevel_start) / sin(ref_A_angle_bisector);
      float l_circle_center = sin(ref_A_angle_bisector - ref_A_bevel_start) /
                              sin(ref_A_angle_bisector);
      float l_coord_R_l_bevel_start =
          cos(nearest_ref_SA_coord) * l_circle_center +
          sqrtf(math::square(cos(nearest_ref_SA_coord) * l_circle_center) +
                math::square(l_circle_radius) - math::square(l_circle_center));
      return cos(ref_A_angle_bisector - ref_A_bevel_start) * l_projection_2d /
             l_coord_R_l_bevel_start;
    }
  }
  else {
    if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis + inner_last_bevel_start_A_x_axis) &&
        (x_axis_A_coord < M_TAU_F - inner_last_bevel_start_A_x_axis))
    {
      float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cos(ref_A_angle_bisector) /
                                                             cos(last_angle_bisector_A_x_axis);
      return l_projection_2d * cos(last_angle_bisector_A_x_axis - ref_A_coord) *
             l_angle_bisector_2d_R_l_last_angle_bisector_2d;
    }
    else {
      /* MSA == Mirrored Signed Angle. The values are mirrored around the last angle bisector
       * to avoid a case distinction. */
      float nearest_ref_MSA_coord = atan2(coord.y, coord.x);
      if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - x_axis_A_outer_last_bevel_start) &&
          (x_axis_A_coord < M_TAU_F - last_angle_bisector_A_x_axis))
      {
        nearest_ref_MSA_coord += last_ref_A_x_axis;
        nearest_ref_MSA_coord *= -1;
      }
      float l_coord_R_l_last_angle_bisector_2d =
          sin(nearest_ref_MSA_coord) * last_circle_center.y +
          cos(nearest_ref_MSA_coord) * last_circle_center.x +
          sqrtf(math::square(sin(nearest_ref_MSA_coord) * last_circle_center.y +
                             cos(nearest_ref_MSA_coord) * last_circle_center.x) +
                math::square(l_last_circle_radius) - math::square(last_circle_center.x) -
                math::square(last_circle_center.y));
      return (cos(ref_A_angle_bisector) * l_projection_2d) /
             (cos(last_angle_bisector_A_x_axis) * l_coord_R_l_last_angle_bisector_2d);
    }
  }
}

float calculate_l_angle_bisector_2d(bool integer_sides,
                                    bool elliptical_corners,
                                    float r_gon_sides,
                                    float r_gon_roundness,
                                    float r_gon_exponent,
                                    float2 coord)
{
  float l_projection_2d = p_norm(coord, r_gon_exponent);

  if (integer_sides || (fractf(r_gon_sides) == 0.0f)) {
    if (r_gon_roundness == 0.0f) {
      float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0f) * M_TAU_F;
      float ref_A_angle_bisector = M_PI_F / r_gon_sides;
      float ref_A_next_ref = 2.0f * ref_A_angle_bisector;
      float segment_id = floorf(x_axis_A_coord / ref_A_next_ref);
      float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;
      return l_projection_2d * cos(ref_A_angle_bisector - ref_A_coord);
    }
    if (r_gon_roundness == 1.0f) {
      return l_projection_2d;
    }
    else {
      float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0f) * M_TAU_F;
      float ref_A_angle_bisector = M_PI_F / r_gon_sides;
      float ref_A_next_ref = 2.0f * ref_A_angle_bisector;
      float segment_id = floorf(x_axis_A_coord / ref_A_next_ref);
      float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;
      float ref_A_bevel_start = ref_A_angle_bisector -
                                atan((1 - r_gon_roundness) * tan(ref_A_angle_bisector));

      if ((ref_A_coord >= ref_A_next_ref - ref_A_bevel_start) || (ref_A_coord < ref_A_bevel_start))
      {
        /* SA == Signed Angle in [-M_PI_F, M_PI_F]. Counterclockwise angles are positive, clockwise
         * angles are negative.*/
        float nearest_ref_SA_coord = ref_A_coord -
                                     float(ref_A_coord > ref_A_angle_bisector) * ref_A_next_ref;
        float l_circle_radius = sin(ref_A_bevel_start) / sin(ref_A_angle_bisector);
        float l_circle_center = sin(ref_A_angle_bisector - ref_A_bevel_start) /
                                sin(ref_A_angle_bisector);
        float l_coord_R_l_bevel_start =
            cos(nearest_ref_SA_coord) * l_circle_center +
            sqrtf(math::square(cos(nearest_ref_SA_coord) * l_circle_center) +
                  math::square(l_circle_radius) - math::square(l_circle_center));
        return cos(ref_A_angle_bisector - ref_A_bevel_start) * l_projection_2d /
               l_coord_R_l_bevel_start;
      }
      else {
        return l_projection_2d * cos(ref_A_angle_bisector - ref_A_coord);
      }
    }
  }
  else {
    if (r_gon_roundness == 0.0f) {
      float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0f) * M_TAU_F;
      float ref_A_angle_bisector = M_PI_F / r_gon_sides;
      float ref_A_next_ref = 2.0f * ref_A_angle_bisector;
      float segment_id = floorf(x_axis_A_coord / ref_A_next_ref);
      float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;

      float last_angle_bisector_A_x_axis = M_PI_F - floorf(r_gon_sides) * ref_A_angle_bisector;
      float last_ref_A_x_axis = 2.0f * last_angle_bisector_A_x_axis;

      if (x_axis_A_coord < M_TAU_F - last_ref_A_x_axis) {
        return l_projection_2d * cos(ref_A_angle_bisector - ref_A_coord);
      }
      else {
        float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cos(ref_A_angle_bisector) /
                                                               cos(last_angle_bisector_A_x_axis);
        return l_projection_2d * cos(last_angle_bisector_A_x_axis - ref_A_coord) *
               l_angle_bisector_2d_R_l_last_angle_bisector_2d;
      }
    }
    if (r_gon_roundness == 1.0f) {
      if (elliptical_corners) {
        return calculate_l_angle_bisector_2d_full_roundness_irregular_elliptical(
            r_gon_sides, coord, l_projection_2d);
      }
      else {
        return calculate_l_angle_bisector_2d_full_roundness_irregular_circular(
            r_gon_sides, coord, l_projection_2d);
      }
    }
    else {
      if (elliptical_corners) {
        return calculate_l_angle_bisector_2d_irregular_elliptical(
            r_gon_sides, r_gon_roundness, coord, l_projection_2d);
      }
      else {
        return calculate_l_angle_bisector_2d_irregular_circular(
            r_gon_sides, r_gon_roundness, coord, l_projection_2d);
      }
    }
  }
}

float calculate_l_angle_bisector_4d(bool integer_sides,
                                    bool elliptical_corners,
                                    float r_gon_sides,
                                    float r_gon_roundness,
                                    float r_gon_exponent,
                                    float sphere_exponent,
                                    float4 coord)
{
  return p_norm(
      float3(calculate_l_angle_bisector_2d(integer_sides,
                                           elliptical_corners,
                                           integer_sides ? math::ceil(r_gon_sides) : r_gon_sides,
                                           r_gon_roundness,
                                           r_gon_exponent,
                                           float2(coord.x, coord.y)),
             coord.z,
             coord.w),
      sphere_exponent);
}

float4 calculate_out_fields_2d_full_roundness_irregular_elliptical(
    bool calculate_r_sphere_field,
    bool calculate_r_gon_parameter_field,
    bool normalize_r_gon_parameter,
    float r_gon_sides,
    float2 coord,
    float l_projection_2d)
{
  float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0f) * M_TAU_F;
  float ref_A_angle_bisector = M_PI_F / r_gon_sides;
  float ref_A_next_ref = 2.0f * ref_A_angle_bisector;
  float segment_id = floorf(x_axis_A_coord / ref_A_next_ref);
  float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;

  float last_angle_bisector_A_x_axis = M_PI_F - floorf(r_gon_sides) * ref_A_angle_bisector;
  float last_ref_A_x_axis = 2.0f * last_angle_bisector_A_x_axis;

  if ((x_axis_A_coord >= ref_A_angle_bisector) &&
      (x_axis_A_coord < M_TAU_F - last_ref_A_x_axis - ref_A_angle_bisector))
  {
    float r_gon_parameter_2d = 0.0f;
    if (calculate_r_gon_parameter_field) {
      r_gon_parameter_2d = fabsf(ref_A_angle_bisector - ref_A_coord);
      if (ref_A_coord < ref_A_angle_bisector) {
        r_gon_parameter_2d *= -1.0f;
      }
      if (normalize_r_gon_parameter) {
        r_gon_parameter_2d /= ref_A_angle_bisector;
      }
    }
    return float4(l_projection_2d, r_gon_parameter_2d, ref_A_angle_bisector, segment_id);
  }
  else {
    /* MSA == Mirrored Signed Angle. The values are mirrored around the last angle bisector
     * to avoid a case distinction. */
    float nearest_ref_MSA_coord = atan2(coord.y, coord.x);
    if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - ref_A_angle_bisector) &&
        (x_axis_A_coord < M_TAU_F - last_angle_bisector_A_x_axis))
    {
      nearest_ref_MSA_coord += last_ref_A_x_axis;
      nearest_ref_MSA_coord *= -1;
    }
    float l_angle_bisector_2d = 0.0f;
    float r_gon_parameter_2d = 0.0f;
    float max_unit_parameter_2d = 0.0f;
    if (calculate_r_sphere_field) {
      float l_basis_vector_1 = tan(ref_A_angle_bisector);
      /* When the fractional part of r_gon_sides is very small division by l_basis_vector_2 causes
       * precision issues. Change to double if necessary */
      float l_basis_vector_2 = sin(last_angle_bisector_A_x_axis) *
                               sqrtf(math::square(tan(ref_A_angle_bisector)) + 1.0f);
      float2 ellipse_center =
          float2(cos(ref_A_angle_bisector) / cos(ref_A_angle_bisector - ref_A_angle_bisector),
                 sin(ref_A_angle_bisector) / cos(ref_A_angle_bisector - ref_A_angle_bisector)) -
          l_basis_vector_2 *
              float2(sin(last_angle_bisector_A_x_axis), cos(last_angle_bisector_A_x_axis));
      float2 transformed_direction_vector = float2(
          cos(last_angle_bisector_A_x_axis + nearest_ref_MSA_coord) /
              (l_basis_vector_1 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)),
          cos(ref_A_angle_bisector - nearest_ref_MSA_coord) /
              (l_basis_vector_2 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)));
      float2 transformed_origin = float2(
          (ellipse_center.y * sin(last_angle_bisector_A_x_axis) -
           ellipse_center.x * cos(last_angle_bisector_A_x_axis)) /
              (l_basis_vector_1 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)),
          -(ellipse_center.y * sin(ref_A_angle_bisector) +
            ellipse_center.x * cos(ref_A_angle_bisector)) /
              (l_basis_vector_2 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)));
      float l_coord_R_l_angle_bisector_2d =
          (-(transformed_direction_vector.x * transformed_origin.x +
             transformed_direction_vector.y * transformed_origin.y) +
           sqrtf(math::square(transformed_direction_vector.x * transformed_origin.x +
                              transformed_direction_vector.y * transformed_origin.y) -
                 (math::square(transformed_direction_vector.x) +
                  math::square(transformed_direction_vector.y)) *
                     (math::square(transformed_origin.x) + math::square(transformed_origin.y) -
                      1.0f))) /
          (math::square(transformed_direction_vector.x) +
           math::square(transformed_direction_vector.y));
      l_angle_bisector_2d = l_projection_2d / l_coord_R_l_angle_bisector_2d;
      if (nearest_ref_MSA_coord < 0.0f) {
        float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cos(ref_A_angle_bisector) /
                                                               cos(last_angle_bisector_A_x_axis);
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = (last_angle_bisector_A_x_axis + nearest_ref_MSA_coord) *
                               l_angle_bisector_2d_R_l_last_angle_bisector_2d;
          if (ref_A_coord < last_angle_bisector_A_x_axis) {
            r_gon_parameter_2d *= -1.0f;
          }
          if (normalize_r_gon_parameter) {
            r_gon_parameter_2d /= last_angle_bisector_A_x_axis *
                                  l_angle_bisector_2d_R_l_last_angle_bisector_2d;
          }
        }
        max_unit_parameter_2d = last_angle_bisector_A_x_axis *
                                l_angle_bisector_2d_R_l_last_angle_bisector_2d;
      }
      else {
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = fabsf(ref_A_angle_bisector - ref_A_coord);
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter_2d *= -1.0f;
          }
          if (normalize_r_gon_parameter) {
            r_gon_parameter_2d /= ref_A_angle_bisector;
          }
        }
        max_unit_parameter_2d = ref_A_angle_bisector;
      }
    }
    return float4(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d, segment_id);
  }
}

float4 calculate_out_fields_2d_irregular_elliptical(bool calculate_r_sphere_field,
                                                    bool calculate_r_gon_parameter_field,
                                                    bool calculate_max_unit_parameter_field,
                                                    bool normalize_r_gon_parameter,
                                                    float r_gon_sides,
                                                    float r_gon_roundness,
                                                    float2 coord,
                                                    float l_projection_2d)
{
  float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0f) * M_TAU_F;
  float ref_A_angle_bisector = M_PI_F / r_gon_sides;
  float ref_A_next_ref = 2.0f * ref_A_angle_bisector;
  float segment_id = floorf(x_axis_A_coord / ref_A_next_ref);
  float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;
  float ref_A_bevel_start = ref_A_angle_bisector -
                            atan((1 - r_gon_roundness) * tan(ref_A_angle_bisector));

  float last_angle_bisector_A_x_axis = M_PI_F - floorf(r_gon_sides) * ref_A_angle_bisector;
  float last_ref_A_x_axis = 2.0f * last_angle_bisector_A_x_axis;
  float inner_last_bevel_start_A_x_axis = last_angle_bisector_A_x_axis -
                                          atan((1 - r_gon_roundness) *
                                               tan(last_angle_bisector_A_x_axis));

  if ((x_axis_A_coord >= ref_A_bevel_start) &&
      (x_axis_A_coord < M_TAU_F - last_ref_A_x_axis - ref_A_bevel_start))
  {
    if ((ref_A_coord >= ref_A_bevel_start) && (ref_A_coord < ref_A_next_ref - ref_A_bevel_start)) {
      float l_angle_bisector_2d = 0.0f;
      float r_gon_parameter_2d = 0.0f;
      float max_unit_parameter_2d = 0.0f;
      if (calculate_r_sphere_field) {
        l_angle_bisector_2d = l_projection_2d * cos(ref_A_angle_bisector - ref_A_coord);
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d *
                               tan(fabsf(ref_A_angle_bisector - ref_A_coord));
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter_2d *= -1.0f;
          }
          if (normalize_r_gon_parameter) {
            r_gon_parameter_2d /= l_angle_bisector_2d *
                                      tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                  ref_A_bevel_start;
          }
        }
        if (calculate_max_unit_parameter_field) {
          max_unit_parameter_2d = tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                  ref_A_bevel_start;
        }
      }
      return float4(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d, segment_id);
    }
    else {
      /* SA == Signed Angle in [-M_PI_F, M_PI_F]. Counterclockwise angles are positive, clockwise
       * angles are negative.*/
      float nearest_ref_SA_coord = ref_A_coord -
                                   float(ref_A_coord > ref_A_angle_bisector) * ref_A_next_ref;
      float l_angle_bisector_2d = 0.0f;
      float r_gon_parameter_2d = 0.0f;
      float max_unit_parameter_2d = 0.0f;
      if (calculate_r_sphere_field) {
        float l_circle_radius = sin(ref_A_bevel_start) / sin(ref_A_angle_bisector);
        float l_circle_center = sin(ref_A_angle_bisector - ref_A_bevel_start) /
                                sin(ref_A_angle_bisector);
        float l_coord_R_l_bevel_start =
            cos(nearest_ref_SA_coord) * l_circle_center +
            sqrtf(math::square(cos(nearest_ref_SA_coord) * l_circle_center) +
                  math::square(l_circle_radius) - math::square(l_circle_center));
        l_angle_bisector_2d = cos(ref_A_angle_bisector - ref_A_bevel_start) * l_projection_2d /
                              l_coord_R_l_bevel_start;
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d *
                                   tan(ref_A_angle_bisector - ref_A_bevel_start) +
                               ref_A_bevel_start - fabsf(nearest_ref_SA_coord);
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter_2d *= -1.0f;
          }
          if (normalize_r_gon_parameter) {
            r_gon_parameter_2d /= l_angle_bisector_2d *
                                      tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                  ref_A_bevel_start;
          }
        }
        if (calculate_max_unit_parameter_field) {
          max_unit_parameter_2d = tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                  ref_A_bevel_start;
        }
      }
      return float4(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d, segment_id);
    }
  }
  else {
    if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis + inner_last_bevel_start_A_x_axis) &&
        (x_axis_A_coord < M_TAU_F - inner_last_bevel_start_A_x_axis))
    {
      float l_angle_bisector_2d = 0.0f;
      float r_gon_parameter_2d = 0.0f;
      float max_unit_parameter_2d = 0.0f;
      if (calculate_r_sphere_field) {
        float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cos(ref_A_angle_bisector) /
                                                               cos(last_angle_bisector_A_x_axis);
        l_angle_bisector_2d = l_projection_2d * cos(last_angle_bisector_A_x_axis - ref_A_coord) *
                              l_angle_bisector_2d_R_l_last_angle_bisector_2d;
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d *
                               tan(fabsf(last_angle_bisector_A_x_axis - ref_A_coord));
          if (ref_A_coord < last_angle_bisector_A_x_axis) {
            r_gon_parameter_2d *= -1.0f;
          }
          if (normalize_r_gon_parameter) {
            r_gon_parameter_2d /= l_angle_bisector_2d * tan(last_angle_bisector_A_x_axis -
                                                            inner_last_bevel_start_A_x_axis) +
                                  inner_last_bevel_start_A_x_axis *
                                      l_angle_bisector_2d_R_l_last_angle_bisector_2d;
          }
        }
        if (calculate_max_unit_parameter_field) {
          max_unit_parameter_2d = tan(last_angle_bisector_A_x_axis -
                                      inner_last_bevel_start_A_x_axis) +
                                  inner_last_bevel_start_A_x_axis *
                                      l_angle_bisector_2d_R_l_last_angle_bisector_2d;
        }
      }
      return float4(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d, segment_id);
    }
    else {
      /* MSA == Mirrored Signed Angle. The values are mirrored around the last angle bisector
       * to avoid a case distinction. */
      float nearest_ref_MSA_coord = atan2(coord.y, coord.x);
      if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - ref_A_bevel_start) &&
          (x_axis_A_coord < M_TAU_F - last_angle_bisector_A_x_axis))
      {
        nearest_ref_MSA_coord += last_ref_A_x_axis;
        nearest_ref_MSA_coord *= -1;
      }
      float l_angle_bisector_2d = 0.0f;
      float r_gon_parameter_2d = 0.0f;
      float max_unit_parameter_2d = 0.0f;
      if (calculate_r_sphere_field) {
        float l_basis_vector_1 = r_gon_roundness * tan(ref_A_angle_bisector);
        float l_basis_vector_2 = r_gon_roundness * sin(last_angle_bisector_A_x_axis) *
                                 sqrtf(math::square(tan(ref_A_angle_bisector)) + 1.0f);
        float2 ellipse_center =
            float2(cos(ref_A_bevel_start) / cos(ref_A_angle_bisector - ref_A_bevel_start),
                   sin(ref_A_bevel_start) / cos(ref_A_angle_bisector - ref_A_bevel_start)) -
            l_basis_vector_2 *
                float2(sin(last_angle_bisector_A_x_axis), cos(last_angle_bisector_A_x_axis));
        float2 transformed_direction_vector = float2(
            cos(last_angle_bisector_A_x_axis + nearest_ref_MSA_coord) /
                (l_basis_vector_1 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)),
            cos(ref_A_angle_bisector - nearest_ref_MSA_coord) /
                (l_basis_vector_2 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)));
        float2 transformed_origin = float2(
            (ellipse_center.y * sin(last_angle_bisector_A_x_axis) -
             ellipse_center.x * cos(last_angle_bisector_A_x_axis)) /
                (l_basis_vector_1 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)),
            -(ellipse_center.y * sin(ref_A_angle_bisector) +
              ellipse_center.x * cos(ref_A_angle_bisector)) /
                (l_basis_vector_2 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)));
        float l_coord_R_l_angle_bisector_2d =
            (-(transformed_direction_vector.x * transformed_origin.x +
               transformed_direction_vector.y * transformed_origin.y) +
             sqrtf(math::square(transformed_direction_vector.x * transformed_origin.x +
                                transformed_direction_vector.y * transformed_origin.y) -
                   (math::square(transformed_direction_vector.x) +
                    math::square(transformed_direction_vector.y)) *
                       (math::square(transformed_origin.x) + math::square(transformed_origin.y) -
                        1.0f))) /
            (math::square(transformed_direction_vector.x) +
             math::square(transformed_direction_vector.y));
        l_angle_bisector_2d = l_projection_2d / l_coord_R_l_angle_bisector_2d;
        if (nearest_ref_MSA_coord < 0.0f) {
          float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cos(ref_A_angle_bisector) /
                                                                 cos(last_angle_bisector_A_x_axis);
          if (calculate_r_gon_parameter_field) {
            r_gon_parameter_2d = l_angle_bisector_2d *
                                     tan(fabsf(last_angle_bisector_A_x_axis -
                                               inner_last_bevel_start_A_x_axis)) +
                                 (inner_last_bevel_start_A_x_axis + nearest_ref_MSA_coord) *
                                     l_angle_bisector_2d_R_l_last_angle_bisector_2d;
            if (ref_A_coord < last_angle_bisector_A_x_axis) {
              r_gon_parameter_2d *= -1.0f;
            }
            if (normalize_r_gon_parameter) {
              r_gon_parameter_2d /= l_angle_bisector_2d * tan(last_angle_bisector_A_x_axis -
                                                              inner_last_bevel_start_A_x_axis) +
                                    inner_last_bevel_start_A_x_axis *
                                        l_angle_bisector_2d_R_l_last_angle_bisector_2d;
            }
          }
          if (calculate_max_unit_parameter_field) {
            max_unit_parameter_2d = tan(last_angle_bisector_A_x_axis -
                                        inner_last_bevel_start_A_x_axis) +
                                    inner_last_bevel_start_A_x_axis *
                                        l_angle_bisector_2d_R_l_last_angle_bisector_2d;
          }
        }
        else {
          if (calculate_r_gon_parameter_field) {
            r_gon_parameter_2d = l_angle_bisector_2d *
                                     tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                 ref_A_bevel_start - nearest_ref_MSA_coord;
            if (ref_A_coord < ref_A_angle_bisector) {
              r_gon_parameter_2d *= -1.0f;
            }
            if (normalize_r_gon_parameter) {
              r_gon_parameter_2d /= l_angle_bisector_2d *
                                        tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                    ref_A_bevel_start;
            }
          }
          if (calculate_max_unit_parameter_field) {
            max_unit_parameter_2d = tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                    ref_A_bevel_start;
          }
        }
      }
      return float4(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d, segment_id);
    }
  }
}

float4 calculate_out_fields_2d_full_roundness_irregular_circular(
    bool calculate_r_sphere_field,
    bool calculate_r_gon_parameter_field,
    bool calculate_max_unit_parameter_field,
    bool normalize_r_gon_parameter,
    float r_gon_sides,
    float2 coord,
    float l_projection_2d)
{
  float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0f) * M_TAU_F;
  float ref_A_angle_bisector = M_PI_F / r_gon_sides;
  float ref_A_next_ref = 2.0f * ref_A_angle_bisector;
  float segment_id = floorf(x_axis_A_coord / ref_A_next_ref);
  float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;

  float last_angle_bisector_A_x_axis = M_PI_F - floorf(r_gon_sides) * ref_A_angle_bisector;
  float last_ref_A_x_axis = 2.0f * last_angle_bisector_A_x_axis;
  float l_last_circle_radius = tan(last_angle_bisector_A_x_axis) /
                               tan(0.5f * (ref_A_angle_bisector + last_angle_bisector_A_x_axis));
  float2 last_circle_center = float2(cos(last_angle_bisector_A_x_axis) -
                                         l_last_circle_radius * cos(last_angle_bisector_A_x_axis),
                                     l_last_circle_radius * sin(last_angle_bisector_A_x_axis) -
                                         sin(last_angle_bisector_A_x_axis));
  float2 outer_last_bevel_start = last_circle_center +
                                  l_last_circle_radius *
                                      float2(cos(ref_A_angle_bisector), sin(ref_A_angle_bisector));
  float x_axis_A_outer_last_bevel_start = atan(outer_last_bevel_start.y /
                                               outer_last_bevel_start.x);

  if ((x_axis_A_coord >= x_axis_A_outer_last_bevel_start) &&
      (x_axis_A_coord < M_TAU_F - last_ref_A_x_axis - x_axis_A_outer_last_bevel_start))
  {
    if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - ref_A_angle_bisector) ||
        (x_axis_A_coord < ref_A_angle_bisector))
    {
      float l_angle_bisector_2d = 0.0f;
      float r_gon_parameter_2d = 0.0f;
      float max_unit_parameter_2d = 0.0f;
      if (calculate_r_sphere_field) {
        l_angle_bisector_2d = l_projection_2d * cos(ref_A_angle_bisector - ref_A_coord);
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d *
                               tan(fabsf(ref_A_angle_bisector - ref_A_coord));
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter_2d *= -1.0f;
          }
          if (normalize_r_gon_parameter) {
            r_gon_parameter_2d /= l_angle_bisector_2d *
                                      tan(ref_A_angle_bisector - x_axis_A_outer_last_bevel_start) +
                                  x_axis_A_outer_last_bevel_start;
          }
        }
        if (calculate_max_unit_parameter_field) {
          max_unit_parameter_2d = tan(ref_A_angle_bisector - x_axis_A_outer_last_bevel_start) +
                                  x_axis_A_outer_last_bevel_start;
        }
      }
      return float4(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d, segment_id);
    }
    else {
      float r_gon_parameter_2d = 0.0f;
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter_2d = fabsf(ref_A_angle_bisector - ref_A_coord);
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter_2d *= -1.0f;
        }
        if (normalize_r_gon_parameter) {
          r_gon_parameter_2d /= ref_A_angle_bisector;
        }
      }
      return float4(l_projection_2d, r_gon_parameter_2d, ref_A_angle_bisector, segment_id);
    }
  }
  else {
    /* MSA == Mirrored Signed Angle. The values are mirrored around the last angle bisector
     * to avoid a case distinction. */
    float nearest_ref_MSA_coord = atan2(coord.y, coord.x);
    if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - x_axis_A_outer_last_bevel_start) &&
        (x_axis_A_coord < M_TAU_F - last_angle_bisector_A_x_axis))
    {
      nearest_ref_MSA_coord += last_ref_A_x_axis;
      nearest_ref_MSA_coord *= -1;
    }
    float l_angle_bisector_2d = 0.0f;
    float r_gon_parameter_2d = 0.0f;
    float max_unit_parameter_2d = 0.0f;
    if (calculate_r_sphere_field) {
      float l_coord_R_l_last_angle_bisector_2d =
          sin(nearest_ref_MSA_coord) * last_circle_center.y +
          cos(nearest_ref_MSA_coord) * last_circle_center.x +
          sqrtf(math::square(sin(nearest_ref_MSA_coord) * last_circle_center.y +
                             cos(nearest_ref_MSA_coord) * last_circle_center.x) +
                math::square(l_last_circle_radius) - math::square(last_circle_center.x) -
                math::square(last_circle_center.y));
      l_angle_bisector_2d = (cos(ref_A_angle_bisector) * l_projection_2d) /
                            (cos(last_angle_bisector_A_x_axis) *
                             l_coord_R_l_last_angle_bisector_2d);
      if (nearest_ref_MSA_coord < 0.0f) {
        float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cos(ref_A_angle_bisector) /
                                                               cos(last_angle_bisector_A_x_axis);
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = (last_angle_bisector_A_x_axis + nearest_ref_MSA_coord) *
                               l_angle_bisector_2d_R_l_last_angle_bisector_2d;
          if (ref_A_coord < last_angle_bisector_A_x_axis) {
            r_gon_parameter_2d *= -1.0f;
          }
          if (normalize_r_gon_parameter) {
            r_gon_parameter_2d /= last_angle_bisector_A_x_axis *
                                  l_angle_bisector_2d_R_l_last_angle_bisector_2d;
          }
        }
        max_unit_parameter_2d = last_angle_bisector_A_x_axis *
                                l_angle_bisector_2d_R_l_last_angle_bisector_2d;
      }
      else {
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d * tan(fabsf(ref_A_angle_bisector -
                                                               x_axis_A_outer_last_bevel_start)) +
                               x_axis_A_outer_last_bevel_start - nearest_ref_MSA_coord;
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter_2d *= -1.0f;
          }
          if (normalize_r_gon_parameter) {
            r_gon_parameter_2d /= l_angle_bisector_2d *
                                      tan(ref_A_angle_bisector - x_axis_A_outer_last_bevel_start) +
                                  x_axis_A_outer_last_bevel_start;
          }
        }
        if (calculate_max_unit_parameter_field) {
          max_unit_parameter_2d = tan(ref_A_angle_bisector - x_axis_A_outer_last_bevel_start) +
                                  x_axis_A_outer_last_bevel_start;
        }
      }
    }
    return float4(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d, segment_id);
  }
}

float4 calculate_out_fields_2d_irregular_circular(bool calculate_r_sphere_field,
                                                  bool calculate_r_gon_parameter_field,
                                                  bool calculate_max_unit_parameter_field,
                                                  bool normalize_r_gon_parameter,
                                                  float r_gon_sides,
                                                  float r_gon_roundness,
                                                  float2 coord,
                                                  float l_projection_2d)
{
  float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0f) * M_TAU_F;
  float ref_A_angle_bisector = M_PI_F / r_gon_sides;
  float ref_A_next_ref = 2.0f * ref_A_angle_bisector;
  float segment_id = floorf(x_axis_A_coord / ref_A_next_ref);
  float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;
  float ref_A_bevel_start = ref_A_angle_bisector -
                            atan((1 - r_gon_roundness) * tan(ref_A_angle_bisector));

  float last_angle_bisector_A_x_axis = M_PI_F - floorf(r_gon_sides) * ref_A_angle_bisector;
  float last_ref_A_x_axis = 2.0f * last_angle_bisector_A_x_axis;
  float inner_last_bevel_start_A_x_axis = last_angle_bisector_A_x_axis -
                                          atan((1 - r_gon_roundness) *
                                               tan(last_angle_bisector_A_x_axis));
  float l_last_circle_radius = r_gon_roundness * tan(last_angle_bisector_A_x_axis) /
                               tan(0.5f * (ref_A_angle_bisector + last_angle_bisector_A_x_axis));
  float2 last_circle_center = float2(
      (cos(inner_last_bevel_start_A_x_axis) /
       cos(last_angle_bisector_A_x_axis - inner_last_bevel_start_A_x_axis)) -
          l_last_circle_radius * cos(last_angle_bisector_A_x_axis),
      l_last_circle_radius * sin(last_angle_bisector_A_x_axis) -
          (sin(inner_last_bevel_start_A_x_axis) /
           cos(last_angle_bisector_A_x_axis - inner_last_bevel_start_A_x_axis)));
  float2 outer_last_bevel_start = last_circle_center +
                                  l_last_circle_radius *
                                      float2(cos(ref_A_angle_bisector), sin(ref_A_angle_bisector));
  float x_axis_A_outer_last_bevel_start = atan(outer_last_bevel_start.y /
                                               outer_last_bevel_start.x);

  if ((x_axis_A_coord >= x_axis_A_outer_last_bevel_start) &&
      (x_axis_A_coord < M_TAU_F - last_ref_A_x_axis - x_axis_A_outer_last_bevel_start))
  {
    if (((ref_A_coord >= ref_A_bevel_start) &&
         (ref_A_coord < ref_A_next_ref - ref_A_bevel_start)) ||
        (x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - ref_A_bevel_start) ||
        (x_axis_A_coord < ref_A_bevel_start))
    {
      float l_angle_bisector_2d = 0.0f;
      float r_gon_parameter_2d = 0.0f;
      float max_unit_parameter_2d = 0.0f;
      if (calculate_r_sphere_field) {
        l_angle_bisector_2d = l_projection_2d * cos(ref_A_angle_bisector - ref_A_coord);
        if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - ref_A_angle_bisector) ||
            (x_axis_A_coord < ref_A_angle_bisector))
        {
          if (calculate_r_gon_parameter_field) {
            r_gon_parameter_2d = l_angle_bisector_2d *
                                 tan(fabsf(ref_A_angle_bisector - ref_A_coord));
            if (ref_A_coord < ref_A_angle_bisector) {
              r_gon_parameter_2d *= -1.0f;
            }
            if (normalize_r_gon_parameter) {
              r_gon_parameter_2d /= l_angle_bisector_2d * tan(ref_A_angle_bisector -
                                                              x_axis_A_outer_last_bevel_start) +
                                    x_axis_A_outer_last_bevel_start;
            }
          }
          if (calculate_max_unit_parameter_field) {
            max_unit_parameter_2d = tan(ref_A_angle_bisector - x_axis_A_outer_last_bevel_start) +
                                    x_axis_A_outer_last_bevel_start;
          }
        }
        else {
          if (calculate_r_gon_parameter_field) {
            r_gon_parameter_2d = l_angle_bisector_2d *
                                 tan(fabsf(ref_A_angle_bisector - ref_A_coord));
            if (ref_A_coord < ref_A_angle_bisector) {
              r_gon_parameter_2d *= -1.0f;
            }
            if (normalize_r_gon_parameter) {
              r_gon_parameter_2d /= l_angle_bisector_2d *
                                        tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                    ref_A_bevel_start;
            }
          }
          if (calculate_max_unit_parameter_field) {
            max_unit_parameter_2d = tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                    ref_A_bevel_start;
          }
        }
      }
      return float4(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d, segment_id);
    }
    else {
      /* SA == Signed Angle in [-M_PI_F, M_PI_F]. Counterclockwise angles are positive, clockwise
       * angles are negative.*/
      float nearest_ref_SA_coord = ref_A_coord -
                                   float(ref_A_coord > ref_A_angle_bisector) * ref_A_next_ref;
      float l_angle_bisector_2d = 0.0f;
      float r_gon_parameter_2d = 0.0f;
      float max_unit_parameter_2d = 0.0f;
      if (calculate_r_sphere_field) {
        float l_circle_radius = sin(ref_A_bevel_start) / sin(ref_A_angle_bisector);
        float l_circle_center = sin(ref_A_angle_bisector - ref_A_bevel_start) /
                                sin(ref_A_angle_bisector);
        float l_coord_R_l_bevel_start =
            cos(nearest_ref_SA_coord) * l_circle_center +
            sqrtf(math::square(cos(nearest_ref_SA_coord) * l_circle_center) +
                  math::square(l_circle_radius) - math::square(l_circle_center));
        l_angle_bisector_2d = cos(ref_A_angle_bisector - ref_A_bevel_start) * l_projection_2d /
                              l_coord_R_l_bevel_start;
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d *
                                   tan(ref_A_angle_bisector - ref_A_bevel_start) +
                               ref_A_bevel_start - fabsf(nearest_ref_SA_coord);
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter_2d *= -1.0f;
          }
          if (normalize_r_gon_parameter) {
            r_gon_parameter_2d /= l_angle_bisector_2d *
                                      tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                  ref_A_bevel_start;
          }
        }
        if (calculate_max_unit_parameter_field) {
          max_unit_parameter_2d = tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                  ref_A_bevel_start;
        }
      }
      return float4(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d, segment_id);
    }
  }
  else {
    if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis + inner_last_bevel_start_A_x_axis) &&
        (x_axis_A_coord < M_TAU_F - inner_last_bevel_start_A_x_axis))
    {
      float l_angle_bisector_2d = 0.0f;
      float r_gon_parameter_2d = 0.0f;
      float max_unit_parameter_2d = 0.0f;
      if (calculate_r_sphere_field) {
        float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cos(ref_A_angle_bisector) /
                                                               cos(last_angle_bisector_A_x_axis);
        l_angle_bisector_2d = l_projection_2d * cos(last_angle_bisector_A_x_axis - ref_A_coord) *
                              l_angle_bisector_2d_R_l_last_angle_bisector_2d;
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d *
                               tan(fabsf(last_angle_bisector_A_x_axis - ref_A_coord));
          if (ref_A_coord < last_angle_bisector_A_x_axis) {
            r_gon_parameter_2d *= -1.0f;
          }
          if (normalize_r_gon_parameter) {
            r_gon_parameter_2d /= l_angle_bisector_2d * tan(last_angle_bisector_A_x_axis -
                                                            inner_last_bevel_start_A_x_axis) +
                                  inner_last_bevel_start_A_x_axis *
                                      l_angle_bisector_2d_R_l_last_angle_bisector_2d;
          }
        }
        if (calculate_max_unit_parameter_field) {
          max_unit_parameter_2d = tan(last_angle_bisector_A_x_axis -
                                      inner_last_bevel_start_A_x_axis) +
                                  inner_last_bevel_start_A_x_axis *
                                      l_angle_bisector_2d_R_l_last_angle_bisector_2d;
        }
      }
      return float4(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d, segment_id);
    }
    else {
      /* MSA == Mirrored Signed Angle. The values are mirrored around the last angle bisector
       * to avoid a case distinction. */
      float nearest_ref_MSA_coord = atan2(coord.y, coord.x);
      if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - x_axis_A_outer_last_bevel_start) &&
          (x_axis_A_coord < M_TAU_F - last_angle_bisector_A_x_axis))
      {
        nearest_ref_MSA_coord += last_ref_A_x_axis;
        nearest_ref_MSA_coord *= -1;
      }
      float l_angle_bisector_2d = 0.0f;
      float r_gon_parameter_2d = 0.0f;
      float max_unit_parameter_2d = 0.0f;
      if (calculate_r_sphere_field) {
        float l_coord_R_l_last_angle_bisector_2d =
            sin(nearest_ref_MSA_coord) * last_circle_center.y +
            cos(nearest_ref_MSA_coord) * last_circle_center.x +
            sqrtf(math::square(sin(nearest_ref_MSA_coord) * last_circle_center.y +
                               cos(nearest_ref_MSA_coord) * last_circle_center.x) +
                  math::square(l_last_circle_radius) - math::square(last_circle_center.x) -
                  math::square(last_circle_center.y));
        l_angle_bisector_2d = (cos(ref_A_angle_bisector) * l_projection_2d) /
                              (cos(last_angle_bisector_A_x_axis) *
                               l_coord_R_l_last_angle_bisector_2d);
        if (nearest_ref_MSA_coord < 0.0f) {
          float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cos(ref_A_angle_bisector) /
                                                                 cos(last_angle_bisector_A_x_axis);
          if (calculate_r_gon_parameter_field) {
            r_gon_parameter_2d = l_angle_bisector_2d *
                                     tan(fabsf(last_angle_bisector_A_x_axis -
                                               inner_last_bevel_start_A_x_axis)) +
                                 (inner_last_bevel_start_A_x_axis + nearest_ref_MSA_coord) *
                                     l_angle_bisector_2d_R_l_last_angle_bisector_2d;
            if (ref_A_coord < last_angle_bisector_A_x_axis) {
              r_gon_parameter_2d *= -1.0f;
            }
            if (normalize_r_gon_parameter) {
              r_gon_parameter_2d /= l_angle_bisector_2d * tan(last_angle_bisector_A_x_axis -
                                                              inner_last_bevel_start_A_x_axis) +
                                    inner_last_bevel_start_A_x_axis *
                                        l_angle_bisector_2d_R_l_last_angle_bisector_2d;
            }
          }
          if (calculate_max_unit_parameter_field) {
            max_unit_parameter_2d = tan(last_angle_bisector_A_x_axis -
                                        inner_last_bevel_start_A_x_axis) +
                                    inner_last_bevel_start_A_x_axis *
                                        l_angle_bisector_2d_R_l_last_angle_bisector_2d;
          }
        }
        else {
          if (calculate_r_gon_parameter_field) {
            r_gon_parameter_2d = l_angle_bisector_2d *
                                     tan(fabsf(ref_A_angle_bisector -
                                               x_axis_A_outer_last_bevel_start)) +
                                 x_axis_A_outer_last_bevel_start - nearest_ref_MSA_coord;
            if (ref_A_coord < ref_A_angle_bisector) {
              r_gon_parameter_2d *= -1.0f;
            }
            if (normalize_r_gon_parameter) {
              r_gon_parameter_2d /= l_angle_bisector_2d * tan(ref_A_angle_bisector -
                                                              x_axis_A_outer_last_bevel_start) +
                                    x_axis_A_outer_last_bevel_start;
            }
          }
          if (calculate_max_unit_parameter_field) {
            max_unit_parameter_2d = tan(ref_A_angle_bisector - x_axis_A_outer_last_bevel_start) +
                                    x_axis_A_outer_last_bevel_start;
          }
        }
      }
      return float4(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d, segment_id);
    }
  }
}

float4 calculate_out_fields_2d(bool calculate_r_sphere_field,
                               bool calculate_r_gon_parameter_field,
                               bool calculate_max_unit_parameter_field,
                               bool normalize_r_gon_parameter,
                               bool integer_sides,
                               bool elliptical_corners,
                               float r_gon_sides,
                               float r_gon_roundness,
                               float r_gon_exponent,
                               float2 coord)
{
  float l_projection_2d = p_norm(coord, r_gon_exponent);

  if (integer_sides || (fractf(r_gon_sides) == 0.0f)) {
    float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0f) * M_TAU_F;
    float ref_A_angle_bisector = M_PI_F / r_gon_sides;
    float ref_A_next_ref = 2.0f * ref_A_angle_bisector;
    float segment_id = floorf(x_axis_A_coord / ref_A_next_ref);
    float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;

    if (r_gon_roundness == 0.0f) {
      float l_angle_bisector_2d = 0.0f;
      float r_gon_parameter_2d = 0.0f;
      float max_unit_parameter_2d = 0.0f;
      if (calculate_r_sphere_field) {
        l_angle_bisector_2d = l_projection_2d * cos(ref_A_angle_bisector - ref_A_coord);
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d *
                               tan(fabsf(ref_A_angle_bisector - ref_A_coord));
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter_2d *= -1.0f;
          }
          if (normalize_r_gon_parameter && (r_gon_sides != 2.0f)) {
            r_gon_parameter_2d /= l_angle_bisector_2d * tan(ref_A_angle_bisector);
          }
        }
        if (calculate_max_unit_parameter_field) {
          max_unit_parameter_2d = (r_gon_sides != 2.0f) ? tan(ref_A_angle_bisector) : 0.0f;
        }
      }
      return float4(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d, segment_id);
    }
    if (r_gon_roundness == 1.0f) {
      float r_gon_parameter_2d = 0.0f;
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter_2d = fabsf(ref_A_angle_bisector - ref_A_coord);
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter_2d *= -1.0f;
        }
        if (normalize_r_gon_parameter) {
          r_gon_parameter_2d /= ref_A_angle_bisector;
        }
      }
      return float4(l_projection_2d, r_gon_parameter_2d, ref_A_angle_bisector, segment_id);
    }
    else {
      float ref_A_bevel_start = ref_A_angle_bisector -
                                atan((1 - r_gon_roundness) * tan(ref_A_angle_bisector));

      if ((ref_A_coord >= ref_A_next_ref - ref_A_bevel_start) || (ref_A_coord < ref_A_bevel_start))
      {
        /* SA == Signed Angle in [-M_PI_F, M_PI_F]. Counterclockwise angles are positive, clockwise
         * angles are negative.*/
        float nearest_ref_SA_coord = ref_A_coord -
                                     float(ref_A_coord > ref_A_angle_bisector) * ref_A_next_ref;
        float l_angle_bisector_2d = 0.0f;
        float r_gon_parameter_2d = 0.0f;
        float max_unit_parameter_2d = 0.0f;
        if (calculate_r_sphere_field) {
          float l_circle_radius = sin(ref_A_bevel_start) / sin(ref_A_angle_bisector);
          float l_circle_center = sin(ref_A_angle_bisector - ref_A_bevel_start) /
                                  sin(ref_A_angle_bisector);
          float l_coord_R_l_bevel_start =
              cos(nearest_ref_SA_coord) * l_circle_center +
              sqrtf(math::square(cos(nearest_ref_SA_coord) * l_circle_center) +
                    math::square(l_circle_radius) - math::square(l_circle_center));
          l_angle_bisector_2d = l_projection_2d * cos(ref_A_angle_bisector - ref_A_bevel_start) /
                                l_coord_R_l_bevel_start;
          if (calculate_r_gon_parameter_field) {
            r_gon_parameter_2d = l_angle_bisector_2d *
                                     tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                 ref_A_bevel_start - fabsf(nearest_ref_SA_coord);
            if (ref_A_coord < ref_A_angle_bisector) {
              r_gon_parameter_2d *= -1.0f;
            }
            if (normalize_r_gon_parameter) {
              r_gon_parameter_2d /= l_angle_bisector_2d *
                                        tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                    ref_A_bevel_start;
            }
          }
          if (calculate_max_unit_parameter_field) {
            max_unit_parameter_2d = tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                    ref_A_bevel_start;
          }
        }
        return float4(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d, segment_id);
      }
      else {
        float l_angle_bisector_2d = 0.0f;
        float r_gon_parameter_2d = 0.0f;
        float max_unit_parameter_2d = 0.0f;
        if (calculate_r_sphere_field) {
          l_angle_bisector_2d = l_projection_2d * cos(ref_A_angle_bisector - ref_A_coord);
          if (calculate_r_gon_parameter_field) {
            r_gon_parameter_2d = l_angle_bisector_2d *
                                 tan(fabsf(ref_A_angle_bisector - ref_A_coord));
            if (ref_A_coord < ref_A_angle_bisector) {
              r_gon_parameter_2d *= -1.0f;
            }
            if (normalize_r_gon_parameter) {
              r_gon_parameter_2d /= l_angle_bisector_2d *
                                        tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                    ref_A_bevel_start;
            }
          }
          if (calculate_max_unit_parameter_field) {
            max_unit_parameter_2d = tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                    ref_A_bevel_start;
          }
        }
        return float4(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d, segment_id);
      }
    }
  }
  else {
    if (r_gon_roundness == 0.0f) {
      float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0f) * M_TAU_F;
      float ref_A_angle_bisector = M_PI_F / r_gon_sides;
      float ref_A_next_ref = 2.0f * ref_A_angle_bisector;
      float segment_id = floorf(x_axis_A_coord / ref_A_next_ref);
      float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;

      float last_angle_bisector_A_x_axis = M_PI_F - floorf(r_gon_sides) * ref_A_angle_bisector;
      float last_ref_A_x_axis = 2.0f * last_angle_bisector_A_x_axis;

      if (x_axis_A_coord < M_TAU_F - last_ref_A_x_axis) {
        float l_angle_bisector_2d = 0.0f;
        float r_gon_parameter_2d = 0.0f;
        float max_unit_parameter_2d = 0.0f;
        if (calculate_r_sphere_field) {
          l_angle_bisector_2d = l_projection_2d * cos(ref_A_angle_bisector - ref_A_coord);
          if (calculate_r_gon_parameter_field) {
            r_gon_parameter_2d = l_angle_bisector_2d *
                                 tan(fabsf(ref_A_angle_bisector - ref_A_coord));
            if (ref_A_coord < ref_A_angle_bisector) {
              r_gon_parameter_2d *= -1.0f;
            }
            if (normalize_r_gon_parameter) {
              r_gon_parameter_2d /= l_angle_bisector_2d * tan(ref_A_angle_bisector);
            }
          }
          if (calculate_max_unit_parameter_field) {
            max_unit_parameter_2d = tan(ref_A_angle_bisector);
          }
        }
        return float4(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d, segment_id);
      }
      else {
        float l_angle_bisector_2d = 0.0f;
        float r_gon_parameter_2d = 0.0f;
        float max_unit_parameter_2d = 0.0f;
        if (calculate_r_sphere_field) {
          float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cos(ref_A_angle_bisector) /
                                                                 cos(last_angle_bisector_A_x_axis);
          l_angle_bisector_2d = l_projection_2d * cos(last_angle_bisector_A_x_axis - ref_A_coord) *
                                l_angle_bisector_2d_R_l_last_angle_bisector_2d;
          if (calculate_r_gon_parameter_field) {
            r_gon_parameter_2d = l_angle_bisector_2d *
                                 tan(fabsf(last_angle_bisector_A_x_axis - ref_A_coord));
            if (ref_A_coord < last_angle_bisector_A_x_axis) {
              r_gon_parameter_2d *= -1.0f;
            }
            if (normalize_r_gon_parameter) {
              r_gon_parameter_2d /= l_angle_bisector_2d * tan(last_angle_bisector_A_x_axis);
            }
          }
          if (calculate_max_unit_parameter_field) {
            max_unit_parameter_2d = tan(last_angle_bisector_A_x_axis);
          }
        }
        return float4(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d, segment_id);
      }
    }
    if (r_gon_roundness == 1.0f) {
      if (elliptical_corners) {
        return calculate_out_fields_2d_full_roundness_irregular_elliptical(
            calculate_r_sphere_field,
            calculate_r_gon_parameter_field,
            normalize_r_gon_parameter,
            r_gon_sides,
            coord,
            l_projection_2d);
      }
      else {
        return calculate_out_fields_2d_full_roundness_irregular_circular(
            calculate_r_sphere_field,
            calculate_r_gon_parameter_field,
            calculate_max_unit_parameter_field,
            normalize_r_gon_parameter,
            r_gon_sides,
            coord,
            l_projection_2d);
      }
    }
    else {
      if (elliptical_corners) {
        return calculate_out_fields_2d_irregular_elliptical(calculate_r_sphere_field,
                                                            calculate_r_gon_parameter_field,
                                                            calculate_max_unit_parameter_field,
                                                            normalize_r_gon_parameter,
                                                            r_gon_sides,
                                                            r_gon_roundness,
                                                            coord,
                                                            l_projection_2d);
      }
      else {
        return calculate_out_fields_2d_irregular_circular(calculate_r_sphere_field,
                                                          calculate_r_gon_parameter_field,
                                                          calculate_max_unit_parameter_field,
                                                          normalize_r_gon_parameter,
                                                          r_gon_sides,
                                                          r_gon_roundness,
                                                          coord,
                                                          l_projection_2d);
      }
    }
  }
}

float4 calculate_out_fields_4d(bool calculate_r_sphere_field,
                               bool calculate_r_gon_parameter_field,
                               bool calculate_max_unit_parameter_field,
                               bool normalize_r_gon_parameter,
                               bool integer_sides,
                               bool elliptical_corners,
                               float r_gon_sides,
                               float r_gon_roundness,
                               float r_gon_exponent,
                               float sphere_exponent,
                               float4 coord)
{
  float4 out_fields = calculate_out_fields_2d(calculate_r_sphere_field,
                                              calculate_r_gon_parameter_field,
                                              calculate_max_unit_parameter_field,
                                              normalize_r_gon_parameter,
                                              integer_sides,
                                              elliptical_corners,
                                              integer_sides ? math::ceil(r_gon_sides) :
                                                              r_gon_sides,
                                              r_gon_roundness,
                                              r_gon_exponent,
                                              float2(coord.x, coord.y));
  out_fields.x = p_norm(float3(out_fields.x, coord.z, coord.w), sphere_exponent);
  return out_fields;
}

void randomize_scale(float scale_randomized[],
                     float scale_randomness[],
                     int scale_index_list[],
                     int scale_index_count,
                     bool uniform_scale_randomness,
                     float3 seed_offset)
{
  if (!uniform_scale_randomness) {
    for (int i = 0; i < scale_index_count; ++i) {
      scale_randomized[scale_index_list[i]] *= powf(
          2.0f,
          mix(-scale_randomness[i],
              scale_randomness[i],
              hash_float_to_float(
                  float3(scale_index_list[i], scale_index_list[i], scale_index_list[i]) +
                  seed_offset)));
    }
  }
  else if (scale_index_count != 0) {
    float random_scale_factor = powf(
        2.0f, mix(-scale_randomness[0], scale_randomness[0], hash_float_to_float(seed_offset)));
    for (int i = 0; i < 4; i++) {
      scale_randomized[i] *= random_scale_factor;
    }
  }
}

void randomize_scale(float scale_randomized[],
                     float scale_randomness[],
                     int scale_index_list[],
                     int scale_index_count,
                     bool uniform_scale_randomness,
                     float4 seed_offset)
{
  if (!uniform_scale_randomness) {
    for (int i = 0; i < scale_index_count; ++i) {
      scale_randomized[scale_index_list[i]] *= powf(
          2.0f,
          mix(-scale_randomness[i],
              scale_randomness[i],
              hash_float_to_float(float4(scale_index_list[i],
                                         scale_index_list[i],
                                         scale_index_list[i],
                                         scale_index_list[i]) +
                                  seed_offset)));
    }
  }
  else if (scale_index_count != 0) {
    float random_scale_factor = powf(
        2.0f, mix(-scale_randomness[0], scale_randomness[0], hash_float_to_float(seed_offset)));
    for (int i = 0; i < 4; i++) {
      scale_randomized[i] *= random_scale_factor;
    }
  }
}

void randomize_float_array(
    float array[], float min[], float max[], int index_list[], int index_count, float3 seed_offset)
{
  for (int i = 0; i < index_count; ++i) {
    array[index_list[i]] = mix(
        min[i],
        max[i],
        hash_float_to_float(float3(index_list[i], index_list[i], index_list[i]) + seed_offset));
  }
}

void randomize_float_array(
    float array[], float min[], float max[], int index_list[], int index_count, float4 seed_offset)
{
  for (int i = 0; i < index_count; ++i) {
    array[index_list[i]] = mix(
        min[i],
        max[i],
        hash_float_to_float(float4(index_list[i], index_list[i], index_list[i], index_list[i]) +
                            seed_offset));
  }
}

float elliptical_ramp(float value, float ellipse_height, float ellipse_width)
{
  if (value < 0.0f) {
    return 0.0f;
  }
  else if (value < ellipse_width + ellipse_height * (1.0f - ellipse_width)) {
    return (ellipse_height *
            (value * ellipse_height * (1.0f - ellipse_width) + math::square(ellipse_width) -
             ellipse_width * sqrtf(math::square(ellipse_width) - math::square(value) +
                                   2.0f * value * ellipse_height * (1.0f - ellipse_width)))) /
           (math::square(ellipse_height * (1.0f - ellipse_width)) + math::square(ellipse_width));
  }
  else {
    return (ellipse_width == 1.0f) ? ellipse_height :
                                     (value - ellipse_width) / (1.0f - ellipse_width);
  }
}

float elliptical_unit_step(float value,
                           float ellipse_height,
                           float ellipse_width,
                           float inflection_point)
{
  if (inflection_point == 0.0f) {
    return (value < 0.0f) ? 0.0f :
                            1.0f - elliptical_ramp(1.0f - value, ellipse_height, ellipse_width);
  }
  else if (inflection_point == 1.0f) {
    return (value < 1.0f) ? elliptical_ramp(value, ellipse_height, ellipse_width) : 1.0f;
  }
  else {
    return (value < inflection_point) ?
               inflection_point *
                   elliptical_ramp(value / inflection_point, ellipse_height, ellipse_width) :
               1.0f - (1.0f - inflection_point) *
                          elliptical_ramp((1.0f - value) / (1.0f - inflection_point),
                                          ellipse_height,
                                          ellipse_width);
  }
}

float inverse_mix(float value, float from_min, float from_max)
{
  return (value - from_min) / (from_max - from_min);
}

float elliptical_remap(float value,
                       float from_min,
                       float from_max,
                       float to_min,
                       float to_max,
                       float ellipse_height,
                       float ellipse_width,
                       float inflection_point)
{
  if (from_min == from_max) {
    return (value >= from_min) ? to_max : to_min;
  }
  else {
    return mix(to_min,
               to_max,
               elliptical_unit_step(inverse_mix(value, from_min, from_max),
                                    ellipse_height,
                                    ellipse_width,
                                    inflection_point));
  }
}

float chained_elliptical_remap_1_step(float remap[], float value)
{
  return elliptical_remap(value,
                          /* Step Center 1 - 0.5f * Step Width 1 */
                          remap[0] - 0.5f * remap[1],
                          /* Step Center 1 + 0.5f * Step Width 1 */
                          remap[0] + 0.5f * remap[1],
                          /* Step Value 1 */
                          remap[2],
                          0.0f,
                          /* Ellipse Height 1 */
                          remap[3],
                          /* Ellipse Width 1 */
                          remap[4],
                          /* Inflection Point 1 */
                          remap[5]);
}

float chained_elliptical_remap_2_steps(float remap[], float value)
{
  float result = elliptical_remap(value,
                                  /* Step Center 1 - 0.5f * Step Width 1 */
                                  remap[0] - 0.5f * remap[1],
                                  /* Step Center 1 + 0.5f * Step Width 1 */
                                  remap[0] + 0.5f * remap[1],
                                  /* Step Value 1 */
                                  remap[2],
                                  /* Step Value 2 */
                                  remap[8],
                                  /* Ellipse Height 1 */
                                  remap[3],
                                  /* Ellipse Width 1 */
                                  remap[4],
                                  /* Inflection Point 1 */
                                  remap[5]);
  return elliptical_remap(value,
                          /* Step Center 2 - 0.5f * Step Width 2 */
                          remap[6] - 0.5f * remap[7],
                          /* Step Center 2 + 0.5f * Step Width 2 */
                          remap[6] + 0.5f * remap[7],
                          result,
                          0.0f,
                          /* Ellipse Height 2 */
                          remap[9],
                          /* Ellipse Width 2 */
                          remap[10],
                          /* Inflection Point 2 */
                          remap[11]);
}

float chained_elliptical_remap_3_steps(float remap[], float value)
{
  float result = elliptical_remap(value,
                                  /* Step Center 1 - 0.5f * Step Width 1 */
                                  remap[0] - 0.5f * remap[1],
                                  /* Step Center 1 + 0.5f * Step Width 1 */
                                  remap[0] + 0.5f * remap[1],
                                  /* Step Value 1 */
                                  remap[2],
                                  /* Step Value 2 */
                                  remap[8],
                                  /* Ellipse Height 1 */
                                  remap[3],
                                  /* Ellipse Width 1 */
                                  remap[4],
                                  /* Inflection Point 1 */
                                  remap[5]);
  result = elliptical_remap(value,
                            /* Step Center 2 - 0.5f * Step Width 2 */
                            remap[6] - 0.5f * remap[7],
                            /* Step Center 2 + 0.5f * Step Width 2 */
                            remap[6] + 0.5f * remap[7],
                            result,
                            /* Step Value 3 */
                            remap[14],
                            /* Ellipse Height 2 */
                            remap[9],
                            /* Ellipse Width 2 */
                            remap[10],
                            /* Inflection Point 2 */
                            remap[11]);
  return elliptical_remap(value,
                          /* Step Center 3 - 0.5f * Step Width 3 */
                          remap[12] - 0.5f * remap[13],
                          /* Step Center 3 + 0.5f * Step Width 3 */
                          remap[12] + 0.5f * remap[13],
                          result,
                          0.0f,
                          /* Ellipse Height 3 */
                          remap[15],
                          /* Ellipse Width 3 */
                          remap[16],
                          /* Inflection Point 3 */
                          remap[17]);
}

float chained_elliptical_remap_4_steps(float remap[], float value)
{
  float result = elliptical_remap(value,
                                  /* Step Center 1 - 0.5f * Step Width 1 */
                                  remap[0] - 0.5f * remap[1],
                                  /* Step Center 1 + 0.5f * Step Width 1 */
                                  remap[0] + 0.5f * remap[1],
                                  /* Step Value 1 */
                                  remap[2],
                                  /* Step Value 2 */
                                  remap[8],
                                  /* Ellipse Height 1 */
                                  remap[3],
                                  /* Ellipse Width 1 */
                                  remap[4],
                                  /* Inflection Point 1 */
                                  remap[5]);
  result = elliptical_remap(value,
                            /* Step Center 2 - 0.5f * Step Width 2 */
                            remap[6] - 0.5f * remap[7],
                            /* Step Center 2 + 0.5f * Step Width 2 */
                            remap[6] + 0.5f * remap[7],
                            result,
                            /* Step Value 3 */
                            remap[14],
                            /* Ellipse Height 2 */
                            remap[9],
                            /* Ellipse Width 2 */
                            remap[10],
                            /* Inflection Point 2 */
                            remap[11]);
  result = elliptical_remap(value,
                            /* Step Center 3 - 0.5f * Step Width 3 */
                            remap[12] - 0.5f * remap[13],
                            /* Step Center 3 + 0.5f * Step Width 3 */
                            remap[12] + 0.5f * remap[13],
                            result,
                            /* Step Value 4 */
                            remap[20],
                            /* Ellipse Height 3 */
                            remap[15],
                            /* Ellipse Width 3 */
                            remap[16],
                            /* Inflection Point 3 */
                            remap[17]);
  return elliptical_remap(value,
                          /* Step Center 4 - 0.5f * Step Width 4 */
                          remap[18] - 0.5f * remap[19],
                          /* Step Center 4 + 0.5f * Step Width 4 */
                          remap[18] + 0.5f * remap[19],
                          result,
                          0.0f,
                          /* Ellipse Height 4 */
                          remap[21],
                          /* Ellipse Width 4 */
                          remap[22],
                          /* Inflection Point 4 */
                          remap[23]);
}

template<typename T>
float chained_elliptical_remap_select_steps(int step_count,
                                            float remap[],
                                            float remap_min[],
                                            float remap_max[],
                                            int remap_index_list[],
                                            int remap_index_count,
                                            T seed_offset,
                                            float value)
{
  float remap_randomized[24];
  float result;
  switch (step_count) {
    case 1: {
      for (int i = 0; i < 6; ++i) {
        remap_randomized[i] = remap[i];
      }
      randomize_float_array(remap_randomized,
                            remap_min,
                            remap_max,
                            remap_index_list,
                            remap_index_count,
                            seed_offset);
      result = chained_elliptical_remap_1_step(remap_randomized, value);
      break;
    }
    case 2: {
      for (int i = 0; i < 12; ++i) {
        remap_randomized[i] = remap[i];
      }
      randomize_float_array(remap_randomized,
                            remap_min,
                            remap_max,
                            remap_index_list,
                            remap_index_count,
                            seed_offset);
      result = chained_elliptical_remap_2_steps(remap_randomized, value);
      break;
    }
    case 3: {
      for (int i = 0; i < 18; ++i) {
        remap_randomized[i] = remap[i];
      }
      randomize_float_array(remap_randomized,
                            remap_min,
                            remap_max,
                            remap_index_list,
                            remap_index_count,
                            seed_offset);
      result = chained_elliptical_remap_3_steps(remap_randomized, value);
      break;
    }
    case 4: {
      for (int i = 0; i < 24; ++i) {
        remap_randomized[i] = remap[i];
      }
      randomize_float_array(remap_randomized,
                            remap_min,
                            remap_max,
                            remap_index_list,
                            remap_index_count,
                            seed_offset);
      result = chained_elliptical_remap_4_steps(remap_randomized, value);
      break;
    }
  }
  return result;
}

float4 rotate_scale(float4 coord,
                    float translation_rotation_randomized[7],
                    float scale_randomized[4],
                    bool invert_order_of_transformation)
{
  if (invert_order_of_transformation) {
    coord = float4(scale_randomized[0],
                   scale_randomized[1],
                   scale_randomized[2],
                   scale_randomized[3]) *
            coord;

    if (translation_rotation_randomized[4] != 0.0f) {
      coord = float4(coord.x,
                     cos(translation_rotation_randomized[4]) * coord.y -
                         sin(translation_rotation_randomized[4]) * coord.z,
                     sin(translation_rotation_randomized[4]) * coord.y +
                         cos(translation_rotation_randomized[4]) * coord.z,
                     coord.w);
    }
    if (translation_rotation_randomized[5] != 0.0f) {
      coord = float4(cos(translation_rotation_randomized[5]) * coord.x +
                         sin(translation_rotation_randomized[5]) * coord.z,
                     coord.y,
                     cos(translation_rotation_randomized[5]) * coord.z -
                         sin(translation_rotation_randomized[5]) * coord.x,
                     coord.w);
    }
    if (translation_rotation_randomized[6] != 0.0f) {
      coord = float4(cos(translation_rotation_randomized[6]) * coord.x -
                         sin(translation_rotation_randomized[6]) * coord.y,
                     sin(translation_rotation_randomized[6]) * coord.x +
                         cos(translation_rotation_randomized[6]) * coord.y,
                     coord.z,
                     coord.w);
    }
  }
  else {
    if (translation_rotation_randomized[4] != 0.0f) {
      coord = float4(coord.x,
                     cos(translation_rotation_randomized[4]) * coord.y -
                         sin(translation_rotation_randomized[4]) * coord.z,
                     sin(translation_rotation_randomized[4]) * coord.y +
                         cos(translation_rotation_randomized[4]) * coord.z,
                     coord.w);
    }
    if (translation_rotation_randomized[5] != 0.0f) {
      coord = float4(cos(translation_rotation_randomized[5]) * coord.x +
                         sin(translation_rotation_randomized[5]) * coord.z,
                     coord.y,
                     cos(translation_rotation_randomized[5]) * coord.z -
                         sin(translation_rotation_randomized[5]) * coord.x,
                     coord.w);
    }
    if (translation_rotation_randomized[6] != 0.0f) {
      coord = float4(cos(translation_rotation_randomized[6]) * coord.x -
                         sin(translation_rotation_randomized[6]) * coord.y,
                     sin(translation_rotation_randomized[6]) * coord.x +
                         cos(translation_rotation_randomized[6]) * coord.y,
                     coord.z,
                     coord.w);
    }

    coord = float4(scale_randomized[0],
                   scale_randomized[1],
                   scale_randomized[2],
                   scale_randomized[3]) *
            coord;
  }

  return coord;
}

/* Noise Texture fBM optimized for Raiko Texture. */
float raiko_noise_fbm(float4 coord, float detail, float roughness, float lacunarity)
{
  float octave_scale = 1.0f;
  float amplitude = 1.0f;
  float max_amplitude = 1.0f;
  float sum = 0.0f;

  for (int i = 0; i <= int(detail); ++i) {
    sum += amplitude * perlin_signed(octave_scale * coord);
    max_amplitude += amplitude;
    amplitude *= roughness;
    octave_scale *= lacunarity;
  }

  float remainder = detail - floorf(detail);
  return (remainder != 0.0f) ?
             ((sum + remainder * amplitude * perlin_signed(octave_scale * coord)) /
              (max_amplitude + remainder * amplitude)) :
             (sum / max_amplitude);
}

/* Random offsets are the same as when calling raiko_noise_fbm_layer_1(DeterministicVariables dv,
 * vec4 coord, float seed_offset) with seed_offset == 0.0f */
float4 raiko_noise_fbm_layer_1(const DeterministicVariables &dv, float4 coord)
{
  return float4(raiko_noise_fbm(dv.noise_scale_1 * coord +
                                    float4(178.498459f, 183.790161f, 114.143784f, 163.889908f),
                                dv.noise_detail_1,
                                dv.noise_roughness_1,
                                dv.noise_lacunarity_1),
                raiko_noise_fbm(dv.noise_scale_1 * coord +
                                    float4(147.634079f, 195.179962f, 158.144135f, 128.116669f),
                                dv.noise_detail_1,
                                dv.noise_roughness_1,
                                dv.noise_lacunarity_1),
                raiko_noise_fbm(dv.noise_scale_1 * coord +
                                    float4(195.063629f, 144.612671f, 155.014709f, 165.883881f),
                                dv.noise_detail_1,
                                dv.noise_roughness_1,
                                dv.noise_lacunarity_1),
                raiko_noise_fbm(dv.noise_scale_1 * coord +
                                    float4(115.671997f, 104.330322f, 135.032425f, 120.330460f),
                                dv.noise_detail_1,
                                dv.noise_roughness_1,
                                dv.noise_lacunarity_1));
}

/* Random offsets are the same as when calling raiko_noise_fbm_layer_2(DeterministicVariables dv,
 * vec4 coord, float seed_offset) with seed_offset == 0.0f */
float4 raiko_noise_fbm_layer_2(const DeterministicVariables &dv, float4 coord)
{
  return float4(raiko_noise_fbm(dv.noise_scale_2 * coord +
                                    float4(115.225372f, 181.849701f, 148.865616f, 148.047165f),
                                dv.noise_detail_2,
                                dv.noise_roughness_2,
                                dv.noise_lacunarity_2),
                raiko_noise_fbm(dv.noise_scale_2 * coord +
                                    float4(132.636856f, 169.415527f, 110.008087f, 130.162735f),
                                dv.noise_detail_2,
                                dv.noise_roughness_2,
                                dv.noise_lacunarity_2),
                raiko_noise_fbm(dv.noise_scale_2 * coord +
                                    float4(187.223145f, 167.974121f, 156.358246f, 121.253998f),
                                dv.noise_detail_2,
                                dv.noise_roughness_2,
                                dv.noise_lacunarity_2),
                raiko_noise_fbm(dv.noise_scale_2 * coord +
                                    float4(119.618362f, 126.933167f, 161.577881f, 147.723999f),
                                dv.noise_detail_2,
                                dv.noise_roughness_2,
                                dv.noise_lacunarity_2));
}

float4 rotate_noise(float4 noise_vector, float noise_fragmentation, float3 index)
{
  float deterministic_angle = noise_fragmentation * M_TAU_F *
                              (math::floored_mod(index.x + 0.0625f, 7.0f) +
                               3.0f * math::floored_mod(index.x + 0.0625f, 2.0f) +
                               13.0f * (math::floored_mod(index.y + 0.0625f, 7.0f) +
                                        3.0f * math::floored_mod(index.y + 0.0625f, 2.0f)) +
                               143.0f * (math::floored_mod(index.z + 0.0625f, 7.0f) +
                                         3.0f * math::floored_mod(index.z + 0.0625f, 2.0f)));
  float4 noise_vector_rotated = float4(
      cos(deterministic_angle) * noise_vector.x - sin(deterministic_angle) * noise_vector.y,
      sin(deterministic_angle) * noise_vector.x + cos(deterministic_angle) * noise_vector.y,
      cos(deterministic_angle) * noise_vector.z - sin(deterministic_angle) * noise_vector.w,
      sin(deterministic_angle) * noise_vector.z + cos(deterministic_angle) * noise_vector.w);
  float random_angle = noise_fragmentation * M_TAU_F * 5.0f * hash_float_to_float(index);
  return float4(
      cos(random_angle) * noise_vector_rotated.x - sin(random_angle) * noise_vector_rotated.z,
      cos(random_angle) * noise_vector_rotated.y - sin(random_angle) * noise_vector_rotated.w,
      sin(random_angle) * noise_vector_rotated.x + cos(random_angle) * noise_vector_rotated.z,
      sin(random_angle) * noise_vector_rotated.y + cos(random_angle) * noise_vector_rotated.w);
}

float4 rotate_noise(float4 noise_vector, float noise_fragmentation, float4 index)
{
  float deterministic_angle = noise_fragmentation * M_TAU_F *
                              (math::floored_mod(index.x + 0.0625f, 7.0f) +
                               3.0f * math::floored_mod(index.x + 0.0625f, 2.0f) +
                               13.0f * (math::floored_mod(index.y + 0.0625f, 7.0f) +
                                        3.0f * math::floored_mod(index.y + 0.0625f, 2.0f)) +
                               143.0f * (math::floored_mod(index.z + 0.0625f, 7.0f) +
                                         3.0f * math::floored_mod(index.z + 0.0625f, 2.0f)) +
                               2431.0f * (math::floored_mod(index.w + 0.0625f, 7.0f) +
                                          3.0f * math::floored_mod(index.w + 0.0625f, 2.0f)));
  float4 noise_vector_rotated = float4(
      cos(deterministic_angle) * noise_vector.x - sin(deterministic_angle) * noise_vector.y,
      sin(deterministic_angle) * noise_vector.x + cos(deterministic_angle) * noise_vector.y,
      cos(deterministic_angle) * noise_vector.z - sin(deterministic_angle) * noise_vector.w,
      sin(deterministic_angle) * noise_vector.z + cos(deterministic_angle) * noise_vector.w);
  float random_angle = noise_fragmentation * M_TAU_F * 5.0f * hash_float_to_float(index);
  return float4(
      cos(random_angle) * noise_vector_rotated.x - sin(random_angle) * noise_vector_rotated.z,
      cos(random_angle) * noise_vector_rotated.y - sin(random_angle) * noise_vector_rotated.w,
      sin(random_angle) * noise_vector_rotated.x + cos(random_angle) * noise_vector_rotated.z,
      sin(random_angle) * noise_vector_rotated.y + cos(random_angle) * noise_vector_rotated.w);
}

OutVariables raiko_select_mode_0d(const DeterministicVariables &dv,
                                  float r_sphere[],
                                  float r_sphere_min[],
                                  float r_sphere_max[],
                                  int r_sphere_index_list[],
                                  int r_sphere_index_count,
                                  float translation_rotation[],
                                  float translation_rotation_min[],
                                  float translation_rotation_max[],
                                  int translation_rotation_index_list[],
                                  int translation_rotation_index_count,
                                  float scale[],
                                  float scale_randomness[],
                                  int scale_index_list[],
                                  int scale_index_count,
                                  float remap[],
                                  float remap_min[],
                                  float remap_max[],
                                  int remap_index_list[],
                                  int remap_index_count)
{
  OutVariables ov;
  float4 fields_noise_layer_1 = dv.calculate_fields_noise_1 ?
                                    raiko_noise_fbm_layer_1(
                                        dv,
                                        dv.transform_fields_noise ?
                                            rotate_scale(dv.coord,
                                                         translation_rotation,
                                                         scale,
                                                         dv.invert_order_of_transformation) :
                                            dv.coord) :
                                    float4(0.0f, 0.0f, 0.0f, 0.0f);
  float4 fields_noise_layer_2 = dv.calculate_fields_noise_2 ?
                                    raiko_noise_fbm_layer_2(
                                        dv,
                                        dv.transform_fields_noise ?
                                            rotate_scale(dv.coord,
                                                         translation_rotation,
                                                         scale,
                                                         dv.invert_order_of_transformation) :
                                            dv.coord) :
                                    float4(0.0f, 0.0f, 0.0f, 0.0f);
  float4 fields_noise_vector = dv.noise_fields_strength_1 * fields_noise_layer_1 +
                               dv.noise_fields_strength_2 * fields_noise_layer_2;

  float r_sphere_randomized[4] = {r_sphere[0], r_sphere[1], r_sphere[2], r_sphere[3]};
  randomize_float_array(r_sphere_randomized,
                        r_sphere_min,
                        r_sphere_max,
                        r_sphere_index_list,
                        r_sphere_index_count,
                        float3(0.0f, 0.0f, 0.0f));
  float translation_rotation_randomized[7];
  for (int n = 0; n < 7; ++n) {
    translation_rotation_randomized[n] = translation_rotation[n];
  }
  randomize_float_array(translation_rotation_randomized,
                        translation_rotation_min,
                        translation_rotation_max,
                        translation_rotation_index_list,
                        translation_rotation_index_count,
                        float3(0.0f, 0.0f, 0.0f));
  float scale_randomized[4] = {scale[0], scale[1], scale[2], scale[3]};
  randomize_scale(scale_randomized,
                  scale_randomness,
                  scale_index_list,
                  scale_index_count,
                  dv.uniform_scale_randomness,
                  float3(0.0f, 0.0f, 0.0f));

  float4 iteration_position = float4(translation_rotation_randomized[0],
                                     translation_rotation_randomized[1],
                                     translation_rotation_randomized[2],
                                     translation_rotation_randomized[3]);
  float4 noiseless_coord = rotate_scale(dv.coord - iteration_position,
                                        translation_rotation_randomized,
                                        scale_randomized,
                                        dv.invert_order_of_transformation);
  float4 iteration_coord = noiseless_coord + (dv.noise_fragmentation_non_zero ?
                                                  rotate_noise(fields_noise_vector,
                                                               dv.noise_fragmentation,
                                                               float3(0.0f, 0.0f, 0.0f)) :
                                                  fields_noise_vector);

  if (dv.mode == SHD_RAIKO_ADDITIVE) {
    ov.out_r_sphere_field = chained_elliptical_remap_select_steps(
        dv.step_count,
        remap,
        remap_min,
        remap_max,
        remap_index_list,
        remap_index_count,
        float3(0.0f, 0.0f, 0.0f),
        calculate_l_angle_bisector_4d(dv.integer_sides,
                                      dv.elliptical_corners,
                                      r_sphere_randomized[0],
                                      r_sphere_randomized[1],
                                      r_sphere_randomized[2],
                                      r_sphere_randomized[3],
                                      iteration_coord));
  }
  else {
    float4 coordinates_noise_layer_1 =
        dv.calculate_coordinates_noise_1 ?
            raiko_noise_fbm_layer_1(
                dv,
                dv.transform_coordinates_noise ?
                    rotate_scale(
                        dv.coord, translation_rotation, scale, dv.invert_order_of_transformation) :
                    dv.coord) :
            fields_noise_layer_1;
    float4 coordinates_noise_layer_2 =
        dv.calculate_coordinates_noise_2 ?
            raiko_noise_fbm_layer_2(
                dv,
                dv.transform_coordinates_noise ?
                    rotate_scale(
                        dv.coord, translation_rotation, scale, dv.invert_order_of_transformation) :
                    dv.coord) :
            fields_noise_layer_2;
    float4 coordinates_noise_vector = dv.noise_coordinates_strength_1 * coordinates_noise_layer_1 +
                                      dv.noise_coordinates_strength_2 * coordinates_noise_layer_2;

    float4 iteration_r_sphere_coordinates = noiseless_coord +
                                            (dv.noise_fragmentation_non_zero ?
                                                 rotate_noise(coordinates_noise_vector,
                                                              dv.noise_fragmentation,
                                                              float3(0.0f, 0.0f, 0.0f)) :
                                                 coordinates_noise_vector);

    float4 out_fields = calculate_out_fields_4d(dv.calculate_r_sphere_field,
                                                dv.calculate_r_gon_parameter_field,
                                                dv.calculate_max_unit_parameter_field,
                                                dv.normalize_r_gon_parameter,
                                                dv.integer_sides,
                                                dv.elliptical_corners,
                                                r_sphere_randomized[0],
                                                r_sphere_randomized[1],
                                                r_sphere_randomized[2],
                                                r_sphere_randomized[3],
                                                iteration_coord);

    ov.out_r_sphere_field = out_fields.x;
    ov.r_gon_parameter_field = out_fields.y;
    ov.max_unit_parameter_field = out_fields.z;
    ov.segment_id_field = out_fields.w;
    ov.out_r_sphere_coordinates = iteration_r_sphere_coordinates;
  }
  return ov;
}

OutVariables raiko_select_mode_1d(const DeterministicVariables &dv,
                                  float r_sphere[],
                                  float r_sphere_min[],
                                  float r_sphere_max[],
                                  int r_sphere_index_list[],
                                  int r_sphere_index_count,
                                  float translation_rotation[],
                                  float translation_rotation_min[],
                                  float translation_rotation_max[],
                                  int translation_rotation_index_list[],
                                  int translation_rotation_index_count,
                                  float scale[],
                                  float scale_randomness[],
                                  int scale_index_list[],
                                  int scale_index_count,
                                  float remap[],
                                  float remap_min[],
                                  float remap_max[],
                                  int remap_index_list[],
                                  int remap_index_count,
                                  float4 initial_index,
                                  float4 initial_position)
{
  float scanning_window_size = math::ceil(4.0f * dv.accuracy);
  OutVariables ov;
  switch (dv.mode) {
    case SHD_RAIKO_ADDITIVE: {
      ov.out_r_sphere_field = 0.0f;
      float4 fields_noise_layer_1 = dv.calculate_fields_noise_1 ?
                                        raiko_noise_fbm_layer_1(
                                            dv,
                                            dv.transform_fields_noise ?
                                                rotate_scale(dv.coord,
                                                             translation_rotation,
                                                             scale,
                                                             dv.invert_order_of_transformation) :
                                                dv.coord) :
                                        float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 fields_noise_layer_2 = dv.calculate_fields_noise_2 ?
                                        raiko_noise_fbm_layer_2(
                                            dv,
                                            dv.transform_fields_noise ?
                                                rotate_scale(dv.coord,
                                                             translation_rotation,
                                                             scale,
                                                             dv.invert_order_of_transformation) :
                                                dv.coord) :
                                        float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 fields_noise_vector = dv.noise_fields_strength_1 * fields_noise_layer_1 +
                                   dv.noise_fields_strength_2 * fields_noise_layer_2;

      for (float i = -scanning_window_size; i <= scanning_window_size; ++i) {
        float3 iteration_index = float3(i, 0.0f, 0.0f) + float3(initial_index.x, 0.0f, 0.0f);
        float r_sphere_randomized[4] = {r_sphere[0], r_sphere[1], r_sphere[2], r_sphere[3]};
        randomize_float_array(r_sphere_randomized,
                              r_sphere_min,
                              r_sphere_max,
                              r_sphere_index_list,
                              r_sphere_index_count,
                              5.0f * iteration_index);
        float translation_rotation_randomized[7];
        for (int n = 0; n < 7; ++n) {
          translation_rotation_randomized[n] = translation_rotation[n];
        }
        randomize_float_array(translation_rotation_randomized,
                              translation_rotation_min,
                              translation_rotation_max,
                              translation_rotation_index_list,
                              translation_rotation_index_count,
                              8.0f * iteration_index);
        float scale_randomized[4] = {scale[0], scale[1], scale[2], scale[3]};
        randomize_scale(scale_randomized,
                        scale_randomness,
                        scale_index_list,
                        scale_index_count,
                        dv.uniform_scale_randomness,
                        9.0f * iteration_index);

        float4 iteration_position = initial_position + i * dv.grid_vector_1 +
                                    float4(translation_rotation_randomized[0],
                                           translation_rotation_randomized[1],
                                           translation_rotation_randomized[2],
                                           translation_rotation_randomized[3]);
        float4 noiseless_coord = rotate_scale(dv.coord - iteration_position,
                                              translation_rotation_randomized,
                                              scale_randomized,
                                              dv.invert_order_of_transformation);
        float4 iteration_coord = noiseless_coord + (dv.noise_fragmentation_non_zero ?
                                                        rotate_noise(fields_noise_vector,
                                                                     dv.noise_fragmentation,
                                                                     iteration_index) :
                                                        fields_noise_vector);

        ov.out_r_sphere_field += chained_elliptical_remap_select_steps(
            dv.step_count,
            remap,
            remap_min,
            remap_max,
            remap_index_list,
            remap_index_count,
            27.0f * iteration_index,
            calculate_l_angle_bisector_4d(dv.integer_sides,
                                          dv.elliptical_corners,
                                          r_sphere_randomized[0],
                                          r_sphere_randomized[1],
                                          r_sphere_randomized[2],
                                          r_sphere_randomized[3],
                                          iteration_coord));
      }
      break;
    }
    case SHD_RAIKO_CLOSEST: {
      ov.out_index_field = float4(0.0f, 0.0f, 0.0f, 0.0f);
      float min_distance = FLT_MAX;
      float closest_rotation_randomized[7];
      float closest_scale_randomized[4];

      float4 index_noise_layer_1 = dv.calculate_fields_noise_1 ?
                                       raiko_noise_fbm_layer_1(
                                           dv,
                                           dv.transform_fields_noise ?
                                               rotate_scale(dv.coord,
                                                            translation_rotation,
                                                            scale,
                                                            dv.invert_order_of_transformation) :
                                               dv.coord) :
                                       float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 index_noise_layer_2 = dv.calculate_fields_noise_2 ?
                                       raiko_noise_fbm_layer_2(
                                           dv,
                                           dv.transform_fields_noise ?
                                               rotate_scale(dv.coord,
                                                            translation_rotation,
                                                            scale,
                                                            dv.invert_order_of_transformation) :
                                               dv.coord) :
                                       float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 index_noise_vector = dv.noise_fields_strength_1 * index_noise_layer_1 +
                                  dv.noise_fields_strength_2 * index_noise_layer_2;

      for (float i = -scanning_window_size; i <= scanning_window_size; ++i) {
        float3 iteration_index = float3(i, 0.0f, 0.0f) + float3(initial_index.x, 0.0f, 0.0f);
        float translation_rotation_randomized[7];
        for (int n = 0; n < 7; ++n) {
          translation_rotation_randomized[n] = translation_rotation[n];
        }
        randomize_float_array(translation_rotation_randomized,
                              translation_rotation_min,
                              translation_rotation_max,
                              translation_rotation_index_list,
                              translation_rotation_index_count,
                              8.0f * iteration_index);
        float scale_randomized[4] = {scale[0], scale[1], scale[2], scale[3]};
        randomize_scale(scale_randomized,
                        scale_randomness,
                        scale_index_list,
                        scale_index_count,
                        dv.uniform_scale_randomness,
                        9.0f * iteration_index);

        float4 iteration_position = initial_position + i * dv.grid_vector_1 +
                                    float4(translation_rotation_randomized[0],
                                           translation_rotation_randomized[1],
                                           translation_rotation_randomized[2],
                                           translation_rotation_randomized[3]);
        float4 noiseless_coord = rotate_scale(dv.coord - iteration_position,
                                              translation_rotation_randomized,
                                              scale_randomized,
                                              dv.invert_order_of_transformation);
        float4 iteration_coord = noiseless_coord + (dv.noise_fragmentation_non_zero ?
                                                        rotate_noise(index_noise_vector,
                                                                     dv.noise_fragmentation,
                                                                     iteration_index) :
                                                        index_noise_vector);

        float l_iteration_coord = euclidean_norm(iteration_coord);
        if (l_iteration_coord < min_distance) {
          min_distance = l_iteration_coord;
          ov.out_index_field.x = iteration_index.x;
          ov.out_position_field = iteration_position;
          /* Translation data not needed for subsequent computations. */
          closest_rotation_randomized[4] = translation_rotation_randomized[4];
          closest_rotation_randomized[5] = translation_rotation_randomized[5];
          closest_rotation_randomized[6] = translation_rotation_randomized[6];
          closest_scale_randomized[0] = scale_randomized[0];
          closest_scale_randomized[1] = scale_randomized[1];
          closest_scale_randomized[2] = scale_randomized[2];
          closest_scale_randomized[3] = scale_randomized[3];
        }
      }

      float4 closest_fields_noise_layer_1 =
          dv.calculate_fields_noise_1 ?
              raiko_noise_fbm_layer_1(dv,
                                      dv.transform_fields_noise ?
                                          rotate_scale(dv.coord - ov.out_position_field,
                                                       closest_rotation_randomized,
                                                       closest_scale_randomized,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord - ov.out_position_field) :
              float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 closest_fields_noise_layer_2 =
          dv.calculate_fields_noise_2 ?
              raiko_noise_fbm_layer_2(dv,
                                      dv.transform_fields_noise ?
                                          rotate_scale(dv.coord - ov.out_position_field,
                                                       closest_rotation_randomized,
                                                       closest_scale_randomized,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord - ov.out_position_field) :
              float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 closest_fields_noise_vector = dv.noise_fields_strength_1 *
                                               closest_fields_noise_layer_1 +
                                           dv.noise_fields_strength_2 *
                                               closest_fields_noise_layer_2;
      float4 closest_coordinates_noise_layer_1 =
          dv.calculate_coordinates_noise_1 ?
              raiko_noise_fbm_layer_1(dv,
                                      dv.transform_coordinates_noise ?
                                          rotate_scale(dv.coord - ov.out_position_field,
                                                       closest_rotation_randomized,
                                                       closest_scale_randomized,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord - ov.out_position_field) :
              closest_fields_noise_layer_1;
      float4 closest_coordinates_noise_layer_2 =
          dv.calculate_coordinates_noise_2 ?
              raiko_noise_fbm_layer_2(dv,
                                      dv.transform_coordinates_noise ?
                                          rotate_scale(dv.coord - ov.out_position_field,
                                                       closest_rotation_randomized,
                                                       closest_scale_randomized,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord - ov.out_position_field) :
              closest_fields_noise_layer_2;
      float4 closest_coordinates_noise_vector = dv.noise_coordinates_strength_1 *
                                                    closest_coordinates_noise_layer_1 +
                                                dv.noise_coordinates_strength_2 *
                                                    closest_coordinates_noise_layer_2;

      float4 noiseless_coord = rotate_scale(dv.coord - ov.out_position_field,
                                            closest_rotation_randomized,
                                            closest_scale_randomized,
                                            dv.invert_order_of_transformation);
      ov.out_r_sphere_coordinates =
          noiseless_coord + (dv.noise_fragmentation_non_zero ?
                                 rotate_noise(closest_coordinates_noise_vector,
                                              dv.noise_fragmentation,
                                              7.0f * float3(ov.out_index_field.x, 0.0f, 0.0f)) :
                                 closest_coordinates_noise_vector);

      float r_sphere_randomized[4] = {r_sphere[0], r_sphere[1], r_sphere[2], r_sphere[3]};
      randomize_float_array(r_sphere_randomized,
                            r_sphere_min,
                            r_sphere_max,
                            r_sphere_index_list,
                            r_sphere_index_count,
                            5.0f * float3(ov.out_index_field.x, 0.0f, 0.0f));

      float4 out_fields = calculate_out_fields_4d(
          dv.calculate_r_sphere_field,
          dv.calculate_r_gon_parameter_field,
          dv.calculate_max_unit_parameter_field,
          dv.normalize_r_gon_parameter,
          dv.integer_sides,
          dv.elliptical_corners,
          r_sphere_randomized[0],
          r_sphere_randomized[1],
          r_sphere_randomized[2],
          r_sphere_randomized[3],
          noiseless_coord + (dv.noise_fragmentation_non_zero ?
                                 rotate_noise(closest_fields_noise_vector,
                                              dv.noise_fragmentation,
                                              7.0f * float3(ov.out_index_field.x, 0.0f, 0.0f)) :
                                 closest_fields_noise_vector));
      ov.out_r_sphere_field = out_fields.x;
      ov.r_gon_parameter_field = out_fields.y;
      ov.max_unit_parameter_field = out_fields.z;
      ov.segment_id_field = out_fields.w;
      break;
    }
    case SHD_RAIKO_SMOOTH_MINIMUM: {
      ov.out_r_sphere_field = 0.0f;
      ov.r_gon_parameter_field = 0.0f;
      ov.max_unit_parameter_field = 0.0f;
      ov.segment_id_field = 0.0f;
      ov.out_r_sphere_coordinates = float4(0.0f, 0.0f, 0.0f, 0.0f);
      ov.out_index_field = float4(0.0f, 0.0f, 0.0f, 0.0f);
      ov.out_position_field = float4(0.0f, 0.0f, 0.0f, 0.0f);
      bool first_iteration = true;

      float4 fields_noise_layer_1 = dv.calculate_fields_noise_1 ?
                                        raiko_noise_fbm_layer_1(
                                            dv,
                                            dv.transform_fields_noise ?
                                                rotate_scale(dv.coord,
                                                             translation_rotation,
                                                             scale,
                                                             dv.invert_order_of_transformation) :
                                                dv.coord) :
                                        float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 fields_noise_layer_2 = dv.calculate_fields_noise_2 ?
                                        raiko_noise_fbm_layer_2(
                                            dv,
                                            dv.transform_fields_noise ?
                                                rotate_scale(dv.coord,
                                                             translation_rotation,
                                                             scale,
                                                             dv.invert_order_of_transformation) :
                                                dv.coord) :
                                        float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 fields_noise_vector = dv.noise_fields_strength_1 * fields_noise_layer_1 +
                                   dv.noise_fields_strength_2 * fields_noise_layer_2;
      float4 coordinates_noise_layer_1 =
          dv.calculate_coordinates_noise_1 ?
              raiko_noise_fbm_layer_1(dv,
                                      dv.transform_coordinates_noise ?
                                          rotate_scale(dv.coord,
                                                       translation_rotation,
                                                       scale,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord) :
              fields_noise_layer_1;
      float4 coordinates_noise_layer_2 =
          dv.calculate_coordinates_noise_2 ?
              raiko_noise_fbm_layer_2(dv,
                                      dv.transform_coordinates_noise ?
                                          rotate_scale(dv.coord,
                                                       translation_rotation,
                                                       scale,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord) :
              fields_noise_layer_2;
      float4 coordinates_noise_vector = dv.noise_coordinates_strength_1 *
                                            coordinates_noise_layer_1 +
                                        dv.noise_coordinates_strength_2 *
                                            coordinates_noise_layer_2;

      for (float i = -scanning_window_size; i <= scanning_window_size; ++i) {
        float3 iteration_index = float3(i, 0.0f, 0.0f) + float3(initial_index.x, 0.0f, 0.0f);
        float r_sphere_randomized[4] = {r_sphere[0], r_sphere[1], r_sphere[2], r_sphere[3]};
        randomize_float_array(r_sphere_randomized,
                              r_sphere_min,
                              r_sphere_max,
                              r_sphere_index_list,
                              r_sphere_index_count,
                              5.0f * iteration_index);
        float translation_rotation_randomized[7];
        for (int n = 0; n < 7; ++n) {
          translation_rotation_randomized[n] = translation_rotation[n];
        }
        randomize_float_array(translation_rotation_randomized,
                              translation_rotation_min,
                              translation_rotation_max,
                              translation_rotation_index_list,
                              translation_rotation_index_count,
                              8.0f * iteration_index);
        float scale_randomized[4] = {scale[0], scale[1], scale[2], scale[3]};
        randomize_scale(scale_randomized,
                        scale_randomness,
                        scale_index_list,
                        scale_index_count,
                        dv.uniform_scale_randomness,
                        9.0f * iteration_index);

        float4 iteration_position = initial_position + i * dv.grid_vector_1 +
                                    float4(translation_rotation_randomized[0],
                                           translation_rotation_randomized[1],
                                           translation_rotation_randomized[2],
                                           translation_rotation_randomized[3]);
        float4 noiseless_coord = rotate_scale(dv.coord - iteration_position,
                                              translation_rotation_randomized,
                                              scale_randomized,
                                              dv.invert_order_of_transformation);
        float4 iteration_coord = noiseless_coord + (dv.noise_fragmentation_non_zero ?
                                                        rotate_noise(fields_noise_vector,
                                                                     dv.noise_fragmentation,
                                                                     iteration_index) :
                                                        fields_noise_vector);
        float4 iteration_r_sphere_coordinates =
            noiseless_coord +
            (dv.noise_fragmentation_non_zero ?
                 rotate_noise(coordinates_noise_vector, dv.noise_fragmentation, iteration_index) :
                 coordinates_noise_vector);

        float4 out_fields = calculate_out_fields_4d(dv.calculate_r_sphere_field,
                                                    dv.calculate_r_gon_parameter_field,
                                                    dv.calculate_max_unit_parameter_field,
                                                    dv.normalize_r_gon_parameter,
                                                    dv.integer_sides,
                                                    dv.elliptical_corners,
                                                    r_sphere_randomized[0],
                                                    r_sphere_randomized[1],
                                                    r_sphere_randomized[2],
                                                    r_sphere_randomized[3],
                                                    iteration_coord);
        float iteration_l_angle_bisector_4d = out_fields.x;
        float iteration_r_gon_parameter = out_fields.y;
        float iteration_max_unit_parameter = out_fields.z;
        float iteration_segment_id = out_fields.w;

        if (dv.smoothness_non_zero) {
          float interpolation_factor = first_iteration ?
                                           1.0f :
                                           smoothstep(0.0f,
                                                      1.0f,
                                                      0.5f + 0.5f *
                                                                 (ov.out_r_sphere_field -
                                                                  iteration_l_angle_bisector_4d) /
                                                                 dv.smoothness);
          float substraction_factor = dv.smoothness * interpolation_factor *
                                      (1.0f - interpolation_factor);
          ov.out_r_sphere_field = mix(ov.out_r_sphere_field,
                                      iteration_l_angle_bisector_4d,
                                      interpolation_factor) -
                                  substraction_factor;

          ov.r_gon_parameter_field = mix(
              ov.r_gon_parameter_field, iteration_r_gon_parameter, interpolation_factor);
          ov.max_unit_parameter_field = mix(
              ov.max_unit_parameter_field, iteration_max_unit_parameter, interpolation_factor);
          ov.segment_id_field = mix(
              ov.segment_id_field, iteration_segment_id, interpolation_factor);
          ov.out_index_field.x = mix(
              ov.out_index_field.x, iteration_index.x, interpolation_factor);
          ov.out_position_field = mix(
              ov.out_position_field, iteration_position, interpolation_factor);
          ov.out_r_sphere_coordinates = mix(
              ov.out_r_sphere_coordinates, iteration_r_sphere_coordinates, interpolation_factor);
        }
        else if ((iteration_l_angle_bisector_4d < ov.out_r_sphere_field) || first_iteration) {
          ov.out_r_sphere_field = iteration_l_angle_bisector_4d;
          ov.r_gon_parameter_field = iteration_r_gon_parameter;
          ov.max_unit_parameter_field = iteration_max_unit_parameter;
          ov.segment_id_field = iteration_segment_id;
          ov.out_index_field.x = iteration_index.x;
          ov.out_position_field = iteration_position;
          ov.out_r_sphere_coordinates = iteration_r_sphere_coordinates;
        }
        first_iteration = false;
      }
      break;
    }
  }
  return ov;
}

OutVariables raiko_select_mode_2d(const DeterministicVariables &dv,
                                  float r_sphere[],
                                  float r_sphere_min[],
                                  float r_sphere_max[],
                                  int r_sphere_index_list[],
                                  int r_sphere_index_count,
                                  float translation_rotation[],
                                  float translation_rotation_min[],
                                  float translation_rotation_max[],
                                  int translation_rotation_index_list[],
                                  int translation_rotation_index_count,
                                  float scale[],
                                  float scale_randomness[],
                                  int scale_index_list[],
                                  int scale_index_count,
                                  float remap[],
                                  float remap_min[],
                                  float remap_max[],
                                  int remap_index_list[],
                                  int remap_index_count,
                                  float4 initial_index,
                                  float4 initial_position)
{
  float l_grid_vector1 = euclidean_norm(dv.grid_vector_1);
  float l_grid_vector2 = euclidean_norm(dv.grid_vector_2);
  float l_shortest_grid_vector = math::min(l_grid_vector1, l_grid_vector2);
  float2 scanning_window_size = math::ceil(
      4.0f * dv.accuracy *
      float2(l_shortest_grid_vector / l_grid_vector1, l_shortest_grid_vector / l_grid_vector2));
  OutVariables ov;
  switch (dv.mode) {
    case SHD_RAIKO_ADDITIVE: {
      ov.out_r_sphere_field = 0.0f;
      float4 fields_noise_layer_1 = dv.calculate_fields_noise_1 ?
                                        raiko_noise_fbm_layer_1(
                                            dv,
                                            dv.transform_fields_noise ?
                                                rotate_scale(dv.coord,
                                                             translation_rotation,
                                                             scale,
                                                             dv.invert_order_of_transformation) :
                                                dv.coord) :
                                        float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 fields_noise_layer_2 = dv.calculate_fields_noise_2 ?
                                        raiko_noise_fbm_layer_2(
                                            dv,
                                            dv.transform_fields_noise ?
                                                rotate_scale(dv.coord,
                                                             translation_rotation,
                                                             scale,
                                                             dv.invert_order_of_transformation) :
                                                dv.coord) :
                                        float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 fields_noise_vector = dv.noise_fields_strength_1 * fields_noise_layer_1 +
                                   dv.noise_fields_strength_2 * fields_noise_layer_2;

      for (float j = -scanning_window_size.y; j <= scanning_window_size.y; ++j) {
        for (float i = -scanning_window_size.x; i <= scanning_window_size.x; ++i) {
          float3 iteration_index = float3(i, j, 0.0f) +
                                   float3(initial_index.x, initial_index.y, 0.0f);
          float r_sphere_randomized[4] = {r_sphere[0], r_sphere[1], r_sphere[2], r_sphere[3]};
          randomize_float_array(r_sphere_randomized,
                                r_sphere_min,
                                r_sphere_max,
                                r_sphere_index_list,
                                r_sphere_index_count,
                                5.0f * iteration_index);
          float translation_rotation_randomized[7];
          for (int n = 0; n < 7; ++n) {
            translation_rotation_randomized[n] = translation_rotation[n];
          }
          randomize_float_array(translation_rotation_randomized,
                                translation_rotation_min,
                                translation_rotation_max,
                                translation_rotation_index_list,
                                translation_rotation_index_count,
                                8.0f * iteration_index);
          float scale_randomized[4] = {scale[0], scale[1], scale[2], scale[3]};
          randomize_scale(scale_randomized,
                          scale_randomness,
                          scale_index_list,
                          scale_index_count,
                          dv.uniform_scale_randomness,
                          9.0f * iteration_index);

          float4 iteration_position = initial_position + i * dv.grid_vector_1 +
                                      j * dv.grid_vector_2 +
                                      float4(translation_rotation_randomized[0],
                                             translation_rotation_randomized[1],
                                             translation_rotation_randomized[2],
                                             translation_rotation_randomized[3]);
          float4 noiseless_coord = rotate_scale(dv.coord - iteration_position,
                                                translation_rotation_randomized,
                                                scale_randomized,
                                                dv.invert_order_of_transformation);
          float4 iteration_coord = noiseless_coord + (dv.noise_fragmentation_non_zero ?
                                                          rotate_noise(fields_noise_vector,
                                                                       dv.noise_fragmentation,
                                                                       iteration_index) :
                                                          fields_noise_vector);

          ov.out_r_sphere_field += chained_elliptical_remap_select_steps(
              dv.step_count,
              remap,
              remap_min,
              remap_max,
              remap_index_list,
              remap_index_count,
              27.0f * iteration_index,
              calculate_l_angle_bisector_4d(dv.integer_sides,
                                            dv.elliptical_corners,
                                            r_sphere_randomized[0],
                                            r_sphere_randomized[1],
                                            r_sphere_randomized[2],
                                            r_sphere_randomized[3],
                                            iteration_coord));
        }
      }
      break;
    }
    case SHD_RAIKO_CLOSEST: {
      ov.out_index_field = float4(0.0f, 0.0f, 0.0f, 0.0f);
      float min_distance = FLT_MAX;
      float closest_rotation_randomized[7];
      float closest_scale_randomized[4];
      float4 index_noise_layer_1 = dv.calculate_fields_noise_1 ?
                                       raiko_noise_fbm_layer_1(
                                           dv,
                                           dv.transform_fields_noise ?
                                               rotate_scale(dv.coord,
                                                            translation_rotation,
                                                            scale,
                                                            dv.invert_order_of_transformation) :
                                               dv.coord) :
                                       float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 index_noise_layer_2 = dv.calculate_fields_noise_2 ?
                                       raiko_noise_fbm_layer_2(
                                           dv,
                                           dv.transform_fields_noise ?
                                               rotate_scale(dv.coord,
                                                            translation_rotation,
                                                            scale,
                                                            dv.invert_order_of_transformation) :
                                               dv.coord) :
                                       float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 index_noise_vector = dv.noise_fields_strength_1 * index_noise_layer_1 +
                                  dv.noise_fields_strength_2 * index_noise_layer_2;

      for (float j = -scanning_window_size.y; j <= scanning_window_size.y; ++j) {
        for (float i = -scanning_window_size.x; i <= scanning_window_size.x; ++i) {
          float3 iteration_index = float3(i, j, 0.0f) +
                                   float3(initial_index.x, initial_index.y, 0.0f);
          float translation_rotation_randomized[7];
          for (int n = 0; n < 7; ++n) {
            translation_rotation_randomized[n] = translation_rotation[n];
          }
          randomize_float_array(translation_rotation_randomized,
                                translation_rotation_min,
                                translation_rotation_max,
                                translation_rotation_index_list,
                                translation_rotation_index_count,
                                8.0f * iteration_index);
          float scale_randomized[4] = {scale[0], scale[1], scale[2], scale[3]};
          randomize_scale(scale_randomized,
                          scale_randomness,
                          scale_index_list,
                          scale_index_count,
                          dv.uniform_scale_randomness,
                          9.0f * iteration_index);

          float4 iteration_position = initial_position + i * dv.grid_vector_1 +
                                      j * dv.grid_vector_2 +
                                      float4(translation_rotation_randomized[0],
                                             translation_rotation_randomized[1],
                                             translation_rotation_randomized[2],
                                             translation_rotation_randomized[3]);
          float4 noiseless_coord = rotate_scale(dv.coord - iteration_position,
                                                translation_rotation_randomized,
                                                scale_randomized,
                                                dv.invert_order_of_transformation);
          float4 iteration_coord = noiseless_coord + (dv.noise_fragmentation_non_zero ?
                                                          rotate_noise(index_noise_vector,
                                                                       dv.noise_fragmentation,
                                                                       iteration_index) :
                                                          index_noise_vector);

          float l_iteration_coord = euclidean_norm(iteration_coord);
          if (l_iteration_coord < min_distance) {
            min_distance = l_iteration_coord;
            ov.out_index_field.x = iteration_index.x;
            ov.out_index_field.y = iteration_index.y;
            ov.out_position_field = iteration_position;
            /* Translation data not needed for subsequent computations. */
            closest_rotation_randomized[4] = translation_rotation_randomized[4];
            closest_rotation_randomized[5] = translation_rotation_randomized[5];
            closest_rotation_randomized[6] = translation_rotation_randomized[6];
            closest_scale_randomized[0] = scale_randomized[0];
            closest_scale_randomized[1] = scale_randomized[1];
            closest_scale_randomized[2] = scale_randomized[2];
            closest_scale_randomized[3] = scale_randomized[3];
          }
        }
      }

      float4 closest_fields_noise_layer_1 =
          dv.calculate_fields_noise_1 ?
              raiko_noise_fbm_layer_1(dv,
                                      dv.transform_fields_noise ?
                                          rotate_scale(dv.coord - ov.out_position_field,
                                                       closest_rotation_randomized,
                                                       closest_scale_randomized,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord - ov.out_position_field) :
              float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 closest_fields_noise_layer_2 =
          dv.calculate_fields_noise_2 ?
              raiko_noise_fbm_layer_2(dv,
                                      dv.transform_fields_noise ?
                                          rotate_scale(dv.coord - ov.out_position_field,
                                                       closest_rotation_randomized,
                                                       closest_scale_randomized,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord - ov.out_position_field) :
              float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 closest_fields_noise_vector = dv.noise_fields_strength_1 *
                                               closest_fields_noise_layer_1 +
                                           dv.noise_fields_strength_2 *
                                               closest_fields_noise_layer_2;
      float4 closest_coordinates_noise_layer_1 =
          dv.calculate_coordinates_noise_1 ?
              raiko_noise_fbm_layer_1(dv,
                                      dv.transform_coordinates_noise ?
                                          rotate_scale(dv.coord - ov.out_position_field,
                                                       closest_rotation_randomized,
                                                       closest_scale_randomized,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord - ov.out_position_field) :
              closest_fields_noise_layer_1;
      float4 closest_coordinates_noise_layer_2 =
          dv.calculate_coordinates_noise_2 ?
              raiko_noise_fbm_layer_2(dv,
                                      dv.transform_coordinates_noise ?
                                          rotate_scale(dv.coord - ov.out_position_field,
                                                       closest_rotation_randomized,
                                                       closest_scale_randomized,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord - ov.out_position_field) :
              closest_fields_noise_layer_2;
      float4 closest_coordinates_noise_vector = dv.noise_coordinates_strength_1 *
                                                    closest_coordinates_noise_layer_1 +
                                                dv.noise_coordinates_strength_2 *
                                                    closest_coordinates_noise_layer_2;

      float4 noiseless_coord = rotate_scale(dv.coord - ov.out_position_field,
                                            closest_rotation_randomized,
                                            closest_scale_randomized,
                                            dv.invert_order_of_transformation);
      ov.out_r_sphere_coordinates =
          noiseless_coord +
          (dv.noise_fragmentation_non_zero ?
               rotate_noise(closest_coordinates_noise_vector,
                            dv.noise_fragmentation,
                            7.0f * float3(ov.out_index_field.x, ov.out_index_field.y, 0.0f)) :
               closest_coordinates_noise_vector);

      float r_sphere_randomized[4] = {r_sphere[0], r_sphere[1], r_sphere[2], r_sphere[3]};
      randomize_float_array(r_sphere_randomized,
                            r_sphere_min,
                            r_sphere_max,
                            r_sphere_index_list,
                            r_sphere_index_count,
                            5.0f * float3(ov.out_index_field.x, ov.out_index_field.y, 0.0f));

      float4 out_fields = calculate_out_fields_4d(
          dv.calculate_r_sphere_field,
          dv.calculate_r_gon_parameter_field,
          dv.calculate_max_unit_parameter_field,
          dv.normalize_r_gon_parameter,
          dv.integer_sides,
          dv.elliptical_corners,
          r_sphere_randomized[0],
          r_sphere_randomized[1],
          r_sphere_randomized[2],
          r_sphere_randomized[3],
          noiseless_coord +
              (dv.noise_fragmentation_non_zero ?
                   rotate_noise(closest_fields_noise_vector,
                                dv.noise_fragmentation,
                                7.0f * float3(ov.out_index_field.x, ov.out_index_field.y, 0.0f)) :
                   closest_fields_noise_vector));
      ov.out_r_sphere_field = out_fields.x;
      ov.r_gon_parameter_field = out_fields.y;
      ov.max_unit_parameter_field = out_fields.z;
      ov.segment_id_field = out_fields.w;
      break;
    }
    case SHD_RAIKO_SMOOTH_MINIMUM: {
      ov.out_r_sphere_field = 0.0f;
      ov.r_gon_parameter_field = 0.0f;
      ov.max_unit_parameter_field = 0.0f;
      ov.segment_id_field = 0.0f;
      ov.out_r_sphere_coordinates = float4(0.0f, 0.0f, 0.0f, 0.0f);
      ov.out_index_field = float4(0.0f, 0.0f, 0.0f, 0.0f);
      ov.out_position_field = float4(0.0f, 0.0f, 0.0f, 0.0f);
      bool first_iteration = true;

      float4 fields_noise_layer_1 = dv.calculate_fields_noise_1 ?
                                        raiko_noise_fbm_layer_1(
                                            dv,
                                            dv.transform_fields_noise ?
                                                rotate_scale(dv.coord,
                                                             translation_rotation,
                                                             scale,
                                                             dv.invert_order_of_transformation) :
                                                dv.coord) :
                                        float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 fields_noise_layer_2 = dv.calculate_fields_noise_2 ?
                                        raiko_noise_fbm_layer_2(
                                            dv,
                                            dv.transform_fields_noise ?
                                                rotate_scale(dv.coord,
                                                             translation_rotation,
                                                             scale,
                                                             dv.invert_order_of_transformation) :
                                                dv.coord) :
                                        float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 fields_noise_vector = dv.noise_fields_strength_1 * fields_noise_layer_1 +
                                   dv.noise_fields_strength_2 * fields_noise_layer_2;
      float4 coordinates_noise_layer_1 =
          dv.calculate_coordinates_noise_1 ?
              raiko_noise_fbm_layer_1(dv,
                                      dv.transform_coordinates_noise ?
                                          rotate_scale(dv.coord,
                                                       translation_rotation,
                                                       scale,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord) :
              fields_noise_layer_1;
      float4 coordinates_noise_layer_2 =
          dv.calculate_coordinates_noise_2 ?
              raiko_noise_fbm_layer_2(dv,
                                      dv.transform_coordinates_noise ?
                                          rotate_scale(dv.coord,
                                                       translation_rotation,
                                                       scale,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord) :
              fields_noise_layer_2;
      float4 coordinates_noise_vector = dv.noise_coordinates_strength_1 *
                                            coordinates_noise_layer_1 +
                                        dv.noise_coordinates_strength_2 *
                                            coordinates_noise_layer_2;

      for (float j = -scanning_window_size.y; j <= scanning_window_size.y; ++j) {
        for (float i = -scanning_window_size.x; i <= scanning_window_size.x; ++i) {
          float3 iteration_index = float3(i, j, 0.0f) +
                                   float3(initial_index.x, initial_index.y, 0.0f);
          float r_sphere_randomized[4] = {r_sphere[0], r_sphere[1], r_sphere[2], r_sphere[3]};
          randomize_float_array(r_sphere_randomized,
                                r_sphere_min,
                                r_sphere_max,
                                r_sphere_index_list,
                                r_sphere_index_count,
                                5.0f * iteration_index);
          float translation_rotation_randomized[7];
          for (int n = 0; n < 7; ++n) {
            translation_rotation_randomized[n] = translation_rotation[n];
          }
          randomize_float_array(translation_rotation_randomized,
                                translation_rotation_min,
                                translation_rotation_max,
                                translation_rotation_index_list,
                                translation_rotation_index_count,
                                8.0f * iteration_index);
          float scale_randomized[4] = {scale[0], scale[1], scale[2], scale[3]};
          randomize_scale(scale_randomized,
                          scale_randomness,
                          scale_index_list,
                          scale_index_count,
                          dv.uniform_scale_randomness,
                          9.0f * iteration_index);

          float4 iteration_position = initial_position + i * dv.grid_vector_1 +
                                      j * dv.grid_vector_2 +
                                      float4(translation_rotation_randomized[0],
                                             translation_rotation_randomized[1],
                                             translation_rotation_randomized[2],
                                             translation_rotation_randomized[3]);
          float4 noiseless_coord = rotate_scale(dv.coord - iteration_position,
                                                translation_rotation_randomized,
                                                scale_randomized,
                                                dv.invert_order_of_transformation);
          float4 iteration_coord = noiseless_coord + (dv.noise_fragmentation_non_zero ?
                                                          rotate_noise(fields_noise_vector,
                                                                       dv.noise_fragmentation,
                                                                       iteration_index) :
                                                          fields_noise_vector);
          float4 iteration_r_sphere_coordinates = noiseless_coord +
                                                  (dv.noise_fragmentation_non_zero ?
                                                       rotate_noise(coordinates_noise_vector,
                                                                    dv.noise_fragmentation,
                                                                    iteration_index) :
                                                       coordinates_noise_vector);

          float4 out_fields = calculate_out_fields_4d(dv.calculate_r_sphere_field,
                                                      dv.calculate_r_gon_parameter_field,
                                                      dv.calculate_max_unit_parameter_field,
                                                      dv.normalize_r_gon_parameter,
                                                      dv.integer_sides,
                                                      dv.elliptical_corners,
                                                      r_sphere_randomized[0],
                                                      r_sphere_randomized[1],
                                                      r_sphere_randomized[2],
                                                      r_sphere_randomized[3],
                                                      iteration_coord);
          float iteration_l_angle_bisector_4d = out_fields.x;
          float iteration_r_gon_parameter = out_fields.y;
          float iteration_max_unit_parameter = out_fields.z;
          float iteration_segment_id = out_fields.w;

          if (dv.smoothness_non_zero) {
            float interpolation_factor =
                first_iteration ?
                    1.0f :
                    smoothstep(0.0f,
                               1.0f,
                               0.5f + 0.5f *
                                          (ov.out_r_sphere_field - iteration_l_angle_bisector_4d) /
                                          dv.smoothness);
            float substraction_factor = dv.smoothness * interpolation_factor *
                                        (1.0f - interpolation_factor);
            ov.out_r_sphere_field = mix(ov.out_r_sphere_field,
                                        iteration_l_angle_bisector_4d,
                                        interpolation_factor) -
                                    substraction_factor;

            ov.r_gon_parameter_field = mix(
                ov.r_gon_parameter_field, iteration_r_gon_parameter, interpolation_factor);
            ov.max_unit_parameter_field = mix(
                ov.max_unit_parameter_field, iteration_max_unit_parameter, interpolation_factor);
            ov.segment_id_field = mix(
                ov.segment_id_field, iteration_segment_id, interpolation_factor);
            ov.out_index_field.x = mix(
                ov.out_index_field.x, iteration_index.x, interpolation_factor);
            ov.out_index_field.y = mix(
                ov.out_index_field.y, iteration_index.y, interpolation_factor);
            ov.out_position_field = mix(
                ov.out_position_field, iteration_position, interpolation_factor);
            ov.out_r_sphere_coordinates = mix(
                ov.out_r_sphere_coordinates, iteration_r_sphere_coordinates, interpolation_factor);
          }
          else if ((iteration_l_angle_bisector_4d < ov.out_r_sphere_field) || first_iteration) {
            ov.out_r_sphere_field = iteration_l_angle_bisector_4d;
            ov.r_gon_parameter_field = iteration_r_gon_parameter;
            ov.max_unit_parameter_field = iteration_max_unit_parameter;
            ov.segment_id_field = iteration_segment_id;
            ov.out_index_field.x = iteration_index.x;
            ov.out_index_field.y = iteration_index.y;
            ov.out_position_field = iteration_position;
            ov.out_r_sphere_coordinates = iteration_r_sphere_coordinates;
          }
          first_iteration = false;
        }
      }
      break;
    }
  }
  return ov;
}

OutVariables raiko_select_mode_3d(const DeterministicVariables &dv,
                                  float r_sphere[],
                                  float r_sphere_min[],
                                  float r_sphere_max[],
                                  int r_sphere_index_list[],
                                  int r_sphere_index_count,
                                  float translation_rotation[],
                                  float translation_rotation_min[],
                                  float translation_rotation_max[],
                                  int translation_rotation_index_list[],
                                  int translation_rotation_index_count,
                                  float scale[],
                                  float scale_randomness[],
                                  int scale_index_list[],
                                  int scale_index_count,
                                  float remap[],
                                  float remap_min[],
                                  float remap_max[],
                                  int remap_index_list[],
                                  int remap_index_count,
                                  float4 initial_index,
                                  float4 initial_position)
{
  float l_grid_vector1 = euclidean_norm(dv.grid_vector_1);
  float l_grid_vector2 = euclidean_norm(dv.grid_vector_2);
  float l_grid_vector3 = euclidean_norm(dv.grid_vector_3);
  float l_shortest_grid_vector = math::min(l_grid_vector1,
                                           math::min(l_grid_vector2, l_grid_vector3));
  float3 scanning_window_size = math::ceil(4.0f * dv.accuracy *
                                           float3(l_shortest_grid_vector / l_grid_vector1,
                                                  l_shortest_grid_vector / l_grid_vector2,
                                                  l_shortest_grid_vector / l_grid_vector3));
  OutVariables ov;
  switch (dv.mode) {
    case SHD_RAIKO_ADDITIVE: {
      ov.out_r_sphere_field = 0.0f;
      float4 fields_noise_layer_1 = dv.calculate_fields_noise_1 ?
                                        raiko_noise_fbm_layer_1(
                                            dv,
                                            dv.transform_fields_noise ?
                                                rotate_scale(dv.coord,
                                                             translation_rotation,
                                                             scale,
                                                             dv.invert_order_of_transformation) :
                                                dv.coord) :
                                        float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 fields_noise_layer_2 = dv.calculate_fields_noise_2 ?
                                        raiko_noise_fbm_layer_2(
                                            dv,
                                            dv.transform_fields_noise ?
                                                rotate_scale(dv.coord,
                                                             translation_rotation,
                                                             scale,
                                                             dv.invert_order_of_transformation) :
                                                dv.coord) :
                                        float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 fields_noise_vector = dv.noise_fields_strength_1 * fields_noise_layer_1 +
                                   dv.noise_fields_strength_2 * fields_noise_layer_2;

      for (float k = -scanning_window_size.z; k <= scanning_window_size.z; ++k) {
        for (float j = -scanning_window_size.y; j <= scanning_window_size.y; ++j) {
          for (float i = -scanning_window_size.x; i <= scanning_window_size.x; ++i) {
            float3 iteration_index = float3(i, j, k) +
                                     float3(initial_index.x, initial_index.y, initial_index.z);
            float r_sphere_randomized[4] = {r_sphere[0], r_sphere[1], r_sphere[2], r_sphere[3]};
            randomize_float_array(r_sphere_randomized,
                                  r_sphere_min,
                                  r_sphere_max,
                                  r_sphere_index_list,
                                  r_sphere_index_count,
                                  5.0f * iteration_index);
            float translation_rotation_randomized[7];
            for (int n = 0; n < 7; ++n) {
              translation_rotation_randomized[n] = translation_rotation[n];
            }
            randomize_float_array(translation_rotation_randomized,
                                  translation_rotation_min,
                                  translation_rotation_max,
                                  translation_rotation_index_list,
                                  translation_rotation_index_count,
                                  8.0f * iteration_index);
            float scale_randomized[4] = {scale[0], scale[1], scale[2], scale[3]};
            randomize_scale(scale_randomized,
                            scale_randomness,
                            scale_index_list,
                            scale_index_count,
                            dv.uniform_scale_randomness,
                            9.0f * iteration_index);

            float4 iteration_position = initial_position + i * dv.grid_vector_1 +
                                        j * dv.grid_vector_2 + k * dv.grid_vector_3 +
                                        float4(translation_rotation_randomized[0],
                                               translation_rotation_randomized[1],
                                               translation_rotation_randomized[2],
                                               translation_rotation_randomized[3]);
            float4 noiseless_coord = rotate_scale(dv.coord - iteration_position,
                                                  translation_rotation_randomized,
                                                  scale_randomized,
                                                  dv.invert_order_of_transformation);
            float4 iteration_coord = noiseless_coord + (dv.noise_fragmentation_non_zero ?
                                                            rotate_noise(fields_noise_vector,
                                                                         dv.noise_fragmentation,
                                                                         iteration_index) :
                                                            fields_noise_vector);

            ov.out_r_sphere_field += chained_elliptical_remap_select_steps(
                dv.step_count,
                remap,
                remap_min,
                remap_max,
                remap_index_list,
                remap_index_count,
                27.0f * iteration_index,
                calculate_l_angle_bisector_4d(dv.integer_sides,
                                              dv.elliptical_corners,
                                              r_sphere_randomized[0],
                                              r_sphere_randomized[1],
                                              r_sphere_randomized[2],
                                              r_sphere_randomized[3],
                                              iteration_coord));
          }
        }
      }
      break;
    }
    case SHD_RAIKO_CLOSEST: {
      ov.out_index_field = float4(0.0f, 0.0f, 0.0f, 0.0f);
      float min_distance = FLT_MAX;
      float closest_rotation_randomized[7];
      float closest_scale_randomized[4];
      float4 index_noise_layer_1 = dv.calculate_fields_noise_1 ?
                                       raiko_noise_fbm_layer_1(
                                           dv,
                                           dv.transform_fields_noise ?
                                               rotate_scale(dv.coord,
                                                            translation_rotation,
                                                            scale,
                                                            dv.invert_order_of_transformation) :
                                               dv.coord) :
                                       float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 index_noise_layer_2 = dv.calculate_fields_noise_2 ?
                                       raiko_noise_fbm_layer_2(
                                           dv,
                                           dv.transform_fields_noise ?
                                               rotate_scale(dv.coord,
                                                            translation_rotation,
                                                            scale,
                                                            dv.invert_order_of_transformation) :
                                               dv.coord) :
                                       float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 index_noise_vector = dv.noise_fields_strength_1 * index_noise_layer_1 +
                                  dv.noise_fields_strength_2 * index_noise_layer_2;

      for (float k = -scanning_window_size.z; k <= scanning_window_size.z; ++k) {
        for (float j = -scanning_window_size.y; j <= scanning_window_size.y; ++j) {
          for (float i = -scanning_window_size.x; i <= scanning_window_size.x; ++i) {
            float3 iteration_index = float3(i, j, k) +
                                     float3(initial_index.x, initial_index.y, initial_index.z);
            float translation_rotation_randomized[7];
            for (int n = 0; n < 7; ++n) {
              translation_rotation_randomized[n] = translation_rotation[n];
            }
            randomize_float_array(translation_rotation_randomized,
                                  translation_rotation_min,
                                  translation_rotation_max,
                                  translation_rotation_index_list,
                                  translation_rotation_index_count,
                                  8.0f * iteration_index);
            float scale_randomized[4] = {scale[0], scale[1], scale[2], scale[3]};
            randomize_scale(scale_randomized,
                            scale_randomness,
                            scale_index_list,
                            scale_index_count,
                            dv.uniform_scale_randomness,
                            9.0f * iteration_index);

            float4 iteration_position = initial_position + i * dv.grid_vector_1 +
                                        j * dv.grid_vector_2 + k * dv.grid_vector_3 +
                                        float4(translation_rotation_randomized[0],
                                               translation_rotation_randomized[1],
                                               translation_rotation_randomized[2],
                                               translation_rotation_randomized[3]);
            float4 noiseless_coord = rotate_scale(dv.coord - iteration_position,
                                                  translation_rotation_randomized,
                                                  scale_randomized,
                                                  dv.invert_order_of_transformation);
            float4 iteration_coord = noiseless_coord + (dv.noise_fragmentation_non_zero ?
                                                            rotate_noise(index_noise_vector,
                                                                         dv.noise_fragmentation,
                                                                         iteration_index) :
                                                            index_noise_vector);

            float l_iteration_coord = euclidean_norm(iteration_coord);
            if (l_iteration_coord < min_distance) {
              min_distance = l_iteration_coord;
              ov.out_index_field.x = iteration_index.x;
              ov.out_index_field.y = iteration_index.y;
              ov.out_index_field.z = iteration_index.z;
              ov.out_position_field = iteration_position;
              /* Translation data not needed for subsequent computations. */
              closest_rotation_randomized[4] = translation_rotation_randomized[4];
              closest_rotation_randomized[5] = translation_rotation_randomized[5];
              closest_rotation_randomized[6] = translation_rotation_randomized[6];
              closest_scale_randomized[0] = scale_randomized[0];
              closest_scale_randomized[1] = scale_randomized[1];
              closest_scale_randomized[2] = scale_randomized[2];
              closest_scale_randomized[3] = scale_randomized[3];
            }
          }
        }
      }

      float4 closest_fields_noise_layer_1 =
          dv.calculate_fields_noise_1 ?
              raiko_noise_fbm_layer_1(dv,
                                      dv.transform_fields_noise ?
                                          rotate_scale(dv.coord - ov.out_position_field,
                                                       closest_rotation_randomized,
                                                       closest_scale_randomized,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord - ov.out_position_field) :
              float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 closest_fields_noise_layer_2 =
          dv.calculate_fields_noise_2 ?
              raiko_noise_fbm_layer_2(dv,
                                      dv.transform_fields_noise ?
                                          rotate_scale(dv.coord - ov.out_position_field,
                                                       closest_rotation_randomized,
                                                       closest_scale_randomized,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord - ov.out_position_field) :
              float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 closest_fields_noise_vector = dv.noise_fields_strength_1 *
                                               closest_fields_noise_layer_1 +
                                           dv.noise_fields_strength_2 *
                                               closest_fields_noise_layer_2;
      float4 closest_coordinates_noise_layer_1 =
          dv.calculate_coordinates_noise_1 ?
              raiko_noise_fbm_layer_1(dv,
                                      dv.transform_coordinates_noise ?
                                          rotate_scale(dv.coord - ov.out_position_field,
                                                       closest_rotation_randomized,
                                                       closest_scale_randomized,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord - ov.out_position_field) :
              closest_fields_noise_layer_1;
      float4 closest_coordinates_noise_layer_2 =
          dv.calculate_coordinates_noise_2 ?
              raiko_noise_fbm_layer_2(dv,
                                      dv.transform_coordinates_noise ?
                                          rotate_scale(dv.coord - ov.out_position_field,
                                                       closest_rotation_randomized,
                                                       closest_scale_randomized,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord - ov.out_position_field) :
              closest_fields_noise_layer_2;
      float4 closest_coordinates_noise_vector = dv.noise_coordinates_strength_1 *
                                                    closest_coordinates_noise_layer_1 +
                                                dv.noise_coordinates_strength_2 *
                                                    closest_coordinates_noise_layer_2;

      float4 noiseless_coord = rotate_scale(dv.coord - ov.out_position_field,
                                            closest_rotation_randomized,
                                            closest_scale_randomized,
                                            dv.invert_order_of_transformation);
      ov.out_r_sphere_coordinates = noiseless_coord +
                                    (dv.noise_fragmentation_non_zero ?
                                         rotate_noise(closest_coordinates_noise_vector,
                                                      dv.noise_fragmentation,
                                                      7.0f * float3(ov.out_index_field.x,
                                                                    ov.out_index_field.y,
                                                                    ov.out_index_field.z)) :
                                         closest_coordinates_noise_vector);

      float r_sphere_randomized[4] = {r_sphere[0], r_sphere[1], r_sphere[2], r_sphere[3]};
      randomize_float_array(
          r_sphere_randomized,
          r_sphere_min,
          r_sphere_max,
          r_sphere_index_list,
          r_sphere_index_count,
          5.0f * float3(ov.out_index_field.x, ov.out_index_field.y, ov.out_index_field.z));

      float4 out_fields = calculate_out_fields_4d(
          dv.calculate_r_sphere_field,
          dv.calculate_r_gon_parameter_field,
          dv.calculate_max_unit_parameter_field,
          dv.normalize_r_gon_parameter,
          dv.integer_sides,
          dv.elliptical_corners,
          r_sphere_randomized[0],
          r_sphere_randomized[1],
          r_sphere_randomized[2],
          r_sphere_randomized[3],
          noiseless_coord + (dv.noise_fragmentation_non_zero ?
                                 rotate_noise(closest_fields_noise_vector,
                                              dv.noise_fragmentation,
                                              7.0f * float3(ov.out_index_field.x,
                                                            ov.out_index_field.y,
                                                            ov.out_index_field.z)) :
                                 closest_fields_noise_vector));
      ov.out_r_sphere_field = out_fields.x;
      ov.r_gon_parameter_field = out_fields.y;
      ov.max_unit_parameter_field = out_fields.z;
      ov.segment_id_field = out_fields.w;
      break;
    }
    case SHD_RAIKO_SMOOTH_MINIMUM: {
      ov.out_r_sphere_field = 0.0f;
      ov.r_gon_parameter_field = 0.0f;
      ov.max_unit_parameter_field = 0.0f;
      ov.segment_id_field = 0.0f;
      ov.out_r_sphere_coordinates = float4(0.0f, 0.0f, 0.0f, 0.0f);
      ov.out_index_field = float4(0.0f, 0.0f, 0.0f, 0.0f);
      ov.out_position_field = float4(0.0f, 0.0f, 0.0f, 0.0f);
      bool first_iteration = true;

      float4 fields_noise_layer_1 = dv.calculate_fields_noise_1 ?
                                        raiko_noise_fbm_layer_1(
                                            dv,
                                            dv.transform_fields_noise ?
                                                rotate_scale(dv.coord,
                                                             translation_rotation,
                                                             scale,
                                                             dv.invert_order_of_transformation) :
                                                dv.coord) :
                                        float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 fields_noise_layer_2 = dv.calculate_fields_noise_2 ?
                                        raiko_noise_fbm_layer_2(
                                            dv,
                                            dv.transform_fields_noise ?
                                                rotate_scale(dv.coord,
                                                             translation_rotation,
                                                             scale,
                                                             dv.invert_order_of_transformation) :
                                                dv.coord) :
                                        float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 fields_noise_vector = dv.noise_fields_strength_1 * fields_noise_layer_1 +
                                   dv.noise_fields_strength_2 * fields_noise_layer_2;
      float4 coordinates_noise_layer_1 =
          dv.calculate_coordinates_noise_1 ?
              raiko_noise_fbm_layer_1(dv,
                                      dv.transform_coordinates_noise ?
                                          rotate_scale(dv.coord,
                                                       translation_rotation,
                                                       scale,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord) :
              fields_noise_layer_1;
      float4 coordinates_noise_layer_2 =
          dv.calculate_coordinates_noise_2 ?
              raiko_noise_fbm_layer_2(dv,
                                      dv.transform_coordinates_noise ?
                                          rotate_scale(dv.coord,
                                                       translation_rotation,
                                                       scale,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord) :
              fields_noise_layer_2;
      float4 coordinates_noise_vector = dv.noise_coordinates_strength_1 *
                                            coordinates_noise_layer_1 +
                                        dv.noise_coordinates_strength_2 *
                                            coordinates_noise_layer_2;

      for (float k = -scanning_window_size.z; k <= scanning_window_size.z; ++k) {
        for (float j = -scanning_window_size.y; j <= scanning_window_size.y; ++j) {
          for (float i = -scanning_window_size.x; i <= scanning_window_size.x; ++i) {
            float3 iteration_index = float3(i, j, k) +
                                     float3(initial_index.x, initial_index.y, initial_index.z);
            float r_sphere_randomized[4] = {r_sphere[0], r_sphere[1], r_sphere[2], r_sphere[3]};
            randomize_float_array(r_sphere_randomized,
                                  r_sphere_min,
                                  r_sphere_max,
                                  r_sphere_index_list,
                                  r_sphere_index_count,
                                  5.0f * iteration_index);
            float translation_rotation_randomized[7];
            for (int n = 0; n < 7; ++n) {
              translation_rotation_randomized[n] = translation_rotation[n];
            }
            randomize_float_array(translation_rotation_randomized,
                                  translation_rotation_min,
                                  translation_rotation_max,
                                  translation_rotation_index_list,
                                  translation_rotation_index_count,
                                  8.0f * iteration_index);
            float scale_randomized[4] = {scale[0], scale[1], scale[2], scale[3]};
            randomize_scale(scale_randomized,
                            scale_randomness,
                            scale_index_list,
                            scale_index_count,
                            dv.uniform_scale_randomness,
                            9.0f * iteration_index);

            float4 iteration_position = initial_position + i * dv.grid_vector_1 +
                                        j * dv.grid_vector_2 + k * dv.grid_vector_3 +
                                        float4(translation_rotation_randomized[0],
                                               translation_rotation_randomized[1],
                                               translation_rotation_randomized[2],
                                               translation_rotation_randomized[3]);
            float4 noiseless_coord = rotate_scale(dv.coord - iteration_position,
                                                  translation_rotation_randomized,
                                                  scale_randomized,
                                                  dv.invert_order_of_transformation);
            float4 iteration_coord = noiseless_coord + (dv.noise_fragmentation_non_zero ?
                                                            rotate_noise(fields_noise_vector,
                                                                         dv.noise_fragmentation,
                                                                         iteration_index) :
                                                            fields_noise_vector);
            float4 iteration_r_sphere_coordinates = noiseless_coord +
                                                    (dv.noise_fragmentation_non_zero ?
                                                         rotate_noise(coordinates_noise_vector,
                                                                      dv.noise_fragmentation,
                                                                      iteration_index) :
                                                         coordinates_noise_vector);

            float4 out_fields = calculate_out_fields_4d(dv.calculate_r_sphere_field,
                                                        dv.calculate_r_gon_parameter_field,
                                                        dv.calculate_max_unit_parameter_field,
                                                        dv.normalize_r_gon_parameter,
                                                        dv.integer_sides,
                                                        dv.elliptical_corners,
                                                        r_sphere_randomized[0],
                                                        r_sphere_randomized[1],
                                                        r_sphere_randomized[2],
                                                        r_sphere_randomized[3],
                                                        iteration_coord);
            float iteration_l_angle_bisector_4d = out_fields.x;
            float iteration_r_gon_parameter = out_fields.y;
            float iteration_max_unit_parameter = out_fields.z;
            float iteration_segment_id = out_fields.w;

            if (dv.smoothness_non_zero) {
              float interpolation_factor =
                  first_iteration ?
                      1.0f :
                      smoothstep(
                          0.0f,
                          1.0f,
                          0.5f + 0.5f * (ov.out_r_sphere_field - iteration_l_angle_bisector_4d) /
                                     dv.smoothness);
              float substraction_factor = dv.smoothness * interpolation_factor *
                                          (1.0f - interpolation_factor);
              ov.out_r_sphere_field = mix(ov.out_r_sphere_field,
                                          iteration_l_angle_bisector_4d,
                                          interpolation_factor) -
                                      substraction_factor;

              ov.r_gon_parameter_field = mix(
                  ov.r_gon_parameter_field, iteration_r_gon_parameter, interpolation_factor);
              ov.max_unit_parameter_field = mix(
                  ov.max_unit_parameter_field, iteration_max_unit_parameter, interpolation_factor);
              ov.segment_id_field = mix(
                  ov.segment_id_field, iteration_segment_id, interpolation_factor);
              ov.out_index_field.x = mix(
                  ov.out_index_field.x, iteration_index.x, interpolation_factor);
              ov.out_index_field.y = mix(
                  ov.out_index_field.y, iteration_index.y, interpolation_factor);
              ov.out_index_field.z = mix(
                  ov.out_index_field.z, iteration_index.z, interpolation_factor);
              ov.out_position_field = mix(
                  ov.out_position_field, iteration_position, interpolation_factor);
              ov.out_r_sphere_coordinates = mix(ov.out_r_sphere_coordinates,
                                                iteration_r_sphere_coordinates,
                                                interpolation_factor);
            }
            else if ((iteration_l_angle_bisector_4d < ov.out_r_sphere_field) || first_iteration) {
              ov.out_r_sphere_field = iteration_l_angle_bisector_4d;
              ov.r_gon_parameter_field = iteration_r_gon_parameter;
              ov.max_unit_parameter_field = iteration_max_unit_parameter;
              ov.segment_id_field = iteration_segment_id;
              ov.out_index_field.x = iteration_index.x;
              ov.out_index_field.y = iteration_index.y;
              ov.out_index_field.z = iteration_index.z;
              ov.out_position_field = iteration_position;
              ov.out_r_sphere_coordinates = iteration_r_sphere_coordinates;
            }
            first_iteration = false;
          }
        }
      }
      break;
    }
  }
  return ov;
}

OutVariables raiko_select_mode_4d(const DeterministicVariables &dv,
                                  float r_sphere[],
                                  float r_sphere_min[],
                                  float r_sphere_max[],
                                  int r_sphere_index_list[],
                                  int r_sphere_index_count,
                                  float translation_rotation[],
                                  float translation_rotation_min[],
                                  float translation_rotation_max[],
                                  int translation_rotation_index_list[],
                                  int translation_rotation_index_count,
                                  float scale[],
                                  float scale_randomness[],
                                  int scale_index_list[],
                                  int scale_index_count,
                                  float remap[],
                                  float remap_min[],
                                  float remap_max[],
                                  int remap_index_list[],
                                  int remap_index_count,
                                  float4 initial_index,
                                  float4 initial_position)
{
  float l_grid_vector1 = euclidean_norm(dv.grid_vector_1);
  float l_grid_vector2 = euclidean_norm(dv.grid_vector_2);
  float l_grid_vector3 = euclidean_norm(dv.grid_vector_3);
  float l_grid_vector4 = euclidean_norm(dv.grid_vector_4);
  float l_shortest_grid_vector = math::min(
      l_grid_vector1, math::min(l_grid_vector2, math::min(l_grid_vector3, l_grid_vector4)));
  float4 scanning_window_size = math::ceil(4.0f * dv.accuracy *
                                           float4(l_shortest_grid_vector / l_grid_vector1,
                                                  l_shortest_grid_vector / l_grid_vector2,
                                                  l_shortest_grid_vector / l_grid_vector3,
                                                  l_shortest_grid_vector / l_grid_vector4));
  OutVariables ov;
  switch (dv.mode) {
    case SHD_RAIKO_ADDITIVE: {
      ov.out_r_sphere_field = 0.0f;
      float4 fields_noise_layer_1 = dv.calculate_fields_noise_1 ?
                                        raiko_noise_fbm_layer_1(
                                            dv,
                                            dv.transform_fields_noise ?
                                                rotate_scale(dv.coord,
                                                             translation_rotation,
                                                             scale,
                                                             dv.invert_order_of_transformation) :
                                                dv.coord) :
                                        float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 fields_noise_layer_2 = dv.calculate_fields_noise_2 ?
                                        raiko_noise_fbm_layer_2(
                                            dv,
                                            dv.transform_fields_noise ?
                                                rotate_scale(dv.coord,
                                                             translation_rotation,
                                                             scale,
                                                             dv.invert_order_of_transformation) :
                                                dv.coord) :
                                        float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 fields_noise_vector = dv.noise_fields_strength_1 * fields_noise_layer_1 +
                                   dv.noise_fields_strength_2 * fields_noise_layer_2;

      for (float l = -scanning_window_size.w; l <= scanning_window_size.w; ++l) {
        for (float k = -scanning_window_size.z; k <= scanning_window_size.z; ++k) {
          for (float j = -scanning_window_size.y; j <= scanning_window_size.y; ++j) {
            for (float i = -scanning_window_size.x; i <= scanning_window_size.x; ++i) {
              float r_sphere_randomized[4] = {r_sphere[0], r_sphere[1], r_sphere[2], r_sphere[3]};
              float4 iteration_index = float4(i, j, k, l) + initial_index;
              randomize_float_array(r_sphere_randomized,
                                    r_sphere_min,
                                    r_sphere_max,
                                    r_sphere_index_list,
                                    r_sphere_index_count,
                                    5.0f * iteration_index);
              float translation_rotation_randomized[7];
              for (int n = 0; n < 7; ++n) {
                translation_rotation_randomized[n] = translation_rotation[n];
              }
              randomize_float_array(translation_rotation_randomized,
                                    translation_rotation_min,
                                    translation_rotation_max,
                                    translation_rotation_index_list,
                                    translation_rotation_index_count,
                                    8.0f * iteration_index);
              float scale_randomized[4] = {scale[0], scale[1], scale[2], scale[3]};
              randomize_scale(scale_randomized,
                              scale_randomness,
                              scale_index_list,
                              scale_index_count,
                              dv.uniform_scale_randomness,
                              9.0f * iteration_index);

              float4 iteration_position = initial_position + i * dv.grid_vector_1 +
                                          j * dv.grid_vector_2 + k * dv.grid_vector_3 +
                                          l * dv.grid_vector_4 +
                                          float4(translation_rotation_randomized[0],
                                                 translation_rotation_randomized[1],
                                                 translation_rotation_randomized[2],
                                                 translation_rotation_randomized[3]);
              float4 noiseless_coord = rotate_scale(dv.coord - iteration_position,
                                                    translation_rotation_randomized,
                                                    scale_randomized,
                                                    dv.invert_order_of_transformation);
              float4 iteration_coord = noiseless_coord + (dv.noise_fragmentation_non_zero ?
                                                              rotate_noise(fields_noise_vector,
                                                                           dv.noise_fragmentation,
                                                                           iteration_index) :
                                                              fields_noise_vector);

              ov.out_r_sphere_field += chained_elliptical_remap_select_steps(
                  dv.step_count,
                  remap,
                  remap_min,
                  remap_max,
                  remap_index_list,
                  remap_index_count,
                  27.0f * iteration_index,
                  calculate_l_angle_bisector_4d(dv.integer_sides,
                                                dv.elliptical_corners,
                                                r_sphere_randomized[0],
                                                r_sphere_randomized[1],
                                                r_sphere_randomized[2],
                                                r_sphere_randomized[3],
                                                iteration_coord));
            }
          }
        }
      }
      break;
    }
    case SHD_RAIKO_CLOSEST: {
      ov.out_index_field = float4(0.0f, 0.0f, 0.0f, 0.0f);
      float min_distance = FLT_MAX;
      float closest_rotation_randomized[7];
      float closest_scale_randomized[4];
      float4 index_noise_layer_1 = dv.calculate_fields_noise_1 ?
                                       raiko_noise_fbm_layer_1(
                                           dv,
                                           dv.transform_fields_noise ?
                                               rotate_scale(dv.coord,
                                                            translation_rotation,
                                                            scale,
                                                            dv.invert_order_of_transformation) :
                                               dv.coord) :
                                       float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 index_noise_layer_2 = dv.calculate_fields_noise_2 ?
                                       raiko_noise_fbm_layer_2(
                                           dv,
                                           dv.transform_fields_noise ?
                                               rotate_scale(dv.coord,
                                                            translation_rotation,
                                                            scale,
                                                            dv.invert_order_of_transformation) :
                                               dv.coord) :
                                       float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 index_noise_vector = dv.noise_fields_strength_1 * index_noise_layer_1 +
                                  dv.noise_fields_strength_2 * index_noise_layer_2;

      for (float l = -scanning_window_size.w; l <= scanning_window_size.w; ++l) {
        for (float k = -scanning_window_size.z; k <= scanning_window_size.z; ++k) {
          for (float j = -scanning_window_size.y; j <= scanning_window_size.y; ++j) {
            for (float i = -scanning_window_size.x; i <= scanning_window_size.x; ++i) {
              float4 iteration_index = float4(i, j, k, l) + initial_index;
              float translation_rotation_randomized[7];
              for (int n = 0; n < 7; ++n) {
                translation_rotation_randomized[n] = translation_rotation[n];
              }
              randomize_float_array(translation_rotation_randomized,
                                    translation_rotation_min,
                                    translation_rotation_max,
                                    translation_rotation_index_list,
                                    translation_rotation_index_count,
                                    8.0f * iteration_index);
              float scale_randomized[4] = {scale[0], scale[1], scale[2], scale[3]};
              randomize_scale(scale_randomized,
                              scale_randomness,
                              scale_index_list,
                              scale_index_count,
                              dv.uniform_scale_randomness,
                              9.0f * iteration_index);

              float4 iteration_position = initial_position + i * dv.grid_vector_1 +
                                          j * dv.grid_vector_2 + k * dv.grid_vector_3 +
                                          l * dv.grid_vector_4 +
                                          float4(translation_rotation_randomized[0],
                                                 translation_rotation_randomized[1],
                                                 translation_rotation_randomized[2],
                                                 translation_rotation_randomized[3]);
              float4 noiseless_coord = rotate_scale(dv.coord - iteration_position,
                                                    translation_rotation_randomized,
                                                    scale_randomized,
                                                    dv.invert_order_of_transformation);
              float4 iteration_coord = noiseless_coord + (dv.noise_fragmentation_non_zero ?
                                                              rotate_noise(index_noise_vector,
                                                                           dv.noise_fragmentation,
                                                                           iteration_index) :
                                                              index_noise_vector);

              float l_iteration_coord = euclidean_norm(iteration_coord);
              if (l_iteration_coord < min_distance) {
                min_distance = l_iteration_coord;
                ov.out_index_field = iteration_index;
                ov.out_position_field = iteration_position;
                /* Translation data not needed for subsequent computations. */
                closest_rotation_randomized[4] = translation_rotation_randomized[4];
                closest_rotation_randomized[5] = translation_rotation_randomized[5];
                closest_rotation_randomized[6] = translation_rotation_randomized[6];
                closest_scale_randomized[0] = scale_randomized[0];
                closest_scale_randomized[1] = scale_randomized[1];
                closest_scale_randomized[2] = scale_randomized[2];
                closest_scale_randomized[3] = scale_randomized[3];
              }
            }
          }
        }
      }

      float4 closest_fields_noise_layer_1 =
          dv.calculate_fields_noise_1 ?
              raiko_noise_fbm_layer_1(dv,
                                      dv.transform_fields_noise ?
                                          rotate_scale(dv.coord - ov.out_position_field,
                                                       closest_rotation_randomized,
                                                       closest_scale_randomized,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord - ov.out_position_field) :
              float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 closest_fields_noise_layer_2 =
          dv.calculate_fields_noise_2 ?
              raiko_noise_fbm_layer_2(dv,
                                      dv.transform_fields_noise ?
                                          rotate_scale(dv.coord - ov.out_position_field,
                                                       closest_rotation_randomized,
                                                       closest_scale_randomized,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord - ov.out_position_field) :
              float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 closest_fields_noise_vector = dv.noise_fields_strength_1 *
                                               closest_fields_noise_layer_1 +
                                           dv.noise_fields_strength_2 *
                                               closest_fields_noise_layer_2;
      float4 closest_coordinates_noise_layer_1 =
          dv.calculate_coordinates_noise_1 ?
              raiko_noise_fbm_layer_1(dv,
                                      dv.transform_coordinates_noise ?
                                          rotate_scale(dv.coord - ov.out_position_field,
                                                       closest_rotation_randomized,
                                                       closest_scale_randomized,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord - ov.out_position_field) :
              closest_fields_noise_layer_1;
      float4 closest_coordinates_noise_layer_2 =
          dv.calculate_coordinates_noise_2 ?
              raiko_noise_fbm_layer_2(dv,
                                      dv.transform_coordinates_noise ?
                                          rotate_scale(dv.coord - ov.out_position_field,
                                                       closest_rotation_randomized,
                                                       closest_scale_randomized,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord - ov.out_position_field) :
              closest_fields_noise_layer_2;
      float4 closest_coordinates_noise_vector = dv.noise_coordinates_strength_1 *
                                                    closest_coordinates_noise_layer_1 +
                                                dv.noise_coordinates_strength_2 *
                                                    closest_coordinates_noise_layer_2;

      float4 noiseless_coord = rotate_scale(dv.coord - ov.out_position_field,
                                            closest_rotation_randomized,
                                            closest_scale_randomized,
                                            dv.invert_order_of_transformation);
      ov.out_r_sphere_coordinates = noiseless_coord +
                                    (dv.noise_fragmentation_non_zero ?
                                         rotate_noise(closest_coordinates_noise_vector,
                                                      dv.noise_fragmentation,
                                                      7.0f * ov.out_index_field) :
                                         closest_coordinates_noise_vector);

      float r_sphere_randomized[4] = {r_sphere[0], r_sphere[1], r_sphere[2], r_sphere[3]};
      randomize_float_array(r_sphere_randomized,
                            r_sphere_min,
                            r_sphere_max,
                            r_sphere_index_list,
                            r_sphere_index_count,
                            5.0f * ov.out_index_field);

      float4 out_fields = calculate_out_fields_4d(
          dv.calculate_r_sphere_field,
          dv.calculate_r_gon_parameter_field,
          dv.calculate_max_unit_parameter_field,
          dv.normalize_r_gon_parameter,
          dv.integer_sides,
          dv.elliptical_corners,
          r_sphere_randomized[0],
          r_sphere_randomized[1],
          r_sphere_randomized[2],
          r_sphere_randomized[3],
          noiseless_coord + (dv.noise_fragmentation_non_zero ?
                                 rotate_noise(closest_fields_noise_vector,
                                              dv.noise_fragmentation,
                                              7.0f * ov.out_index_field) :
                                 closest_fields_noise_vector));
      ov.out_r_sphere_field = out_fields.x;
      ov.r_gon_parameter_field = out_fields.y;
      ov.max_unit_parameter_field = out_fields.z;
      ov.segment_id_field = out_fields.w;
      break;
    }
    case SHD_RAIKO_SMOOTH_MINIMUM: {
      ov.out_r_sphere_field = 0.0f;
      ov.r_gon_parameter_field = 0.0f;
      ov.max_unit_parameter_field = 0.0f;
      ov.segment_id_field = 0.0f;
      ov.out_r_sphere_coordinates = float4(0.0f, 0.0f, 0.0f, 0.0f);
      ov.out_index_field = float4(0.0f, 0.0f, 0.0f, 0.0f);
      ov.out_position_field = float4(0.0f, 0.0f, 0.0f, 0.0f);
      bool first_iteration = true;

      float4 fields_noise_layer_1 = dv.calculate_fields_noise_1 ?
                                        raiko_noise_fbm_layer_1(
                                            dv,
                                            dv.transform_fields_noise ?
                                                rotate_scale(dv.coord,
                                                             translation_rotation,
                                                             scale,
                                                             dv.invert_order_of_transformation) :
                                                dv.coord) :
                                        float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 fields_noise_layer_2 = dv.calculate_fields_noise_2 ?
                                        raiko_noise_fbm_layer_2(
                                            dv,
                                            dv.transform_fields_noise ?
                                                rotate_scale(dv.coord,
                                                             translation_rotation,
                                                             scale,
                                                             dv.invert_order_of_transformation) :
                                                dv.coord) :
                                        float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 fields_noise_vector = dv.noise_fields_strength_1 * fields_noise_layer_1 +
                                   dv.noise_fields_strength_2 * fields_noise_layer_2;
      float4 coordinates_noise_layer_1 =
          dv.calculate_coordinates_noise_1 ?
              raiko_noise_fbm_layer_1(dv,
                                      dv.transform_coordinates_noise ?
                                          rotate_scale(dv.coord,
                                                       translation_rotation,
                                                       scale,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord) :
              fields_noise_layer_1;
      float4 coordinates_noise_layer_2 =
          dv.calculate_coordinates_noise_2 ?
              raiko_noise_fbm_layer_2(dv,
                                      dv.transform_coordinates_noise ?
                                          rotate_scale(dv.coord,
                                                       translation_rotation,
                                                       scale,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord) :
              fields_noise_layer_2;
      float4 coordinates_noise_vector = dv.noise_coordinates_strength_1 *
                                            coordinates_noise_layer_1 +
                                        dv.noise_coordinates_strength_2 *
                                            coordinates_noise_layer_2;

      for (float l = -scanning_window_size.w; l <= scanning_window_size.w; ++l) {
        for (float k = -scanning_window_size.z; k <= scanning_window_size.z; ++k) {
          for (float j = -scanning_window_size.y; j <= scanning_window_size.y; ++j) {
            for (float i = -scanning_window_size.x; i <= scanning_window_size.x; ++i) {
              float4 iteration_index = float4(i, j, k, l) + initial_index;
              float r_sphere_randomized[4] = {r_sphere[0], r_sphere[1], r_sphere[2], r_sphere[3]};
              randomize_float_array(r_sphere_randomized,
                                    r_sphere_min,
                                    r_sphere_max,
                                    r_sphere_index_list,
                                    r_sphere_index_count,
                                    5.0f * iteration_index);
              float translation_rotation_randomized[7];
              for (int n = 0; n < 7; ++n) {
                translation_rotation_randomized[n] = translation_rotation[n];
              }
              randomize_float_array(translation_rotation_randomized,
                                    translation_rotation_min,
                                    translation_rotation_max,
                                    translation_rotation_index_list,
                                    translation_rotation_index_count,
                                    8.0f * iteration_index);
              float scale_randomized[4] = {scale[0], scale[1], scale[2], scale[3]};
              randomize_scale(scale_randomized,
                              scale_randomness,
                              scale_index_list,
                              scale_index_count,
                              dv.uniform_scale_randomness,
                              9.0f * iteration_index);

              float4 iteration_position = initial_position + i * dv.grid_vector_1 +
                                          j * dv.grid_vector_2 + k * dv.grid_vector_3 +
                                          l * dv.grid_vector_4 +
                                          float4(translation_rotation_randomized[0],
                                                 translation_rotation_randomized[1],
                                                 translation_rotation_randomized[2],
                                                 translation_rotation_randomized[3]);
              float4 noiseless_coord = rotate_scale(dv.coord - iteration_position,
                                                    translation_rotation_randomized,
                                                    scale_randomized,
                                                    dv.invert_order_of_transformation);
              float4 iteration_coord = noiseless_coord + (dv.noise_fragmentation_non_zero ?
                                                              rotate_noise(fields_noise_vector,
                                                                           dv.noise_fragmentation,
                                                                           iteration_index) :
                                                              fields_noise_vector);
              float4 iteration_r_sphere_coordinates = noiseless_coord +
                                                      (dv.noise_fragmentation_non_zero ?
                                                           rotate_noise(coordinates_noise_vector,
                                                                        dv.noise_fragmentation,
                                                                        iteration_index) :
                                                           coordinates_noise_vector);

              float4 out_fields = calculate_out_fields_4d(dv.calculate_r_sphere_field,
                                                          dv.calculate_r_gon_parameter_field,
                                                          dv.calculate_max_unit_parameter_field,
                                                          dv.normalize_r_gon_parameter,
                                                          dv.integer_sides,
                                                          dv.elliptical_corners,
                                                          r_sphere_randomized[0],
                                                          r_sphere_randomized[1],
                                                          r_sphere_randomized[2],
                                                          r_sphere_randomized[3],
                                                          iteration_coord);
              float iteration_l_angle_bisector_4d = out_fields.x;
              float iteration_r_gon_parameter = out_fields.y;
              float iteration_max_unit_parameter = out_fields.z;
              float iteration_segment_id = out_fields.w;

              if (dv.smoothness_non_zero) {
                float interpolation_factor =
                    first_iteration ?
                        1.0f :
                        smoothstep(
                            0.0f,
                            1.0f,
                            0.5f + 0.5f * (ov.out_r_sphere_field - iteration_l_angle_bisector_4d) /
                                       dv.smoothness);
                float substraction_factor = dv.smoothness * interpolation_factor *
                                            (1.0f - interpolation_factor);
                ov.out_r_sphere_field = mix(ov.out_r_sphere_field,
                                            iteration_l_angle_bisector_4d,
                                            interpolation_factor) -
                                        substraction_factor;

                ov.r_gon_parameter_field = mix(
                    ov.r_gon_parameter_field, iteration_r_gon_parameter, interpolation_factor);
                ov.max_unit_parameter_field = mix(ov.max_unit_parameter_field,
                                                  iteration_max_unit_parameter,
                                                  interpolation_factor);
                ov.segment_id_field = mix(
                    ov.segment_id_field, iteration_segment_id, interpolation_factor);
                ov.out_index_field = mix(
                    ov.out_index_field, iteration_index, interpolation_factor);
                ov.out_position_field = mix(
                    ov.out_position_field, iteration_position, interpolation_factor);
                ov.out_r_sphere_coordinates = mix(ov.out_r_sphere_coordinates,
                                                  iteration_r_sphere_coordinates,
                                                  interpolation_factor);
              }
              else if ((iteration_l_angle_bisector_4d < ov.out_r_sphere_field) || first_iteration)
              {
                ov.out_r_sphere_field = iteration_l_angle_bisector_4d;
                ov.r_gon_parameter_field = iteration_r_gon_parameter;
                ov.max_unit_parameter_field = iteration_max_unit_parameter;
                ov.segment_id_field = iteration_segment_id;
                ov.out_index_field = iteration_index;
                ov.out_position_field = iteration_position;
                ov.out_r_sphere_coordinates = iteration_r_sphere_coordinates;
              }
              first_iteration = false;
            }
          }
        }
      }
      break;
    }
  }
  return ov;
}

OutVariables raiko_select_grid_dimensions(DeterministicVariables dv,
                                          float r_sphere[],
                                          float r_sphere_min[],
                                          float r_sphere_max[],
                                          int r_sphere_index_list[],
                                          int r_sphere_index_count,
                                          float translation_rotation[],
                                          float translation_rotation_min[],
                                          float translation_rotation_max[],
                                          int translation_rotation_index_list[],
                                          int translation_rotation_index_count,
                                          float scale[],
                                          float scale_randomness[],
                                          int scale_index_list[],
                                          int scale_index_count,
                                          float remap[],
                                          float remap_min[],
                                          float remap_max[],
                                          int remap_index_list[],
                                          int remap_index_count)
{
  OutVariables ov;
  switch (dv.grid_dimensions) {
    case 0: {
      ov = raiko_select_mode_0d(dv,
                                r_sphere,
                                r_sphere_min,
                                r_sphere_max,
                                r_sphere_index_list,
                                r_sphere_index_count,
                                translation_rotation,
                                translation_rotation_min,
                                translation_rotation_max,
                                translation_rotation_index_list,
                                translation_rotation_index_count,
                                scale,
                                scale_randomness,
                                scale_index_list,
                                scale_index_count,
                                remap,
                                remap_min,
                                remap_max,
                                remap_index_list,
                                remap_index_count);
      break;
    }
    case 1: {
      if (dv.grid_vector_1.x != 0.0f) {
        dv.grid_vector_1 = float4(dv.grid_vector_1.x, 0.0f, 0.0f, 0.0f);

        float4 initial_index = math::round(
            float4(dv.coord.x / dv.grid_vector_1.x, 0.0f, 0.0f, 0.0f));
        float4 initial_position = initial_index.x * dv.grid_vector_1;

        ov = raiko_select_mode_1d(dv,
                                  r_sphere,
                                  r_sphere_min,
                                  r_sphere_max,
                                  r_sphere_index_list,
                                  r_sphere_index_count,
                                  translation_rotation,
                                  translation_rotation_min,
                                  translation_rotation_max,
                                  translation_rotation_index_list,
                                  translation_rotation_index_count,
                                  scale,
                                  scale_randomness,
                                  scale_index_list,
                                  scale_index_count,
                                  remap,
                                  remap_min,
                                  remap_max,
                                  remap_index_list,
                                  remap_index_count,
                                  initial_index,
                                  initial_position);
      }
      else {
        ov = raiko_select_mode_0d(dv,
                                  r_sphere,
                                  r_sphere_min,
                                  r_sphere_max,
                                  r_sphere_index_list,
                                  r_sphere_index_count,
                                  translation_rotation,
                                  translation_rotation_min,
                                  translation_rotation_max,
                                  translation_rotation_index_list,
                                  translation_rotation_index_count,
                                  scale,
                                  scale_randomness,
                                  scale_index_list,
                                  scale_index_count,
                                  remap,
                                  remap_min,
                                  remap_max,
                                  remap_index_list,
                                  remap_index_count);
      }
      break;
    }
    case 2: {
      float M_det = dv.grid_vector_1.x * dv.grid_vector_2.y -
                    dv.grid_vector_2.x * dv.grid_vector_1.y;
      if (M_det != 0.0f) {
        dv.grid_vector_1 = float4(dv.grid_vector_1.x, dv.grid_vector_1.y, 0.0f, 0.0f);
        dv.grid_vector_2 = float4(dv.grid_vector_2.x, dv.grid_vector_2.y, 0.0f, 0.0f);

        float2 initial_index_xy = math::round(
            fc_linear_system_solve_non_singular_2x2(float2(dv.grid_vector_1.x, dv.grid_vector_1.y),
                                                    float2(dv.grid_vector_2.x, dv.grid_vector_2.y),
                                                    float2(dv.coord.x, dv.coord.y),
                                                    M_det));
        float4 initial_index = float4(initial_index_xy.x, initial_index_xy.y, 0.0f, 0.0f);
        float4 initial_position = initial_index.x * dv.grid_vector_1 +
                                  initial_index.y * dv.grid_vector_2;

        ov = raiko_select_mode_2d(dv,
                                  r_sphere,
                                  r_sphere_min,
                                  r_sphere_max,
                                  r_sphere_index_list,
                                  r_sphere_index_count,
                                  translation_rotation,
                                  translation_rotation_min,
                                  translation_rotation_max,
                                  translation_rotation_index_list,
                                  translation_rotation_index_count,
                                  scale,
                                  scale_randomness,
                                  scale_index_list,
                                  scale_index_count,
                                  remap,
                                  remap_min,
                                  remap_max,
                                  remap_index_list,
                                  remap_index_count,
                                  initial_index,
                                  initial_position);
      }
      else {
        ov = raiko_select_mode_0d(dv,
                                  r_sphere,
                                  r_sphere_min,
                                  r_sphere_max,
                                  r_sphere_index_list,
                                  r_sphere_index_count,
                                  translation_rotation,
                                  translation_rotation_min,
                                  translation_rotation_max,
                                  translation_rotation_index_list,
                                  translation_rotation_index_count,
                                  scale,
                                  scale_randomness,
                                  scale_index_list,
                                  scale_index_count,
                                  remap,
                                  remap_min,
                                  remap_max,
                                  remap_index_list,
                                  remap_index_count);
      }
      break;
    }
    case 3: {
      Fcc_3x3 A_fcc = calculate_Fcc_3x3(
          float3(dv.grid_vector_1.x, dv.grid_vector_1.y, dv.grid_vector_1.z),
          float3(dv.grid_vector_2.x, dv.grid_vector_2.y, dv.grid_vector_2.z),
          float3(dv.grid_vector_3.x, dv.grid_vector_3.y, dv.grid_vector_3.z));
      if (A_fcc.M_det != 0.0f) {
        dv.grid_vector_1 = float4(
            dv.grid_vector_1.x, dv.grid_vector_1.y, dv.grid_vector_1.z, 0.0f);
        dv.grid_vector_2 = float4(
            dv.grid_vector_2.x, dv.grid_vector_2.y, dv.grid_vector_2.z, 0.0f);
        dv.grid_vector_3 = float4(
            dv.grid_vector_3.x, dv.grid_vector_3.y, dv.grid_vector_3.z, 0.0f);

        float3 initial_index_xyz = math::round(fc_linear_system_solve_non_singular_3x3(
            float3(dv.grid_vector_1.x, dv.grid_vector_1.y, dv.grid_vector_1.z),
            float3(dv.grid_vector_2.x, dv.grid_vector_2.y, dv.grid_vector_2.z),
            float3(dv.grid_vector_3.x, dv.grid_vector_3.y, dv.grid_vector_3.z),
            float3(dv.coord.x, dv.coord.y, dv.coord.z),
            A_fcc));
        float4 initial_index = float4(
            initial_index_xyz.x, initial_index_xyz.y, initial_index_xyz.z, 0.0f);
        float4 initial_position = initial_index.x * dv.grid_vector_1 +
                                  initial_index.y * dv.grid_vector_2 +
                                  initial_index.z * dv.grid_vector_3;

        ov = raiko_select_mode_3d(dv,
                                  r_sphere,
                                  r_sphere_min,
                                  r_sphere_max,
                                  r_sphere_index_list,
                                  r_sphere_index_count,
                                  translation_rotation,
                                  translation_rotation_min,
                                  translation_rotation_max,
                                  translation_rotation_index_list,
                                  translation_rotation_index_count,
                                  scale,
                                  scale_randomness,
                                  scale_index_list,
                                  scale_index_count,
                                  remap,
                                  remap_min,
                                  remap_max,
                                  remap_index_list,
                                  remap_index_count,
                                  initial_index,
                                  initial_position);
      }
      else {
        ov = raiko_select_mode_0d(dv,
                                  r_sphere,
                                  r_sphere_min,
                                  r_sphere_max,
                                  r_sphere_index_list,
                                  r_sphere_index_count,
                                  translation_rotation,
                                  translation_rotation_min,
                                  translation_rotation_max,
                                  translation_rotation_index_list,
                                  translation_rotation_index_count,
                                  scale,
                                  scale_randomness,
                                  scale_index_list,
                                  scale_index_count,
                                  remap,
                                  remap_min,
                                  remap_max,
                                  remap_index_list,
                                  remap_index_count);
      }
      break;
    }
    case 4: {
      Fcc_4x4 A_fcc = calculate_Fcc_4x4(
          dv.grid_vector_1, dv.grid_vector_2, dv.grid_vector_3, dv.grid_vector_4);
      if (A_fcc.M_det != 0.0f) {

        float4 initial_index = math::round(
            fc_linear_system_solve_non_singular_4x4(dv.grid_vector_1,
                                                    dv.grid_vector_2,
                                                    dv.grid_vector_3,
                                                    dv.grid_vector_4,
                                                    dv.coord,
                                                    A_fcc));
        float4 initial_position = initial_index.x * dv.grid_vector_1 +
                                  initial_index.y * dv.grid_vector_2 +
                                  initial_index.z * dv.grid_vector_3 +
                                  initial_index.w * dv.grid_vector_4;

        ov = raiko_select_mode_4d(dv,
                                  r_sphere,
                                  r_sphere_min,
                                  r_sphere_max,
                                  r_sphere_index_list,
                                  r_sphere_index_count,
                                  translation_rotation,
                                  translation_rotation_min,
                                  translation_rotation_max,
                                  translation_rotation_index_list,
                                  translation_rotation_index_count,
                                  scale,
                                  scale_randomness,
                                  scale_index_list,
                                  scale_index_count,
                                  remap,
                                  remap_min,
                                  remap_max,
                                  remap_index_list,
                                  remap_index_count,
                                  initial_index,
                                  initial_position);
      }
      else {
        ov = raiko_select_mode_0d(dv,
                                  r_sphere,
                                  r_sphere_min,
                                  r_sphere_max,
                                  r_sphere_index_list,
                                  r_sphere_index_count,
                                  translation_rotation,
                                  translation_rotation_min,
                                  translation_rotation_max,
                                  translation_rotation_index_list,
                                  translation_rotation_index_count,
                                  scale,
                                  scale_randomness,
                                  scale_index_list,
                                  scale_index_count,
                                  remap,
                                  remap_min,
                                  remap_max,
                                  remap_index_list,
                                  remap_index_count);
      }
      break;
    }
  }
  return ov;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Gabor Noise
 *
 * Implements Gabor noise based on the paper:
 *
 *   Lagae, Ares, et al. "Procedural noise using sparse Gabor convolution." ACM Transactions on
 *   Graphics (TOG) 28.3 (2009): 1-10.
 *
 * But with the improvements from the paper:
 *
 *   Tavernier, Vincent, et al. "Making gabor noise fast and normalized." Eurographics 2019-40th
 *   Annual Conference of the European Association for Computer Graphics. 2019.
 *
 * And compute the Phase and Intensity of the Gabor based on the paper:
 *
 *   Tricard, Thibault, et al. "Procedural phasor noise." ACM Transactions on Graphics (TOG) 38.4
 *   (2019): 1-13.
 *
 * \{ */

/* The original Gabor noise paper specifies that the impulses count for each cell should be
 * computed by sampling a Poisson distribution whose mean is the impulse density. However,
 * Tavernier's paper showed that stratified Poisson point sampling is better assuming the weights
 * are sampled using a Bernoulli distribution, as shown in Figure (3). By stratified sampling, they
 * mean a constant number of impulses per cell, so the stratification is the grid itself in that
 * sense, as described in the supplementary material of the paper. */
static constexpr int gabor_impulses_count = 8;

/* Computes a 2D Gabor kernel based on Equation (6) in the original Gabor noise paper. Where the
 * frequency argument is the F_0 parameter and the orientation argument is the w_0 parameter. We
 * assume the Gaussian envelope has a unit magnitude, that is, K = 1. That is because we will
 * eventually normalize the final noise value to the unit range, so the multiplication by the
 * magnitude will be canceled by the normalization. Further, we also assume a unit Gaussian width,
 * that is, a = 1. That is because it does not provide much artistic control. It follows that the
 * Gaussian will be truncated at pi.
 *
 * To avoid the discontinuities caused by the aforementioned truncation, the Gaussian is windowed
 * using a Hann window, that is because contrary to the claim made in the original Gabor paper,
 * truncating the Gaussian produces significant artifacts especially when differentiated for bump
 * mapping. The Hann window is C1 continuous and has limited effect on the shape of the Gaussian,
 * so it felt like an appropriate choice.
 *
 * Finally, instead of computing the Gabor value directly, we instead use the complex phasor
 * formulation described in section 3.1.1 in Tricard's paper. That's done to be able to compute the
 * phase and intensity of the Gabor noise after summation based on equations (8) and (9). The
 * return value of the Gabor kernel function is then a complex number whose real value is the
 * value computed in the original Gabor noise paper, and whose imaginary part is the sine
 * counterpart of the real part, which is the only extra computation in the new formulation.
 *
 * Note that while the original Gabor noise paper uses the cosine part of the phasor, that is, the
 * real part of the phasor, we use the sine part instead, that is, the imaginary part of the
 * phasor, as suggested by Tavernier's paper in "Section 3.3. Instance stationarity and
 * normalization", to ensure a zero mean, which should help with normalization. */
static float2 compute_2d_gabor_kernel(const float2 position,
                                      const float frequency,
                                      const float orientation)
{
  const float distance_squared = math::length_squared(position);
  const float hann_window = 0.5f + 0.5f * math::cos(math::numbers::pi * distance_squared);
  const float gaussian_envelop = math::exp(-math::numbers::pi * distance_squared);
  const float windowed_gaussian_envelope = gaussian_envelop * hann_window;

  const float2 frequency_vector = frequency * float2(cos(orientation), sin(orientation));
  const float angle = 2.0f * math::numbers::pi * math::dot(position, frequency_vector);
  const float2 phasor = float2(math::cos(angle), math::sin(angle));

  return windowed_gaussian_envelope * phasor;
}

/* Computes the approximate standard deviation of the zero mean normal distribution representing
 * the amplitude distribution of the noise based on Equation (9) in the original Gabor noise paper.
 * For simplicity, the Hann window is ignored and the orientation is fixed since the variance is
 * orientation invariant. We start integrating the squared Gabor kernel with respect to x:
 *
 *   \int_{-\infty}^{-\infty} (e^{- \pi (x^2 + y^2)} cos(2 \pi f_0 x))^2 dx
 *
 * Which gives:
 *
 *  \frac{(e^{2 \pi f_0^2}-1) e^{-2 \pi y^2 - 2 pi f_0^2}}{2^\frac{3}{2}}
 *
 * Then we similarly integrate with respect to y to get:
 *
 *  \frac{1 - e^{-2 \pi f_0^2}}{4}
 *
 * Secondly, we note that the second moment of the weights distribution is 0.5 since it is a
 * fair Bernoulli distribution. So the final standard deviation expression is square root the
 * integral multiplied by the impulse density multiplied by the second moment.
 *
 * Note however that the integral is almost constant for all frequencies larger than one, and
 * converges to an upper limit as the frequency approaches infinity, so we replace the expression
 * with the following limit:
 *
 *  \lim_{x \to \infty} \frac{1 - e^{-2 \pi f_0^2}}{4}
 *
 * To get an approximation of 0.25. */
static float compute_2d_gabor_standard_deviation()
{
  const float integral_of_gabor_squared = 0.25f;
  const float second_moment = 0.5f;
  return math::sqrt(gabor_impulses_count * second_moment * integral_of_gabor_squared);
}

/* Computes the Gabor noise value at the given position for the given cell. This is essentially the
 * sum in Equation (8) in the original Gabor noise paper, where we sum Gabor kernels sampled at a
 * random position with a random weight. The orientation of the kernel is constant for anisotropic
 * noise while it is random for isotropic noise. The original Gabor noise paper mentions that the
 * weights should be uniformly distributed in the [-1, 1] range, however, Tavernier's paper showed
 * that using a Bernoulli distribution yields better results, so that is what we do. */
static float2 compute_2d_gabor_noise_cell(const float2 cell,
                                          const float2 position,
                                          const float frequency,
                                          const float isotropy,
                                          const float base_orientation)

{
  float2 noise(0.0f);
  for (const int i : IndexRange(gabor_impulses_count)) {
    /* Compute unique seeds for each of the needed random variables. */
    const float3 seed_for_orientation(cell.x, cell.y, i * 3);
    const float3 seed_for_kernel_center(cell.x, cell.y, i * 3 + 1);
    const float3 seed_for_weight(cell.x, cell.y, i * 3 + 2);

    /* For isotropic noise, add a random orientation amount, while for anisotropic noise, use the
     * base orientation. Linearly interpolate between the two cases using the isotropy factor. Note
     * that the random orientation range spans pi as opposed to two pi, that's because the Gabor
     * kernel is symmetric around pi. */
    const float random_orientation = (noise::hash_float_to_float(seed_for_orientation) - 0.5f) *
                                     math::numbers::pi;
    const float orientation = base_orientation + random_orientation * isotropy;

    const float2 kernel_center = noise::hash_float_to_float2(seed_for_kernel_center);
    const float2 position_in_kernel_space = position - kernel_center;

    /* The kernel is windowed beyond the unit distance, so early exit with a zero for points that
     * are further than a unit radius. */
    if (math::length_squared(position_in_kernel_space) >= 1.0f) {
      continue;
    }

    /* We either add or subtract the Gabor kernel based on a Bernoulli distribution of equal
     * probability. */
    const float weight = noise::hash_float_to_float(seed_for_weight) < 0.5f ? -1.0f : 1.0f;

    noise += weight * compute_2d_gabor_kernel(position_in_kernel_space, frequency, orientation);
  }
  return noise;
}

/* Computes the Gabor noise value by dividing the space into a grid and evaluating the Gabor noise
 * in the space of each cell of the 3x3 cell neighborhood. */
static float2 compute_2d_gabor_noise(const float2 coordinates,
                                     const float frequency,
                                     const float isotropy,
                                     const float base_orientation)
{
  const float2 cell_position = math::floor(coordinates);
  const float2 local_position = coordinates - cell_position;

  float2 sum(0.0f);
  for (int j = -1; j <= 1; j++) {
    for (int i = -1; i <= 1; i++) {
      const float2 cell_offset = float2(i, j);
      const float2 current_cell_position = cell_position + cell_offset;
      const float2 position_in_cell_space = local_position - cell_offset;
      sum += compute_2d_gabor_noise_cell(
          current_cell_position, position_in_cell_space, frequency, isotropy, base_orientation);
    }
  }

  return sum;
}

/* Identical to compute_2d_gabor_kernel, except it is evaluated in 3D space. Notice that Equation
 * (6) in the original Gabor noise paper computes the frequency vector using (cos(w_0), sin(w_0)),
 * which we also do in the 2D variant, however, for 3D, the orientation is already a unit frequency
 * vector, so we just need to scale it by the frequency value. */
static float2 compute_3d_gabor_kernel(const float3 position,
                                      const float frequency,
                                      const float3 orientation)
{
  const float distance_squared = math::length_squared(position);
  const float hann_window = 0.5f + 0.5f * math::cos(math::numbers::pi * distance_squared);
  const float gaussian_envelop = math::exp(-math::numbers::pi * distance_squared);
  const float windowed_gaussian_envelope = gaussian_envelop * hann_window;

  const float3 frequency_vector = frequency * orientation;
  const float angle = 2.0f * math::numbers::pi * math::dot(position, frequency_vector);
  const float2 phasor = float2(math::cos(angle), math::sin(angle));

  return windowed_gaussian_envelope * phasor;
}

/* Identical to compute_2d_gabor_standard_deviation except we do triple integration in 3D. The only
 * difference is the denominator in the integral expression, which is 2^{5 / 2} for the 3D case
 * instead of 4 for the 2D case. Similarly, the limit evaluates to 1 / (4 * sqrt(2)). */
static float compute_3d_gabor_standard_deviation()
{
  const float integral_of_gabor_squared = 1.0f / (4.0f * math::numbers::sqrt2);
  const float second_moment = 0.5f;
  return math::sqrt(gabor_impulses_count * second_moment * integral_of_gabor_squared);
}

/* Computes the orientation of the Gabor kernel such that it is constant for anisotropic
 * noise while it is random for isotropic noise. We randomize in spherical coordinates for a
 * uniform distribution. */
static float3 compute_3d_orientation(const float3 orientation,
                                     const float isotropy,
                                     const float4 seed)
{
  /* Return the base orientation in case we are completely anisotropic. */
  if (isotropy == 0.0) {
    return orientation;
  }

  /* Compute the orientation in spherical coordinates. */
  float inclination = math::acos(orientation.z);
  float azimuth = math::sign(orientation.y) *
                  math::acos(orientation.x / math::length(float2(orientation.x, orientation.y)));

  /* For isotropic noise, add a random orientation amount, while for anisotropic noise, use the
   * base orientation. Linearly interpolate between the two cases using the isotropy factor. Note
   * that the random orientation range is to pi as opposed to two pi, that's because the Gabor
   * kernel is symmetric around pi. */
  const float2 random_angles = noise::hash_float_to_float2(seed) * math::numbers::pi;
  inclination += random_angles.x * isotropy;
  azimuth += random_angles.y * isotropy;

  /* Convert back to Cartesian coordinates, */
  return float3(math::sin(inclination) * math::cos(azimuth),
                math::sin(inclination) * math::sin(azimuth),
                math::cos(inclination));
}

static float2 compute_3d_gabor_noise_cell(const float3 cell,
                                          const float3 position,
                                          const float frequency,
                                          const float isotropy,
                                          const float3 base_orientation)

{
  float2 noise(0.0f);
  for (const int i : IndexRange(gabor_impulses_count)) {
    /* Compute unique seeds for each of the needed random variables. */
    const float4 seed_for_orientation(cell.x, cell.y, cell.z, i * 3);
    const float4 seed_for_kernel_center(cell.x, cell.y, cell.z, i * 3 + 1);
    const float4 seed_for_weight(cell.x, cell.y, cell.z, i * 3 + 2);

    const float3 orientation = compute_3d_orientation(
        base_orientation, isotropy, seed_for_orientation);

    const float3 kernel_center = noise::hash_float_to_float3(seed_for_kernel_center);
    const float3 position_in_kernel_space = position - kernel_center;

    /* The kernel is windowed beyond the unit distance, so early exit with a zero for points that
     * are further than a unit radius. */
    if (math::length_squared(position_in_kernel_space) >= 1.0f) {
      continue;
    }

    /* We either add or subtract the Gabor kernel based on a Bernoulli distribution of equal
     * probability. */
    const float weight = noise::hash_float_to_float(seed_for_weight) < 0.5f ? -1.0f : 1.0f;

    noise += weight * compute_3d_gabor_kernel(position_in_kernel_space, frequency, orientation);
  }
  return noise;
}

/* Identical to compute_2d_gabor_noise but works in the 3D neighborhood of the noise. */
static float2 compute_3d_gabor_noise(const float3 coordinates,
                                     const float frequency,
                                     const float isotropy,
                                     const float3 base_orientation)
{
  const float3 cell_position = math::floor(coordinates);
  const float3 local_position = coordinates - cell_position;

  float2 sum(0.0f);
  for (int k = -1; k <= 1; k++) {
    for (int j = -1; j <= 1; j++) {
      for (int i = -1; i <= 1; i++) {
        const float3 cell_offset = float3(i, j, k);
        const float3 current_cell_position = cell_position + cell_offset;
        const float3 position_in_cell_space = local_position - cell_offset;
        sum += compute_3d_gabor_noise_cell(
            current_cell_position, position_in_cell_space, frequency, isotropy, base_orientation);
      }
    }
  }

  return sum;
}

void gabor(const float2 coordinates,
           const float scale,
           const float frequency,
           const float anisotropy,
           const float orientation,
           float *r_value,
           float *r_phase,
           float *r_intensity)
{
  const float2 scaled_coordinates = coordinates * scale;
  const float isotropy = 1.0f - math::clamp(anisotropy, 0.0f, 1.0f);
  const float sanitized_frequency = math::max(0.001f, frequency);

  const float2 phasor = compute_2d_gabor_noise(
      scaled_coordinates, sanitized_frequency, isotropy, orientation);
  const float standard_deviation = compute_2d_gabor_standard_deviation();

  /* Normalize the noise by dividing by six times the standard deviation, which was determined
   * empirically. */
  const float normalization_factor = 6.0f * standard_deviation;

  /* As discussed in compute_2d_gabor_kernel, we use the imaginary part of the phasor as the Gabor
   * value. But remap to [0, 1] from [-1, 1]. */
  if (r_value) {
    *r_value = (phasor.y / normalization_factor) * 0.5f + 0.5f;
  }

  /* Compute the phase based on equation (9) in Tricard's paper. But remap the phase into the
   * [0, 1] range. */
  if (r_phase) {
    *r_phase = (math::atan2(phasor.y, phasor.x) + math::numbers::pi) / (2.0f * math::numbers::pi);
  }

  /* Compute the intensity based on equation (8) in Tricard's paper. */
  if (r_intensity) {
    *r_intensity = math::length(phasor) / normalization_factor;
  }
}

void gabor(const float3 coordinates,
           const float scale,
           const float frequency,
           const float anisotropy,
           const float3 orientation,
           float *r_value,
           float *r_phase,
           float *r_intensity)
{
  const float3 scaled_coordinates = coordinates * scale;
  const float isotropy = 1.0f - math::clamp(anisotropy, 0.0f, 1.0f);
  const float sanitized_frequency = math::max(0.001f, frequency);

  const float3 normalized_orientation = math::normalize(orientation);
  const float2 phasor = compute_3d_gabor_noise(
      scaled_coordinates, sanitized_frequency, isotropy, normalized_orientation);
  const float standard_deviation = compute_3d_gabor_standard_deviation();

  /* Normalize the noise by dividing by six times the standard deviation, which was determined
   * empirically. */
  const float normalization_factor = 6.0f * standard_deviation;

  /* As discussed in compute_2d_gabor_kernel, we use the imaginary part of the phasor as the Gabor
   * value. But remap to [0, 1] from [-1, 1]. */
  if (r_value) {
    *r_value = (phasor.y / normalization_factor) * 0.5f + 0.5f;
  }

  /* Compute the phase based on equation (9) in Tricard's paper. But remap the phase into the
   * [0, 1] range. */
  if (r_phase) {
    *r_phase = (math::atan2(phasor.y, phasor.x) + math::numbers::pi) / (2.0f * math::numbers::pi);
  }

  /* Compute the intensity based on equation (8) in Tricard's paper. */
  if (r_intensity) {
    *r_intensity = math::length(phasor) / normalization_factor;
  }
}

/** \} */

}  // namespace blender::noise
