/* SPDX-FileCopyrightText: 2011-2024 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

/* Derived from:
 * https://knarkowicz.wordpress.com/2014/04/16/octahedron-normal-vector-encoding/
 * https://twitter.com/Stubbesaurus/status/937994790553227264
 * https://jcgt.org/published/0003/02/01/paper.pdf
 * https://www.shadertoy.com/view/llfcRl */

#ifndef __UTIL_OCTAHEDRAL_NORMAL_H__
#define __UTIL_OCTAHEDRAL_NORMAL_H__

#include "util/types.h"

CCL_NAMESPACE_BEGIN

template<uint SH>
ccl_device_inline uint octahedral_encoder(const float3 in)
{
  ccl_static_constexpr float mu = (float)((1u << SH) - 1u) / 2.0f;
  float2 v = make_float2(in.x / (fabsf(in.x) + fabsf(in.y) + fabsf(in.z)),
                         in.y / (fabsf(in.x) + fabsf(in.y) + fabsf(in.z)));
  v = in.z >= 0.0f ? v : make_float2((1.0f - fabsf(v.y)) * (v.x >= 0.0f ? 1.0f : -1.0f),
                                     (1.0f - fabsf(v.x)) * (v.y >= 0.0f ? 1.0f : -1.0f));
  const uint2 d = make_uint2(uint(roundf(mu + v.x * mu)),
                             uint(roundf(mu + v.y * mu)));
  return d.x | (d.y << SH);
}

template<uint SH>
ccl_device_inline packed_float3 octahedral_decoder(const uint in)
{
  ccl_static_constexpr uint mu = (1u << SH) - 1u;
  ccl_static_constexpr float div = (float)mu / 2.0f;
  const uint2 iv = make_uint2(in & mu, (in >> SH) & mu);
  packed_float3 n = make_float3((float)iv.x / div - 1.0f,
                                (float)iv.y / div - 1.0f,
                                0.0f);
  n.z = 1.0f - fabsf(n.x) - fabsf(n.y);
  const float t = max(-n.z, 0.0f);
  n.x += (n.x > 0.0f) ? -t : t;
  n.y += (n.y > 0.0f) ? -t : t;
  return normalize(n);
}

ccl_device_inline uint encode_normal(const float3 in)
{
    return octahedral_encoder<16>(in);
}

ccl_device_inline packed_float3 decode_normal(const uint in)
{
    return octahedral_decoder<16>(in);
}

ccl_device_inline uint encode_tangent(const float4 in)
{
  const uint result = octahedral_encoder<15>(float4_to_float3(in));
  /* store tangent sign in first 2 bits */
  return in.w < 0.0f ? result | (1 << 30) : result;
}

ccl_device_inline float4 decode_tangent(const uint in)
{
  const packed_float3 result = octahedral_decoder<15>(in);
  const float sign = (in >> 30) == 1 ? -1.0f : 1.0f;
  return make_float4(result.x, result.y, result.z, sign);
}

CCL_NAMESPACE_END

#endif /* __UTIL_OCTAHEDRAL_NORMAL_H__ */
