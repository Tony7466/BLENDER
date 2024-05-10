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

ccl_device_inline uint octahedral_encode(const float3 in)
{
  float2 v = make_float2(in.x / (fabsf(in.x) + fabsf(in.y) + fabsf(in.z)),
                         in.y / (fabsf(in.x) + fabsf(in.y) + fabsf(in.z)));
  v = in.z >= 0.0f ? v : make_float2((1.0f - fabsf(v.y)) * (v.x >= 0.0f ? 1.0f : -1.0f),
                                     (1.0f - fabsf(v.x)) * (v.y >= 0.0f ? 1.0f : -1.0f));
  const uint2 d = make_uint2(uint(roundf(32767.5f + v.x * 32767.5f)),
                             uint(roundf(32767.5f + v.y * 32767.5f)));
  return d.x | (d.y << 16u);
}

ccl_device_inline packed_float3 octahedral_decode(const uint in)
{
  const uint2 iv = make_uint2(in & 65535u, (in >> 16u) & 65535u);
  packed_float3 n = make_float3((float)iv.x / 32767.5f - 1.0f,
                                (float)iv.y / 32767.5f - 1.0f,
                                0.0f);
  n.z = 1.0f - fabsf(n.x) - fabsf(n.y);
  const float t = max(-n.z, 0.0f);
  n.x += (n.x > 0.0f) ? -t : t;
  n.y += (n.y > 0.0f) ? -t : t;
  return normalize(n);
}

CCL_NAMESPACE_END

#endif /* __UTIL_OCTAHEDRAL_NORMAL_H__ */
