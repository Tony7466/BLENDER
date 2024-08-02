/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Library to read packed vertex buffer data of a `gpu::Batch` using a SSBO rather than using input
 * assembly. It is **not** needed to use these macros if the data is known to be aligned and not
 * packed.
 *
 * Implemented as macros to avoid compiler differences with buffer qualifiers.
 */

#define gpu_attr_load_triplet(_type, _data, _i) \
  _type(_data[_i * 3 + 0], _data[_i * 3 + 1], _data[_i * 3 + 2])

/* Assumes _data is declared as an array of float. */
#define gpu_attr_load_float3(_data, _i) gpu_attr_load_triplet(vec3, _data, _i)
/* Assumes _data is declared as an array of uint. */
#define gpu_attr_load_uint3(_data, _i) gpu_attr_load_triplet(ivec3, _data, _i)
/* Assumes _data is declared as an array of int. */
#define gpu_attr_load_int3(_data, _i) gpu_attr_load_triplet(uvec3, _data, _i)
