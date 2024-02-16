/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma BLENDER_REQUIRE(eevee_octahedron_lib.glsl)

SphereProbePixelArea reinterpret_as_write_coord(ivec4 packed_coord)
{
  SphereProbePixelArea unpacked;
  unpacked.offset = packed_coord.xy;
  unpacked.extent = packed_coord.z;
  unpacked.layer = packed_coord.w;
  return unpacked;
}

SphereProbeUvArea reinterpret_as_atlas_coord(ivec4 packed_coord)
{
  SphereProbeUvArea unpacked;
  unpacked.offset = intBitsToFloat(packed_coord.xy);
  unpacked.scale = intBitsToFloat(packed_coord.z);
  unpacked.layer = intBitsToFloat(packed_coord.w);
  return unpacked;
}

/* local_texel is the texel coordinate inside the probe area [0..texel_area.extent] range.
 * Returned vector is not normalized. */
vec3 sphere_probe_texel_to_direction(ivec2 local_texel,
                                     SphereProbePixelArea texel_area,
                                     SphereProbeUvArea uv_area,
                                     int atlas_mip_size,
                                     out vec2 sampling_uv)
{
  /* Texel in probe atlas. */
  ivec2 texel = local_texel + texel_area.offset;
  /* UV in sampling area. */
  sampling_uv = (vec2(texel) + 0.5) / vec2(texel_area.extent);
  /* Direction in world space. */
  return octahedral_uv_to_direction(sampling_uv);
}

/* local_texel is the texel coordinate inside the probe area [0..texel_area.extent] range.
 * Returned vector is not normalized. */
vec3 sphere_probe_texel_to_direction(ivec2 local_texel,
                                     SphereProbePixelArea texel_area,
                                     SphereProbeUvArea uv_area,
                                     int atlas_mip_size)
{
  vec2 sampling_uv_unused;
  return sphere_probe_texel_to_direction(
      local_texel, texel_area, uv_area, atlas_mip_size, sampling_uv_unused);
}

/* Apply correct bias and scale for the given level of detail. */
vec2 sphere_probe_miplvl_scale_bias(float mip_lvl, SphereProbeUvArea uv_area, vec2 uv)
{
  float pixel_count_mip_0 = float(SPHERE_PROBE_ATLAS_RES) * uv_area.scale;
  /* This should be always odd. */
  float pixel_count = floor(pixel_count_mip_0 / exp2(mip_lvl));
  float scale = (pixel_count - 1.0) / pixel_count;
  float offset = 0.5 / pixel_count;
  return uv * scale + offset;
}

void sphere_probe_direction_to_uv(vec3 L,
                                  float lod_min,
                                  float lod_max,
                                  SphereProbeUvArea uv_area,
                                  out vec2 altas_uv_min,
                                  out vec2 altas_uv_max)
{
  /* We place texel centers at the edges of the octahedron, to avoid artifacts caused by
   * interpolating across the edges.
   *
   * This is a diagram of a 5px^2 map with 2 mips. 0 and 1 denote the pixels centers of each
   * mips. The lines shows the octahedron edges.
   *
   * 1---0---1---0---1 <
   * |     / | \     |  |
   * 0   0   0   0   0  |
   * | /     |     \ |  |
   * 1---0---1---0---1  | sampling area
   * | \     |     / |  |
   * 0   0   0   0   0  |
   * |     \ | /     |  |
   * 1---0---1---0---1 <
   * ^---------------^
   *       sampling area
   *
   * To do this, we made all mip levels have an odd number of pixel, by sizing the atlas
   * appropriately. But this makes the interpolation between mips not aligning and we need to use a
   * different uv coordinate for each level of detail. Fortunately, this avoids us to have to worry
   * about padding texels all together.
   */
  vec2 octahedral_uv = octahedral_uv_from_direction(L);
  vec2 local_uv_min = sphere_probe_miplvl_scale_bias(lod_min, uv_area, octahedral_uv);
  vec2 local_uv_max = sphere_probe_miplvl_scale_bias(lod_max, uv_area, octahedral_uv);
  /* Remap into atlas location. */
  altas_uv_min = local_uv_min * uv_area.scale + uv_area.offset;
  altas_uv_max = local_uv_max * uv_area.scale + uv_area.offset;
}

/* Single mip variant. */
vec2 sphere_probe_direction_to_uv(vec3 L, float lod, SphereProbeUvArea uv_area)
{
  vec2 altas_uv_min, altas_uv_max_unused;
  sphere_probe_direction_to_uv(L, lod, 0.0, uv_area, altas_uv_min, altas_uv_max_unused);
  return altas_uv_min;
}
