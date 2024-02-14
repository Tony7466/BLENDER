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

/* Mirror the UV if they are not on the diagonal or unit UV squares.
 * Doesn't extend outside of [-1..2] range. But this is fine since we use it only for borders. */
vec2 mirror_repeat_uv(vec2 uv)
{
  vec2 m = abs(uv - 0.5) + 0.5;
  vec2 f = floor(m);
  float x = f.x - f.y;
  if (x != 0.0) {
    uv.xy = 1.0 - uv.xy;
  }
  return fract(uv);
}

/* local_texel is the texel coordinate inside the probe area [0..texel_area.extent] range. */
vec3 sphere_probe_texel_to_direction(ivec2 local_texel,
                                     SphereProbePixelArea texel_area,
                                     SphereProbeUvArea uv_area,
                                     int atlas_mip_size,
                                     out vec2 wrapped_uv)
{
  /* Texel in probe atlas. */
  ivec2 texel = local_texel + texel_area.offset;
  /* UV in probe atlas. */
  vec2 atlas_uv = (vec2(texel) + 0.5) / vec2(atlas_mip_size);
  /* UV in sampling area. */
  vec2 sampling_uv = (atlas_uv - uv_area.offset) / uv_area.scale;
  wrapped_uv = mirror_repeat_uv(sampling_uv);
  /* Direction in world space. */
  return octahedral_uv_to_direction(wrapped_uv);
}

vec3 sphere_probe_texel_to_direction(ivec2 local_texel,
                                     SphereProbePixelArea texel_area,
                                     SphereProbeUvArea uv_area,
                                     int atlas_mip_size)
{
  vec2 wrapped_uv;
  return sphere_probe_texel_to_direction(
      local_texel, texel_area, uv_area, atlas_mip_size, wrapped_uv);
}
