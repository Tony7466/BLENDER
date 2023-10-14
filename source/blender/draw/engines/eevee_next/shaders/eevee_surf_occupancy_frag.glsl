/* SPDX-FileCopyrightText: 2017-2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Prepass that voxelizes an object on frustum aligned voxels.
 */

#pragma BLENDER_REQUIRE(eevee_sampling_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_nodetree_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_surf_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_velocity_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_volume_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_transparency_lib.glsl)

vec4 closure_to_rgba(Closure cl)
{
  return vec4(0.0);
}

void main()
{
  ivec2 texel = ivec2(gl_FragCoord.xy);
  vec2 uv = gl_FragCoord.xy / vec2(imageSize(occupancy_img).xy);
  vec3 ss_P = vec3(uv, gl_FragCoord.z);

  float volume_z = screen_to_volume(ss_P).z;
  /* TODO(fclem): Check if this quantization is good. */
  int volume_bit = int(volume_z * uniform_buf.volumes.tex_size.z);

  for (int i = 0; i < imageSize(occupancy_img).z; i++) {
    int shift = volume_bit - i * 32;
    if (shift < 32) {
      uint occupancy_bits = 0xFFFFFFFFu;
      if (shift > 0) {
        occupancy_bits >>= uint(shift);
      }
      imageAtomicXor(occupancy_img, ivec3(texel, i), occupancy_bits);
    }
  }
}
