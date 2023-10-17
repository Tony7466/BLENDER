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
#pragma BLENDER_REQUIRE(eevee_occupancy_lib.glsl)

vec4 closure_to_rgba(Closure cl)
{
  return vec4(0.0);
}

void main()
{
  ivec2 texel = ivec2(gl_FragCoord.xy);
  vec2 uv = gl_FragCoord.xy / vec2(imageSize(occupancy_img).xy);
  vec3 ss_P = vec3(uv, gl_FragCoord.z);
  /* Apply jitter here instead of modifying the projection matrix.
   * This is because the depth range and mapping function changes. */
  /* TODO(fclem): Jitter the camera for the other 2 dimension. */
  float jitter = sampling_rng_1D_get(SAMPLING_VOLUME_W) * uniform_buf.volumes.inv_tex_size.z;
  float volume_z = screen_to_volume(ss_P).z - jitter;

#if 0
  /* XOR technique:
   * Gives correct result for manifold meshes in and out of view. */
  OccupancyBits occupancy_bits = occupancy_from_depth(volume_z, uniform_buf.volumes.tex_size.z);

  for (int i = 0; i < imageSize(occupancy_img).z; i++) {
    /* Negate occupancy bits before XORing so that meshes clipped by the near plane fill the space
     * betwen the inner part of the mesh and the near plane.
     * It doesn't change anything for closed meshes. */
    occupancy_bits.bits[i] = ~occupancy_bits.bits[i];
    if (occupancy_bits.bits[i] != 0u) {
      imageAtomicXor(occupancy_img, ivec3(texel, i), occupancy_bits.bits[i]);
    }
  }
#else
  uint hit_id = imageAtomicAdd(hit_count_img, texel, 1u);
  if (hit_id < VOLUME_HIT_DEPTH_MAX) {
    float value = gl_FrontFacing ? volume_z : -volume_z;
    imageStore(hit_depth_img, ivec3(texel, hit_id), vec4(value));
  }
#endif
}
