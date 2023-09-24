/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Process in screen space the diffuse radiance input to mimic subsurface transmission.
 *
 * This implementation follows the technique described in the siggraph presentation:
 * "Efficient screen space subsurface scattering Siggraph 2018"
 * by Evgenii Golubev
 *
 * But, instead of having all the precomputed weights for all three color primaries,
 * we precompute a weight profile texture to be able to support per pixel AND per channel radius.
 */

#pragma BLENDER_REQUIRE(eevee_gbuffer_lib.glsl)
#pragma BLENDER_REQUIRE(common_view_lib.glsl)
#pragma BLENDER_REQUIRE(common_math_lib.glsl)

shared uint has_visible_sss;

void main(void)
{
  ivec2 texel = ivec2(gl_GlobalInvocationID.xy);

  if (all(equal(gl_LocalInvocationID.xy, uvec2(0)))) {
    has_visible_sss = 0u;
  }

  barrier();

  GBufferData gbuf = gbuffer_read(gbuf_header_tx, gbuf_closure_tx, gbuf_color_tx, texel);

  if (gbuf.has_diffuse && gbuf.diffuse.sss_id != 0u) {
    vec3 radiance = imageLoad(direct_light_img, texel).rgb +
                    imageLoad(indirect_light_img, texel).rgb;

    float sss_id_packed = gbuffer_object_id_f16_pack(gbuf.diffuse.sss_id);

    float max_radius = max_v3(gbuf.diffuse.sss_radius);

    imageStore(out_radiance_id_img, texel, vec4(radiance, sss_id_packed));

    vec2 center_uv = (vec2(texel) + 0.5) / vec2(textureSize(gbuf_header_tx, 0));
    float depth = texelFetch(depth_tx, texel, 0).r;
    /* TODO(fclem): Check if this simplifies. */
    float vPz = get_view_z_from_depth(depth);
    float homcoord = ProjectionMatrix[2][3] * vPz + ProjectionMatrix[3][3];
    float sample_scale = ProjectionMatrix[0][0] * (0.5 * max_radius / homcoord);
    float pixel_footprint = sample_scale * float(textureSize(gbuf_header_tx, 0).x);
    if (pixel_footprint > 1.0) {
      /* Race condition doesn't matter here. */
      has_visible_sss = 1u;
    }
  }
  else {
    imageStore(out_radiance_id_img, texel, vec4(0.0));
  }

  barrier();

  if (all(equal(gl_LocalInvocationID.xy, uvec2(0)))) {
    if (has_visible_sss > 0u) {
      uint tile_id = atomicAdd(convolve_dispatch_buf.num_groups_x, 1u);
      convolve_tile_buf[tile_id] = packUvec2x16(gl_WorkGroupID.xy);
      /* Race condition doesn't matter here. */
      convolve_dispatch_buf.num_groups_y = 1;
      convolve_dispatch_buf.num_groups_z = 1;
    }
  }
}
