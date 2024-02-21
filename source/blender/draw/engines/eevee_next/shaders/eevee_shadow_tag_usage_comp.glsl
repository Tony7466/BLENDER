/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Virtual shadow-mapping: Usage tagging
 *
 * Shadow pages are only allocated if they are visible.
 * This pass scan the depth buffer and tag all tiles that are needed for light shadowing as
 * needed.
 */

#pragma BLENDER_REQUIRE(eevee_shadow_tag_usage_lib.glsl)

void shadow_tag_usage_depth(float depth_val, ivec2 local_texel)
{
  vec2 uv = (vec2(local_texel) + 0.5) / vec2(textureSize(depth_tx, 0).xy);
  vec3 vP = drw_point_screen_to_view(vec3(uv, depth_val));
  vec3 P = drw_point_view_to_world(vP);
  vec2 pixel = vec2(local_texel);
  shadow_tag_usage(vP, P, pixel);
}

void shadow_tag_usage_depth_check(float depth_val, ivec2 local_texel)
{
  if (depth_val < 1.0) {
    shadow_tag_usage_depth(depth_val, local_texel);
  }
}

void main()
{
  ivec2 tex_size = textureSize(depth_tx, 0).xy;
  ivec2 base_texel = ivec2(gl_GlobalInvocationID.xy) * SHADOW_TAG_USAGE_NUM_PIXELS_PER_THREAD_DIM;
  ivec2 max_texel = min(base_texel + SHADOW_TAG_USAGE_NUM_PIXELS_PER_THREAD_DIM,
                        ivec2(tex_size - 1));

#if SHADOW_TAG_USAGE_NUM_PIXELS_PER_THREAD_DIM >= 2
  /* Use texture gather to read 2x2 depth blocks. */
  ivec2 max_texel_round = (max_texel / 2) * 2;

  ivec2 texel = base_texel;
  for (texel.y = base_texel.y; texel.y < max_texel_round.y; texel.y += 2) {
    for (texel.x = base_texel.x; texel.x < max_texel_round.x; texel.x += 2) {
      vec4 depths = textureGather(depth_tx, vec2(texel) / vec2(textureSize(depth_tx, 0).xy));

      if (all(lessThan(depths, vec4(1.0)))) {
        shadow_tag_usage_depth(depths.r, texel + ivec2(0, 1));
        shadow_tag_usage_depth(depths.g, texel + ivec2(1, 1));
        shadow_tag_usage_depth(depths.b, texel + ivec2(1, 0));
        shadow_tag_usage_depth(depths.a, texel + ivec2(0, 0));
      }
      else {
        shadow_tag_usage_depth_check(depths.r, texel + ivec2(0, 1));
        shadow_tag_usage_depth_check(depths.g, texel + ivec2(1, 1));
        shadow_tag_usage_depth_check(depths.b, texel + ivec2(1, 0));
        shadow_tag_usage_depth_check(depths.a, texel + ivec2(0, 0));
      }
    }
  }

#endif

  /* Evaluate remaining individual edge fragments when texture is non-even. */
  for (int j = base_texel.y; j < max_texel.y; j++) {
    for (int i = base_texel.x; i < max_texel.x; i++) {
      ivec2 texel = ivec2(i, j);
      float depth = texelFetch(depth_tx, texel, 0).r;

      shadow_tag_usage_depth(depth, base_texel);
    }
  }
}
