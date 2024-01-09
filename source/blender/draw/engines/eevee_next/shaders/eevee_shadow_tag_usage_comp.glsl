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

void main()
{
  ivec2 tex_size = textureSize(depth_tx, 0).xy;
  ivec2 base_texel = ivec2(gl_GlobalInvocationID.xy) * SHADOW_TAG_USAGE_NUM_CELLS_PER_THREAD_DIM;
  ivec2 max_texel = min(base_texel + SHADOW_TAG_USAGE_NUM_CELLS_PER_THREAD_DIM,
                        ivec2(tex_size - 1));

  /* Only consider this initial check for cases where sufficient fragments are processed beyond
   * what the lower depth check would capture. For this to benefit performance, the cells processed
   * per thread invocation must be greater than 4 in each dimension. */
#if SHADOW_TAG_USAGE_NUM_CELLS_PER_THREAD_DIM >= 4
  /* Quick test for filtering out entire regions. */
  vec2 texel_f = vec2(base_texel) / vec2(imageSize(hiz_tx).xy);
  int hiz_mip = int(floor(log2(float(SHADOW_TAG_USAGE_NUM_CELLS_PER_THREAD_DIM))));
  float hiz_depth = texture(hiz_tx, texel_f, hiz_mip).r;
  if (hiz_depth == 1.0) {
    return;
  }
#endif

#define SHADOW_TAG_USAGE_DEPTH(depth_val, offset) \
  { \
    ivec2 local_texel = base_texel + offset; \
    vec2 uv = (vec2(local_texel) + 0.5) / vec2(tex_size); \
    vec3 vP = drw_point_screen_to_view(vec3(uv, depth_val)); \
    vec3 P = drw_point_view_to_world(vP); \
    vec2 pixel = vec2(local_texel); \
    shadow_tag_usage(vP, P, pixel); \
  }

#define SHADOW_TAG_USAGE_DEPTH_CHECK(depth_val, offset) \
  if (depth_val < 1.0) { \
    SHADOW_TAG_USAGE_DEPTH(depth_val, offset) \
  }

#if SHADOW_TAG_USAGE_NUM_CELLS_PER_THREAD_DIM >= 2
  /* Use texture gather to read 2x2 depth blocks. */
  ivec2 max_texel_round = (max_texel / 2) * 2;
  for (; base_texel.x < max_texel_round.x; base_texel.x += 2) {
    for (; base_texel.y < max_texel_round.y; base_texel.y += 2) {
      vec4 depths = textureGather(depth_tx, vec2(base_texel) / vec2(imageSize(depth_tx).xy));

      /* Perform faster boolean check to take single branch for the case where all depths flag
       * updates. */
      if (all(lessThan(depths, vec4(1.0)))) {
        SHADOW_TAG_USAGE_DEPTH(depths.r, ivec2(0, 0))
        SHADOW_TAG_USAGE_DEPTH(depths.g, ivec2(1, 1))
        SHADOW_TAG_USAGE_DEPTH(depths.b, ivec2(1, 0))
        SHADOW_TAG_USAGE_DEPTH(depths.a, ivec2(0, 1))
      }
      else {
        SHADOW_TAG_USAGE_DEPTH_CHECK(depths.r, ivec2(0, 0))
        SHADOW_TAG_USAGE_DEPTH_CHECK(depths.g, ivec2(1, 1))
        SHADOW_TAG_USAGE_DEPTH_CHECK(depths.b, ivec2(1, 0))
        SHADOW_TAG_USAGE_DEPTH_CHECK(depths.a, ivec2(0, 1))
      }
    }
  }

#endif

  /* Evaluate remaining individual edge fragments when texture is non-even. */
  for (int i = base_texel.x; i < max_texel.x; i++) {
    for (int j = base_texel.y; j < max_texel.y; j++) {
      ivec2 texel = ivec2(i, j);
      float depth = texelFetch(depth_tx, texel, 0).r;

      SHADOW_TAG_USAGE_DEPTH(depth, ivec2(0, 0))
    }
  }
}
