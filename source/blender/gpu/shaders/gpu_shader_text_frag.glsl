/* SPDX-FileCopyrightText: 2016-2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma BLENDER_REQUIRE(gpu_shader_colorspace_lib.glsl)

float texel_fetch(int index)
{
  // glyph_tex_size: upper 8 bits is log2 of texture width, lower 24 bits is width-1
  int col_mask = glyph_tex_size & 0xFFFFFF;
  int row_shift = glyph_tex_size >> 24;
  ivec2 uv = ivec2(index & col_mask, index >> row_shift);
  return texelFetch(glyph, uv, 0).r;
}

bool is_inside_box(ivec2 v)
{
  return all(greaterThanEqual(v, ivec2(0))) && all(lessThan(v, glyph_dim));
}

float texture_1D_custom_bilinear_filter(vec2 f, vec2 uv)
{
  ivec2 texel_2d_near = ivec2(uv) - 1;
  int frag_offset = glyph_offset + texel_2d_near.y * glyph_dim.x + texel_2d_near.x;

  int offset_x = 1;
  int offset_y = glyph_dim.x;
  float tl = texel_fetch(frag_offset);
  float tr = texel_fetch(frag_offset + offset_x);
  float bl = texel_fetch(frag_offset + offset_y);
  float br = texel_fetch(frag_offset + offset_x + offset_y);
  if (!is_inside_box(texel_2d_near)) tl = 0.0;
  if (!is_inside_box(texel_2d_near + ivec2(1, 0))) tr = 0.0;
  if (!is_inside_box(texel_2d_near + ivec2(0, 1))) bl = 0.0;
  if (!is_inside_box(texel_2d_near + ivec2(1, 1))) br = 0.0;

  float tA = mix(tl, tr, f.x);
  float tB = mix(bl, br, f.x);

  return mix(tA, tB, f.y);
}

vec4 texture_1D_custom_bilinear_filter_color(vec2 uv)
{
  vec2 texel_2d = uv + 0.5;
  ivec2 texel_2d_near = ivec2(texel_2d) - 1;

  int frag_offset = glyph_offset + ((texel_2d_near.y * glyph_dim.x * glyph_comp_len) +
                                    (texel_2d_near.x * glyph_comp_len));

  float tr = 0.0;
  float tg = 0.0;
  float tb = 0.0;
  float ta = 0.0;

  if (is_inside_box(texel_2d_near)) {
    tr = texel_fetch(frag_offset);
    tg = texel_fetch(frag_offset + 1);
    tb = texel_fetch(frag_offset + 2);
    ta = texel_fetch(frag_offset + 3);
  }
  return vec4(tr, tg, tb, ta);
}

void main()
{
  vec2 uv_base = texCoord_interp;

  if (glyph_comp_len == 4) {
    fragColor.rgba = texture_1D_custom_bilinear_filter_color(uv_base).rgba;
    return;
  }

  // input color replaces texture color
  fragColor.rgb = color_flat.rgb;

  // modulate input alpha & texture alpha
  if (interp_size == 0) {
    uv_base += vec2(0.5);
    vec2 bilin_f = fract(uv_base);
    fragColor.a = texture_1D_custom_bilinear_filter(bilin_f, uv_base);
  }
  else {
    fragColor.a = 0.0;

    vec2 bilin_f = fract(uv_base);
    if (interp_size == 1) {
      /* NOTE(Metal): Declaring constant array in function scope to avoid increasing local shader
       * memory pressure. */
      const vec2 offsets4[4] = vec2[4](
          vec2(0.0, 1.0), vec2(1.0, 1.0), vec2(0.0, 0.0), vec2(0.0, 0.0)); //@TODO: one of offsets is wrong, (1.0, 0.0) is missing

      /* 3x3 blur */
      /* Manual unroll for performance (stupid GLSL compiler). */
      fragColor.a += texture_1D_custom_bilinear_filter(bilin_f, uv_base + offsets4[0]);
      fragColor.a += texture_1D_custom_bilinear_filter(bilin_f, uv_base + offsets4[1]);
      fragColor.a += texture_1D_custom_bilinear_filter(bilin_f, uv_base + offsets4[2]);
      fragColor.a += texture_1D_custom_bilinear_filter(bilin_f, uv_base + offsets4[3]);
      fragColor.a *= (1.0 / 4.0);
    }
    else {
      /* NOTE(Metal): Declaring constant array in function scope to avoid increasing local shader
       * memory pressure. */
      const vec2 offsets16[16] = vec2[16](vec2(-1.0, 2.0),
                                          vec2( 0.0, 2.0),
                                          vec2( 1.0, 2.0),
                                          vec2( 2.0, 2.0),
                                          vec2(-1.0, 1.0),
                                          vec2( 0.0, 1.0),
                                          vec2( 1.0, 1.0),
                                          vec2( 2.0, 1.0),
                                          vec2(-1.0, 0.0),
                                          vec2( 0.0, 0.0),
                                          vec2( 1.0, 0.0),
                                          vec2( 2.0, 0.0),
                                          vec2(-1.0, -1.0),
                                          vec2( 0.0, -1.0),
                                          vec2( 1.0, -1.0),
                                          vec2( 2.0, -1.0));

      /* 5x5 blur */
      /* Manual unroll for performance (stupid GLSL compiler). */
      vec2 f = fract(uv_base);
      fragColor.a += texture_1D_custom_bilinear_filter(bilin_f, uv_base + offsets16[0]);
      fragColor.a += texture_1D_custom_bilinear_filter(bilin_f, uv_base + offsets16[1]);
      fragColor.a += texture_1D_custom_bilinear_filter(bilin_f, uv_base + offsets16[2]);
      fragColor.a += texture_1D_custom_bilinear_filter(bilin_f, uv_base + offsets16[3]);
      fragColor.a += texture_1D_custom_bilinear_filter(bilin_f, uv_base + offsets16[4]);
      fragColor.a += texture_1D_custom_bilinear_filter(bilin_f, uv_base + offsets16[5]) * 2.0;
      fragColor.a += texture_1D_custom_bilinear_filter(bilin_f, uv_base + offsets16[6]) * 2.0;
      fragColor.a += texture_1D_custom_bilinear_filter(bilin_f, uv_base + offsets16[7]);
      fragColor.a += texture_1D_custom_bilinear_filter(bilin_f, uv_base + offsets16[8]);
      fragColor.a += texture_1D_custom_bilinear_filter(bilin_f, uv_base + offsets16[9]) * 2.0;
      fragColor.a += texture_1D_custom_bilinear_filter(bilin_f, uv_base + offsets16[10]) * 2.0;
      fragColor.a += texture_1D_custom_bilinear_filter(bilin_f, uv_base + offsets16[11]);
      fragColor.a += texture_1D_custom_bilinear_filter(bilin_f, uv_base + offsets16[12]);
      fragColor.a += texture_1D_custom_bilinear_filter(bilin_f, uv_base + offsets16[13]);
      fragColor.a += texture_1D_custom_bilinear_filter(bilin_f, uv_base + offsets16[14]);
      fragColor.a += texture_1D_custom_bilinear_filter(bilin_f, uv_base + offsets16[15]);
      fragColor.a *= (1.0 / 20.0);
    }
  }

  fragColor.a *= color_flat.a;
  fragColor = blender_srgb_to_framebuffer_space(fragColor);
}
