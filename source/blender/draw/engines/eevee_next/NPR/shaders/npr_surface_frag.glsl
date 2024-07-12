/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma BLENDER_REQUIRE(draw_view_lib.glsl)
#pragma BLENDER_REQUIRE(npr_nodetree_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_utildefines_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_math_base_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_codegen_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_deferred_combine_lib.glsl)

void main()
{
  ivec2 texel = ivec2(gl_FragCoord.xy);
  if (texelFetch(npr_index_tx, texel, 0).r != npr_index) {
    /* TODO(NPR): Convert NPR index to depth and use depth testing instead. */
    discard;
    return;
  }

  DeferredCombine dc = deferred_combine(texel);
  deferred_combine_clamp(dc);

  g_combined_color = deferred_combine_final_output(dc);
  g_combined_color.a = saturate(1.0 - g_combined_color.a);
  g_diffuse_color = vec4(dc.diffuse_color, 1.0);
  g_diffuse_direct = vec4(dc.diffuse_direct, 1.0);
  g_diffuse_indirect = vec4(dc.diffuse_indirect, 1.0);
  g_specular_color = vec4(dc.specular_color, 1.0);
  g_specular_direct = vec4(dc.specular_direct, 1.0);
  g_specular_indirect = vec4(dc.specular_indirect, 1.0);

  float depth = texelFetch(depth_tx, texel, 0).r;
  vec3 P = drw_point_screen_to_world(vec3(uvcoordsvar.xy, depth));
  vec3 N = dc.average_normal;

  g_data.P = P;
  g_data.Ni = N;
  g_data.N = N;
  g_data.Ng = N;
  g_data.is_strand = false;
  g_data.hair_time = 0.0;
  g_data.hair_thickness = 0.0;
  g_data.hair_strand_id = 0;
  g_data.ray_type = RAY_TYPE_CAMERA;
  g_data.ray_depth = 0.0;
  g_data.ray_length = distance(P, drw_view_position());
  g_data.barycentric_coords = vec2(0.0);
  g_data.barycentric_dists = vec3(0.0);

  out_color = nodetree_npr().rgb;
}
