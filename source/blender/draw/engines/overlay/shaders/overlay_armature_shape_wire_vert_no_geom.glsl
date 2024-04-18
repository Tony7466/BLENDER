/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma BLENDER_REQUIRE(common_view_clipping_lib.glsl)
#pragma BLENDER_REQUIRE(common_view_lib.glsl)

void main()
{
  vec4 bone_color, state_color;
  mat4 model_mat = extract_matrix_packed_data(inst_obmat, state_color, bone_color);

  vec3 world_pos = (model_mat * vec4(pos, 1.0)).xyz;
  gl_Position = point_world_to_ndc(world_pos);

  /* Due to packing, the wire width is passed in compressed. If the RNA range is increased, this
   * needs to change as well. */
  const float wire_width = bone_color.a * 16.0;
  geometry_out.wire_width = wire_width;

  geometry_out.finalColor.rgb = mix(state_color.rgb, bone_color.rgb, 0.5);
  geometry_out.finalColor.a = 1.0;
  view_clipping_distances(world_pos);
}
