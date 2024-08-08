/* SPDX-FileCopyrightText: 2017-2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma BLENDER_REQUIRE(common_view_clipping_lib.glsl)
#pragma BLENDER_REQUIRE(common_view_lib.glsl)
#pragma BLENDER_REQUIRE(overlay_edit_mesh_common_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_utildefines_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_math_vector_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_attribute_load_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_index_load_lib.glsl)

struct VertIn {
  /* Local Position. */
  vec3 lP;
  /* Local Vertex Normal. */
  vec3 lN;
  /* Edit Flags and Data. */
  uvec4 data;
};

VertIn input_assembly(uint in_vertex_id)
{
  uint v_i = gpu_index_load(in_vertex_id);

  VertIn vert_in;
  vert_in.lP = gpu_attr_load_float3(pos, gpu_attr_0, v_i);
  if (gpu_attr_1.x == 1) {
    vert_in.lN = gpu_attr_load_uint_1010102_snorm(vnor, gpu_attr_1, v_i).xyz;
  }
  else {
    vert_in.lN.x = uintBitsToFloat(vnor[gpu_attr_load_index(v_i, gpu_attr_1) + 0]);
    vert_in.lN.y = uintBitsToFloat(vnor[gpu_attr_load_index(v_i, gpu_attr_1) + 1]);
    vert_in.lN.z = uintBitsToFloat(vnor[gpu_attr_load_index(v_i, gpu_attr_1) + 2]);
  }
  vert_in.data = gpu_attr_load_uchar4(data, gpu_attr_2, v_i);
  return vert_in;
}

bool test_occlusion(vec4 gpu_position)
{
  vec3 ndc = (gpu_position.xyz / gpu_position.w) * 0.5 + 0.5;
  return ndc.z > texture(depthTex, ndc.xy).r;
}

vec3 non_linear_blend_color(vec3 col1, vec3 col2, float fac)
{
  col1 = pow(col1, vec3(1.0 / 2.2));
  col2 = pow(col2, vec3(1.0 / 2.2));
  vec3 col = mix(col1, col2, fac);
  return pow(col, vec3(2.2));
}

struct VertOut {
  vec4 gpu_position;
  vec4 final_color;
  vec4 final_color_outer;
  uint select_override;
};

VertOut vertex_main(VertIn vert_in)
{
  GPU_INTEL_VERTEX_SHADER_WORKAROUND

  VertOut vert_out;

  vec3 world_pos = point_object_to_world(vert_in.lP);
  vec3 view_pos = point_world_to_view(world_pos);
  vert_out.gpu_position = point_view_to_ndc(view_pos);

  /* Offset Z position for retopology overlay. */
  vert_out.gpu_position.z += get_homogenous_z_offset(
      view_pos.z, vert_out.gpu_position.w, retopologyOffset);

  uvec4 m_data = vert_in.data & uvec4(dataMask);

#if defined(VERT)
  vertexCrease = float(m_data.z >> 4) / 15.0;
  vert_out.final_color = EDIT_MESH_vertex_color(m_data.y, vertexCrease);
  gl_PointSize = sizeVertex * ((vertexCrease > 0.0) ? 3.0 : 2.0);
  /* Make selected and active vertex always on top. */
  if ((data.x & VERT_SELECTED) != 0u) {
    vert_out.gpu_position.z -= 5e-7 * abs(vert_out.gpu_position.w);
  }
  if ((data.x & VERT_ACTIVE) != 0u) {
    vert_out.gpu_position.z -= 5e-7 * abs(vert_out.gpu_position.w);
  }

  bool occluded = test_occlusion(vert_out.gpu_position);

#elif defined(EDGE)
  if (use_vertex_selection) {
    vert_out.final_color = EDIT_MESH_edge_vertex_color(m_data.y);
    vert_out.select_override = (m_data.y & EDGE_SELECTED);
  }
  else {
    vert_out.final_color = EDIT_MESH_edge_color_inner(m_data.y);
    vert_out.select_override = 1u;
  }

  float edge_crease = float(m_data.z & 0xFu) / 15.0;
  float bweight = float(m_data.w) / 255.0;
  vert_out.final_color_outer = EDIT_MESH_edge_color_outer(
      m_data.y, m_data.x, edge_crease, bweight);

  if (vert_out.final_color_outer.a > 0.0) {
    vert_out.gpu_position.z -= 5e-7 * abs(vert_out.gpu_position.w);
  }

  bool occluded = false; /* Done in fragment shader */

#elif defined(FACE)
  vert_out.final_color = EDIT_MESH_face_color(m_data.x);
  bool occluded = true;

#  ifdef GPU_METAL
  /* Apply depth bias to overlay in order to prevent z-fighting on Apple Silicon GPUs. */
  vert_out.gpu_position.z -= 5e-5;
#  endif

#elif defined(FACEDOT)
  vert_out.final_color = EDIT_MESH_facedot_color(.w);

  /* Bias Face-dot Z position in clip-space. */
  vert_out.gpu_position.z -= (drw_view.winmat[3][3] == 0.0) ? 0.00035 : 1e-6;
  gl_PointSize = sizeFaceDot;

  bool occluded = test_occlusion(vert_out.gpu_position);

#endif

  vert_out.final_color.a *= (occluded) ? alpha : 1.0;

#if !defined(FACE)
  /* Facing based color blend */
  vec3 view_normal = normalize(normal_object_to_view(vert_in.lN) + 1e-4);
  vec3 view_vec = (drw_view.winmat[3][3] == 0.0) ? normalize(view_pos) : vec3(0.0, 0.0, 1.0);
  float facing = dot(view_vec, view_normal);
  facing = 1.0 - abs(facing) * 0.2;

  /* Do interpolation in a non-linear space to have a better visual result. */
  vert_out.final_color.rgb = mix(
      vert_out.final_color.rgb,
      non_linear_blend_color(colorEditMeshMiddle.rgb, vert_out.final_color.rgb, facing),
      fresnelMixEdit);
#endif

  view_clipping_distances(world_pos);

  return vert_out;
}

struct GeomOut {
  vec4 gpu_position;
  vec4 final_color;
  float edge_coord;
};

void export_vertex(GeomOut geom_out)
{
  geometry_out.finalColor = geom_out.final_color;
  geometry_noperspective_out.edgeCoord = geom_out.edge_coord;
  gl_Position = geom_out.gpu_position;
}

void strip_EmitVertex(const uint strip_index,
                      uint out_vertex_id,
                      uint out_primitive_id,
                      GeomOut geom_out)
{
  bool is_odd_primitive = (out_primitive_id & 1u) != 0u;
  /* Maps triangle list primitives to triangle strip indices. */
  uint out_strip_index = (is_odd_primitive ? (2u - out_vertex_id) : out_vertex_id) +
                         out_primitive_id;

  if (out_strip_index == strip_index) {
    export_vertex(geom_out);
  }
}

void do_vertex(const uint strip_index,
               uint out_vertex_id,
               uint out_primitive_id,
               vec4 color,
               vec4 pos,
               float coord,
               vec2 offset)
{
  GeomOut geom_out;
  geom_out.final_color = color;
  geom_out.edge_coord = coord;
  geom_out.gpu_position = pos;
  /* Multiply offset by 2 because gl_Position range is [-1..1]. */
  geom_out.gpu_position.xy += offset * 2.0 * pos.w;
  strip_EmitVertex(strip_index, out_vertex_id, out_primitive_id, geom_out);
}

void geometry_main(VertOut geom_in[2],
                   uint out_vertex_id,
                   uint out_primitive_id,
                   uint out_invocation_id)
{
  vec2 ss_pos[2];

  /* Clip line against near plane to avoid deformed lines. */
  vec4 pos0 = geom_in[0].gpu_position;
  vec4 pos1 = geom_in[1].gpu_position;
  vec2 pz_ndc = vec2(pos0.z / pos0.w, pos1.z / pos1.w);
  bvec2 clipped = lessThan(pz_ndc, vec2(-1.0));
  if (all(clipped)) {
    /* Totally clipped. */
    return;
  }

  vec4 pos01 = pos0 - pos1;
  float ofs = abs((pz_ndc.y + 1.0) / (pz_ndc.x - pz_ndc.y));
  if (clipped.y) {
    pos1 += pos01 * ofs;
  }
  else if (clipped.x) {
    pos0 -= pos01 * (1.0 - ofs);
  }

  ss_pos[0] = pos0.xy / pos0.w;
  ss_pos[1] = pos1.xy / pos1.w;

  vec2 line = ss_pos[0] - ss_pos[1];
  line = abs(line) * sizeViewport.xy;

  geometry_flat_out.finalColorOuter = geom_in[0].final_color_outer;
  float half_size = sizeEdge;
  /* Enlarge edge for flag display. */
  half_size += (geometry_flat_out.finalColorOuter.a > 0.0) ? max(sizeEdge, 1.0) : 0.0;

  if (do_smooth_wire) {
    /* Add 1px for AA */
    half_size += 0.5;
  }

  vec3 edge_ofs = vec3(half_size * sizeViewportInv, 0.0);

  bool horizontal = line.x > line.y;
  edge_ofs = (horizontal) ? edge_ofs.zyz : edge_ofs.xzz;

  /* Due to an AMD glitch, this line was moved out of the `do_vertex`
   * function (see #62792). */
  // view_clipping_distances_set(geom_in[0]);
  do_vertex(
      0, out_vertex_id, out_primitive_id, geom_in[0].final_color, pos0, half_size, edge_ofs.xy);
  do_vertex(
      1, out_vertex_id, out_primitive_id, geom_in[0].final_color, pos0, -half_size, -edge_ofs.xy);

  // view_clipping_distances_set(geom_in[1]);
  vec4 final_color = (geom_in[0].select_override == 0u) ? geom_in[1].final_color :
                                                          geom_in[0].final_color;
  do_vertex(2, out_vertex_id, out_primitive_id, final_color, pos1, half_size, edge_ofs.xy);
  do_vertex(3, out_vertex_id, out_primitive_id, final_color, pos1, -half_size, -edge_ofs.xy);
}

void main()
{
#if defined(EDGE)
  /* Edges generate a triangle strip per input edge.
   * This allows to draw edges with anti-aliasing and with variable size. */

  /* Line list primitive. */
  const uint input_primitive_vertex_count = 2u;
  /* Triangle list primitive. */
  const uint ouput_primitive_vertex_count = 3u;
  const uint ouput_primitive_count = 2u;
  const uint ouput_invocation_count = 1u;
  const uint output_vertex_count_per_invocation = ouput_primitive_count *
                                                  ouput_primitive_vertex_count;
  const uint output_vertex_count_per_input_primitive = output_vertex_count_per_invocation *
                                                       ouput_invocation_count;

  uint in_primitive_id = uint(gl_VertexID) / output_vertex_count_per_input_primitive;
  uint in_primitive_first_vertex = in_primitive_id * input_primitive_vertex_count;

  uint out_vertex_id = uint(gl_VertexID) % ouput_primitive_vertex_count;
  uint out_primitive_id = (uint(gl_VertexID) / ouput_primitive_vertex_count) %
                          ouput_primitive_count;
  uint out_invocation_id = (uint(gl_VertexID) / output_vertex_count_per_invocation) %
                           ouput_invocation_count;

  VertIn vert_in[input_primitive_vertex_count];
  vert_in[0] = input_assembly(in_primitive_first_vertex + 0u);
  vert_in[1] = input_assembly(in_primitive_first_vertex + 1u);

  VertOut vert_out[input_primitive_vertex_count];
  vert_out[0] = vertex_main(vert_in[0]);
  vert_out[1] = vertex_main(vert_in[1]);

  /* Discard by default. */
  gl_Position = vec4(NAN_FLT);
  geometry_main(vert_out, out_vertex_id, out_primitive_id, out_invocation_id);
#else
  /* TODO(fclem): Bypass vertex pull in this case. */
  VertIn vert_in = input_assembly(0);

  /* Vertex, Facedot and Face case. */
  VertOut vert_out = vertex_main(vert_in);

  gl_Position = vert_out.gpu_position;
#endif
}
