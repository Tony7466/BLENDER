/* SPDX-FileCopyrightText: 2019-2022 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma USE_SSBO_VERTEX_FETCH(TriangleList, 6)
#pragma BLENDER_REQUIRE(common_view_clipping_lib.glsl)
#pragma BLENDER_REQUIRE(common_view_lib.glsl)

/* "geometry" shader area */

/* Clips point to near clip plane before perspective divide. */
vec4 clip_line_point_homogeneous_space(vec4 p, vec4 q)
{
  if (p.z < -p.w) {
    /* Just solves p + (q - p) * A; for A when p.z / p.w = -1.0. */
    float denom = q.z - p.z + q.w - p.w;
    if (denom == 0.0) {
      /* No solution. */
      return p;
    }
    float A = (-p.z - p.w) / denom;
    p = p + (q - p) * A;
  }
  return p;
}

void do_vertex(int index, vec4 pos, vec2 ofs, float flip)
{
  interp_noperspective.smoothline = flip * (LINE_WIDTH + SMOOTH_WIDTH) * 0.5;
  gl_Position = pos;
  gl_Position.xy += flip * ofs * pos.w;
}

/* "veretex" shader area */

bool test_occlusion()
{
  vec3 ndc = (gl_Position.xyz / gl_Position.w) * 0.5 + 0.5;
  return (ndc.z - 0.00035) > texture(depthTex, ndc.xy).r;
}

vec4 vertex_main(uint src_index
                 vec3 in_pos,
                 vec4 in_lnor,
                 vec4 in_vnor,
                 vec4 in_norAndFlag)
{
  /* Avoid undefined behavior after return. */
  interp.final_color = vec4(0.0);
  gl_Position = vec4(0.0);

  vec3 nor;
  /* Select the right normal by checking if the generic attribute is used. */
  if (!all(equal(in_lnor.xyz, vec3(0)))) {
    if (in_lnor.w < 0.0) {
      return;
    }
    nor = in_lnor.xyz;
    interp.final_color = colorLNormal;
  }
  else if (!all(equal(in_vnor.xyz, vec3(0)))) {
    if (in_vnor.w < 0.0) {
      return;
    }
    nor = in_vnor.xyz;
    interp.final_color = colorVNormal;
  }
  else {
    nor = in_norAndFlag.xyz;
    if (all(equal(nor, vec3(0)))) {
      return;
    }
    interp.final_color = colorNormal;
  }

  vec3 n = normalize(normal_object_to_world(nor));
  vec3 world_pos = point_object_to_world(in_pos);

  if (src_index == 0) {
    if (isConstantScreenSizeNormals) {
      bool is_persp = (drw_view.winmat[3][3] == 0.0);
      if (is_persp) {
        float dist_fac = length(cameraPos - world_pos);
        float cos_fac = dot(cameraForward, cameraVec(world_pos));
        world_pos += n * normalScreenSize * dist_fac * cos_fac * pixelFac * sizePixel;
      }
      else {
        float frustrum_fac = mul_project_m4_v3_zfac(n) * sizePixel;
        world_pos += n * normalScreenSize * frustrum_fac;
      }
    }
    else {
      world_pos += n * normalSize;
    }
  }

  interp.final_color.a *= (test_occlusion()) ? alpha : 1.0;
  view_clipping_distances(world_pos);

  return point_world_to_ndc(world_pos);
}

/* Real main */

void main()
{
  /* Index of the quad primitive. Each quad corresponds to one line in the input primitive. */
  int quad_id = gl_VertexID / 6;
  /* Determine vertex ID. */
  int quad_vertex_id = gl_VertexID % 6;

  /* Get current point attributes. */
  uint src_index_a = 0;
  uint src_index_b = 1;

  vec3 in_pos[2];
  in_pos[0] = vertex_fetch_attribute(src_index_a, pos, vec3);
  in_pos[1] = vertex_fetch_attribute(src_index_b, pos, vec3);

  vec4 in_lnor[2];
  in_lnor[0] = vertex_fetch_attribute(src_index_a, lnor, vec4);
  in_lnor[1] = vertex_fetch_attribute(src_index_b, lnor, vec4);

  vec4 in_vnor[2];
  in_vnor[0] = vertex_fetch_attribute(src_index_a, vnor, vec4);
  in_vnor[1] = vertex_fetch_attribute(src_index_b, vnor, vec4);

  vec4 in_norAndFlag[2];
  in_norAndFlag[0] = vertex_fetch_attribute(src_index_a, norAndFlag, vec4);
  in_norAndFlag[1] = vertex_fetch_attribute(src_index_b, norAndFlag, vec4);

  /* Convert to ndc space. Vertex shader main area. */
  vec4 out_pos[2];
  out_pos[0] = vert_main(src_index_a, in_pos[0], in_lnor[0], in_vnor[0], in_norAndFlag[0]);
  out_pos[1] = vert_main(src_index_b, in_pos[1], in_lnor[1], in_vnor[1], in_norAndFlag[1]);

  /* Geometry shader main area. */
  vec4 p[2];
  vec4 p[0] = clip_line_point_homogeneous_space(out_pos[0], out_pos[1]);
  vec4 p[1] = clip_line_point_homogeneous_space(out_pos[1], out_pos[0]);
  vec2 e = normalize(((p[1].xy / p[1].w) - (p[0].xy / p[0].w)) * viewportSize.xy);

#if 0 /* Hard turn when line direction changes quadrant. */
  e = abs(e);
  vec2 ofs = (e.x > e.y) ? vec2(0.0, 1.0 / e.x) : vec2(1.0 / e.y, 0.0);
#else /* Use perpendicular direction. */
  vec2 ofs = vec2(-e.y, e.x);
#endif

  ofs /= viewportSize.xy;
  ofs *= LINE_WIDTH + SMOOTH_WIDTH;

  if (quad_vertex_id == 0) {
    do_vertex(0, p[0], ofs, 1.0);
  }
  else if (quad_vertex_id == 1 || quad_vertex_id == 3) {
    do_vertex(0, p[0], ofs, -1.0);
  }
  else if (quad_vertex_id == 2 || quad_vertex_id == 5) {
    do_vertex(1, p[1], ofs, 1.0);
  }
  else if (quad_vertex_id == 4) {
    do_vertex(1, p[1], ofs, -1.0);
  }
}
