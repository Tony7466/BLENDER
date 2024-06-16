/* SPDX-FileCopyrightText: 2021-2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* To be compiled with common_subdiv_lib.glsl */

layout(std430, binding = 0) readonly buffer inputUVs
{
  vec2 uvs[];
};

layout(std430, binding = 2) writeonly buffer outputNormals
{
  vec3 uv_normals[];
};

void main()
{
  /* We execute for each quad. */
  uint quad_index = get_global_invocation_index();
  if (quad_index >= total_dispatch_size) {
    return;
  }

  /* The start index of the loop is quad_index * 4. */
  uint start_loop_index = quad_index * 4;

  uint triangle_loop_index = quad_index * 6;

  uint loop_0 = start_loop_index;
  uint loop_1 = start_loop_index + 1;
  uint loop_2 = start_loop_index + 2;
  uint loop_3 = start_loop_index + 3;

  vec2 luv_0= uvs[src_offset + loop_0];
  vec2 luv_1= uvs[src_offset + loop_1];
  vec2 luv_2= uvs[src_offset + loop_2];
  vec2 luv_3= uvs[src_offset + loop_3];

  vec3 e1 = vec3(luv_0 - luv_1, 0.0);
  vec3 e2 = vec3(luv_2 - luv_0, 0.0);
  vec3 e3 = vec3(luv_3 - luv_2, 0.0);

  vec3 normal_1 = normalize(cross(e1, e2));
  vec3 normal_2 = normalize(cross(e2, e3));

  uv_normals[loop_1] = normal_1;
  uv_normals[loop_3] = normal_2;
  if (normal_1.z > 0.0) {
    uv_normals[loop_0] = normal_1;
    uv_normals[loop_2] = normal_1;
  }
  else {
    uv_normals[loop_0] = normal_2;
    uv_normals[loop_2] = normal_2;

  }
}