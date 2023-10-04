/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Apply lights contribution to scene surfel representation.
 */

#pragma BLENDER_REQUIRE(eevee_light_eval_lib.glsl)

void main()
{
  int index = int(gl_GlobalInvocationID.x);
  if (index >= int(capture_info_buf.surfel_len)) {
    return;
  }

  Surfel surfel = surfel_buf[index];

  ClosureLight cl_diff;
  cl_diff.N = surfel.normal;
  cl_diff.ltc_mat = LTC_LAMBERT_MAT;
  cl_diff.type = LIGHT_DIFFUSE;

  /* There is no view dependent effect as we evaluate everything using diffuse. */
  vec3 V = surfel.normal;
  vec3 Ng = surfel.normal;
  vec3 P = surfel.position;
  float unused_vPz = 0.0;
  float unused_thickness = 0.0;
  light_eval(cl_diff, P, Ng, V, unused_vPz, unused_thickness);

  if (capture_info_buf.capture_indirect) {
    surfel_buf[index].radiance_direct.front.rgb += cl_diff.light_shadowed * surfel.albedo_front;
  }

  light_eval(cl_diff, P, Ng, V, unused_vPz, unused_thickness);

  if (capture_info_buf.capture_indirect) {
    surfel_buf[index].radiance_direct.back.rgb += cl_diff.light_shadowed * surfel.albedo_back;
  }
}
