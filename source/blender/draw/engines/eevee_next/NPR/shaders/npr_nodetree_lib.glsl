/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma BLENDER_REQUIRE(draw_view_lib.glsl)

vec4 g_combined_color;

vec4 nodetree_npr();

/* -------------------------------------------------------------------- */
/** \name Coordinate implementations
 *
 * Callbacks for the texture coordinate node.
 *
 * \{ */

vec3 coordinate_camera(vec3 P)
{
  vec3 vP = drw_point_world_to_view(P);
  vP.z = -vP.z;
  return vP;
}

vec3 coordinate_screen(vec3 P)
{
  vec3 window = vec3(0.0);
  /* TODO(fclem): Actual camera transform. */
  window.xy = drw_point_world_to_screen(P).xy;
  // window.xy = window.xy * uniform_buf.camera.uv_scale + uniform_buf.camera.uv_bias;
  return window;
}

vec3 coordinate_reflect(vec3 P, vec3 N)
{
  return -reflect(drw_world_incident_vector(P), N);
}

vec3 coordinate_incoming(vec3 P)
{
  return drw_world_incident_vector(P);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Mixed render resolution
 *
 * Callbacks image texture sampling.
 *
 * \{ */

float texture_lod_bias_get()
{
  // return uniform_buf.film.texture_lod_bias;
  return 0.0;
}

/** \} */
