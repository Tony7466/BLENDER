/* SPDX-FileCopyrightText: 2018-2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef DRAW_VIEW_CREATE_INFO
#  error Missing draw_view additional create info on shader create info
#endif

/* Returns true if the current view has a perspective projection matrix. */
bool drw_view_is_perspective()
{
  return drw_view.winmat[3][3] == 0.0;
}

/* Returns the view forward vector, going towards the viewer. */
vec3 drw_view_forward()
{
  return drw_view.viewinv[2].xyz;
}

/* Returns the view origin. */
vec3 drw_view_position()
{
  return drw_view.viewinv[3].xyz;
}

/**
 * Return the world view vector `V` (going towards the viewer)
 * from the world position `P` and the current view.
 */
vec3 drw_view_vector_get(vec3 P)
{
  return drw_view_is_perspective() ? normalize(drw_view_position() - P) : drw_view_forward();
}

/* -------------------------------------------------------------------- */
/** \name Transform Normal
 *
 * Space conversion helpers for normal vectors.
 * \{ */

vec3 drw_normal_view_to_world(vec3 vN)
{
  return (mat3x3(drw_view.viewinv) * vN);
}

vec3 drw_normal_world_to_view(vec3 N)
{
  return (mat3x3(drw_view.viewmat) * N);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Transform Normal
 *
 * Space conversion helpers for points (coordinates).
 * \{ */

vec3 drw_point_homogenous_to_ndc(vec4 hs_P)
{
  return hs_P.xyz / hs_P.w;
}

vec3 drw_point_view_to_world(vec3 vP)
{
  return (drw_view.viewinv * vec4(vP, 1.0)).xyz;
}
vec4 drw_point_view_to_homogenous(vec3 vP)
{
  return (drw_view.winmat * vec4(vP, 1.0));
}
vec3 drw_point_view_to_ndc(vec3 vP)
{
  return drw_point_homogenous_to_ndc(drw_point_view_to_homogenous(vP));
}

vec3 drw_point_world_to_view(vec3 P)
{
  return (drw_view.viewmat * vec4(P, 1.0)).xyz;
}
vec4 drw_point_world_to_homogenous(vec3 P)
{
  return (drw_view.viewmat * (drw_view.viewmat * vec4(P, 1.0)));
}
vec3 drw_point_world_to_ndc(vec3 P)
{
  return drw_point_homogenous_to_ndc(drw_point_world_to_homogenous(P));
}

/** \} */
