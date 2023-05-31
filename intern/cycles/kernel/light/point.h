/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include "kernel/light/common.h"

CCL_NAMESPACE_BEGIN

template<bool in_volume_segment>
ccl_device_inline bool point_light_sample(const ccl_global KernelLight *klight,
                                          const float2 rand,
                                          const float3 P,
                                          ccl_private LightSample *ls)
{
  float3 lightN = P - klight->co;
  const float d_sqr = len_squared(lightN);
  const float d = sqrtf(d_sqr);
  lightN /= d;

  const float r_sqr = sqr(klight->spot.radius);

  float cos_theta;
  if (d_sqr > r_sqr) {
    const float one_minus_cos = sin_sqr_to_one_minus_cos(r_sqr / d_sqr);
    sample_uniform_cone_concentric(lightN, one_minus_cos, rand, &cos_theta, &ls->Ng, &ls->pdf);
  }
  else {
    ls->Ng = sample_uniform_sphere(rand);
    ls->pdf = M_1_2PI_F * 0.5f;
    cos_theta = dot(ls->Ng, lightN);
  }

  /* We set the light normal to the outgoing direction to support texturing. */
  ls->D = -ls->Ng;

  /* Law of cosines. */
  ls->t = d * cos_theta -
          copysignf(safe_sqrtf(r_sqr - d_sqr + d_sqr * sqr(cos_theta)), d_sqr - r_sqr);

  ls->P = P + ls->D * ls->t;

  /* TODO: change invarea to that of a sphere. */
  ls->eval_fac = M_1_PI_F * 0.25f * klight->spot.invarea;
  if (r_sqr == 0) {
    /* Use intensity instead of radiance for point light. */
    ls->eval_fac /= sqr(ls->t);
  }

  const float2 uv = map_to_sphere(ls->Ng);
  ls->u = uv.x;
  ls->v = uv.y;

  return true;
}

/* TODO: could there be visibility problem? */
ccl_device_forceinline void point_light_mnee_sample_update(const ccl_global KernelLight *klight,
                                                           ccl_private LightSample *ls,
                                                           const float3 P)
{
  ls->D = normalize_len(ls->P - P, &ls->t);
  ls->Ng = -ls->D;

  const float2 uv = map_to_sphere(ls->Ng);
  ls->u = uv.x;
  ls->v = uv.y;

  const float radius = klight->spot.radius;

  if (radius > 0) {
    const float d_sqr = len_squared(P - klight->co);
    const float r_sqr = sqr(radius);
    const float t_sqr = sqr(ls->t);

    ls->pdf = (d_sqr > r_sqr) ? M_1_2PI_F / sin_sqr_to_one_minus_cos(r_sqr / d_sqr) :
                                M_1_2PI_F * 0.5f;
    /* NOTE : preserve pdf in area measure. */
    ls->pdf *= 0.5f * fabsf(d_sqr - r_sqr - t_sqr) / (radius * ls->t * t_sqr);
  }
  else {
    ls->eval_fac = M_1_PI_F * 0.25f * klight->spot.invarea;
    /* PDF does not change. */
  }
}

ccl_device_inline bool point_light_intersect(const ccl_global KernelLight *klight,
                                             const ccl_private Ray *ccl_restrict ray,
                                             ccl_private float *t)
{
  const float radius = klight->spot.radius;
  if (radius == 0.0f) {
    return false;
  }

  float3 P;
  return ray_sphere_intersect(ray->P, ray->D, ray->tmin, ray->tmax, klight->co, radius, &P, t);
}

ccl_device_inline bool point_light_sample_from_intersection(
    const ccl_global KernelLight *klight,
    ccl_private const Intersection *ccl_restrict isect,
    const float3 ray_P,
    const float3 ray_D,
    ccl_private LightSample *ccl_restrict ls)
{
  /* We set the light normal to the outgoing direction to support texturing. */
  ls->Ng = -ls->D;

  ls->eval_fac = (0.25f * M_1_PI_F) * klight->spot.invarea;

  const float2 uv = map_to_sphere(ls->Ng);
  ls->u = uv.x;
  ls->v = uv.y;

  if (ls->t == FLT_MAX) {
    ls->pdf = 0.0f;
  }
  else {
    const float d_sqr = len_squared(ray_P - klight->co);
    const float r_sqr = sqr(klight->spot.radius);
    ls->pdf = (d_sqr > r_sqr) ? M_1_2PI_F / sin_sqr_to_one_minus_cos(r_sqr / d_sqr) :
                                M_1_2PI_F * 0.5f;
  }

  return true;
}

template<bool in_volume_segment>
ccl_device_forceinline bool point_light_tree_parameters(const ccl_global KernelLight *klight,
                                                        const float3 centroid,
                                                        const float3 P,
                                                        ccl_private float &cos_theta_u,
                                                        ccl_private float2 &distance,
                                                        ccl_private float3 &point_to_centroid)
{
  if (in_volume_segment) {
    /* TODO: does this still hold? */
    cos_theta_u = 1.0f; /* Any value in [-1, 1], irrelevant since theta = 0 */
    return true;
  }

  float dist_point_to_centroid;
  point_to_centroid = safe_normalize_len(centroid - P, &dist_point_to_centroid);

  const float radius = klight->spot.radius;
  if (dist_point_to_centroid > radius) {
    /* Equivalent to a disk light with the same angular span. */
    const float max_dist = sqrtf(sqr(dist_point_to_centroid) - sqr(radius));
    cos_theta_u = max_dist / dist_point_to_centroid;
    distance = make_float2(dist_point_to_centroid, max_dist);
  }
  else {
    /* Similar to background light. */
    cos_theta_u = -1.0f;
    /* HACK: pack radiance scaling in the distance. */
    distance = one_float2() * radius / M_SQRT2_F;
  }

  return true;
}

CCL_NAMESPACE_END
