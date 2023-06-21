/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "kernel/light/common.h"

CCL_NAMESPACE_BEGIN

ccl_device float spot_light_attenuation(const ccl_global KernelSpotLight *spot, float3 ray)
{
  const float3 scaled_ray = safe_normalize(make_float3(dot(ray, spot->scaled_axis_u),
                                                       dot(ray, spot->scaled_axis_v),
                                                       dot(ray, spot->dir * spot->inv_len_z)));

  return smoothstepf((scaled_ray.z - spot->cos_half_spot_angle) * spot->spot_smooth);
}

/* TODO: is attenuation needed inside the sphere? */
/* TODO: sampling valid cone. */
/* TODO: what's the expected behaviour of texturing? */
template<bool in_volume_segment>
ccl_device_inline bool spot_light_sample(const ccl_global KernelLight *klight,
                                         const float2 rand,
                                         const float3 P,
                                         const float3 N,
                                         const int shader_flags,
                                         ccl_private LightSample *ls)
{
  point_light_sample(klight, rand, P, N, shader_flags, ls);

  /* Spot light attenuation. */
  ls->eval_fac *= spot_light_attenuation(&klight->spot, -ls->D);
  if (!in_volume_segment && ls->eval_fac == 0.0f) {
    return false;
  }

  return true;
}

ccl_device_forceinline void spot_light_mnee_sample_update(const ccl_global KernelLight *klight,
                                                          ccl_private LightSample *ls,
                                                          const float3 P,
                                                          const float3 N,
                                                          const uint32_t path_flag)
{
  point_light_mnee_sample_update(klight, ls, P, N, path_flag);

  /* Spot light attenuation. */
  ls->eval_fac *= spot_light_attenuation(&klight->spot, ls->Ng);
}

ccl_device_inline bool spot_light_intersect(const ccl_global KernelLight *klight,
                                            const ccl_private Ray *ccl_restrict ray,
                                            ccl_private float *t)
{
  /* One sided. */
  if (dot(ray->D, ray->P - klight->co) >= 0.0f) {
    return false;
  }

  /* TODO: when to check if outside cone? */
  return point_light_intersect(klight, ray, t);
}

ccl_device_inline bool spot_light_sample_from_intersection(
    const ccl_global KernelLight *klight,
    ccl_private const Intersection *ccl_restrict isect,
    const float3 ray_P,
    const float3 ray_D,
    const float3 N,
    const uint32_t path_flag,
    ccl_private LightSample *ccl_restrict ls)
{
  point_light_sample_from_intersection(klight, isect, ray_P, ray_D, N, path_flag, ls);

  /* Spot light attenuation. */
  ls->eval_fac *= spot_light_attenuation(&klight->spot, -ls->D);

  return (ls->eval_fac > 0);
}

template<bool in_volume_segment>
ccl_device_forceinline bool spot_light_tree_parameters(const ccl_global KernelLight *klight,
                                                       const float3 centroid,
                                                       const float3 P,
                                                       ccl_private float &cos_theta_u,
                                                       ccl_private float2 &distance,
                                                       ccl_private float3 &point_to_centroid)
{
  float dist_point_to_centroid;
  const float3 point_to_centroid_ = safe_normalize_len(centroid - P, &dist_point_to_centroid);

  const float radius = klight->spot.radius;
  cos_theta_u = (dist_point_to_centroid > radius) ? cos_from_sin(radius / dist_point_to_centroid) :
                                                    -1.0f;

  if (in_volume_segment) {
    return true;
  }

  distance = (dist_point_to_centroid > radius) ?
                 dist_point_to_centroid * make_float2(1.0f / cos_theta_u, 1.0f) :
                 one_float2() * radius / M_SQRT2_F;
  point_to_centroid = point_to_centroid_;

  return true;
}

CCL_NAMESPACE_END
