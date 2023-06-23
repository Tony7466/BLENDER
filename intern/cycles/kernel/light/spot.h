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
/* TODO: what's the expected behaviour of texturing? */
template<bool in_volume_segment>
ccl_device_inline bool spot_light_sample(const ccl_global KernelLight *klight,
                                         const float2 rand,
                                         const float3 P,
                                         const float3 N,
                                         const int shader_flags,
                                         ccl_private LightSample *ls)
{
  const float radius = klight->spot.radius;
  const float r_sq = sqr(klight->spot.radius);

  const float3 center = klight->co;

  float3 lightN = P - center;
  const float d_sq = len_squared(lightN);
  const float d = sqrtf(d_sq);
  lightN /= d;

  float cos_theta;
  ls->t = FLT_MAX;
  if (d_sq > r_sq) {
    const float one_minus_cos_half_spot_spread = 1.0f - klight->spot.cos_half_spot_angle;
    const float one_minus_cos_half_angle = sin_sqr_to_one_minus_cos(r_sq / d_sq);

    if (in_volume_segment || one_minus_cos_half_angle < one_minus_cos_half_spot_spread) {
      /* Sample visible part of the sphere. */
      sample_uniform_cone_concentric(
          -lightN, one_minus_cos_half_angle, rand, &cos_theta, &ls->D, &ls->pdf);
    }
    else {
      /* Sample spread cone. */
      sample_uniform_cone_concentric(
          -klight->spot.dir, one_minus_cos_half_spot_spread, rand, &cos_theta, &ls->D, &ls->pdf);

      if (!ray_sphere_intersect(P, ls->D, 0.0f, FLT_MAX, center, radius, &ls->P, &ls->t)) {
        /* Sampled direction does not intersect with the light. */
        return false;
      }
    }
  }
  else {
    const bool has_transmission = (shader_flags & SD_BSDF_HAS_TRANSMISSION);
    if (has_transmission) {
      ls->D = sample_uniform_sphere(rand);
      ls->pdf = M_1_2PI_F * 0.5f;
    }
    else {
      sample_cos_hemisphere(N, rand, &ls->D, &ls->pdf);
    }
    cos_theta = -dot(ls->D, lightN);
  }

  if (ls->t == FLT_MAX) {
    /* Law of cosines. */
    ls->t = d * cos_theta -
            copysignf(safe_sqrtf(r_sq - d_sq + d_sq * sqr(cos_theta)), d_sq - r_sq);
    ls->P = P + ls->D * ls->t;
  }
  else {
    /* Already computed when sampling the spread cone. */
  }

  ls->eval_fac = klight->spot.eval_fac * spot_light_attenuation(&klight->spot, -ls->D);
  if (!in_volume_segment && ls->eval_fac == 0.0f) {
    return false;
  }

  if (r_sq == 0) {
    /* Use intensity instead of radiance for point light. */
    ls->eval_fac /= sqr(ls->t);
    /* `ls->Ng` is not well-defined for point light, so use the incoming direction instead.  */
    ls->Ng = -ls->D;
  }
  else {
    ls->Ng = normalize(ls->P - center);
    /* Remap sampled point onto the sphere to prevent precision issues with small radius. */
    ls->P = ls->Ng * radius + center;
  }

  const Transform itfm = klight->itfm;
  const float2 uv = map_to_sphere(transform_direction(&itfm, ls->Ng));
  /* NOTE: Return barycentric coordinates in the same notation as Embree and OptiX. */
  ls->u = uv.y;
  ls->v = 1.0f - uv.x - uv.y;

  return true;
}

ccl_device_forceinline float spot_light_pdf(const float cos_half_spread,
                                            const float d_sq,
                                            const float r_sq,
                                            const float3 N,
                                            const float3 D,
                                            const uint32_t path_flag)
{
  if (d_sq > r_sq) {
    return M_1_2PI_F / min(sin_sqr_to_one_minus_cos(r_sq / d_sq), 1.0f - cos_half_spread);
  }

  const bool has_transmission = (path_flag & PATH_RAY_MIS_HAD_TRANSMISSION);
  return has_transmission ? M_1_2PI_F * 0.5f : pdf_cos_hemisphere(N, D);
}

ccl_device_forceinline void spot_light_mnee_sample_update(const ccl_global KernelLight *klight,
                                                          ccl_private LightSample *ls,
                                                          const float3 P,
                                                          const float3 N,
                                                          const uint32_t path_flag)
{
  ls->D = normalize_len(ls->P - P, &ls->t);

  const float radius = klight->spot.radius;

  if (radius > 0) {
    const float d_sq = len_squared(P - klight->co);
    const float r_sq = sqr(radius);
    const float t_sq = sqr(ls->t);

    ls->pdf = spot_light_pdf(klight->spot.cos_half_spot_angle, d_sq, r_sq, N, ls->D, path_flag);

    /* NOTE : preserve pdf in area measure. */
    ls->pdf *= 0.5f * fabsf(d_sq - r_sq - t_sq) / (radius * ls->t * t_sq);

    ls->Ng = normalize(ls->P - klight->co);
  }
  else {
    ls->Ng = -ls->D;

    /* PDF does not change. */
  }

  const Transform itfm = klight->itfm;
  const float2 uv = map_to_sphere(transform_direction(&itfm, ls->Ng));
  /* NOTE: Return barycentric coordinates in the same notation as Embree and OptiX. */
  ls->u = uv.y;
  ls->v = 1.0f - uv.x - uv.y;

  ls->eval_fac = klight->spot.eval_fac * spot_light_attenuation(&klight->spot, -ls->D);
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
  ls->eval_fac = klight->spot.eval_fac * spot_light_attenuation(&klight->spot, -ray_D);
  if (ls->eval_fac == 0) {
    return false;
  }

  const float radius = klight->spot.radius;

  ls->Ng = radius > 0 ? normalize(ls->P - klight->co) : -ray_D;

  const Transform itfm = klight->itfm;
  const float2 uv = map_to_sphere(transform_direction(&itfm, ls->Ng));
  /* NOTE: Return barycentric coordinates in the same notation as Embree and OptiX. */
  ls->u = uv.y;
  ls->v = 1.0f - uv.x - uv.y;

  if (ls->t == FLT_MAX) {
    ls->pdf = 0.0f;
  }
  else {
    ls->pdf = spot_light_pdf(klight->spot.cos_half_spot_angle,
                             len_squared(ray_P - klight->co),
                             sqr(radius),
                             N,
                             ray_D,
                             path_flag);
  }

  return true;
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
