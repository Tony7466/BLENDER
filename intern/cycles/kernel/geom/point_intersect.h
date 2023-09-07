/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#pragma once

CCL_NAMESPACE_BEGIN

/* Point primitive intersection functions. */

#ifdef __POINTCLOUD__

ccl_device_forceinline bool point_intersect(KernelGlobals kg,
                                            ccl_private Intersection *isect,
                                            const float3 ray_P,
                                            const float3 ray_D,
                                            const float ray_tmin,
                                            const float ray_tmax,
                                            const int object,
                                            const int prim,
                                            const float time,
                                            const int type)
{
  const float4 point = (type & PRIMITIVE_MOTION) ? motion_point(kg, object, prim, time) :
                                                   kernel_data_fetch(points, prim);

    float3 discard;
  if (!ray_sphere_intersect(ray_P, ray_D, ray_tmin, ray_tmax, float4_to_float3(point), point.w, &discard, &isect->t, true)) {
      return false;
  }

  isect->prim = prim;
  isect->object = object;
  isect->type = type;
  isect->u = 0.0f;
  isect->v = 0.0f;
  return true;
}

ccl_device_inline void point_shader_setup(KernelGlobals kg,
                                          ccl_private ShaderData *sd,
                                          ccl_private const Intersection *isect,
                                          ccl_private const Ray *ray)
{
  sd->shader = kernel_data_fetch(points_shader, isect->prim);
  sd->P = ray->P + ray->D * isect->t;

  /* Texture coordinates, zero for now. */
#  ifdef __UV__
  sd->u = isect->u;
  sd->v = isect->v;
#  endif

  /* Compute point center for normal. */
  float3 center = float4_to_float3((isect->type & PRIMITIVE_MOTION) ?
                                       motion_point(kg, sd->object, sd->prim, sd->time) :
                                       kernel_data_fetch(points, sd->prim));
  if (!(sd->object_flag & SD_OBJECT_TRANSFORM_APPLIED)) {
    object_position_transform_auto(kg, sd, &center);
  }

  /* Normal */
  sd->Ng = normalize(sd->P - center);
  sd->N = sd->Ng;

#  ifdef __DPDU__
  /* dPdu/dPdv */
  sd->dPdu = make_float3(0.0f, 0.0f, 0.0f);
  sd->dPdv = make_float3(0.0f, 0.0f, 0.0f);
#  endif
}

#endif

CCL_NAMESPACE_END
