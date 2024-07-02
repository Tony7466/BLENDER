/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "kernel/bvh/bvh.h"
#include "kernel/integrator/path_state.h"
#include "kernel/integrator/subsurface_disk.h"
#include "kernel/integrator/subsurface_random_walk.h"
#include "kernel/integrator/volume_stack.h"

CCL_NAMESPACE_BEGIN

#ifdef __VOLUME__
ccl_device void integrator_volume_stack_update_for_subsurface(KernelGlobals kg,
                                                              IntegratorState state,
                                                              const float3 from_P,
                                                              const float3 to_P)
{
  PROFILING_INIT(kg, PROFILING_INTERSECT_VOLUME_STACK);

  ShaderDataTinyStorage stack_sd_storage;
  ccl_private ShaderData *stack_sd = AS_SHADER_DATA(&stack_sd_storage);

  kernel_assert(kernel_data.integrator.use_volumes);

  Ray volume_ray ccl_optional_struct_init;
  volume_ray.P = from_P;
  volume_ray.D = normalize_len(to_P - from_P, &volume_ray.tmax);
  volume_ray.tmin = 0.0f;
  volume_ray.self.object = INTEGRATOR_STATE(state, isect, object);
  volume_ray.self.prim = INTEGRATOR_STATE(state, isect, prim);
  volume_ray.self.light_object = OBJECT_NONE;
  volume_ray.self.light_prim = PRIM_NONE;
  volume_ray.self.light = LAMP_NONE;
  /* Store to avoid global fetches on every intersection step. */
  const uint volume_stack_size = kernel_data.volume_stack_size;

  const uint32_t path_flag = INTEGRATOR_STATE(state, path, flag);
  const uint32_t visibility = SHADOW_CATCHER_PATH_VISIBILITY(path_flag, PATH_RAY_ALL_VISIBILITY);

#  ifdef __VOLUME_RECORD_ALL__
  Intersection hits[2 * MAX_VOLUME_STACK_SIZE + 1];
  uint num_hits = scene_intersect_volume(kg, &volume_ray, hits, 2 * volume_stack_size, visibility);
  if (num_hits > 0) {
    Intersection *isect = hits;

    qsort(hits, num_hits, sizeof(Intersection), intersections_compare);

    for (uint hit = 0; hit < num_hits; ++hit, ++isect) {
      /* Ignore self, SSS itself already enters and exits the object. */
      if (isect->object == volume_ray.self.object) {
        continue;
      }
      shader_setup_from_ray(kg, stack_sd, &volume_ray, isect);
      volume_stack_enter_exit(kg, state, stack_sd);
    }
  }
#  else
  Intersection isect;
  int step = 0;
  while (step < 2 * volume_stack_size &&
         scene_intersect_volume(kg, &volume_ray, &isect, visibility))
  {
    /* Ignore self, SSS itself already enters and exits the object. */
    if (isect.object != volume_ray.self.object) {
      shader_setup_from_ray(kg, stack_sd, &volume_ray, &isect);
      volume_stack_enter_exit(kg, state, stack_sd);
    }
    /* Move ray forward. */
    volume_ray.tmin = intersection_t_offset(isect.t);
    volume_ray.self.object = isect.object;
    volume_ray.self.prim = isect.prim;
    ++step;
  }
#  endif
}
#endif

#ifdef __SUBSURFACE__
ccl_device_inline bool subsurface_scatter(KernelGlobals kg, IntegratorState state)
{
  RNGState rng_state;
  path_state_rng_load(state, &rng_state);

  Ray ray ccl_optional_struct_init;
  LocalIntersection ss_isect ccl_optional_struct_init;

  if (INTEGRATOR_STATE(state, path, flag) & PATH_RAY_SUBSURFACE_RANDOM_WALK) {
    if (!subsurface_random_walk(kg, state, rng_state, ray, ss_isect)) {
      return false;
    }
  }
  else {
    if (!subsurface_disk(kg, state, rng_state, ray, ss_isect)) {
      return false;
    }
  }

#  ifdef __VOLUME__
  /* Update volume stack if needed. */
  if (kernel_data.integrator.use_volumes) {
    const int object = ss_isect.hits[0].object;
    const int object_flag = kernel_data_fetch(object_flag, object);

    if (object_flag & SD_OBJECT_INTERSECTS_VOLUME) {
      float3 P = INTEGRATOR_STATE(state, ray, P);

      integrator_volume_stack_update_for_subsurface(kg, state, P, ray.P);
    }
  }
#  endif /* __VOLUME__ */

  /* Pretend ray is coming from the outside towards the exit point. This ensures
   * correct front/back facing normals.
   * TODO: find a more elegant solution? */
  ray.P += ray.D * ray.tmax * 2.0f;
  ray.D = -ray.D;

  integrator_state_write_isect(state, &ss_isect.hits[0]);
  integrator_state_write_ray(state, &ray);

  /* Advance random number offset for bounce. */
  INTEGRATOR_STATE_WRITE(state, path, rng_offset) += PRNG_BOUNCE_NUM;

  const int shader = intersection_get_shader(kg, &ss_isect.hits[0]);
  const int shader_flags = kernel_data_fetch(shaders, shader).flags;
  const int object_flags = intersection_get_object_flags(kg, &ss_isect.hits[0]);
  const bool use_caustics = kernel_data.integrator.use_caustics &&
                            (object_flags & SD_OBJECT_CAUSTICS);
  const bool use_raytrace_kernel = (shader_flags & SD_HAS_RAYTRACE);

  if (use_caustics) {
    integrator_path_next_sorted(kg,
                                state,
                                DEVICE_KERNEL_INTEGRATOR_INTERSECT_SUBSURFACE,
                                DEVICE_KERNEL_INTEGRATOR_SHADE_SURFACE_MNEE,
                                shader);
  }
  else if (use_raytrace_kernel) {
    integrator_path_next_sorted(kg,
                                state,
                                DEVICE_KERNEL_INTEGRATOR_INTERSECT_SUBSURFACE,
                                DEVICE_KERNEL_INTEGRATOR_SHADE_SURFACE_RAYTRACE,
                                shader);
  }
  else {
    integrator_path_next_sorted(kg,
                                state,
                                DEVICE_KERNEL_INTEGRATOR_INTERSECT_SUBSURFACE,
                                DEVICE_KERNEL_INTEGRATOR_SHADE_SURFACE,
                                shader);
  }

  return true;
}
#endif

ccl_device void integrator_intersect_subsurface(KernelGlobals kg, IntegratorState state)
{
  PROFILING_INIT(kg, PROFILING_INTERSECT_SUBSURFACE);

#ifdef __SUBSURFACE__
  if (subsurface_scatter(kg, state)) {
    return;
  }
#endif

  integrator_path_terminate(kg, state, DEVICE_KERNEL_INTEGRATOR_INTERSECT_SUBSURFACE);
}

CCL_NAMESPACE_END
