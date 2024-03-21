/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "kernel/bvh/util.h"
#include "kernel/film/light_passes.h"
#include "kernel/integrator/state_flow.h"
#include "kernel/integrator/state_util.h"

CCL_NAMESPACE_BEGIN

/* Check whether current surface bounce is where path is to be split for the shadow catcher. */
ccl_device_inline bool kernel_shadow_catcher_is_path_split_bounce(KernelGlobals kg,
                                                                  IntegratorState state,
                                                                  const int object_flag)
{
#ifdef __SHADOW_CATCHER__
  if (!kernel_data.integrator.has_shadow_catcher) {
    return false;
  }

  /* Check the flag first, avoiding fetches form global memory. */
  if ((object_flag & SD_OBJECT_SHADOW_CATCHER) == 0) {
    return false;
  }
  if (object_flag & SD_OBJECT_HOLDOUT_MASK) {
    return false;
  }

  const uint32_t path_flag = INTEGRATOR_STATE(state, path, flag);

  if ((path_flag & PATH_RAY_TRANSPARENT_BACKGROUND) == 0) {
    /* Split only on primary rays, secondary bounces are to treat shadow catcher as a regular
     * object. */
    return false;
  }

  if (path_flag & PATH_RAY_SHADOW_CATCHER_HIT) {
    return false;
  }

  return true;
#else
  (void)object_flag;
  return false;
#endif
}

/* Check whether the current path can still split. */
ccl_device_inline bool kernel_shadow_catcher_path_can_split(KernelGlobals kg,
                                                            ConstIntegratorState state)
{
  if (integrator_path_is_terminated(state)) {
    return false;
  }

  const uint32_t path_flag = INTEGRATOR_STATE(state, path, flag);

  if (path_flag & PATH_RAY_SHADOW_CATCHER_HIT) {
    /* Shadow catcher was already hit and the state was split. No further split is allowed. */
    return false;
  }

  return (path_flag & PATH_RAY_TRANSPARENT_BACKGROUND) != 0;
}

#ifdef __SHADOW_CATCHER__

/* Schedule next kernel to be executed after updating volume stack for shadow catcher. */
template<DeviceKernel current_kernel>
ccl_device_forceinline void integrator_intersect_next_kernel_after_shadow_catcher_volume(
    KernelGlobals kg, IntegratorState state)
{
  /* Continue with shading shadow catcher surface. Same as integrator_split_shadow_catcher, but
   * using NEXT instead of INIT. */
  Intersection isect ccl_optional_struct_init;
  integrator_state_read_isect(state, &isect);

  const int shader = intersection_get_shader(kg, &isect);
  const int flags = kernel_data_fetch(shaders, shader).flags;
  const int object_flags = intersection_get_object_flags(kg, &isect);
  const bool use_caustics = kernel_data.integrator.use_caustics &&
                            (object_flags & SD_OBJECT_CAUSTICS);
  const bool use_raytrace_kernel = (flags & SD_HAS_RAYTRACE);

  if (use_caustics) {
    integrator_path_next_sorted(
        kg, state, current_kernel, DEVICE_KERNEL_INTEGRATOR_SHADE_SURFACE_MNEE, shader);
  }
  else if (use_raytrace_kernel) {
    integrator_path_next_sorted(
        kg, state, current_kernel, DEVICE_KERNEL_INTEGRATOR_SHADE_SURFACE_RAYTRACE, shader);
  }
  else {
    integrator_path_next_sorted(
        kg, state, current_kernel, DEVICE_KERNEL_INTEGRATOR_SHADE_SURFACE, shader);
  }
}

/* Schedule next kernel to be executed after executing background shader for shadow catcher. */
template<DeviceKernel current_kernel>
ccl_device_forceinline void integrator_intersect_next_kernel_after_shadow_catcher_background(
    KernelGlobals kg, IntegratorState state)
{
  /* Same logic as integrator_split_shadow_catcher, but using NEXT instead of INIT. */
  if (!integrator_state_volume_stack_is_empty(kg, state)) {
    /* Volume stack is not empty. Re-init the volume stack to exclude any non-shadow catcher
     * objects from it, and then continue shading volume and shadow catcher surface after. */
    integrator_path_next(
        kg, state, current_kernel, DEVICE_KERNEL_INTEGRATOR_INTERSECT_VOLUME_STACK);
    return;
  }

  /* Continue with shading shadow catcher surface. */
  integrator_intersect_next_kernel_after_shadow_catcher_volume<current_kernel>(kg, state);
}

/* Split path if a shadow catcher was hit. */
ccl_device_forceinline void integrator_split_shadow_catcher(
    KernelGlobals kg,
    IntegratorState state,
    ccl_private const Intersection *ccl_restrict isect,
    ccl_global float *ccl_restrict render_buffer)
{
  /* Test if we hit a shadow catcher object, and potentially split the path to continue tracing two
   * paths from here. */
  const int object_flags = intersection_get_object_flags(kg, isect);
  if (!kernel_shadow_catcher_is_path_split_bounce(kg, state, object_flags)) {
    return;
  }

  film_write_shadow_catcher_bounce_data(kg, state, render_buffer);

  /* Mark state as having done a shadow catcher split so that it stops contributing to
   * the shadow catcher matte pass, but keeps contributing to the combined pass. */
  INTEGRATOR_STATE_WRITE(state, path, flag) |= PATH_RAY_SHADOW_CATCHER_HIT;

  /* Copy current state to new state. */
  state = integrator_state_shadow_catcher_split(kg, state);

  /* Initialize new state.
   *
   * Note that the splitting leaves kernel and sorting counters as-is, so use INIT semantic for
   * the matte path. */

  /* Mark current state so that it will only track contribution of shadow catcher objects ignoring
   * non-catcher objects. */
  INTEGRATOR_STATE_WRITE(state, path, flag) |= PATH_RAY_SHADOW_CATCHER_PASS;

  if (kernel_data.film.pass_background != PASS_UNUSED && !kernel_data.background.transparent) {
    /* If using background pass, schedule background shading kernel so that we have a background
     * to alpha-over on. The background kernel will then continue the path afterwards. */
    INTEGRATOR_STATE_WRITE(state, path, flag) |= PATH_RAY_SHADOW_CATCHER_BACKGROUND;
    integrator_path_init(kg, state, DEVICE_KERNEL_INTEGRATOR_SHADE_BACKGROUND);
    return;
  }

  if (!integrator_state_volume_stack_is_empty(kg, state)) {
    /* Volume stack is not empty. Re-init the volume stack to exclude any non-shadow catcher
     * objects from it, and then continue shading volume and shadow catcher surface after. */
    integrator_path_init(kg, state, DEVICE_KERNEL_INTEGRATOR_INTERSECT_VOLUME_STACK);
    return;
  }

  /* Continue with shading shadow catcher surface. */
  const int shader = intersection_get_shader(kg, isect);
  const int flags = kernel_data_fetch(shaders, shader).flags;
  const bool use_caustics = kernel_data.integrator.use_caustics &&
                            (object_flags & SD_OBJECT_CAUSTICS);
  const bool use_raytrace_kernel = (flags & SD_HAS_RAYTRACE);

  if (use_caustics) {
    integrator_path_init_sorted(kg, state, DEVICE_KERNEL_INTEGRATOR_SHADE_SURFACE_MNEE, shader);
  }
  else if (use_raytrace_kernel) {
    integrator_path_init_sorted(
        kg, state, DEVICE_KERNEL_INTEGRATOR_SHADE_SURFACE_RAYTRACE, shader);
  }
  else {
    integrator_path_init_sorted(kg, state, DEVICE_KERNEL_INTEGRATOR_SHADE_SURFACE, shader);
  }
}

#endif /* __SHADOW_CATCHER__ */

CCL_NAMESPACE_END
