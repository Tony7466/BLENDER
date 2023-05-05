/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2023 Blender Foundation */

#pragma once

#include "kernel/integrator/path_state.h"
#include "kernel/integrator/state_util.h"

CCL_NAMESPACE_BEGIN

#ifdef __SHADOW_LINKING__

/* Check whether special shadow rays for shadow linking are needed in the current scene
 * configuration. */
ccl_device_forceinline bool shadow_linking_scene_need_shadow_ray(KernelGlobals kg,
                                                                 IntegratorState state)
{
  if (!(kernel_data.kernel_features & KERNEL_FEATURE_SHADOW_LINKING)) {
    /* No shadow linking in the scene, so no need to trace any extra rays. */
    return false;
  }

  if (!kernel_data.integrator.use_light_mis) {
    /* No need to cast extra shadow linking path if there are no lights with MIS in the scene. */
    return false;
  }

  return true;
}

/* Shadow linking re-used the main path intersection to store information about the light to which
 * the extra ray is to be traced (this intersection communicates light between the shadow blocker
 * intersection and shading kernels).
 * These utilities makes a copy of the fields from the main intersection which are needed by the
 * intersect_closest kernel after the surface bounce. */

ccl_device_forceinline void shadow_linking_store_last_primitives(IntegratorState state)
{
  INTEGRATOR_STATE_WRITE(state, shadow_link, last_isect_prim) = INTEGRATOR_STATE(
      state, isect, prim);
  INTEGRATOR_STATE_WRITE(state, shadow_link, last_isect_object) = INTEGRATOR_STATE(
      state, isect, object);
}

ccl_device_forceinline void shadow_linking_restore_last_primitives(IntegratorState state)
{
  INTEGRATOR_STATE_WRITE(state, isect, prim) = INTEGRATOR_STATE(
      state, shadow_link, last_isect_prim);
  INTEGRATOR_STATE_WRITE(state, isect, object) = INTEGRATOR_STATE(
      state, shadow_link, last_isect_object);
}

#endif /* __SHADOW_LINKING__ */

CCL_NAMESPACE_END
