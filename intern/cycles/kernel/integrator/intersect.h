/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#pragma once

CCL_NAMESPACE_BEGIN

ccl_device_forceinline bool integrator_intersect_skip_lights(KernelGlobals kg,
                                                             IntegratorState state)
{
  /* When direct lighting is disabled for baking, we skip light sampling in
   * integrate_surface_direct_light for the first bounce. Therefore, in order
   * for MIS to be consistent, we also need to skip evaluating lights here. */
  return (kernel_data.integrator.filter_closures & FILTER_CLOSURE_DIRECT_LIGHT) &&
         (INTEGRATOR_STATE(state, path, bounce) == 1);
}

CCL_NAMESPACE_END
