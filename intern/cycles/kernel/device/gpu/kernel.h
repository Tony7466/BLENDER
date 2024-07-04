/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

/* Common GPU kernels. */

#include "kernel/device/gpu/parallel_active_index.h"
#include "kernel/device/gpu/parallel_prefix_sum.h"
#include "kernel/device/gpu/parallel_sorted_index.h"

#include "kernel/sample/lcg.h"

/* Include constant tables before entering Metal's context class scope (context_begin.h) */
#include "kernel/tables.h"

#ifdef __KERNEL_METAL__
#  include "kernel/device/metal/context_begin.h"
#elif defined(__KERNEL_ONEAPI__)
#  include "kernel/device/oneapi/context_begin.h"
#endif

#include "kernel/device/gpu/work_stealing.h"

#include "kernel/integrator/state.h"
#include "kernel/integrator/state_flow.h"
#include "kernel/integrator/state_util.h"

#include "kernel/integrator/init_from_bake.h"
#include "kernel/integrator/init_from_camera.h"
#include "kernel/integrator/intersect_closest.h"
#include "kernel/integrator/intersect_dedicated_light.h"
#include "kernel/integrator/intersect_shadow.h"
#include "kernel/integrator/intersect_subsurface.h"
#include "kernel/integrator/intersect_volume_stack.h"
#include "kernel/integrator/shade_background.h"
#include "kernel/integrator/shade_dedicated_light.h"
#include "kernel/integrator/shade_light.h"
#include "kernel/integrator/shade_shadow.h"
#include "kernel/integrator/shade_surface.h"
#include "kernel/integrator/shade_volume.h"

#include "kernel/bake/bake.h"

#include "kernel/film/adaptive_sampling.h"

#ifdef __KERNEL_METAL__
#  include "kernel/device/metal/context_end.h"
#elif defined(__KERNEL_ONEAPI__)
#  include "kernel/device/oneapi/context_end.h"
#endif

#include "kernel/film/read.h"

#if defined(__HIPRT__)
#  include "kernel/device/hiprt/hiprt_kernels.h"
#endif

/* --------------------------------------------------------------------
 * Integrator.
 */

#include "kernel/device/gpu/kernel_integrator.h"
#include "kernel/device/gpu/kernel_integrator_init_from_bake.h"
#include "kernel/device/gpu/kernel_integrator_init_from_camera.h"
#include "kernel/device/gpu/kernel_integrator_intersect_closest.h"
#include "kernel/device/gpu/kernel_integrator_intersect_dedicated_light.h"
#include "kernel/device/gpu/kernel_integrator_intersect_shadow.h"
#include "kernel/device/gpu/kernel_integrator_intersect_subsurface.h"
#include "kernel/device/gpu/kernel_integrator_intersect_volume_stack.h"
#include "kernel/device/gpu/kernel_integrator_shade_background.h"
#include "kernel/device/gpu/kernel_integrator_shade_dedicated_light.h"
#include "kernel/device/gpu/kernel_integrator_shade_light.h"
#include "kernel/device/gpu/kernel_integrator_shade_shadow.h"
#include "kernel/device/gpu/kernel_integrator_shade_surface.h"
#include "kernel/device/gpu/kernel_integrator_shade_volume.h"

/* --------------------------------------------------------------------
 * Adaptive sampling.
 */

#include "kernel/device/gpu/kernel_adaptive_sampling.h"

/* --------------------------------------------------------------------
 * Cryptomatte.
 */

#include "kernel/device/gpu/kernel_cryptomatte_passes.h"

/* --------------------------------------------------------------------
 * Film.
 */

#include "kernel/device/gpu/kernel_film.h"

/* --------------------------------------------------------------------
 * Shader evaluation.
 */

#include "kernel/device/gpu/kernel_bake.h"

/* --------------------------------------------------------------------
 * Denoising.
 */

#include "kernel/device/gpu/kernel_denoising.h"

/* --------------------------------------------------------------------
 * Shadow catcher.
 */

#include "kernel/device/gpu/kernel_integrator_shadow_catcher.h"
