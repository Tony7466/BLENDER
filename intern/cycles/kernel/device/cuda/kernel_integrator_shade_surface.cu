/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#define CCL_EXTERN_DECLS

#include "kernel/device/cuda/compat.h"
#include "kernel/device/cuda/config.h"
#include "kernel/device/cuda/globals.h"

#include "kernel/device/gpu/image.h"
#include "kernel/tables_extern.h"

#include "kernel/integrator/shade_surface.h"

#include "kernel/device/gpu/kernel_integrator_shade_surface.h"
