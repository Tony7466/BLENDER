/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#define CCL_EXTERN_DECLS

#include "kernel/device/cuda/compat.h"
#include "kernel/device/cuda/config.h"
#include "kernel/device/cuda/globals.h"

#include "kernel/device/gpu/image.h"
#include "kernel/device/gpu/work_stealing.h"
#include "kernel/tables_extern.h"

#include "kernel/integrator/init_from_bake.h"

#include "kernel/device/gpu/kernel_integrator_init_from_bake.h"
