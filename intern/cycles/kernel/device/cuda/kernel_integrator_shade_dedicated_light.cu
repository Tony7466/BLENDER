/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "kernel/device/cuda/compat.h"
#include "kernel/device/cuda/config.h"
#include "kernel/device/cuda/globals.h"

#include "kernel/device/gpu/image.h"
#include "kernel/tables.h"

#include "kernel/integrator/state.h"
#include "kernel/integrator/state_flow.h"
#include "kernel/integrator/state_util.h"

#include "kernel/integrator/shade_dedicated_light.h"

#include "kernel/device/gpu/kernel_integrator_shade_dedicated_light.h"
