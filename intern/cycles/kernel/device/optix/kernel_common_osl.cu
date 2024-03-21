/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#define __OSL__

#include "kernel/device/optix/compat.h"
#include "kernel/device/optix/globals.h"

#include "kernel/tables.h"

#include "kernel/device/gpu/image.h"

#include "kernel/integrator/state_util.h"

#include "kernel/bvh/bvh.h"
#include "kernel/geom/motion_triangle_shader.h"
#include "kernel/geom/subd_triangle.h"
