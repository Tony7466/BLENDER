/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "kernel/device/cuda/compat.h"
#include "kernel/device/cuda/config.h"
#include "kernel/device/cuda/globals.h"

#include "kernel/tables.h"

__constant__ KernelParamsCUDA kernel_params;

#include "kernel/device/gpu/image.h"

#include "kernel/bvh/bvh.h"
#include "kernel/geom/motion_triangle_shader.h"
#include "kernel/geom/subd_triangle.h"
#include "kernel/svm/svm.h"
