/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#define __OSL__

#include "kernel/device/optix/compat.h"
#include "kernel/device/optix/globals.h"

#include "kernel/device/gpu/image.h" /* Texture lookup uses normal CUDA intrinsics. */

#include "kernel/osl/services_gpu.h"

extern "C" __device__ void __direct_callable__dummy_services() {}
