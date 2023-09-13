/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "util/openimagedenoise.h"
#include "device/device.h"
#include "integrator/denoiser_oidn2.h"

CCL_NAMESPACE_BEGIN

bool openimagedenoise_gpu_supported()
{
#if defined(WITH_OPENIMAGEDENOISE) && OIDN_VERSION_MAJOR >= 2
#  ifdef OIDN_DEVICE_SYCL
  if (Device::available_devices(DEVICE_MASK_ONEAPI).size() > 0 &&
      OIDN2Denoiser::is_device_type_supported(DEVICE_ONEAPI))
  {
    return true;
  }
#  endif
#endif
  return false;
}
CCL_NAMESPACE_END
