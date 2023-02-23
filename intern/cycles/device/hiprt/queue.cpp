/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#ifdef WITH_HIPRT

#  include "device/hiprt/queue.h"

#  include "device/hip/graphics_interop.h"
#  include "device/hip/kernel.h"
#  include "device/hiprt/device_impl.h"

CCL_NAMESPACE_BEGIN

HIPRTDeviceQueue::HIPRTDeviceQueue(HIPRTDevice *device)
    : HIPDeviceQueue((HIPDevice *)device), hiprt_device_(device)
{
}

bool HIPRTDeviceQueue::enqueue(DeviceKernel kernel,
                               const int work_size,
                               DeviceKernelArguments const &args)
{
  if (hiprt_device_->have_error()) {
    return false;
  }

  if (!device_kernel_has_intersection(kernel))
    return HIPDeviceQueue::enqueue(kernel, work_size, args);

  DeviceKernelArguments arg_copy = args;

  const HIPContextScope scope(hiprt_device_);
  const HIPDeviceKernel &hip_kernel = hiprt_device_->kernels.get(kernel);

  /* Compute kernel launch parameters. */
  const int num_threads_per_block = HIPRT_THREAD_GROUP_SIZE;
  const int num_blocks = divide_up(work_size, num_threads_per_block);

  int shared_mem_bytes = 0;

  assert_success(hipModuleLaunchKernel(hip_kernel.function,
                                       num_blocks,
                                       1,
                                       1,
                                       num_threads_per_block,
                                       1,
                                       1,
                                       shared_mem_bytes,
                                       hip_stream_,
                                       const_cast<void **>(arg_copy.values),
                                       0),
                 "enqueue");

  return !(hiprt_device_->have_error());
}

CCL_NAMESPACE_END

#endif /* WITH_HIPRT */
