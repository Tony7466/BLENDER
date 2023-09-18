/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#pragma once

#ifdef WITH_CUDA

#  include "device/kernel.h"
#  include "device/memory.h"
#  include "device/queue.h"

#  include "device/cuda/util.h"

#  include <algorithm>
#  include <vector>

CCL_NAMESPACE_BEGIN

class CUDADevice;
class device_memory;

struct TimingInterval {
  int work_size;
  CUstream stream;
  CUevent start, stop;

  TimingInterval(CUstream _stream, int _work_size)
  {
    work_size = _work_size;
    stream = _stream;
    cuEventCreate(&start, CU_EVENT_DEFAULT);
    cuEventCreate(&stop, CU_EVENT_DEFAULT);

    cuEventRecord(start, stream);
  }

  void Stop()
  {
    cuEventRecord(stop, stream);
  }

  float Poll()
  {
    float milliseconds = 0;
    if (cuEventElapsedTime(&milliseconds, start, stop) == CUDA_SUCCESS) {
      cuEventDestroy(start);
      cuEventDestroy(stop);
      return milliseconds / 1000.0f;
    }
    return -1.0f;
  }
};

class CUDADeviceQueue;
struct TimingStats {

  DeviceKernel kernel;
  double total_time = 0.0f;
  uint64_t total_work_size = 0;
  uint64_t num_dispatches = 0;

  void AddInterval(DeviceKernel _kernel, TimingInterval *interval)
  {
    kernel = _kernel;
    interval->Stop();
    openIntervals.push_back(interval);
  }

  void Poll()
  {
    for (size_t j = 0; j < openIntervals.size();) {
      TimingInterval *interval = openIntervals[j];
      float elapsed = interval->Poll();
      if (elapsed < 0.0f) {
        /* dispatch is still in flight */
        j++;
      }
      else {
        /* dispatch is complete - accumulate timing stats and free */
        num_dispatches += 1;
        total_work_size += interval->work_size;
        total_time += elapsed;
        openIntervals.erase(openIntervals.begin() + j);
        delete interval;
      }
    }
  }

  std::vector<TimingInterval *> openIntervals;

  static void Print(CUDADeviceQueue *queue);
};

/* Base class for CUDA queues. */
class CUDADeviceQueue : public DeviceQueue {
 public:
  CUDADeviceQueue(CUDADevice *device);
  ~CUDADeviceQueue();

  virtual int num_concurrent_states(const size_t state_size) const override;
  virtual int num_concurrent_busy_states(const size_t state_size) const override;

  virtual void init_execution() override;

  virtual bool enqueue(DeviceKernel kernel,
                       const int work_size,
                       DeviceKernelArguments const &args) override;

  virtual bool synchronize() override;

  virtual void zero_to_device(device_memory &mem) override;
  virtual void copy_to_device(device_memory &mem) override;
  virtual void copy_from_device(device_memory &mem) override;

  virtual CUstream stream()
  {
    return cuda_stream_;
  }

  virtual unique_ptr<DeviceGraphicsInterop> graphics_interop_create() override;

 protected:
  CUDADevice *cuda_device_;
  CUstream cuda_stream_;

  void assert_success(CUresult result, const char *operation);
};

CCL_NAMESPACE_END

#endif /* WITH_CUDA */
