/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#ifdef WITH_CUDA

#  include "device/cuda/queue.h"

#  include "device/cuda/device_impl.h"
#  include "device/cuda/graphics_interop.h"
#  include "device/cuda/kernel.h"

#  include "util/path.h"
#  include "util/string.h"
#  include "util/time.h"

CCL_NAMESPACE_BEGIN

void TimingStats::Print(CUDADeviceQueue *queue)
{
  if (!queue->done_any_rendering) return;

  const char *device_name = (queue->cuda_device_->info.type == DEVICE_CUDA) ? "CUDA" : "Optix";
  TimingStats *stats = queue->timing_stats;

  printf("\n%s dispatch stats:\n\n", device_name);
  int num_unfinished = 0;
  double total_time = 0.0;
  double kernel_time[DEVICE_KERNEL_NUM] = {0};
  int64_t num_dispatches = 0;
  for (int i = 0; i < DEVICE_KERNEL_NUM; i++) {
    num_unfinished += stats[i].openIntervals.size();
    total_time += stats[i].total_time;
    kernel_time[i] = stats[i].total_time;
    num_dispatches += stats[i].num_dispatches;
  }
  if (num_unfinished) {
    printf("Warning: There are %d dispatches still in flight!\n", num_unfinished);
  }

  printf("\nCUDA dispatch stats:\n\n");
  auto header = string_printf("%-40s %16s %12s %12s %7s %7s",
                              "Kernel name",
                              "Total threads",
                              "Dispatches",
                              "Avg. T/D",
                              "Time",
                              "Time%");
  auto divider = string(header.length(), '-');
  printf("%s\n%s\n%s\n", divider.c_str(), header.c_str(), divider.c_str());

  for (int i = 0; i < DEVICE_KERNEL_NUM; i++) {
    TimingStats &stat = stats[i];
    if (stat.num_dispatches > 0) {
      printf("%-40s %16s %12s %12s %6.2fs %6.2f%%\n",
            device_kernel_as_string(DeviceKernel(i)),
            string_human_readable_number(stat.total_work_size).c_str(),
            string_human_readable_number(stat.num_dispatches).c_str(),
            string_human_readable_number(stat.total_work_size / stat.num_dispatches).c_str(),
            float(stat.total_time),
            total_time ? float(stat.total_time) * 100.0f / total_time : 0.0f);
    }
  }
  printf("%s\n", divider.c_str());
  printf("%-40s %16s %12s %12s %6.2fs %6.2f%%\n",
         "",
         "",
         string_human_readable_number(num_dispatches).c_str(),
         "",
         total_time,
         100.0);
  printf("%s\n\n", divider.c_str());
}

/* CUDADeviceQueue */

CUDADeviceQueue::CUDADeviceQueue(CUDADevice *device)
    : DeviceQueue(device), cuda_device_(device), cuda_stream_(nullptr)
{
  const CUDAContextScope scope(cuda_device_);
  cuda_device_assert(cuda_device_, cuStreamCreate(&cuda_stream_, CU_STREAM_NON_BLOCKING));
}

CUDADeviceQueue::~CUDADeviceQueue()
{
  TimingStats::Print(this);

  const CUDAContextScope scope(cuda_device_);
  cuStreamDestroy(cuda_stream_);
}

int CUDADeviceQueue::num_concurrent_states(const size_t state_size) const
{
  const int max_num_threads = cuda_device_->get_num_multiprocessors() *
                              cuda_device_->get_max_num_threads_per_multiprocessor();
  int num_states = max(max_num_threads, 65536) * 16;

  const char *factor_str = getenv("CYCLES_CONCURRENT_STATES_FACTOR");
  if (factor_str) {
    const float factor = (float)atof(factor_str);
    if (factor != 0.0f) {
      num_states = max((int)(num_states * factor), 1024);
    }
    else {
      VLOG_DEVICE_STATS << "CYCLES_CONCURRENT_STATES_FACTOR evaluated to 0";
    }
  }

  VLOG_DEVICE_STATS << "GPU queue concurrent states: " << num_states << ", using up to "
                    << string_human_readable_size(num_states * state_size);

  return num_states;
}

int CUDADeviceQueue::num_concurrent_busy_states(const size_t /*state_size*/) const
{
  const int max_num_threads = cuda_device_->get_num_multiprocessors() *
                              cuda_device_->get_max_num_threads_per_multiprocessor();

  if (max_num_threads == 0) {
    return 65536;
  }

  return 4 * max_num_threads;
}

void CUDADeviceQueue::init_execution()
{
  /* Synchronize all textures and memory copies before executing task. */
  CUDAContextScope scope(cuda_device_);
  cuda_device_->load_texture_info();
  cuda_device_assert(cuda_device_, cuCtxSynchronize());

  debug_init_execution();
}

bool CUDADeviceQueue::enqueue(DeviceKernel kernel,
                              const int work_size,
                              DeviceKernelArguments const &args)
{
  if (cuda_device_->have_error()) {
    return false;
  }

  debug_enqueue_begin(kernel, work_size);

  const CUDAContextScope scope(cuda_device_);
  const CUDADeviceKernel &cuda_kernel = cuda_device_->kernels.get(kernel);

  /* Compute kernel launch parameters. */
  const int num_threads_per_block = cuda_kernel.num_threads_per_block;
  const int num_blocks = divide_up(work_size, num_threads_per_block);

  int shared_mem_bytes = 0;

  switch (kernel) {
    case DEVICE_KERNEL_INTEGRATOR_QUEUED_PATHS_ARRAY:
    case DEVICE_KERNEL_INTEGRATOR_QUEUED_SHADOW_PATHS_ARRAY:
    case DEVICE_KERNEL_INTEGRATOR_ACTIVE_PATHS_ARRAY:
    case DEVICE_KERNEL_INTEGRATOR_TERMINATED_PATHS_ARRAY:
    case DEVICE_KERNEL_INTEGRATOR_SORTED_PATHS_ARRAY:
    case DEVICE_KERNEL_INTEGRATOR_COMPACT_PATHS_ARRAY:
    case DEVICE_KERNEL_INTEGRATOR_TERMINATED_SHADOW_PATHS_ARRAY:
    case DEVICE_KERNEL_INTEGRATOR_COMPACT_SHADOW_PATHS_ARRAY:
      /* See parall_active_index.h for why this amount of shared memory is needed. */
      shared_mem_bytes = (num_threads_per_block + 1) * sizeof(int);
      break;

    default:
      break;
  }

  /* Launch kernel. */
  assert_success(cuLaunchKernel(cuda_kernel.function,
                                num_blocks,
                                1,
                                1,
                                num_threads_per_block,
                                1,
                                1,
                                shared_mem_bytes,
                                cuda_stream_,
                                const_cast<void **>(args.values),
                                0),
                 "enqueue");

  timing_stats[kernel].AddInterval(kernel, interval);
  if (kernel == DEVICE_KERNEL_INTEGRATOR_INTERSECT_CLOSEST) {
    done_any_rendering = true;
  }

  debug_enqueue_end();

  return !(cuda_device_->have_error());
}

bool CUDADeviceQueue::synchronize()
{
  if (cuda_device_->have_error()) {
    return false;
  }

  const CUDAContextScope scope(cuda_device_);
  assert_success(cuStreamSynchronize(cuda_stream_), "synchronize");

  debug_synchronize();

  /* naive poll loop for all in-flight dispatches (TODO: this is crude) */
  for (int i = 0; i < DEVICE_KERNEL_NUM; i++) {
    timing_stats[i].Poll();
  }

  return !(cuda_device_->have_error());
}

void CUDADeviceQueue::zero_to_device(device_memory &mem)
{
  assert(mem.type != MEM_GLOBAL && mem.type != MEM_TEXTURE);

  if (mem.memory_size() == 0) {
    return;
  }

  /* Allocate on demand. */
  if (mem.device_pointer == 0) {
    cuda_device_->mem_alloc(mem);
  }

  /* Zero memory on device. */
  assert(mem.device_pointer != 0);

  const CUDAContextScope scope(cuda_device_);
  assert_success(
      cuMemsetD8Async((CUdeviceptr)mem.device_pointer, 0, mem.memory_size(), cuda_stream_),
      "zero_to_device");
}

void CUDADeviceQueue::copy_to_device(device_memory &mem)
{
  assert(mem.type != MEM_GLOBAL && mem.type != MEM_TEXTURE);

  if (mem.memory_size() == 0) {
    return;
  }

  /* Allocate on demand. */
  if (mem.device_pointer == 0) {
    cuda_device_->mem_alloc(mem);
  }

  assert(mem.device_pointer != 0);
  assert(mem.host_pointer != nullptr);

  /* Copy memory to device. */
  const CUDAContextScope scope(cuda_device_);
  assert_success(
      cuMemcpyHtoDAsync(
          (CUdeviceptr)mem.device_pointer, mem.host_pointer, mem.memory_size(), cuda_stream_),
      "copy_to_device");
}

void CUDADeviceQueue::copy_from_device(device_memory &mem)
{
  assert(mem.type != MEM_GLOBAL && mem.type != MEM_TEXTURE);

  if (mem.memory_size() == 0) {
    return;
  }

  assert(mem.device_pointer != 0);
  assert(mem.host_pointer != nullptr);

  /* Copy memory from device. */
  const CUDAContextScope scope(cuda_device_);
  assert_success(
      cuMemcpyDtoHAsync(
          mem.host_pointer, (CUdeviceptr)mem.device_pointer, mem.memory_size(), cuda_stream_),
      "copy_from_device");
}

void CUDADeviceQueue::assert_success(CUresult result, const char *operation)
{
  if (result != CUDA_SUCCESS) {
    const char *name = cuewErrorString(result);
    cuda_device_->set_error(string_printf(
        "%s in CUDA queue %s (%s)", name, operation, debug_active_kernels().c_str()));
  }
}

unique_ptr<DeviceGraphicsInterop> CUDADeviceQueue::graphics_interop_create()
{
  return make_unique<CUDADeviceGraphicsInterop>(this);
}

CCL_NAMESPACE_END

#endif /* WITH_CUDA */
