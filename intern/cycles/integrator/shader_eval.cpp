/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "integrator/shader_eval.h"

#include "kernel/integrator/state.h"

#include "device/device.h"
#include "device/queue.h"

#include "device/cpu/kernel.h"
#include "device/cpu/kernel_thread_globals.h"

#include "util/log.h"
#include "util/progress.h"
#include "util/tbb.h"

CCL_NAMESPACE_BEGIN

ShaderEval::ShaderEval(Device *device, Progress &progress, const uint kernel_features)
    : device_(device), progress_(progress), kernel_features_(kernel_features)
{
  DCHECK_NE(device_, nullptr);
}

bool ShaderEval::eval(const ShaderEvalType type,
                      const int max_num_inputs,
                      const int num_channels,
                      const function<int(device_vector<KernelShaderEvalInput> &)> &fill_input,
                      const function<void(device_vector<float> &)> &read_output)
{
  bool first_device = true;
  bool success = true;

  device_->foreach_device([&](Device *device) {
    if (!first_device) {
      VLOG_WORK << "Multi-devices are not yet fully implemented, will evaluate shader on a "
                   "single device.";
      return;
    }
    first_device = false;

    device_vector<KernelShaderEvalInput> input(device, "ShaderEval input", MEM_READ_ONLY);
    device_vector<float> output(device, "ShaderEval output", MEM_READ_WRITE);

    /* Allocate and copy device buffers. */
    DCHECK_EQ(input.device, device);
    DCHECK_EQ(output.device, device);
    DCHECK_LE(output.size(), input.size());

    input.alloc(max_num_inputs);
    int num_points = fill_input(input);
    if (num_points == 0) {
      return;
    }

    input.copy_to_device();
    output.alloc(num_points * num_channels);
    output.zero_to_device();

    /* Evaluate on CPU or GPU. */
    success = (device->info.type == DEVICE_CPU) ?
                  eval_cpu(device, type, input, output, num_points) :
                  eval_gpu(device, type, input, output, num_points);

    /* Copy data back from device if not canceled. */
    if (success) {
      output.copy_from_device(0, 1, output.size());
      read_output(output);
    }

    input.free();
    output.free();
  });

  return success;
}

bool ShaderEval::eval_cpu(Device *device,
                          const ShaderEvalType type,
                          device_vector<KernelShaderEvalInput> &input,
                          device_vector<float> &output,
                          const int64_t work_size)
{
  vector<CPUKernelThreadGlobals> kernel_thread_globals;
  device->get_cpu_kernel_thread_globals(kernel_thread_globals);

  /* Find required kernel function. */
  const CPUKernels &kernels = Device::get_cpu_kernels();

  /* Simple parallel_for over all work items. */
  KernelShaderEvalInput *input_data = input.data();
  float *output_data = output.data();
  bool success = true;

  tbb::task_arena local_arena(device->info.cpu_threads);
  local_arena.execute([&]() {
    parallel_for(int64_t(0), work_size, [&](int64_t work_index) {
      /* TODO: is this fast enough? */
      if (progress_.get_cancel()) {
        success = false;
        return;
      }

      const int thread_index = tbb::this_task_arena::current_thread_index();
      const KernelGlobalsCPU *kg = &kernel_thread_globals[thread_index];

      IntegratorStateCPU state = {0};

      switch (type) {
        case SHADER_EVAL_DISPLACE:
          kernels.shader_eval_displace(kg, &state, input_data, output_data, work_index);
          break;
        case SHADER_EVAL_BACKGROUND:
          kernels.shader_eval_background(kg, &state, input_data, output_data, work_index);
          break;
        case SHADER_EVAL_CURVE_SHADOW_TRANSPARENCY:
          kernels.shader_eval_curve_shadow_transparency(
              kg, &state, input_data, output_data, work_index);
          break;
      }
    });
  });

  return success;
}

static size_t estimate_single_state_size(const uint kernel_features)
{
  size_t state_size = 0;

#define KERNEL_STRUCT_BEGIN(name) for (int array_index = 0;; array_index++) {
#define KERNEL_STRUCT_MEMBER(parent_struct, type, name, feature) \
  state_size += (kernel_features & (feature)) ? sizeof(type) : 0;
#define KERNEL_STRUCT_ARRAY_MEMBER(parent_struct, type, name, feature) \
  state_size += (kernel_features & (feature)) ? sizeof(type) : 0;
#define KERNEL_STRUCT_END(name) \
  break; \
  }
#define KERNEL_STRUCT_END_ARRAY(name, cpu_array_size, gpu_array_size) \
  if (array_index >= gpu_array_size - 1) { \
    break; \
  } \
  }
/* TODO(sergey): Look into better estimation for fields which depend on scene features. Maybe
 * maximum state calculation should happen as `alloc_work_memory()`, so that we can react to an
 * updated scene state here.
 * For until then use common value. Currently this size is only used for logging, but is weak to
 * rely on this. */
#define KERNEL_STRUCT_VOLUME_STACK_SIZE 4

#include "kernel/integrator/state_template.h"

#include "kernel/integrator/shadow_state_template.h"

#undef KERNEL_STRUCT_BEGIN
#undef KERNEL_STRUCT_MEMBER
#undef KERNEL_STRUCT_ARRAY_MEMBER
#undef KERNEL_STRUCT_END
#undef KERNEL_STRUCT_END_ARRAY
#undef KERNEL_STRUCT_VOLUME_STACK_SIZE

  return state_size;
}

bool ShaderEval::eval_gpu(Device *device,
                          const ShaderEvalType type,
                          device_vector<KernelShaderEvalInput> &input,
                          device_vector<float> &output,
                          const int64_t work_size)
{
  /* Find required kernel function. */
  DeviceKernel kernel;
  switch (type) {
    case SHADER_EVAL_DISPLACE:
      kernel = DEVICE_KERNEL_SHADER_EVAL_DISPLACE;
      break;
    case SHADER_EVAL_BACKGROUND:
      kernel = DEVICE_KERNEL_SHADER_EVAL_BACKGROUND;
      break;
    case SHADER_EVAL_CURVE_SHADOW_TRANSPARENCY:
      kernel = DEVICE_KERNEL_SHADER_EVAL_CURVE_SHADOW_TRANSPARENCY;
      break;
  };

  /* Create device queue. */
  unique_ptr<DeviceQueue> queue = device->gpu_queue_create();

  const auto kernel_features = kernel_features_;
  const size_t single_state_size = estimate_single_state_size(kernel_features);
  const auto max_num_paths = queue->num_concurrent_states(single_state_size);

  IntegratorStateGPU integrator_state_gpu;
  vector<unique_ptr<device_memory>> integrator_state_soa;

  memset(&integrator_state_gpu, 0, sizeof(integrator_state_gpu));

  /* Allocate a device only memory buffer before for each struct member, and then
   * write the pointers into a struct that resides in constant memory.
   *
   * TODO: store float3 in separate XYZ arrays. */
#define KERNEL_STRUCT_BEGIN(name) for (int array_index = 0;; array_index++) {
#define KERNEL_STRUCT_MEMBER(parent_struct, type, name, feature) \
  if ((kernel_features & (feature)) && (integrator_state_gpu.parent_struct.name == nullptr)) { \
    device_only_memory<type> *array = new device_only_memory<type>(device, \
                                                                   "integrator_state_" #name); \
    array->alloc_to_device(max_num_paths); \
    integrator_state_soa.emplace_back(array); \
    integrator_state_gpu.parent_struct.name = (type *)array->device_pointer; \
  }
#define KERNEL_STRUCT_ARRAY_MEMBER(parent_struct, type, name, feature) \
  if ((kernel_features & (feature)) && \
      (integrator_state_gpu.parent_struct[array_index].name == nullptr)) { \
    device_only_memory<type> *array = new device_only_memory<type>(device, \
                                                                   "integrator_state_" #name); \
    array->alloc_to_device(max_num_paths); \
    integrator_state_soa.emplace_back(array); \
    integrator_state_gpu.parent_struct[array_index].name = (type *)array->device_pointer; \
  }
#define KERNEL_STRUCT_END(name) \
  break; \
  }
#define KERNEL_STRUCT_END_ARRAY(name, cpu_array_size, gpu_array_size) \
  if (array_index >= gpu_array_size - 1) { \
    break; \
  } \
  }

#define KERNEL_STRUCT_VOLUME_STACK_SIZE 0

#include "kernel/integrator/state_template.h"

#include "kernel/integrator/shadow_state_template.h"

#undef KERNEL_STRUCT_BEGIN
#undef KERNEL_STRUCT_MEMBER
#undef KERNEL_STRUCT_ARRAY_MEMBER
#undef KERNEL_STRUCT_END
#undef KERNEL_STRUCT_END_ARRAY
#undef KERNEL_STRUCT_VOLUME_STACK_SIZE

  device->const_copy_to("integrator_state", &integrator_state_gpu, sizeof(integrator_state_gpu));

  queue->init_execution();

  /* Execute work on GPU in chunks, so we can cancel.
   * TODO: query appropriate size from device. */
  const int32_t chunk_size = 65536;

  device_ptr d_input = input.device_pointer;
  device_ptr d_output = output.device_pointer;

  assert(work_size <= 0x7fffffff);
  for (int32_t d_offset = 0; d_offset < int32_t(work_size); d_offset += chunk_size) {
    int32_t d_work_size = std::min(chunk_size, int32_t(work_size) - d_offset);

    DeviceKernelArguments args(&d_input, &d_output, &d_offset, &d_work_size);

    queue->enqueue(kernel, d_work_size, args);
    queue->synchronize();

    if (progress_.get_cancel()) {
      return false;
    }
  }

  return true;
}

CCL_NAMESPACE_END
