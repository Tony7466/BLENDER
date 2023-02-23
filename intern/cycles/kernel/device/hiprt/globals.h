/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include "kernel/types.h"

#include "kernel/integrator/state.h"

#include "kernel/util/profiling.h"

#define HIPRT_SHARED_STACK

#define HIPRT_GLOBAL_STACK_SIZE 512 * 1024 * 1024
#define HIPRT_SHARED_STACK_SIZE 24  // LDS (Local Data Storage) allocation for each thread
#define HIPRT_THREAD_STACK_SIZE \
  64  // global stack allocation per thread (memory reserved for each thread in
      // global_stack_buffer)
#define HIPRT_THREAD_GROUP_SIZE \
  256  // total locaal stack size would be number of threads * HIPRT_SHARED_STACK_SIZE

CCL_NAMESPACE_BEGIN

struct KernelGlobalsGPU {

#ifdef HIPRT_SHARED_STACK
  int *shared_stack;
#else
  int unused[1];
#endif
};
typedef ccl_global KernelGlobalsGPU *ccl_restrict KernelGlobals;

#if defined(HIPRT_SHARED_STACK)
// this macro allocate shared memory and to pass the shared memory down to intersection functions
// KernelGlobals is used
#  define HIPRT_SET_SHARED_MEMORY() \
    ccl_gpu_shared int shared_stack[HIPRT_SHARED_STACK_SIZE * HIPRT_THREAD_GROUP_SIZE]; \
    ccl_global KernelGlobalsGPU kg_gpu; \
    KernelGlobals kg = &kg_gpu; \
    kg->shared_stack = &shared_stack[0];
#else
#  define HIPRT_SET_SHARED_MEMORY() KernelGlobals kg = NULL;
#endif

struct KernelParamsHIPRT {
  KernelData data;
#define KERNEL_DATA_ARRAY(type, name) const type *name;
  KERNEL_DATA_ARRAY(int, user_instance_id)
  KERNEL_DATA_ARRAY(uint64_t, blas_ptr)
  KERNEL_DATA_ARRAY(int2, custom_prim_info)
  KERNEL_DATA_ARRAY(int2, custom_prim_info_offset)
  KERNEL_DATA_ARRAY(float2, prims_time)
  KERNEL_DATA_ARRAY(int, prim_time_offset)
#include "kernel/data_arrays.h"

  /* Integrator state */
  IntegratorStateGPU integrator_state;

  hiprtFuncTable table_closest_intersect;
  hiprtFuncTable table_shadow_intersect;
  hiprtFuncTable table_local_intersect;
  hiprtFuncTable table_volume_intersect;
};

#ifdef __KERNEL_GPU__
__constant__ KernelParamsHIPRT kernel_params;

// global_stack_buffer is defined in global memory and the size is hard coded otherwise it causes
// instablity the correct size of global_stack_buffer is (total number of threads) x STACK_SIZE it
// is a fallback space for HIPRT traversal if the stack on the local memory overflows each thread
// can store up to HIPRT_SHARED_STACK_SIZE elements in local memory and up to
// HIPRT_THREAD_STACK_SIZE elements in global memory.

__attribute__((device)) int global_stack_buffer[HIPRT_GLOBAL_STACK_SIZE];

#  ifdef HIPRT_SHARED_STACK
typedef hiprtGlobalStack Stack;
#  endif

#endif

/* Abstraction macros */
#define kernel_data kernel_params.data
#define kernel_data_fetch(name, index) kernel_params.name[(index)]
#define kernel_data_array(name) (kernel_params.name)
#define kernel_integrator_state kernel_params.integrator_state

CCL_NAMESPACE_END
