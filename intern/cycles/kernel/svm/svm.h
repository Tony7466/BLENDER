/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#pragma once

/* Shader Virtual Machine
 *
 * A shader is a list of nodes to be executed. These are simply read one after
 * the other and executed, using an node counter. Each node and its associated
 * data is encoded as one or more uint4's in a 1D texture. If the data is larger
 * than an uint4, the node can increase the node counter to compensate for this.
 * Floats are encoded as int and then converted to float again.
 *
 * Nodes write their output into a stack. All stack data in the stack is
 * floats, since it's all factors, colors and vectors. The stack will be stored
 * in local memory on the GPU, as it would take too many register and indexes in
 * ways not known at compile time. This seems the only solution even though it
 * may be slow, with two positive factors. If the same shader is being executed,
 * memory access will be coalesced and cached.
 *
 * The result of shader execution will be a single closure. This means the
 * closure type, associated label, data and weight. Sampling from multiple
 * closures is supported through the mix closure node, the logic for that is
 * mostly taken care of in the SVM compiler.
 */

#include "kernel/svm/types.h"

CCL_NAMESPACE_BEGIN

/* Stack */

ccl_device_inline float3 stack_load_float3(ccl_private float *stack, uint a)
{
  kernel_assert(a + 2 < SVM_STACK_SIZE);

  ccl_private float *stack_a = stack + a;
  return make_float3(stack_a[0], stack_a[1], stack_a[2]);
}

ccl_device_inline void stack_store_float3(ccl_private float *stack, uint a, float3 f)
{
  kernel_assert(a + 2 < SVM_STACK_SIZE);

  ccl_private float *stack_a = stack + a;
  stack_a[0] = f.x;
  stack_a[1] = f.y;
  stack_a[2] = f.z;
}

ccl_device_inline float stack_load_float(ccl_private float *stack, uint a)
{
  kernel_assert(a < SVM_STACK_SIZE);

  return stack[a];
}

ccl_device_inline float stack_load_float_default(ccl_private float *stack, uint a, uint value)
{
  return (a == (uint)SVM_STACK_INVALID) ? __uint_as_float(value) : stack_load_float(stack, a);
}

ccl_device_inline void stack_store_float(ccl_private float *stack, uint a, float f)
{
  kernel_assert(a < SVM_STACK_SIZE);

  stack[a] = f;
}

ccl_device_inline int stack_load_int(ccl_private float *stack, uint a)
{
  kernel_assert(a < SVM_STACK_SIZE);

  return __float_as_int(stack[a]);
}

ccl_device_inline int stack_load_int_default(ccl_private float *stack, uint a, uint value)
{
  return (a == (uint)SVM_STACK_INVALID) ? (int)value : stack_load_int(stack, a);
}

ccl_device_inline void stack_store_int(ccl_private float *stack, uint a, int i)
{
  kernel_assert(a < SVM_STACK_SIZE);

  stack[a] = __int_as_float(i);
}

ccl_device_inline bool stack_valid(uint a)
{
  return a != (uint)SVM_STACK_INVALID;
}

/* Reading Nodes */

ccl_device_inline uint4 read_node(KernelGlobals kg, ccl_private int *offset)
{
  uint4 node = kernel_data_fetch(svm_nodes, *offset);
  (*offset)++;
  return node;
}

ccl_device_inline float4 read_node_float(KernelGlobals kg, ccl_private int *offset)
{
  uint4 node = kernel_data_fetch(svm_nodes, *offset);
  float4 f = make_float4(__uint_as_float(node.x),
                         __uint_as_float(node.y),
                         __uint_as_float(node.z),
                         __uint_as_float(node.w));
  (*offset)++;
  return f;
}

ccl_device_inline float4 fetch_node_float(KernelGlobals kg, int offset)
{
  uint4 node = kernel_data_fetch(svm_nodes, offset);
  return make_float4(__uint_as_float(node.x),
                     __uint_as_float(node.y),
                     __uint_as_float(node.z),
                     __uint_as_float(node.w));
}

ccl_device_forceinline void svm_unpack_node_uchar2(uint i,
                                                   ccl_private uint *x,
                                                   ccl_private uint *y)
{
  *x = (i & 0xFF);
  *y = ((i >> 8) & 0xFF);
}

ccl_device_forceinline void svm_unpack_node_uchar3(uint i,
                                                   ccl_private uint *x,
                                                   ccl_private uint *y,
                                                   ccl_private uint *z)
{
  *x = (i & 0xFF);
  *y = ((i >> 8) & 0xFF);
  *z = ((i >> 16) & 0xFF);
}

ccl_device_forceinline void svm_unpack_node_uchar4(
    uint i, ccl_private uint *x, ccl_private uint *y, ccl_private uint *z, ccl_private uint *w)
{
  *x = (i & 0xFF);
  *y = ((i >> 8) & 0xFF);
  *z = ((i >> 16) & 0xFF);
  *w = ((i >> 24) & 0xFF);
}

CCL_NAMESPACE_END

/* Nodes */

#include "kernel/svm/aov.h"
#include "kernel/svm/attribute.h"
#include "kernel/svm/blackbody.h"
#include "kernel/svm/brick.h"
#include "kernel/svm/brightness.h"
#include "kernel/svm/bump.h"
#include "kernel/svm/camera.h"
#include "kernel/svm/checker.h"
#include "kernel/svm/clamp.h"
#include "kernel/svm/closure.h"
#include "kernel/svm/convert.h"
#include "kernel/svm/displace.h"
#include "kernel/svm/fresnel.h"
#include "kernel/svm/gamma.h"
#include "kernel/svm/geometry.h"
#include "kernel/svm/gradient.h"
#include "kernel/svm/hsv.h"
#include "kernel/svm/ies.h"
#include "kernel/svm/image.h"
#include "kernel/svm/invert.h"
#include "kernel/svm/light_path.h"
#include "kernel/svm/magic.h"
#include "kernel/svm/map_range.h"
#include "kernel/svm/mapping.h"
#include "kernel/svm/math.h"
#include "kernel/svm/mix.h"
#include "kernel/svm/musgrave.h"
#include "kernel/svm/noisetex.h"
#include "kernel/svm/normal.h"
#include "kernel/svm/ramp.h"
#include "kernel/svm/sepcomb_color.h"
#include "kernel/svm/sepcomb_hsv.h"
#include "kernel/svm/sepcomb_vector.h"
#include "kernel/svm/sky.h"
#include "kernel/svm/tex_coord.h"
#include "kernel/svm/value.h"
#include "kernel/svm/vector_rotate.h"
#include "kernel/svm/vector_transform.h"
#include "kernel/svm/vertex_color.h"
#include "kernel/svm/voronoi.h"
#include "kernel/svm/voxel.h"
#include "kernel/svm/wave.h"
#include "kernel/svm/wavelength.h"
#include "kernel/svm/white_noise.h"
#include "kernel/svm/wireframe.h"

#ifdef __SHADER_RAYTRACE__
#  include "kernel/svm/ao.h"
#  include "kernel/svm/bevel.h"
#endif

CCL_NAMESPACE_BEGIN

#ifdef __KERNEL_USE_DATA_CONSTANTS__
#  define SVM_CASE(node) \
    case node: \
      if (!kernel_data_svm_usage_##node) \
        break;
#else
#  define SVM_CASE(node) case node:
#endif

/*CUSTOMIZED_SVM_EVAL_NODES_HERE*/

#ifndef USE_CUSTOMIZED_SVM_EVAL_NODES
#include "kernel/svm/svm_eval_nodes_decl.h"
    while (1) {
      uint4 node = read_node(kg, &offset);
      switch(node.x)
      {
        
#include "kernel/svm/svm_node_handler.h"
              
        default:
          kernel_assert(!"Unknown node type was passed to the SVM machine");
          return;
      }
    }
}
#endif

CCL_NAMESPACE_END
