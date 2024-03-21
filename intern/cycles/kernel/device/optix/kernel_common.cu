/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "kernel/device/optix/compat.h"
#include "kernel/device/optix/globals.h"

#include "kernel/tables.h"

#include "kernel/device/gpu/image.h"

#include "kernel/integrator/state_util.h"

#include "kernel/bvh/bvh.h"
#include "kernel/geom/motion_triangle_shader.h"
#include "kernel/geom/subd_triangle.h"
#include "kernel/svm/svm.h"

// Instantiate SVM node functions
#define INSTANTIATE_FUNCTION1(ret, name, args) \
  template ccl_device_noinline ret name<KERNEL_FEATURE_NODE_MASK_SURFACE, SHADER_TYPE_SURFACE> \
      args; \
  template ccl_device_noinline ret \
      name<KERNEL_FEATURE_NODE_MASK_SURFACE & ~KERNEL_FEATURE_NODE_RAYTRACE, SHADER_TYPE_SURFACE> \
          args; \
  template ccl_device_noinline ret name< \
      (KERNEL_FEATURE_NODE_MASK_SURFACE & ~KERNEL_FEATURE_NODE_RAYTRACE) | KERNEL_FEATURE_MNEE, \
      SHADER_TYPE_SURFACE> \
      args; \
  template ccl_device_noinline ret \
      name<KERNEL_FEATURE_NODE_MASK_SURFACE_LIGHT & \
               ~(KERNEL_FEATURE_NODE_RAYTRACE | KERNEL_FEATURE_NODE_LIGHT_PATH), \
           SHADER_TYPE_SURFACE> \
          args; \
  template ccl_device_noinline ret \
      name<KERNEL_FEATURE_NODE_MASK_SURFACE_SHADOW & \
               ~(KERNEL_FEATURE_NODE_RAYTRACE | KERNEL_FEATURE_NODE_LIGHT_PATH), \
           SHADER_TYPE_SURFACE> \
          args; \
  template ccl_device_noinline ret \
      name<KERNEL_FEATURE_NODE_MASK_SURFACE_BACKGROUND, SHADER_TYPE_SURFACE> \
          args; \
  template ccl_device_noinline ret \
      name<KERNEL_FEATURE_NODE_MASK_SURFACE_LIGHT, SHADER_TYPE_SURFACE> \
          args; \
  template ccl_device_noinline ret \
      name<KERNEL_FEATURE_NODE_MASK_SURFACE_SHADOW, SHADER_TYPE_SURFACE> \
          args; \
  template ccl_device_noinline ret name<KERNEL_FEATURE_NODE_MASK_BUMP, SHADER_TYPE_SURFACE> args; \
  template ccl_device_noinline ret \
      name<KERNEL_FEATURE_NODE_MASK_DISPLACEMENT, SHADER_TYPE_DISPLACEMENT> \
          args; \
  template ccl_device_noinline ret name<KERNEL_FEATURE_NODE_MASK_VOLUME, SHADER_TYPE_VOLUME> args;
#define INSTANTIATE_FUNCTION2(ret, name, args) \
  template ccl_device_noinline ret name<KERNEL_FEATURE_NODE_MASK_SURFACE> args; \
  template ccl_device_noinline ret \
      name<KERNEL_FEATURE_NODE_MASK_SURFACE & ~KERNEL_FEATURE_NODE_RAYTRACE> \
          args; \
  template ccl_device_noinline ret name< \
      (KERNEL_FEATURE_NODE_MASK_SURFACE & ~KERNEL_FEATURE_NODE_RAYTRACE) | KERNEL_FEATURE_MNEE> \
      args; \
  template ccl_device_noinline ret \
      name<KERNEL_FEATURE_NODE_MASK_SURFACE_LIGHT & \
           ~(KERNEL_FEATURE_NODE_RAYTRACE | KERNEL_FEATURE_NODE_LIGHT_PATH)> \
          args; \
  template ccl_device_noinline ret \
      name<KERNEL_FEATURE_NODE_MASK_SURFACE_SHADOW & \
           ~(KERNEL_FEATURE_NODE_RAYTRACE | KERNEL_FEATURE_NODE_LIGHT_PATH)> \
          args; \
  template ccl_device_noinline ret name<KERNEL_FEATURE_NODE_MASK_SURFACE_BACKGROUND> args; \
  template ccl_device_noinline ret name<KERNEL_FEATURE_NODE_MASK_SURFACE_LIGHT> args; \
  template ccl_device_noinline ret name<KERNEL_FEATURE_NODE_MASK_SURFACE_SHADOW> args; \
  template ccl_device_noinline ret name<KERNEL_FEATURE_NODE_MASK_DISPLACEMENT> args; \
  template ccl_device_noinline ret name<KERNEL_FEATURE_NODE_MASK_VOLUME> args;
#define INSTANTIATE_FUNCTION3(ret, name, args) \
  template ccl_device_noinline ret name<SHADER_TYPE_SURFACE> args; \
  template ccl_device_noinline ret name<SHADER_TYPE_DISPLACEMENT> args; \
  template ccl_device_noinline ret name<SHADER_TYPE_VOLUME> args;

INSTANTIATE_FUNCTION1(int,
                      svm_node_closure_bsdf,
                      (KernelGlobals kg,
                       ccl_private ShaderData *sd,
                       ccl_private float *stack,
                       Spectrum closure_weight,
                       uint4 node,
                       uint32_t path_flag,
                       int offset))
INSTANTIATE_FUNCTION2(
    void,
    svm_node_attr,
    (KernelGlobals kg, ccl_private ShaderData *sd, ccl_private float *stack, uint4 node))
INSTANTIATE_FUNCTION2(
    void,
    svm_node_set_displacement,
    (KernelGlobals kg, ccl_private ShaderData *sd, ccl_private float *stack, uint fac_offset))
INSTANTIATE_FUNCTION2(
    void,
    svm_node_displacement,
    (KernelGlobals kg, ccl_private ShaderData *sd, ccl_private float *stack, uint4 node))
INSTANTIATE_FUNCTION2(int,
                      svm_node_vector_displacement,
                      (KernelGlobals kg,
                       ccl_private ShaderData *sd,
                       ccl_private float *stack,
                       uint4 node,
                       int offset))
INSTANTIATE_FUNCTION2(
    void,
    svm_node_set_bump,
    (KernelGlobals kg, ccl_private ShaderData *sd, ccl_private float *stack, uint4 node))
INSTANTIATE_FUNCTION3(void,
                      svm_node_closure_volume,
                      (KernelGlobals kg,
                       ccl_private ShaderData *sd,
                       ccl_private float *stack,
                       Spectrum closure_weight,
                       uint4 node))
INSTANTIATE_FUNCTION3(int,
                      svm_node_principled_volume,
                      (KernelGlobals kg,
                       ccl_private ShaderData *sd,
                       ccl_private float *stack,
                       Spectrum closure_weight,
                       uint4 node,
                       uint32_t path_flag,
                       int offset))
INSTANTIATE_FUNCTION2(void,
                      svm_node_light_path,
                      (KernelGlobals kg,
                       ConstIntegratorState state,
                       ccl_private const ShaderData *sd,
                       ccl_private float *stack,
                       uint type,
                       uint out_offset,
                       uint32_t path_flag))
INSTANTIATE_FUNCTION2(int,
                      svm_node_tex_voronoi,
                      (KernelGlobals kg,
                       ccl_private ShaderData *sd,
                       ccl_private float *stack,
                       uint dimensions,
                       uint feature,
                       uint metric,
                       int offset))
#if defined(__SHADER_RAYTRACE__)
INSTANTIATE_FUNCTION2(void,
                      svm_node_bevel,
                      (KernelGlobals kg,
                       ConstIntegratorState state,
                       ccl_private ShaderData *sd,
                       ccl_private float *stack,
                       uint4 node))
INSTANTIATE_FUNCTION2(void,
                      svm_node_ao,
                      (KernelGlobals kg,
                       ConstIntegratorState state,
                       ccl_private ShaderData *sd,
                       ccl_private float *stack,
                       uint4 node))
#endif
INSTANTIATE_FUNCTION2(int,
                      svm_node_tex_voxel,
                      (KernelGlobals kg,
                       ccl_private ShaderData *sd,
                       ccl_private float *stack,
                       uint4 node,
                       int offset))
