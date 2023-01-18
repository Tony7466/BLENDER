/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2023 Blender Foundation */

CCL_NAMESPACE_BEGIN

template<typename ConstIntegratorGenericState>
ccl_device void svm_node_rgb_to_spectrum(KernelGlobals kg,
                                         ConstIntegratorGenericState state,
                                         uint32_t path_flag,
                                         ShaderData *sd,
                                         float *stack,
                                         uint in_color_offset,
                                         uint out_spectrum_offset)
{
  float3 in_color = stack_load_float3(stack, in_color_offset);
  stack_store_spectrum(
      stack, out_spectrum_offset, rgb_to_spectrum(kg, state, path_flag, in_color));
}

CCL_NAMESPACE_END