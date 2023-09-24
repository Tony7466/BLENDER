/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#ifndef FUNCTION_DECLERATION
#define FUNCTION_DECLERATION(...) __VA_ARGS__
#endif

FUNCTION_DECLERATION(
template<uint node_feature_mask, ShaderType type, typename ConstIntegratorGenericState>
ccl_device void svm_eval_nodes(KernelGlobals kg,
                               ConstIntegratorGenericState state,
                               ccl_private ShaderData *sd,
                               ccl_global float *render_buffer,
                               uint32_t path_flag)
{
    int offset = sd->shader & SHADER_MASK;
    float stack[SVM_STACK_SIZE];
    Spectrum closure_weight;
)

