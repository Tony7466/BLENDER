/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2023 Blender Foundation */

CCL_NAMESPACE_BEGIN

ccl_device void svm_node_float_to_spectrum(float *stack,
                                           uint in_float_offset,
                                           uint out_spectrum_offset)
{
  float in_float = stack_load_float(stack, in_float_offset);
  stack_store_spectrum(stack, out_spectrum_offset, make_spectrum(in_float));
}

CCL_NAMESPACE_END