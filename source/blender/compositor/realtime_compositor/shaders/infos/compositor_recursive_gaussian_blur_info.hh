/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "gpu_shader_create_info.hh"

GPU_SHADER_CREATE_INFO(compositor_recursive_gaussian_blur)
    .local_group_size(256)
    .push_constant(Type::VEC4, "causal_feedforward_coefficients")
    .push_constant(Type::VEC4, "non_causal_feedforward_coefficients")
    .push_constant(Type::VEC4, "feedback_coefficients")
    .sampler(0, ImageType::FLOAT_2D, "input_tx")
    .image(0, GPU_RGBA16F, Qualifier::READ_WRITE, ImageType::FLOAT_2D, "output_img")
    .compute_source("compositor_recursive_gaussian_blur.glsl")
    .do_static_compilation(true);
