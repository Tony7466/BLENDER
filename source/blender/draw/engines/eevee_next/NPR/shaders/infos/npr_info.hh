/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "gpu_shader_create_info.hh"

#include "NPR/npr_defines.hh"

GPU_SHADER_CREATE_INFO(npr_surface)
    .sampler(INDEX_TX_SLOT, ImageType::UINT_2D, "npr_index_tx")
    .sampler(DEPTH_TX_SLOT, ImageType::DEPTH_2D, "depth_tx")
    .sampler(NORMAL_TX_SLOT, ImageType::FLOAT_2D, "normal_tx")
    .sampler(COMBINED_TX_SLOT, ImageType::FLOAT_2D, "combined_tx")
    .push_constant(Type::INT, "npr_index")
    .fragment_out(0, Type::VEC4, "out_color")
    .fragment_source("npr_surface_frag.glsl")
    .additional_info("draw_fullscreen")
    .define("Closure", "int");
