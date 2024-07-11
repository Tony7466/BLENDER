/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "gpu_shader_create_info.hh"

#include "NPR/npr_defines.hh"

GPU_SHADER_CREATE_INFO(npr_surface)
    .sampler(INDEX_TX_SLOT, ImageType::UINT_2D, "npr_index_tx")
    .sampler(DEPTH_TX_SLOT, ImageType::DEPTH_2D, "depth_tx")
    /* eevee_gbuffer_data */
    .define("GBUFFER_LOAD")
    .sampler(GBUF_NORMAL_TX_SLOT, ImageType::FLOAT_2D_ARRAY, "gbuf_normal_tx")
    .sampler(GBUF_HEADER_TX_SLOT, ImageType::UINT_2D, "gbuf_header_tx")
    .sampler(GBUF_CLOSURE_TX_SLOT, ImageType::FLOAT_2D_ARRAY, "gbuf_closure_tx")
    /* eevee_gbuffer_data */
    /* eevee_deferred_combine */
    .sampler(DIRECT_RADIANCE_TX_SLOT_1 + 0, ImageType::UINT_2D, "direct_radiance_1_tx")
    .sampler(DIRECT_RADIANCE_TX_SLOT_1 + 1, ImageType::UINT_2D, "direct_radiance_2_tx")
    .sampler(DIRECT_RADIANCE_TX_SLOT_1 + 2, ImageType::UINT_2D, "direct_radiance_3_tx")
    .sampler(INDIRECT_RADIANCE_TX_SLOT_1 + 0, ImageType::FLOAT_2D, "indirect_radiance_1_tx")
    .sampler(INDIRECT_RADIANCE_TX_SLOT_1 + 1, ImageType::FLOAT_2D, "indirect_radiance_2_tx")
    .sampler(INDIRECT_RADIANCE_TX_SLOT_1 + 2, ImageType::FLOAT_2D, "indirect_radiance_3_tx")
    .push_constant(Type::BOOL, "use_split_radiance")
    /* eevee_deferred_combine */
    .push_constant(Type::INT, "npr_index")
    .fragment_out(0, Type::VEC3, "out_color")
    .fragment_source("npr_surface_frag.glsl")
    .define("Closure", "int")
    .define("NPR_SHADER")
    .additional_info("draw_fullscreen", "draw_view", "eevee_shared", "eevee_global_ubo");
