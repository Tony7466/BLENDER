/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "eevee_defines.hh"
#include "gpu_shader_create_info.hh"

GPU_SHADER_CREATE_INFO(npr_surface_common)
    .sampler(INDEX_NPR_TX_SLOT, ImageType::UINT_2D, "npr_index_tx")
    .sampler(DEPTH_NPR_TX_SLOT, ImageType::DEPTH_2D, "depth_tx")
    /* eevee_gbuffer_data */
    .define("GBUFFER_LOAD")
    .sampler(GBUF_NORMAL_NPR_TX_SLOT, ImageType::FLOAT_2D_ARRAY, "gbuf_normal_tx")
    .sampler(GBUF_HEADER_NPR_TX_SLOT, ImageType::UINT_2D, "gbuf_header_tx")
    .sampler(GBUF_CLOSURE_NPR_TX_SLOT, ImageType::FLOAT_2D_ARRAY, "gbuf_closure_tx")
    /* eevee_gbuffer_data */
    /* eevee_deferred_combine */
    .sampler(DIRECT_RADIANCE_NPR_TX_SLOT_1 + 0, ImageType::UINT_2D, "direct_radiance_1_tx")
    .sampler(DIRECT_RADIANCE_NPR_TX_SLOT_1 + 1, ImageType::UINT_2D, "direct_radiance_2_tx")
    .sampler(DIRECT_RADIANCE_NPR_TX_SLOT_1 + 2, ImageType::UINT_2D, "direct_radiance_3_tx")
    .sampler(INDIRECT_RADIANCE_NPR_TX_SLOT_1 + 0, ImageType::FLOAT_2D, "indirect_radiance_1_tx")
    .sampler(INDIRECT_RADIANCE_NPR_TX_SLOT_1 + 1, ImageType::FLOAT_2D, "indirect_radiance_2_tx")
    .sampler(INDIRECT_RADIANCE_NPR_TX_SLOT_1 + 2, ImageType::FLOAT_2D, "indirect_radiance_3_tx")
    .push_constant(Type::BOOL, "use_split_radiance")
    /* eevee_deferred_combine */
    .push_constant(Type::INT, "npr_index")
    .define("NPR_SHADER")
    .fragment_out(0, Type::VEC3, "out_color")
    .additional_info("draw_view",
                     "eevee_shared",
                     "eevee_global_ubo",
                     "eevee_light_data",
                     "eevee_lightprobe_data",
                     "eevee_shadow_data",
                     "eevee_utility_texture",
                     "eevee_sampling_data",
                     "eevee_hiz_data");

GPU_SHADER_CREATE_INFO(eevee_surf_npr)
    .fragment_source("eevee_surf_deferred_npr_frag.glsl")
    .additional_info("npr_surface_common");

/** NOTE: Screen-space NPR evaluation. Not currently used. */
GPU_SHADER_INTERFACE_INFO(npr_surface_iface, "").smooth(Type::VEC4, "uvcoordsvar");

GPU_SHADER_CREATE_INFO(npr_surface)
    .define("Closure", "int")
    .vertex_source("npr_surface_vert.glsl")
    .vertex_out(npr_surface_iface)
    .fragment_source("npr_surface_frag.glsl")
    .define("ModelMatrix", "mat4(1.0)")
    .define("ModelMatrixInverse", "mat4(1.0)")
    .additional_info("npr_surface_common");
