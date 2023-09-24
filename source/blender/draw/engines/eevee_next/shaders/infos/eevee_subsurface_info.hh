/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "eevee_defines.hh"
#include "gpu_shader_create_info.hh"

GPU_SHADER_CREATE_INFO(eevee_subsurface_setup)
    .do_static_compilation(true)
    .local_group_size(RAYTRACE_GROUP_SIZE, RAYTRACE_GROUP_SIZE)
    .typedef_source("draw_shader_shared.h")
    .additional_info("draw_view", "eevee_shared", "eevee_gbuffer_data")
    .sampler(2, ImageType::DEPTH_2D, "depth_tx")
    .image(0, GPU_RGBA16F, Qualifier::READ, ImageType::FLOAT_2D, "direct_light_img")
    .image(1, GPU_RGBA16F, Qualifier::READ, ImageType::FLOAT_2D, "indirect_light_img")
    .image(2, GPU_RGBA16F, Qualifier::WRITE, ImageType::FLOAT_2D, "out_radiance_id_img")
    .storage_buf(0, Qualifier::WRITE, "uint", "convolve_tile_buf[]")
    .storage_buf(1, Qualifier::WRITE, "DispatchCommand", "convolve_dispatch_buf")
    .compute_source("eevee_subsurface_setup_comp.glsl");

GPU_SHADER_CREATE_INFO(eevee_subsurface_convolve)
    .do_static_compilation(true)
    .local_group_size(RAYTRACE_GROUP_SIZE, RAYTRACE_GROUP_SIZE)
    .additional_info(
        "draw_view", "eevee_hiz_data", "eevee_shared", "eevee_gbuffer_data", "eevee_global_ubo")
    .sampler(2, ImageType::FLOAT_2D, "radiance_id_tx")
    .storage_buf(0, Qualifier::READ, "uint", "tiles_coord_buf[]")
    .image(0, GPU_RGBA16F, Qualifier::WRITE, ImageType::FLOAT_2D, "out_direct_light_img")
    .image(1, GPU_RGBA16F, Qualifier::WRITE, ImageType::FLOAT_2D, "out_indirect_light_img")
    .compute_source("eevee_subsurface_convolve_comp.glsl");
