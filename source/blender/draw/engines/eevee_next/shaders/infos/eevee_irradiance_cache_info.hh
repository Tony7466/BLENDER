/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "eevee_defines.hh"
#include "gpu_shader_create_info.hh"

GPU_SHADER_INTERFACE_INFO(eevee_debug_surfel_iface, "")
    .smooth(Type::VEC3, "P")
    .flat(Type::INT, "surfel_index");

GPU_SHADER_CREATE_INFO(eevee_debug_surfels)
    .additional_info("eevee_shared", "draw_view")
    .vertex_source("eevee_debug_surfels_vert.glsl")
    .vertex_out(eevee_debug_surfel_iface)
    .fragment_source("eevee_debug_surfels_frag.glsl")
    .fragment_out(0, Type::VEC4, "out_color")
    .storage_buf(0, Qualifier::READ, "Surfel", "surfels_buf[]")
    .push_constant(Type::FLOAT, "surfel_radius")
    .push_constant(Type::INT, "debug_mode")
    .do_static_compilation(true);

GPU_SHADER_CREATE_INFO(eevee_surfel_common)
    .storage_buf(SURFEL_BUF_SLOT, Qualifier::READ_WRITE, "Surfel", "surfel_buf[]")
    .storage_buf(CAPTURE_BUF_SLOT, Qualifier::READ, "CaptureInfoData", "capture_info_buf");

GPU_SHADER_CREATE_INFO(eevee_surfel_light)
    .local_group_size(SURFEL_GROUP_SIZE)
    .additional_info("eevee_shared",
                     "draw_view",
                     "eevee_utility_texture",
                     "eevee_surfel_common",
                     "eevee_light_data",
                     "eevee_shadow_data")
    .compute_source("eevee_surfel_light_comp.glsl")
    .do_static_compilation(true);

GPU_SHADER_CREATE_INFO(eevee_surfel_bounce)
    .local_group_size(SURFEL_GROUP_SIZE)
    .additional_info("eevee_shared", "eevee_surfel_common")
    .compute_source("eevee_surfel_bounce_comp.glsl")
    .do_static_compilation(true);

GPU_SHADER_CREATE_INFO(eevee_surfel_list_build)
    .local_group_size(SURFEL_GROUP_SIZE)
    .additional_info("eevee_shared", "eevee_surfel_common", "draw_view")
    .storage_buf(0, Qualifier::READ_WRITE, "int", "list_start_buf[]")
    .storage_buf(6, Qualifier::READ_WRITE, "SurfelListInfoData", "list_info_buf")
    .compute_source("eevee_surfel_list_build_comp.glsl")
    .do_static_compilation(true);

GPU_SHADER_CREATE_INFO(eevee_surfel_list_sort)
    .local_group_size(SURFEL_LIST_GROUP_SIZE)
    .additional_info("eevee_shared", "eevee_surfel_common", "draw_view")
    .storage_buf(0, Qualifier::READ_WRITE, "int", "list_start_buf[]")
    .storage_buf(6, Qualifier::READ, "SurfelListInfoData", "list_info_buf")
    .compute_source("eevee_surfel_list_sort_comp.glsl")
    .do_static_compilation(true);

GPU_SHADER_CREATE_INFO(eevee_surfel_ray)
    .local_group_size(SURFEL_GROUP_SIZE)
    .additional_info("eevee_shared", "eevee_surfel_common", "draw_view")
    .compute_source("eevee_surfel_ray_comp.glsl")
    .do_static_compilation(true);

GPU_SHADER_INTERFACE_INFO(eevee_display_probe_grid_iface, "")
    .smooth(Type::VEC2, "lP")
    .flat(Type::IVEC3, "cell");

GPU_SHADER_CREATE_INFO(eevee_display_probe_grid)
    .additional_info("eevee_shared", "draw_view")
    .vertex_source("eevee_display_probe_grid_vert.glsl")
    .vertex_out(eevee_display_probe_grid_iface)
    .fragment_source("eevee_display_probe_grid_frag.glsl")
    .fragment_out(0, Type::VEC4, "out_color")
    .push_constant(Type::FLOAT, "sphere_radius")
    .push_constant(Type::IVEC3, "grid_resolution")
    .push_constant(Type::MAT4, "grid_to_world")
    .sampler(0, ImageType::FLOAT_3D, "irradiance_a_tx")
    .sampler(1, ImageType::FLOAT_3D, "irradiance_b_tx")
    .sampler(2, ImageType::FLOAT_3D, "irradiance_c_tx")
    .sampler(3, ImageType::FLOAT_3D, "irradiance_d_tx")
    .do_static_compilation(true);

GPU_SHADER_CREATE_INFO(eevee_lightprobe_irradiance_ray)
    .local_group_size(IRRADIANCE_GRID_GROUP_SIZE,
                      IRRADIANCE_GRID_GROUP_SIZE,
                      IRRADIANCE_GRID_GROUP_SIZE)
    .additional_info("eevee_shared", "eevee_surfel_common", "draw_view")
    .storage_buf(0, Qualifier::READ, "int", "list_start_buf[]")
    .storage_buf(6, Qualifier::READ, "SurfelListInfoData", "list_info_buf")
    .image(0, GPU_RGBA16F, Qualifier::READ_WRITE, ImageType::FLOAT_3D, "irradiance_L0_img")
    .image(1, GPU_RGBA16F, Qualifier::READ_WRITE, ImageType::FLOAT_3D, "irradiance_L1_a_img")
    .image(2, GPU_RGBA16F, Qualifier::READ_WRITE, ImageType::FLOAT_3D, "irradiance_L1_b_img")
    .image(3, GPU_RGBA16F, Qualifier::READ_WRITE, ImageType::FLOAT_3D, "irradiance_L1_c_img")
    .compute_source("eevee_lightprobe_irradiance_ray_comp.glsl")
    .do_static_compilation(true);
