/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "eevee_defines.hh"
#include "gpu_shader_create_info.hh"

GPU_SHADER_INTERFACE_INFO(eeve_debug_surfel_iface, "")
    .smooth(Type::VEC3, "P")
    .flat(Type::INT, "surfel_index");

GPU_SHADER_CREATE_INFO(eevee_debug_surfels)
    .additional_info("eevee_shared", "draw_view")
    .vertex_source("eevee_debug_surfels_vert.glsl")
    .vertex_out(eeve_debug_surfel_iface)
    .fragment_source("eevee_debug_surfels_frag.glsl")
    .fragment_out(0, Type::VEC4, "out_color")
    .storage_buf(0, Qualifier::READ, "Surfel", "surfels_buf[]")
    .push_constant(Type::FLOAT, "surfel_radius")
    .push_constant(Type::INT, "debug_mode")
    .do_static_compilation(true);

GPU_SHADER_CREATE_INFO(eevee_surfel_light)
    .local_group_size(SURFEL_GROUP_SIZE)
    .additional_info("eevee_shared",
                     "draw_view",
                     "eevee_utility_texture",
                     "eevee_light_data",
                     "eevee_shadow_data")
    .compute_source("eevee_surfel_light_comp.glsl")
    .storage_buf(SURFEL_BUF_SLOT, Qualifier::READ_WRITE, "Surfel", "surfel_buf[]")
    .storage_buf(CAPTURE_BUF_SLOT, Qualifier::READ, "CaptureInfoData", "capture_info_buf")
    .do_static_compilation(true);

GPU_SHADER_CREATE_INFO(eevee_surfel_list_build)
    .local_group_size(SURFEL_GROUP_SIZE)
    .additional_info("eevee_shared", "draw_view")
    .storage_buf(0, Qualifier::READ_WRITE, "int", "list_start_buf[]")
    .storage_buf(SURFEL_BUF_SLOT, Qualifier::READ_WRITE, "Surfel", "surfel_buf[]")
    .storage_buf(CAPTURE_BUF_SLOT, Qualifier::READ, "CaptureInfoData", "capture_info_buf")
    .storage_buf(6, Qualifier::READ_WRITE, "SurfelListInfoData", "list_info_buf")
    .compute_source("eevee_surfel_list_build_comp.glsl")
    .do_static_compilation(true);

GPU_SHADER_CREATE_INFO(eevee_surfel_list_sort)
    .local_group_size(SURFEL_LIST_GROUP_SIZE)
    .additional_info("eevee_shared", "draw_view")
    .storage_buf(0, Qualifier::READ_WRITE, "int", "list_start_buf[]")
    .storage_buf(SURFEL_BUF_SLOT, Qualifier::READ_WRITE, "Surfel", "surfel_buf[]")
    .storage_buf(6, Qualifier::READ, "SurfelListInfoData", "list_info_buf")
    .compute_source("eevee_surfel_list_sort_comp.glsl")
    .do_static_compilation(true);
