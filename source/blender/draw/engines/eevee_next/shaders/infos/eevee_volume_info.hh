/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "gpu_shader_create_info.hh"

#pragma once

GPU_SHADER_CREATE_INFO(eevee_volume_lib)
    .additional_info("eevee_shared")
    .additional_info("draw_view")
    .additional_info("eevee_light_data")
    .additional_info("eevee_shadow_data")
    .uniform_buf(VOLUMES_BUF_SLOT, "VolumesData", "volumes_buf")
    .sampler(VOLUME_SCATTERING_TEX_SLOT, ImageType::FLOAT_3D, "inScattering")
    .sampler(VOLUME_TRANSMITTANCE_TEX_SLOT, ImageType::FLOAT_3D, "inTransmittance");

GPU_SHADER_CREATE_INFO(eevee_volume_clear)
    .additional_info("eevee_shared")
    .uniform_buf(VOLUMES_BUF_SLOT, "VolumesData", "volumes_buf")
    .compute_source("eevee_volume_clear_comp.glsl")
    .local_group_size(VOLUME_GROUP_SIZE, VOLUME_GROUP_SIZE, VOLUME_GROUP_SIZE)
    .image(0, GPU_R11F_G11F_B10F, Qualifier::WRITE, ImageType::FLOAT_3D, "out_scattering")
    .image(1, GPU_R11F_G11F_B10F, Qualifier::WRITE, ImageType::FLOAT_3D, "out_extinction")
    .image(2, GPU_R11F_G11F_B10F, Qualifier::WRITE, ImageType::FLOAT_3D, "out_emissive")
    .image(3, GPU_RG16F, Qualifier::WRITE, ImageType::FLOAT_3D, "out_phase")
    .do_static_compilation(true);

GPU_SHADER_CREATE_INFO(eevee_volume_scatter)
    .additional_info("draw_resource_id_varying")
    .additional_info("eevee_volume_lib")
    .compute_source("eevee_volume_scatter_comp.glsl")
    .local_group_size(VOLUME_GROUP_SIZE, VOLUME_GROUP_SIZE, VOLUME_GROUP_SIZE)
    .define("VOLUME_SHADOW")
    /** NOTE: Unique sampler IDs assigned for consistency between library includes,
     * and to avoid unique assignment collision validation error. */
    .sampler(17, ImageType::FLOAT_3D, "scattering_tx")
    .sampler(18, ImageType::FLOAT_3D, "extinction_tx")
    .sampler(19, ImageType::FLOAT_3D, "emission_tx")
    .sampler(20, ImageType::FLOAT_3D, "phase_tx")
    .image(0, GPU_R11F_G11F_B10F, Qualifier::WRITE, ImageType::FLOAT_3D, "out_scattering")
    .image(1, GPU_R11F_G11F_B10F, Qualifier::WRITE, ImageType::FLOAT_3D, "out_transmittance")
    .do_static_compilation(true);

GPU_SHADER_CREATE_INFO(eevee_volume_scatter_with_lights)
    .additional_info("eevee_volume_scatter")
    .define("VOLUME_LIGHTING")
    .do_static_compilation(true);

GPU_SHADER_CREATE_INFO(eevee_volume_integration)
    .additional_info("eevee_volume_lib")
    .compute_source("eevee_volume_integration_comp.glsl")
    .local_group_size(VOLUME_2D_GROUP_SIZE, VOLUME_2D_GROUP_SIZE, 1)
    /** NOTE: Unique sampler IDs assigned for consistency between library includes,
     * and to avoid unique assignment collision validation error. */
    .sampler(17, ImageType::FLOAT_3D, "scattering_tx")
    .sampler(18, ImageType::FLOAT_3D, "extinction_tx")
    .image(0, GPU_R11F_G11F_B10F, Qualifier::WRITE, ImageType::FLOAT_3D, "out_scattering")
    .image(1, GPU_R11F_G11F_B10F, Qualifier::WRITE, ImageType::FLOAT_3D, "out_transmittance")
    .do_static_compilation(true);

GPU_SHADER_CREATE_INFO(eevee_volume_resolve_common)
    .additional_info("draw_fullscreen")
    .additional_info("eevee_volume_lib")
    .sampler(0, ImageType::DEPTH_2D, "inSceneDepth")
    .fragment_source("eevee_volume_resolve_frag.glsl");

GPU_SHADER_CREATE_INFO(eevee_volume_resolve)
    .additional_info("eevee_volume_resolve_common")
    .fragment_out(0, Type::VEC4, "out_radiance", DualBlend::SRC_0)
    .fragment_out(0, Type::VEC4, "out_transmittance", DualBlend::SRC_1)
    .do_static_compilation(true);

GPU_SHADER_CREATE_INFO(eevee_volume_resolve_accum)
    .define("VOLUMETRICS_ACCUM")
    .additional_info("eevee_volume_resolve_common")
    .fragment_out(0, Type::VEC4, "out_radiance")
    .fragment_out(1, Type::VEC4, "out_transmittance")
    .do_static_compilation(true);

GPU_SHADER_CREATE_INFO(eevee_volume_accum)
    .additional_info("draw_fullscreen")
    .additional_info("eevee_volume_lib")
    .fragment_out(0, Type::VEC4, "out_radiance")
    .fragment_out(1, Type::VEC4, "out_transmittance")
    .fragment_source("eevee_volume_accum_frag.glsl")
    .do_static_compilation(true);
