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

GPU_SHADER_INTERFACE_INFO(eevee_volume_vert_geom_iface, "volume_vert_iface")
    .smooth(Type::VEC4, "vPos");

GPU_SHADER_INTERFACE_INFO(eevee_volume_geom_frag_iface, "volume_geom_iface")
    .flat(Type::INT, "slice");

GPU_SHADER_CREATE_INFO(eevee_volume_base)
    .additional_info("draw_resource_id_varying", "eevee_volume_lib")
    .define("STANDALONE")
    .define("VOLUMETRICS")
    .geometry_source("eevee_volume_geom.glsl")
    .vertex_out(eevee_volume_vert_geom_iface)
    .geometry_out(eevee_volume_geom_frag_iface)
    .geometry_layout(PrimitiveIn::TRIANGLES, PrimitiveOut::TRIANGLE_STRIP, 3)
    .fragment_out(0, Type::VEC4, "volumeScattering")
    .fragment_out(1, Type::VEC4, "volumeExtinction")
    .fragment_out(2, Type::VEC4, "volumeEmissive")
    .fragment_out(3, Type::VEC4, "volumePhase");

GPU_SHADER_CREATE_INFO(eevee_volume_common)
    .additional_info("eevee_volume_base")
    .vertex_source("eevee_volume_vert.glsl");

GPU_SHADER_CREATE_INFO(eevee_volume_clear)
    .additional_info("eevee_volume_common")
    .fragment_source("eevee_volume_clear_frag.glsl")
    .do_static_compilation(true);

/* TODO (Miguel Pozo) */
#ifdef WITH_METAL_BACKEND
/* Non-geometry shader equivalent for multilayered rendering.
 * NOTE: Layer selection can be done in vertex shader, and thus
 * vertex shader emits both vertex and geometry shader output
 * interfaces. */
GPU_SHADER_CREATE_INFO(eevee_volume_clear_no_geom)
    .define("STANDALONE")
    .define("VOLUMETRICS")
    .define("CLEAR")
    .additional_info("draw_resource_id_varying")
    .additional_info("eevee_volume_lib")
    .vertex_source("eevee_volume_vert.glsl")
    .fragment_source("eevee_volume_frag.glsl")
    .vertex_out(eevee_volume_vert_geom_iface)
    .vertex_out(eevee_volume_geom_frag_iface)
    .fragment_out(0, Type::VEC4, "volumeScattering")
    .fragment_out(1, Type::VEC4, "volumeExtinction")
    .fragment_out(2, Type::VEC4, "volumeEmissive")
    .fragment_out(3, Type::VEC4, "volumePhase")
    .metal_backend_only(true)
    .do_static_compilation(true)
    .auto_resource_location(true);
#endif

GPU_SHADER_CREATE_INFO(eevee_volume_scatter_common)
    .define("STANDALONE")
    .define("VOLUMETRICS")
    .define("VOLUME_SHADOW")
    .additional_info("draw_resource_id_varying")
    .additional_info("eevee_volume_lib")
    /* NOTE: Unique sampler IDs assigned for consistency between library includes,
     * and to avoid unique assignment collision validation error.
     * However, resources will be auto assigned locations within shader usage. */
    .sampler(17, ImageType::FLOAT_3D, "volumeScattering")
    .sampler(18, ImageType::FLOAT_3D, "volumeExtinction")
    .sampler(19, ImageType::FLOAT_3D, "volumeEmission")
    .sampler(20, ImageType::FLOAT_3D, "volumePhase")
    .sampler(21, ImageType::FLOAT_3D, "historyScattering")
    .sampler(22, ImageType::FLOAT_3D, "historyTransmittance")
    .fragment_out(0, Type::VEC4, "outScattering")
    .fragment_out(1, Type::VEC4, "outTransmittance")
    .vertex_source("eevee_volume_vert.glsl")
    .fragment_source("eevee_volume_scatter_frag.glsl")
    .vertex_out(eevee_volume_vert_geom_iface);

GPU_SHADER_CREATE_INFO(eevee_volume_scatter)
    .additional_info("eevee_volume_scatter_common")
    .geometry_source("eevee_volume_geom.glsl")
    .geometry_out(eevee_volume_geom_frag_iface)
    .geometry_layout(PrimitiveIn::TRIANGLES, PrimitiveOut::TRIANGLE_STRIP, 3)
    .do_static_compilation(true);

#ifdef WITH_METAL_BACKEND
GPU_SHADER_CREATE_INFO(eevee_volume_scatter_no_geom)
    .additional_info("eevee_volume_scatter_common")
    .vertex_out(eevee_volume_geom_frag_iface)
    .metal_backend_only(true)
    .do_static_compilation(true)
    .auto_resource_location(true);
#endif

GPU_SHADER_CREATE_INFO(eevee_volume_scatter_with_lights_common).define("VOLUME_LIGHTING");

GPU_SHADER_CREATE_INFO(eevee_volume_scatter_with_lights)
    .additional_info("eevee_volume_scatter_with_lights_common")
    .additional_info("eevee_volume_scatter")
    .do_static_compilation(true);

#ifdef WITH_METAL_BACKEND
GPU_SHADER_CREATE_INFO(eevee_volume_scatter_with_lights_no_geom)
    .additional_info("eevee_volume_scatter_with_lights_common")
    .additional_info("eevee_volume_scatter_no_geom")
    .metal_backend_only(true)
    .do_static_compilation(true)
    .auto_resource_location(true);
#endif

GPU_SHADER_CREATE_INFO(eevee_volume_integration_common)
    .define("STANDALONE")
    .additional_info("eevee_volume_lib")
    .additional_info("draw_resource_id_varying")
    /* NOTE: Unique sampler IDs assigned for consistency between library includes,
     * and to avoid unique assignment collision validation error.
     * However, resources will be auto assigned locations within shader usage. */
    .sampler(20, ImageType::FLOAT_3D, "volumeScattering")
    .sampler(21, ImageType::FLOAT_3D, "volumeExtinction")
    .vertex_out(eevee_volume_vert_geom_iface)
    .vertex_source("eevee_volume_vert.glsl")
    .fragment_source("eevee_volume_integration_frag.glsl")
    /*
    .image(0, GPU_R11F_G11F_B10F, Qualifier::WRITE, ImageType::FLOAT_3D, "finalScattering_img")
    .image(1, GPU_R11F_G11F_B10F, Qualifier::WRITE, ImageType::FLOAT_3D, "finalTransmittance_img")
    */
    .fragment_out(0, Type::VEC3, "finalScattering")
    .fragment_out(1, Type::VEC3, "finalTransmittance");

GPU_SHADER_CREATE_INFO(eevee_volume_integration)
    .additional_info("eevee_volume_integration_common")
    .geometry_source("eevee_volume_geom.glsl")
    .geometry_out(eevee_volume_geom_frag_iface)
    .geometry_layout(PrimitiveIn::TRIANGLES, PrimitiveOut::TRIANGLE_STRIP, 3)
    .do_static_compilation(true);

#ifdef WITH_METAL_BACKEND
GPU_SHADER_CREATE_INFO(eevee_volume_integration_no_geom)
    .additional_info("eevee_volume_integration_common_no_geom")
    .additional_info("eevee_volume_integration_common")
    .vertex_out(eevee_volume_geom_frag_iface)
    .metal_backend_only(true)
    .do_static_compilation(true)
    .auto_resource_location(true);
#endif

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

/* EEVEE_shaders_volumes_accum_sh_get */
GPU_SHADER_CREATE_INFO(eevee_volume_accum)
    .additional_info("draw_fullscreen")
    .additional_info("eevee_volume_lib")
    .fragment_out(0, Type::VEC4, "out_radiance")
    .fragment_out(1, Type::VEC4, "out_transmittance")
    .fragment_source("eevee_volume_accum_frag.glsl")
    .do_static_compilation(true);
