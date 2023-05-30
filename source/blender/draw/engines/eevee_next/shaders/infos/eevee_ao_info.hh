/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "eevee_defines.hh"
#include "gpu_shader_create_info.hh"

GPU_SHADER_CREATE_INFO(eevee_raytrace_data)
    .additional_info("draw_view", "eevee_shared", "eevee_hiz_data")
    .uniform_buf(RAYTRACE_BUF_SLOT, "RayTracingData", "rt_buf");

GPU_SHADER_CREATE_INFO(eevee_ao_data)
    .additional_info("eevee_raytrace_data", "eevee_sampling_data", "eevee_utility_texture")
    .sampler(AO_HORIZONS_TEX_SLOT, ImageType::FLOAT_2D, "horizons_tx")
    .uniform_buf(AO_BUF_SLOT, "AOData", "ao_buf");

GPU_SHADER_CREATE_INFO(eevee_ao)
    .additional_info("eevee_ao_data")
    .additional_info("draw_fullscreen")
    .fragment_source("eevee_ao_frag.glsl")
    .fragment_out(0, Type::VEC4, "out_horizons")
    .sampler(0, ImageType::FLOAT_2D, "normal_tx")
    .do_static_compilation(true);
