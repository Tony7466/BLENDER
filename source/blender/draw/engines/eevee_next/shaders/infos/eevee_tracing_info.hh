
#include "eevee_defines.hh"
#include "gpu_shader_create_info.hh"

/* -------------------------------------------------------------------- */
/** \name Reflection Pipeline
 * \{ */

GPU_SHADER_CREATE_INFO(eevee_ray_tile_classify)
    .do_static_compilation(true)
    .additional_info("eevee_shared")
    .sampler(0, ImageType::FLOAT_2D_ARRAY, "gbuffer_closure_tx")
    .sampler(1, ImageType::UINT_2D, "stencil_tx")
    .image(0, GPU_R32UI, Qualifier::WRITE, ImageType::UINT_2D, "tile_mask_img")
    .compute_source("eevee_ray_tile_classify_comp.glsl");

GPU_SHADER_CREATE_INFO(eevee_ray_tile_compact)
    .do_static_compilation(true)
    .compute_source("eevee_ray_tile_compact_comp.glsl");

GPU_SHADER_CREATE_INFO(eevee_ray_generate)
    .do_static_compilation(true)
    .compute_source("eevee_ray_generate_comp.glsl");

GPU_SHADER_CREATE_INFO(eevee_ray_trace_screen)
    .do_static_compilation(true)
    .compute_source("eevee_ray_trace_screen_comp.glsl");

GPU_SHADER_CREATE_INFO(eevee_ray_denoise_spatial)
    .do_static_compilation(true)
    .compute_source("eevee_ray_denoise_spatial_comp.glsl");

GPU_SHADER_CREATE_INFO(eevee_ray_denoise_temporal)
    .do_static_compilation(true)
    .compute_source("eevee_ray_denoise_temporal_comp.glsl");

GPU_SHADER_CREATE_INFO(eevee_ray_denoise_bilateral)
    .do_static_compilation(true)
    .compute_source("eevee_ray_denoise_bilateral_comp.glsl");

/** \} */
