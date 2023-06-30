
#include "eevee_defines.hh"
#include "gpu_shader_create_info.hh"

/* -------------------------------------------------------------------- */
/** \name Reflection Pipeline
 * \{ */

GPU_SHADER_CREATE_INFO(eevee_ray_tile_classify)
    .do_static_compilation(true)
    .local_group_size(RAYTRACE_GROUP_SIZE, RAYTRACE_GROUP_SIZE)
    .additional_info("eevee_shared", "draw_view")
    .sampler(0, ImageType::FLOAT_2D_ARRAY, "gbuffer_closure_tx")
    .sampler(1, ImageType::UINT_2D, "stencil_tx")
    .image(0, GPU_R32UI, Qualifier::WRITE, ImageType::UINT_2D, "tile_mask_img")
    /** TODO(fclem): Move to compaction phase. */
    .storage_buf(1, Qualifier::READ_WRITE, "DispatchCommand", "dispatch_reflect_buf")
    .storage_buf(2, Qualifier::READ_WRITE, "DispatchCommand", "dispatch_refract_buf")
    .storage_buf(4, Qualifier::WRITE, "uint", "tiles_reflect_buf[]")
    .storage_buf(5, Qualifier::WRITE, "uint", "tiles_refract_buf[]")
    .compute_source("eevee_ray_tile_classify_comp.glsl");

GPU_SHADER_CREATE_INFO(eevee_ray_tile_compact)
    .do_static_compilation(true)
    .local_group_size(RAYTRACE_GROUP_SIZE, RAYTRACE_GROUP_SIZE)
    .compute_source("eevee_ray_tile_compact_comp.glsl");

GPU_SHADER_CREATE_INFO(eevee_ray_generate)
    .do_static_compilation(true)
    .local_group_size(RAYTRACE_GROUP_SIZE, RAYTRACE_GROUP_SIZE)
    .additional_info("eevee_shared", "eevee_sampling_data", "draw_view", "eevee_utility_texture")
    .push_constant(Type::INT, "active_closure_type")
    .sampler(0, ImageType::FLOAT_2D, "depth_tx")
    .sampler(1, ImageType::UINT_2D, "stencil_tx")
    .sampler(3, ImageType::FLOAT_2D_ARRAY, "gbuffer_closure_tx")
    .sampler(4, ImageType::FLOAT_2D_ARRAY, "gbuffer_color_tx")
    .image(0, GPU_RGBA32F, Qualifier::WRITE, ImageType::FLOAT_2D, "out_ray_data_img")
    .storage_buf(4, Qualifier::READ, "uint", "tiles_coord_buf[]")
    .uniform_buf(1, "RaytraceData", "raytrace_buf")
    .compute_source("eevee_ray_generate_comp.glsl");

GPU_SHADER_CREATE_INFO(eevee_ray_trace_screen)
    .do_static_compilation(true)
    .local_group_size(RAYTRACE_GROUP_SIZE, RAYTRACE_GROUP_SIZE)
    .compute_source("eevee_ray_trace_screen_comp.glsl");

GPU_SHADER_CREATE_INFO(eevee_ray_denoise_spatial)
    .do_static_compilation(true)
    .local_group_size(RAYTRACE_GROUP_SIZE, RAYTRACE_GROUP_SIZE)
    .compute_source("eevee_ray_denoise_spatial_comp.glsl");

GPU_SHADER_CREATE_INFO(eevee_ray_denoise_temporal)
    .do_static_compilation(true)
    .local_group_size(RAYTRACE_GROUP_SIZE, RAYTRACE_GROUP_SIZE)
    .compute_source("eevee_ray_denoise_temporal_comp.glsl");

GPU_SHADER_CREATE_INFO(eevee_ray_denoise_bilateral)
    .do_static_compilation(true)
    .local_group_size(RAYTRACE_GROUP_SIZE, RAYTRACE_GROUP_SIZE)
    .compute_source("eevee_ray_denoise_bilateral_comp.glsl");

/** \} */
