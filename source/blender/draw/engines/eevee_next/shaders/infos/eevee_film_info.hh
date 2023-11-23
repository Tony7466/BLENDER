/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "eevee_defines.hh"
#include "gpu_shader_create_info.hh"

GPU_SHADER_CREATE_INFO(eevee_film)
    .sampler(0, ImageType::DEPTH_2D, "depth_tx")
    .sampler(1, ImageType::FLOAT_2D, "combined_tx")
    .sampler(2, ImageType::FLOAT_2D, "vector_tx")
    .sampler(3, ImageType::FLOAT_2D_ARRAY, "rp_color_tx")
    .sampler(4, ImageType::FLOAT_2D_ARRAY, "rp_value_tx")
    /* Color History for TAA needs to be sampler to leverage bilinear sampling. */
    .sampler(5, ImageType::FLOAT_2D, "in_combined_tx")
    .sampler(6, ImageType::FLOAT_2D, "cryptomatte_tx")
    .image(0, GPU_R32F, Qualifier::READ, ImageType::FLOAT_2D_ARRAY, "in_weight_img")
    .image(1, GPU_R32F, Qualifier::WRITE, ImageType::FLOAT_2D_ARRAY, "out_weight_img")
    /* Color History for TAA needs to be sampler to leverage bilinear sampling. */
    //.image(2, GPU_RGBA16F, Qualifier::READ, ImageType::FLOAT_2D, "in_combined_img")
    .image(3, GPU_RGBA16F, Qualifier::WRITE, ImageType::FLOAT_2D, "out_combined_img")
    .image(4, GPU_R32F, Qualifier::READ_WRITE, ImageType::FLOAT_2D, "depth_img")
    .image(5, GPU_RGBA16F, Qualifier::READ_WRITE, ImageType::FLOAT_2D_ARRAY, "color_accum_img")
    .image(6, GPU_R16F, Qualifier::READ_WRITE, ImageType::FLOAT_2D_ARRAY, "value_accum_img")
    .image(7, GPU_RGBA32F, Qualifier::READ_WRITE, ImageType::FLOAT_2D_ARRAY, "cryptomatte_img")
    .additional_info("eevee_shared")
    .additional_info("eevee_global_ubo")
    .additional_info("eevee_velocity_camera")
    .specialization_constant(SC_display_only_SLOT,
                             Type::INT,
                             "SC_display_only",
                             "uniform_buf.film.display_only")
    .specialization_constant(SC_combined_id_SLOT,
                             Type::INT,
                             "SC_combined_id",
                             "uniform_buf.film.combined_id")
    .specialization_constant(SC_has_data_SLOT,
                             Type::INT,
                             "SC_has_data",
                             "uniform_buf.film.has_data")
    .specialization_constant(SC_use_reprojection_SLOT,
                             Type::INT,
                             "SC_use_reprojection",
                             "uniform_buf.film.use_reprojection")
    .specialization_constant(SC_use_history_SLOT,
                             Type::INT,
                             "SC_use_history",
                             "uniform_buf.film.use_history")
    .specialization_constant(SC_normal_id_SLOT,
                             Type::INT,
                             "SC_normal_id",
                             "uniform_buf.film.normal_id")
    .specialization_constant(SC_any_render_pass_1_SLOT,
                             Type::INT,
                             "SC_any_render_pass_1",
                             "uniform_buf.film.any_render_pass_1")
    .specialization_constant(SC_any_render_pass_2_SLOT,
                             Type::INT,
                             "SC_any_render_pass_2",
                             "uniform_buf.film.any_render_pass_2")
    .specialization_constant(SC_any_render_pass_3_SLOT,
                             Type::INT,
                             "SC_any_render_pass_3",
                             "uniform_buf.film.any_render_pass_3")
    .specialization_constant(SC_aov_color_len_SLOT,
                             Type::INT,
                             "SC_aov_color_len",
                             "uniform_buf.film.aov_color_len")
    .specialization_constant(SC_cryptomatte_samples_len_SLOT,
                             Type::INT,
                             "SC_cryptomatte_samples_len",
                             "uniform_buf.film.cryptomatte_samples_len")
    .additional_info("draw_view");

GPU_SHADER_CREATE_INFO(eevee_film_frag)
    .do_static_compilation(true)
    .fragment_out(0, Type::VEC4, "out_color")
    .fragment_source("eevee_film_frag.glsl")
    .additional_info("draw_fullscreen", "eevee_film")
    .depth_write(DepthWrite::ANY);

GPU_SHADER_CREATE_INFO(eevee_film_comp)
    .do_static_compilation(true)
    .local_group_size(FILM_GROUP_SIZE, FILM_GROUP_SIZE)
    .compute_source("eevee_film_comp.glsl")
    .additional_info("eevee_film");

GPU_SHADER_CREATE_INFO(eevee_film_cryptomatte_post)
    .do_static_compilation(true)
    .image(0, GPU_RGBA32F, Qualifier::READ_WRITE, ImageType::FLOAT_2D_ARRAY, "cryptomatte_img")
    .image(1, GPU_R32F, Qualifier::READ, ImageType::FLOAT_2D_ARRAY, "weight_img")
    .push_constant(Type::INT, "cryptomatte_layer_len")
    .push_constant(Type::INT, "cryptomatte_samples_per_layer")
    .local_group_size(FILM_GROUP_SIZE, FILM_GROUP_SIZE)
    .compute_source("eevee_film_cryptomatte_post_comp.glsl")
    .additional_info("eevee_shared");
