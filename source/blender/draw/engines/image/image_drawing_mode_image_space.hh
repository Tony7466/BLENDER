/* SPDX-FileCopyrightText: 2021 Blender Foundation.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup draw_engine
 */

#pragma once

#include "image_private.hh"

namespace blender::draw::image_engine {
/**
 * Drawing mode optimized for textures that fits within the GPU specifications.
 *
 * Each GPU has a max texture size. Textures larger than this size aren't able to be allocated on
 * the GPU. For large textures use #ScreenSpaceDrawingMode.
 */
class ImageDrawingMode : public AbstractDrawingMode {
 public:
  void begin_sync(IMAGE_Data *vedata) const override
  {
    IMAGE_InstanceData &instance_data = *vedata->instance_data;
    instance_data.passes.image_pass = DRW_pass_create(
        "image_ps", DRW_STATE_WRITE_COLOR | DRW_STATE_WRITE_DEPTH | DRW_STATE_DEPTH_ALWAYS);

    instance_data.passes.depth_pass = nullptr;
  }  // namespace blender::draw::image_engine

  void image_sync(IMAGE_Data *vedata, Image *image, ImageUser *image_user) const override
  {
    IMAGE_InstanceData &instance_data = *vedata->instance_data;
    GPUBatch *geom = DRW_cache_quad_get();
    const bool is_tiled = image->source == IMA_SRC_TILED;
    GPUShader *shader = is_tiled ? IMAGE_shader_image_tiled_get() : IMAGE_shader_image_get();

    ImageUser tile_user = {0};
    if (image_user) {
      tile_user = *image_user;
    }

    GPUTexture *tiles = nullptr;
    GPUTexture *tile_data = nullptr;
    if (is_tiled) {
      tiles = BKE_image_get_gpu_tiles(image, &tile_user, nullptr);
      tile_data = BKE_image_get_gpu_tilemap(image, &tile_user, nullptr);
    }

    LISTBASE_FOREACH (ImageTile *, image_tile_ptr, &image->tiles) {
      const bke::image::ImageTileWrapper image_tile(image_tile_ptr);
      const int tile_x = image_tile.get_tile_x_offset();
      const int tile_y = image_tile.get_tile_y_offset();
      tile_user.tile = image_tile.get_tile_number();

      ShaderParameters &sh_params = instance_data.sh_params;
      DRWShadingGroup *grp = DRW_shgroup_create(shader, instance_data.passes.image_pass);
      DRW_shgroup_uniform_vec2_copy(grp, "tile_offset", float2(tile_x, tile_y));
      DRW_shgroup_uniform_vec2_copy(grp, "farNearDistances", sh_params.far_near);
      DRW_shgroup_uniform_vec4_copy(grp, "shuffle", sh_params.shuffle);
      DRW_shgroup_uniform_int_copy(grp, "drawFlags", static_cast<int32_t>(sh_params.flags));
      DRW_shgroup_uniform_bool_copy(grp, "imgPremultiplied", sh_params.use_premul_alpha);

      if (is_tiled) {
        DRW_shgroup_uniform_texture(grp, "imageTileArray", tiles);
        DRW_shgroup_uniform_texture(grp, "imageTileData", tile_data);
      }
      else {
        GPUTexture *texture = BKE_image_get_gpu_texture(image, &tile_user, nullptr);
        DRW_shgroup_uniform_texture(grp, "imageTexture", texture);
      }

      DRW_shgroup_call(grp, geom, nullptr);
    }
  }

  void draw_viewport(IMAGE_Data *vedata) const override
  {
    IMAGE_InstanceData &instance_data = *vedata->instance_data;
    DefaultFramebufferList *dfbl = DRW_viewport_framebuffer_list_get();
    GPU_framebuffer_clear_color_depth(dfbl->default_fb, float4(0.0), 1.0f);
    DRW_draw_pass(instance_data.passes.image_pass);
  }

  void draw_finish(IMAGE_Data *vedata) const override {}
};
};  // namespace blender::draw::image_engine
