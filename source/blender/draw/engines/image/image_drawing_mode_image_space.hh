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
struct ImageDrawingMode {
  void begin_sync(IMAGE_Data *vedata)
  {
    IMAGE_InstanceData &instance_data = *vedata->instance_data;
    instance_data.passes.image_pass = DRW_pass_create(
        "image_ps", DRW_STATE_WRITE_COLOR | DRW_STATE_WRITE_DEPTH | DRW_STATE_DEPTH_ALWAYS);

    instance_data.passes.depth_pass = nullptr;
  }
  void image_sync(IMAGE_Data *vedata, Image *image, ImageUser *image_user)
  {
    IMAGE_InstanceData &instance_data = *vedata->instance_data;
    GPUBatch *geom = DRW_cache_quad_get();
    GPUShader *shader = IMAGE_shader_image_get();

    ImageUser tile_user = {0};
    if (image_user) {
      tile_user = *image_user;
    }
    LISTBASE_FOREACH (ImageTile *, image_tile_ptr, &image->tiles) {
      const ImageTileWrapper image_tile(image_tile_ptr);
      const int tile_x = image_tile.get_tile_x_offset();
      const int tile_y = image_tile.get_tile_y_offset();
      tile_user.tile = image_tile.get_tile_number();
      GPUTexture *texture = BKE_image_get_gpu_texture(image, &tile_user, nullptr);

      DRWShadingGroup *grp = DRW_shgroup_create(shader, instance_data.passes.image_pass);
      DRW_shgroup_uniform_vec2_copy(grp, "tile_offset", float2(tile_x, tile_y));
      DRW_shgroup_uniform_texture(grp, "imageTexture", texture);
      DRW_shgroup_call(grp, geom, nullptr);
    }
  }

  void draw_viewport(IMAGE_Data *vedata)
  {
    IMAGE_InstanceData &instance_data = *vedata->instance_data;
    DefaultFramebufferList *dfbl = DRW_viewport_framebuffer_list_get();
    GPU_framebuffer_clear_color_depth(dfbl->default_fb, float4(0.0), 1.0f);
    DRW_draw_pass(instance_data.passes.image_pass);
  }
  void draw_finish(IMAGE_Data *vedata) {}
};

};  // namespace blender::draw::image_engine
