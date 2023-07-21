/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include <epoxy/gl.h>

#include "GPU_framebuffer.h"
#include "GPU_texture.h"

#include "IMB_imbuf_types.h"

#include "preview_engine.h"

namespace blender::render::hydra {

void PreviewEngine::render(Depsgraph *depsgraph)
{
  prepare_for_render(depsgraph);
  render_task_delegate_->set_renderer_aov(pxr::HdAovTokens->color);

  engine_->Execute(render_index_.get(), &tasks_);

  std::vector<float> &pixels = render_images_["Combined"];
  while (true) {
    if (RE_engine_test_break(bl_engine_)) {
      break;
    }

    if (render_task_delegate_->is_converged()) {
      break;
    }

    render_task_delegate_->get_renderer_aov_data(pxr::HdAovTokens->color, pixels.data());
    update_render_result(pixels);
  }

  render_task_delegate_->get_renderer_aov_data(pxr::HdAovTokens->color, pixels.data());
  update_render_result(pixels);
}

void PreviewEngineGPU::render(Depsgraph *depsgraph)
{
  prepare_for_render(depsgraph);

  GPUFrameBuffer *framebuffer = GPU_framebuffer_create("fb_render_mat_prev_hydra");
  GPUTexture *tex_color = GPU_texture_create_2d("tex_render_mat_prev_hydra_color",
                                                resolution_[0],
                                                resolution_[1],
                                                1,
                                                GPU_RGBA32F,
                                                GPU_TEXTURE_USAGE_GENERAL,
                                                nullptr);
  GPUTexture *tex_depth = GPU_texture_create_2d("tex_render_mat_prev_hydra_depth",
                                                resolution_[0],
                                                resolution_[1],
                                                1,
                                                GPU_DEPTH32F_STENCIL8,
                                                GPU_TEXTURE_USAGE_GENERAL,
                                                nullptr);
  GPU_texture_filter_mode(tex_color, true);
  GPU_texture_mipmap_mode(tex_color, true, true);
  GPU_texture_filter_mode(tex_depth, true);
  GPU_texture_mipmap_mode(tex_depth, true, true);

  GPU_framebuffer_ensure_config(
      &framebuffer, {GPU_ATTACHMENT_TEXTURE(tex_depth), GPU_ATTACHMENT_TEXTURE(tex_color)});

  GPU_framebuffer_bind(framebuffer);

  float clear_color[4] = {0.0f, 0.0f, 0.0f, 1.0f};

  GPU_framebuffer_clear_color_depth(framebuffer, clear_color, 1.0f);

  /* Important: we have to create and bind at least one Vertex Array Object (VAO) before render
     execution: More info at https://open.gl/drawing */
  GLuint VAO;
  glGenVertexArrays(1, &VAO);
  glBindVertexArray(VAO);

  engine_->Execute(render_index_.get(), &tasks_);

  std::vector<float> &pixels = render_images_["Combined"];

  while (true) {
    if (RE_engine_test_break(bl_engine_)) {
      break;
    }

    if (render_task_delegate_->is_converged()) {
      break;
    }

    void *data = GPU_texture_read(tex_color, GPU_DATA_FLOAT, 0);
    memcpy(pixels.data(), data, pixels.size() * sizeof(float));
    MEM_freeN(data);
    update_render_result();
  }

  void *data = GPU_texture_read(tex_color, GPU_DATA_FLOAT, 0);
  memcpy(pixels.data(), data, pixels.size() * sizeof(float));
  MEM_freeN(data);
  update_render_result();

  glDeleteVertexArrays(1, &VAO);
  GPU_framebuffer_free(framebuffer);
  GPU_texture_free(tex_color);
  GPU_texture_free(tex_depth);
}

void PreviewEngine::update_render_result(std::vector<float> &pixels)
{
  RenderResult *result = RE_engine_begin_result(
      bl_engine_, 0, 0, resolution_[0], resolution_[1], layer_name_.c_str(), nullptr);

  RenderLayer *layer = (RenderLayer *)result->layers.first;
  RenderPass *pass = (RenderPass *)layer->passes.first;
  memcpy(pass->ibuf->float_buffer.data,
         pixels.data(),
         sizeof(float) * pass->rectx * pass->recty * pass->channels);

  RE_engine_end_result(bl_engine_, result, false, false, false);
}

}  // namespace blender::render::hydra
