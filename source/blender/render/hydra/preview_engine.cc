/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

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
