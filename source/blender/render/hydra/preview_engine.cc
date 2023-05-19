/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include "DEG_depsgraph_query.h"

#include "BLI_timer.h"
#include "camera.h"
#include "preview_engine.h"

namespace blender::render::hydra {

const double LIFETIME = 180.0;

std::unique_ptr<PreviewEngine> PreviewEngine::instance_;

PreviewEngine *PreviewEngine::create(RenderEngine *bl_engine,
                                     const std::string &render_delegate_name)
{
  if (!instance_) {
    instance_ = std::make_unique<PreviewEngine>(bl_engine, render_delegate_name);
  }
  else if (instance_->render_delegate_name != render_delegate_name) {
    instance_->render_delegate_->Stop();
    instance_ = std::make_unique<PreviewEngine>(bl_engine, render_delegate_name);
  }
  else {
    instance_->bl_engine_ = bl_engine;
  }

  instance_->scene_delegate_->clear();

  if (BLI_timer_is_registered((uintptr_t)&instance_)) {
    /* Unregister timer while PreviewEngine is working */
    BLI_timer_unregister((uintptr_t)&instance_);
  }

  return instance_.get();
}

void PreviewEngine::free()
{
  instance_->render_delegate_->Stop();

  /* Register timer for schedule free PreviewEngine instance */
  BLI_timer_register((uintptr_t)&instance_, free_instance, nullptr, nullptr, LIFETIME, true);
}

void PreviewEngine::render(Depsgraph *depsgraph)
{
  prepare_for_render(depsgraph);

  std::vector<float> &pixels = render_images_["Combined"];

  {
    /* Release the GIL before calling into hydra, in case any hydra plugins call into python. */
    pxr::TF_PY_ALLOW_THREADS_IN_SCOPE();
    engine_->Execute(render_index_.get(), &tasks_);
  }

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

double PreviewEngine::free_instance(uintptr_t uuid, void *user_data)
{
  if (!instance_->render_task_delegate_->is_converged()) {
    /* Restart timer if render isn't completed */
    return LIFETIME;
  }

  CLOG_INFO(LOG_RENDER_HYDRA, 2, "");
  instance_ = nullptr;
  return -1;
}

void PreviewEngine::update_render_result(std::vector<float> &pixels)
{
  RenderResult *result = RE_engine_begin_result(
      bl_engine_, 0, 0, resolution_[0], resolution_[1], layer_name_.c_str(), nullptr);

  RenderLayer *layer = (RenderLayer *)result->layers.first;
  RenderPass *pass = (RenderPass *)layer->passes.first;
  memcpy(pass->rect, pixels.data(), sizeof(float) * pass->rectx * pass->recty * pass->channels);

  RE_engine_end_result(bl_engine_, result, false, false, false);
}

}  // namespace blender::render::hydra
