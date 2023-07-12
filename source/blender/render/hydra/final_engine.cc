/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include "BKE_lib_id.h"
#include "BLI_timecode.h"
#include "DEG_depsgraph_query.h"
#include "GPU_framebuffer.h"
#include "GPU_texture.h"
#include "PIL_time.h"

#include "IMB_imbuf_types.h"

#include "camera.h"
#include "final_engine.h"

namespace blender::render::hydra {

void FinalEngine::render(Depsgraph *depsgraph)
{
  prepare_for_render(depsgraph);

  std::vector<float> &pixels = render_images_["Combined"];

  {
    /* Release the GIL before calling into hydra, in case any hydra plugins call into python. */
    engine_->Execute(render_index_.get(), &tasks_);
  }

  char elapsed_time[32];
  double time_begin = PIL_check_seconds_timer();
  float percent_done = 0.0;

  while (true) {
    if (RE_engine_test_break(bl_engine_)) {
      break;
    }

    percent_done = renderer_percent_done();

    BLI_timecode_string_from_time_simple(
        elapsed_time, sizeof(elapsed_time), PIL_check_seconds_timer() - time_begin);

    notify_status(percent_done / 100.0,
                  scene_name_ + ": " + layer_name_,
                  std::string("Render Time: ") + elapsed_time +
                      " | Done: " + std::to_string(int(percent_done)) + "%");

    if (render_task_delegate_->is_converged()) {
      break;
    }

    render_task_delegate_->get_renderer_aov_data(pxr::HdAovTokens->color, pixels.data());
    update_render_result();
  }

  render_task_delegate_->get_renderer_aov_data(pxr::HdAovTokens->color, pixels.data());
  update_render_result();
}

void FinalEngine::update_render_result()
{
  RenderResult *result = RE_engine_begin_result(
      bl_engine_, 0, 0, resolution_[0], resolution_[1], layer_name_.c_str(), nullptr);

  /* TODO: only for the first render layer */
  RenderLayer *layer = (RenderLayer *)result->layers.first;
  for (RenderPass *pass = (RenderPass *)layer->passes.first; pass != nullptr; pass = pass->next) {
    auto it_image = render_images_.find(pass->name);
    if (it_image == render_images_.end()) {
      continue;
    }
    memcpy(pass->ibuf->float_buffer.data,
           it_image->second.data(),
           sizeof(float) * pass->rectx * pass->recty * pass->channels);
  }

  RE_engine_end_result(bl_engine_, result, false, false, false);
}

void FinalEngine::notify_status(float progress, const std::string &title, const std::string &info)
{
  RE_engine_update_progress(bl_engine_, progress);
  RE_engine_update_stats(bl_engine_, title.c_str(), info.c_str());
}

void FinalEngine::prepare_for_render(Depsgraph *depsgraph)
{
  const Scene *scene = DEG_get_evaluated_scene(depsgraph);
  const ViewLayer *view_layer = DEG_get_evaluated_view_layer(depsgraph);

  BKE_id_full_name_get(scene_name_.data(), (ID *)scene, 0);
  layer_name_ = view_layer->name;

  const RenderData &r = scene->r;
  pxr::GfVec4f border(0, 0, 1, 1);
  if (r.mode & R_BORDER) {
    border.Set(r.border.xmin,
               r.border.ymin,
               r.border.xmax - r.border.xmin,
               r.border.ymax - r.border.ymin);
  }
  pxr::GfVec2i image_res(r.xsch * r.size / 100, r.ysch * r.size / 100);
  resolution_ = pxr::GfVec2i(int(image_res[0] * border[2]), int(image_res[1] * border[3]));
  pxr::GfCamera camera =
      CameraData(scene->camera, image_res, pxr::GfVec4f(0, 0, 1, 1)).gf_camera(border);

  free_camera_delegate_->SetCamera(camera);
  render_task_delegate_->set_camera_and_viewport(
      free_camera_delegate_->GetCameraId(), pxr::GfVec4d(0, 0, resolution_[0], resolution_[1]));
  render_task_delegate_->set_renderer_aov(pxr::HdAovTokens->color);

  if (simple_light_task_delegate_) {
    simple_light_task_delegate_->set_camera_path(free_camera_delegate_->GetCameraId());
    tasks_.push_back(simple_light_task_delegate_->get_task());
  }
  tasks_.push_back(render_task_delegate_->get_task());

  render_images_.emplace(
      "Combined",
      std::vector<float>(resolution_[0] * resolution_[1] * 4)); /* 4 - number of channels. */
}

void FinalEngineGL::render(Depsgraph *depsgraph)
{
  prepare_for_render(depsgraph);

  std::vector<float> &pixels = render_images_["Combined"];

  GPUFrameBuffer *framebuffer = GPU_framebuffer_create("fb_hdyra_render_final");
  GPUTexture *texture = GPU_texture_create_2d("tex_hydra_render_final",
                                              resolution_[0],
                                              resolution_[1],
                                              1,
                                              GPU_RGBA32F,
                                              GPU_TEXTURE_USAGE_GENERAL,
                                              nullptr);
  GPU_texture_filter_mode(texture, true);
  GPU_texture_mipmap_mode(texture, true, true);
  GPU_framebuffer_texture_attach(framebuffer, texture, 0, 0);

  GPU_framebuffer_bind(framebuffer);
  float clear_color[4] = {0.0, 0.0, 0.0, 0.0};
  GPU_framebuffer_clear_color_depth(framebuffer, clear_color, 1.0);

  {
    /* Release the GIL before calling into hydra, in case any hydra plugins call into python. */
    engine_->Execute(render_index_.get(), &tasks_);
  }

  char elapsed_time[32];
  double time_begin = PIL_check_seconds_timer();
  float percent_done = 0.0;

  while (true) {
    if (RE_engine_test_break(bl_engine_)) {
      break;
    }

    percent_done = renderer_percent_done();

    BLI_timecode_string_from_time_simple(
        elapsed_time, sizeof(elapsed_time), PIL_check_seconds_timer() - time_begin);

    notify_status(percent_done / 100.0,
                  scene_name_ + ": " + layer_name_,
                  std::string("Render Time: ") + elapsed_time +
                      " | Done: " + std::to_string(int(percent_done)) + "%");

    if (render_task_delegate_->is_converged()) {
      break;
    }

    void *data = GPU_texture_read(texture, GPU_DATA_FLOAT, 0);
    memcpy(pixels.data(), data, pixels.size() * sizeof(float));
    MEM_freeN(data);
    update_render_result();
  }

  void *data = GPU_texture_read(texture, GPU_DATA_FLOAT, 0);
  memcpy(pixels.data(), data, pixels.size() * sizeof(float));
  MEM_freeN(data);
  update_render_result();

  GPU_framebuffer_free(framebuffer);
  GPU_texture_free(texture);
}

}  // namespace blender::render::hydra
