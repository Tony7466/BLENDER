/* SPDX-License-Identifier: GPL-2.0-or-later
 * SPDX-FileCopyrightText: 2011-2022 Blender Foundation */

#include "final_engine.h"

#include <pxr/imaging/hd/light.h>
#include <pxr/imaging/hd/renderBuffer.h>

#include "DNA_scene_types.h"

#include "BLI_timecode.h"
#include "PIL_time.h"

#include "BKE_lib_id.h"

#include "DEG_depsgraph_query.h"

#include "IMB_imbuf_types.h"

#include "RE_engine.h"

#include "hydra/camera.h"

namespace blender::render::hydra {

/* FinalEngine implementation */

void FinalEngine::render(Depsgraph *depsgraph)
{
  const Scene *scene = DEG_get_evaluated_scene(depsgraph);
  const ViewLayer *view_layer = DEG_get_evaluated_view_layer(depsgraph);

  char scene_name[MAX_ID_FULL_NAME];
  BKE_id_full_name_get(scene_name, &scene->id, 0);
  scene_name_ = scene_name;
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
      io::hydra::CameraData(scene->camera, image_res, pxr::GfVec4f(0, 0, 1, 1)).gf_camera(border);

  free_camera_delegate_->SetCamera(camera);
  render_task_delegate_->set_viewport(pxr::GfVec4d(0, 0, resolution_[0], resolution_[1]));
  if (light_tasks_delegate_) {
    light_tasks_delegate_->set_viewport(pxr::GfVec4d(0, 0, resolution_[0], resolution_[1]));
  }

  render_task_delegate_->add_aov(pxr::HdAovTokens->color);
  render_task_delegate_->add_aov(pxr::HdAovTokens->depth);

  pxr::HdTaskSharedPtrVector tasks;
  if (light_tasks_delegate_) {
    if (scene->r.alphamode != R_ALPHAPREMUL) {
      tasks.push_back(light_tasks_delegate_->skydome_task());
    }
    tasks.push_back(light_tasks_delegate_->simple_task());
  }
  tasks.push_back(render_task_delegate_->task());

  render_task_delegate_->bind();

  engine_->Execute(render_index_.get(), &tasks);

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

    update_render_result();
  }

  update_render_result();
  render_task_delegate_->unbind();
}

void FinalEngine::notify_status(float progress, const std::string &title, const std::string &info)
{
  RE_engine_update_progress(bl_engine_, progress);
  RE_engine_update_stats(bl_engine_, title.c_str(), info.c_str());
}

void FinalEngine::update_render_result()
{
  RenderResult *rr = RE_engine_begin_result(
      bl_engine_, 0, 0, resolution_[0], resolution_[1], layer_name_.c_str(), nullptr);

  RenderLayer *rlayer = static_cast<RenderLayer *>(
      BLI_findstring(&rr->layers, layer_name_.c_str(), offsetof(RenderLayer, name)));

  if (rlayer) {
    LISTBASE_FOREACH (RenderPass *, rpass, &rlayer->passes) {
      pxr::TfToken aov_key;
      if (STREQ(rpass->name, "Combined")) {
        aov_key = pxr::HdAovTokens->color;
      }
      else if (STREQ(rpass->name, "Depth")) {
        aov_key = pxr::HdAovTokens->depth;
      }
      render_task_delegate_->read_aov(aov_key, rpass->ibuf->float_buffer.data);
    }
  }

  RE_engine_end_result(bl_engine_, rr, false, false, false);
}

}  // namespace blender::render::hydra
