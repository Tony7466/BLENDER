/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include "DEG_depsgraph_query.h"

#include "camera.h"
#include "preview_engine.h"
#include "BLI_timer.h"

namespace blender::render::hydra {

const double LIFETIME = 180.0;

std::unique_ptr<PreviewEngine> PreviewEngine::instance;

void PreviewEngine::schedule_free()
{
  instance->render_delegate->Stop();

  /* Register timer for schedule free PreviewEngine instance */
  BLI_timer_register((uintptr_t)instance.get(),
                      free_instance,
                      nullptr,
                      nullptr,
                      LIFETIME,
                      true);
}

PreviewEngine *PreviewEngine::get_instance(RenderEngine *bl_engine, const std::string &render_delegate_id)
{
  if (!instance) {
    instance = std::make_unique<PreviewEngine>(bl_engine, render_delegate_id);
  }
  if (BLI_timer_is_registered((uintptr_t)instance.get())) {
    /* Unregister timer while PreviewEngine is working */
    BLI_timer_unregister((uintptr_t)instance.get());
  }
  instance->update(bl_engine, render_delegate_id);

  return instance.get();
}

double PreviewEngine::free_instance(uintptr_t uuid, void *user_data)
{
  if (!instance->render_task_delegate->is_converged()) {
    /* Restart timer if render isn't completed */
    return LIFETIME;
  }

  CLOG_INFO(LOG_EN, 2, "");
  instance = nullptr;
  return -1;
}

void PreviewEngine::sync(Depsgraph *depsgraph,
                         bContext *context,
                         pxr::HdRenderSettingsMap &render_settings)
{
  if (!scene_delegate) {
    scene_delegate = std::make_unique<BlenderSceneDelegate>(
        render_index.get(),
        pxr::SdfPath::AbsoluteRootPath().AppendElementString("scene"),
        BlenderSceneDelegate::EngineType::PREVIEW);
  }
  scene_delegate->clear();
  scene_delegate->populate(depsgraph, context);

  for (auto const &setting : render_settings) {
    render_delegate->SetRenderSetting(setting.first, setting.second);
  }
}

void PreviewEngine::render(Depsgraph *depsgraph)
{
  Scene *scene = DEG_get_input_scene(depsgraph);
  ViewLayer *view_layer = DEG_get_input_view_layer(depsgraph);

  std::string layer_name = view_layer->name;
  pxr::GfVec2i res(scene->r.xsch, scene->r.ysch);

  pxr::GfCamera camera =
      CameraData(scene->camera, res, pxr::GfVec4f(0, 0, 1, 1)).gf_camera(pxr::GfVec4f(0, 0, 1, 1));

  free_camera_delegate->SetCamera(camera);
  render_task_delegate->set_camera_and_viewport(free_camera_delegate->GetCameraId(),
                                                pxr::GfVec4d(0, 0, res[0], res[1]));
  render_task_delegate->set_renderer_aov(pxr::HdAovTokens->color);

  pxr::HdTaskSharedPtrVector tasks;
  if (simple_light_task_delegate) {
    tasks.push_back(simple_light_task_delegate->get_task());
  }
  tasks.push_back(render_task_delegate->get_task());

  std::vector<float> pixels = std::vector<float>(res[0] * res[1] * 4);  // 4 - number of channels

  {
    // Release the GIL before calling into hydra, in case any hydra plugins call into python.
    pxr::TF_PY_ALLOW_THREADS_IN_SCOPE();
    engine->Execute(render_index.get(), &tasks);
  }

  while (true) {
    if (RE_engine_test_break(bl_engine)) {
      break;
    }

    if (render_task_delegate->is_converged()) {
      break;
    }

    render_task_delegate->get_renderer_aov_data(pxr::HdAovTokens->color, pixels.data());
    update_render_result(layer_name, res[0], res[1], pixels);
  }

  render_task_delegate->get_renderer_aov_data(pxr::HdAovTokens->color, pixels.data());
  update_render_result(layer_name, res[0], res[1], pixels);
}

void PreviewEngine::update(RenderEngine *bl_engine, const std::string &render_delegate_id)
{
  this->bl_engine = bl_engine;
  /* TODO: recreate render_delegate when render_delegate_id is changed */
}

void PreviewEngine::update_render_result(const std::string &layer_name,
                                         int width,
                                         int height,
                                         std::vector<float> &pixels)
{
  RenderResult *result = RE_engine_begin_result(
      bl_engine, 0, 0, width, height, layer_name.c_str(), nullptr);

  RenderLayer *layer = (RenderLayer *)result->layers.first;
  RenderPass *pass = (RenderPass *)layer->passes.first;
  memcpy(pass->rect, pixels.data(), sizeof(float) * pass->rectx * pass->recty * pass->channels);

  RE_engine_end_result(bl_engine, result, false, false, false);
}

}  // namespace blender::render::hydra
