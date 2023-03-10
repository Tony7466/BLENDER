/* SPDX-License-Identifier: Apache-2.0
* Copyright 2011-2022 Blender Foundation */

#include "DEG_depsgraph_query.h"

#include "previewEngine.h"
#include "camera.h"

using namespace pxr;
using namespace std;

namespace blender::render::hydra {

void PreviewEngine::sync(Depsgraph *depsgraph, bContext *context, HdRenderSettingsMap &renderSettings)
{
  scene_delegate = make_unique<BlenderSceneDelegate>(render_index.get(), 
    SdfPath::AbsoluteRootPath().AppendElementString("scene"), BlenderSceneDelegate::EngineType::Preview);
  scene_delegate->populate(depsgraph, context);

  for (auto const& setting : renderSettings) {
    render_delegate->SetRenderSetting(setting.first, setting.second);
  }
}

void PreviewEngine::render(Depsgraph *depsgraph)
{
  Scene *scene = DEG_get_input_scene(depsgraph);
  ViewLayer *view_layer = DEG_get_input_view_layer(depsgraph);

  string layerName = view_layer->name;
  GfVec2i res(scene->r.xsch, scene->r.ysch);

  GfCamera camera = CameraData(scene->camera, res, GfVec4f(0, 0, 1, 1)).gf_camera(GfVec4f(0, 0, 1, 1));

  free_camera_delegate->SetCamera(camera);
  render_task_delegate->SetCameraAndViewport(free_camera_delegate->GetCameraId(), GfVec4d(0, 0, res[0], res[1]));
  render_task_delegate->SetRendererAov(HdAovTokens->color);

  HdTaskSharedPtrVector tasks;
  if (simple_light_task_delegate) {
    tasks.push_back(simple_light_task_delegate->GetTask());
  }
  tasks.push_back(render_task_delegate->GetTask());

  vector<float> pixels = vector<float>(res[0] * res[1] * 4);  // 4 - number of channels

  {
    // Release the GIL before calling into hydra, in case any hydra plugins call into python.
    TF_PY_ALLOW_THREADS_IN_SCOPE();
    engine->Execute(render_index.get(), &tasks);
  }

  while (true) {
    if (RE_engine_test_break(bl_engine)) {
      break;
    }

    if (render_task_delegate->IsConverged()) {
      break;
    }

    render_task_delegate->GetRendererAovData(HdAovTokens->color, pixels.data());
    updateRenderResult(layerName, res[0], res[1], pixels);
  }

  render_task_delegate->GetRendererAovData(HdAovTokens->color, pixels.data());
  updateRenderResult(layerName, res[0], res[1], pixels);
}

void PreviewEngine::updateRenderResult(const string &layerName, int width, int height, vector<float> &pixels)
{
  RenderResult *result = RE_engine_begin_result(bl_engine, 0, 0, width, height, layerName.c_str(), nullptr);

  RenderLayer *layer = (RenderLayer *)result->layers.first;
  RenderPass *pass = (RenderPass *)layer->passes.first;
  memcpy(pass->rect, pixels.data(), sizeof(float) * pass->rectx * pass->recty * pass->channels);

  RE_engine_end_result(bl_engine, result, false, false, false);
}

}   // namespace blender::render::hydra
