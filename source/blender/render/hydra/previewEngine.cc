/* SPDX-License-Identifier: Apache-2.0
* Copyright 2011-2022 Blender Foundation */

#include <pxr/usdImaging/usdAppUtils/camera.h>

#include "previewEngine.h"
#include "camera.h"

using namespace pxr;
using namespace std;

namespace blender::render::hydra {

void PreviewEngine::sync(BL::Depsgraph &b_depsgraph, BL::Context &b_context, HdRenderSettingsMap &renderSettings)
{
  sceneDelegate = std::make_unique<BlenderSceneDelegate>(renderIndex.get(), 
    SdfPath::AbsoluteRootPath().AppendElementString("scene"), BlenderSceneDelegate::EngineType::Preview);
  sceneDelegate->populate((Depsgraph *)b_depsgraph.ptr.data, (bContext *)b_context.ptr.data);

  for (auto const& setting : renderSettings) {
    renderDelegate->SetRenderSetting(setting.first, setting.second);
  }
}

void PreviewEngine::render(BL::Depsgraph &b_depsgraph)
{
  BL::Scene b_scene = b_depsgraph.scene();
  string layerName = b_depsgraph.view_layer().name();
  GfVec2i buffer_res {b_scene.render().resolution_x(),
                      b_scene.render().resolution_y()};

  GfCamera gfCamera = CameraData((Object *)b_scene.camera().ptr.data, buffer_res, GfVec4f(0, 0, 1, 1)).gf_camera(GfVec4f(0, 0, 1, 1));

  freeCameraDelegate->SetCamera(gfCamera);
  renderTaskDelegate->SetCameraAndViewport(freeCameraDelegate->GetCameraId(), GfVec4d(0, 0, buffer_res[0], buffer_res[1]));
  renderTaskDelegate->SetRendererAov(HdAovTokens->color);

  HdTaskSharedPtrVector tasks;
  if (simpleLightTaskDelegate) {
    tasks.push_back(simpleLightTaskDelegate->GetTask());
  }
  tasks.push_back(renderTaskDelegate->GetTask());

  vector<float> pixels = vector<float>(buffer_res[0] * buffer_res[1] * 4);  // 4 - number of channels

  {
    // Release the GIL before calling into hydra, in case any hydra plugins call into python.
    TF_PY_ALLOW_THREADS_IN_SCOPE();
    engine->Execute(renderIndex.get(), &tasks);
  }

  while (true) {
    if (b_engine.test_break()) {
      break;
    }

    if (renderTaskDelegate->IsConverged()) {
      break;
    }

    renderTaskDelegate->GetRendererAovData(HdAovTokens->color, pixels.data());
    updateRenderResult(layerName, buffer_res[0], buffer_res[1], pixels);
  }

  renderTaskDelegate->GetRendererAovData(HdAovTokens->color, pixels.data());
  updateRenderResult(layerName, buffer_res[0], buffer_res[1], pixels);
}

void PreviewEngine::updateRenderResult(const string &layerName, int width, int height, vector<float> &pixels)
{
  BL::RenderResult b_result = b_engine.begin_result(0, 0, width, height, layerName.c_str(), NULL);
  BL::CollectionRef b_passes = b_result.layers[0].passes;
  b_passes[0].rect(pixels.data());
  b_engine.end_result(b_result, false, false, false);
}

}   // namespace blender::render::hydra
