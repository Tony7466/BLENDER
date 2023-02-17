/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include <memory>

#include <pxr/imaging/hd/engine.h>
#include <pxr/imaging/hdx/freeCameraSceneDelegate.h>
#include <pxr/imaging/glf/drawTarget.h>
#include <pxr/usdImaging/usdAppUtils/camera.h>

#include "glog/logging.h"

#include "finalEngine.h"
#include "camera.h"
#include "utils.h"

using namespace std;
using namespace pxr;

namespace blender::render::hydra {

void FinalEngine::sync(BL::Depsgraph &b_depsgraph, BL::Context &b_context, pxr::HdRenderSettingsMap &renderSettings)
{
  sceneDelegate = std::make_unique<BlenderSceneDelegate>(renderIndex.get(), 
    SdfPath::AbsoluteRootPath().AppendElementString("scene"));
  sceneDelegate->Populate(b_depsgraph, b_context);

  for (auto const& setting : renderSettings) {
    renderDelegate->SetRenderSetting(setting.first, setting.second);
  }
}

void FinalEngine::render(BL::Depsgraph &b_depsgraph)
{
  BL::Scene b_scene = b_depsgraph.scene();
  BL::ViewLayer b_view_layer = b_depsgraph.view_layer();
  string sceneName = b_scene.name(), layerName = b_view_layer.name();
  GfVec2i res = get_resolution(b_scene.render());
  GfCamera gfCamera = CameraData((Object *)b_scene.camera().ptr.data, res, GfVec4f(0, 0, 1, 1)).gf_camera();
  freeCameraDelegate->SetCamera(gfCamera);
  renderTaskDelegate->SetCameraAndViewport(freeCameraDelegate->GetCameraId(), GfVec4d(0, 0, res[0], res[1]));
  renderTaskDelegate->SetRendererAov(HdAovTokens->color);
  
  HdTaskSharedPtrVector tasks = renderTaskDelegate->GetTasks();

  chrono::time_point<chrono::steady_clock> timeBegin = chrono::steady_clock::now(), timeCurrent;
  chrono::milliseconds elapsedTime;

  float percentDone = 0.0;

  map<string, vector<float>> renderImages{
    {"Combined", vector<float>(res[0] * res[1] * 4)}};  // 4 - number of channels
  vector<float> &pixels = renderImages["Combined"];

  {
    // Release the GIL before calling into hydra, in case any hydra plugins call into python.
    TF_PY_ALLOW_THREADS_IN_SCOPE();
    engine->Execute(renderIndex.get(), &tasks);
  }

  while (true) {
    if (b_engine.test_break()) {
      break;
    }

    percentDone = getRendererPercentDone();
    timeCurrent = chrono::steady_clock::now();
    elapsedTime = chrono::duration_cast<chrono::milliseconds>(timeCurrent - timeBegin);

    notifyStatus(percentDone / 100.0, sceneName + ": " + layerName,
      "Render Time: " + format_duration(elapsedTime) + " | Done: " + to_string(int(percentDone)) + "%");

    if (renderTaskDelegate->IsConverged()) {
      break;
    }

    renderTaskDelegate->GetRendererAovData(HdAovTokens->color, pixels.data());
    updateRenderResult(renderImages, layerName, res[0], res[1]);
  }

  renderTaskDelegate->GetRendererAovData(HdAovTokens->color, pixels.data());
  updateRenderResult(renderImages, layerName, res[0], res[1]);
}

GfVec2i FinalEngine::get_resolution(BL::RenderSettings b_render)
{
  float border_w = 1.0, border_h = 1.0;
  if (b_render.use_border()) {
    border_w = b_render.border_max_x() - b_render.border_min_x();
    border_h = b_render.border_max_y() - b_render.border_min_y();
  }

  return GfVec2i(int(b_render.resolution_x() * border_w * b_render.resolution_percentage() / 100),
                 int(b_render.resolution_y() * border_h * b_render.resolution_percentage() / 100));
}

void FinalEngine::updateRenderResult(map<string, vector<float>>& renderImages, const string &layerName, int width, int height)
{
  BL::RenderResult b_result = b_engine.begin_result(0, 0, width, height, layerName.c_str(), NULL);
  BL::CollectionRef b_passes = b_result.layers[0].passes;

  for (BL::RenderPass b_pass : b_passes) {
    auto it_image = renderImages.find(b_pass.name());
    if (it_image == renderImages.end()) {
      continue;
    }
    b_pass.rect(it_image->second.data());
  }
  b_engine.end_result(b_result, false, false, false);
}

void FinalEngine::notifyStatus(float progress, const string &title, const string &info)
{
  b_engine.update_progress(progress);
  b_engine.update_stats(title.c_str(), info.c_str());
}

void FinalEngineGL::render(BL::Depsgraph &b_depsgraph)
{
  BL::Scene b_scene = b_depsgraph.scene();
  BL::ViewLayer b_view_layer = b_depsgraph.view_layer();
  string sceneName = b_scene.name(), layerName = b_view_layer.name();
  GfVec2i res = get_resolution(b_scene.render());
  GfCamera gfCamera = CameraData((Object *)b_scene.camera().ptr.data, res, GfVec4f(0, 0, 1, 1)).gf_camera();
  freeCameraDelegate->SetCamera(gfCamera);
  renderTaskDelegate->SetCameraAndViewport(freeCameraDelegate->GetCameraId(), GfVec4d(0, 0, res[0], res[1]));

  HdTaskSharedPtrVector tasks = renderTaskDelegate->GetTasks();

  chrono::time_point<chrono::steady_clock> timeBegin = chrono::steady_clock::now(), timeCurrent;
  chrono::milliseconds elapsedTime;

  float percentDone = 0.0;

  map<string, vector<float>> renderImages{
    {"Combined", vector<float>(res[0] * res[1] * 4)}};  // 4 - number of channels
  vector<float> &pixels = renderImages["Combined"];

  GLuint FramebufferName = 0;
  glGenFramebuffers(1, &FramebufferName);
  glBindFramebuffer(GL_FRAMEBUFFER, FramebufferName);

  // The texture we're going to render to
  GLuint renderedTexture;
  glGenTextures(1, &renderedTexture);

  // "Bind" the newly created texture : all future texture functions will modify this texture
  glBindTexture(GL_TEXTURE_2D, renderedTexture);

  // Give an empty image to OpenGL ( the last "0" )
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, res[0], res[1], 0, GL_RGBA, GL_FLOAT, 0);

  // Poor filtering. Needed !
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

  // Set "renderedTexture" as our colour attachement #0
  glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, renderedTexture, 0);

  // Generate vertex array
  GLuint VAO;
  glGenVertexArrays(1, &VAO);
  glBindVertexArray(VAO);

  {
    // Release the GIL before calling into hydra, in case any hydra plugins call into python.
    TF_PY_ALLOW_THREADS_IN_SCOPE();
    engine->Execute(renderIndex.get(), &tasks);
  }

  while (true) {
    if (b_engine.test_break()) {
      break;
    }

    percentDone = getRendererPercentDone();
    timeCurrent = chrono::steady_clock::now();
    elapsedTime = chrono::duration_cast<chrono::milliseconds>(timeCurrent - timeBegin);

    notifyStatus(percentDone / 100.0,
                 sceneName + ": " + layerName,
                 "Render Time: " + format_duration(elapsedTime) +
                     " | Done: " + to_string(int(percentDone)) + "%");

    if (renderTaskDelegate->IsConverged()) {
      break;
    }

    glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_FLOAT, pixels.data());
    updateRenderResult(renderImages, layerName, res[0], res[1]);
  }

  glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_FLOAT, pixels.data());
  updateRenderResult(renderImages, layerName, res[0], res[1]);
}

}   // namespace blender::render::hydra
