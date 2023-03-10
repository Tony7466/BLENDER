/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include <pxr/imaging/glf/drawTarget.h>

#include "DEG_depsgraph_query.h"
#include "BKE_lib_id.h"

#include "glog/logging.h"

#include "finalEngine.h"
#include "camera.h"
#include "utils.h"

using namespace std;
using namespace pxr;

namespace blender::render::hydra {

void FinalEngine::sync(Depsgraph *depsgraph, bContext *context, HdRenderSettingsMap &renderSettings)
{
  scene_delegate = std::make_unique<BlenderSceneDelegate>(render_index.get(), 
    SdfPath::AbsoluteRootPath().AppendElementString("scene"), BlenderSceneDelegate::EngineType::Final);
  scene_delegate->populate(depsgraph, context);

  for (auto const& setting : renderSettings) {
    render_delegate->SetRenderSetting(setting.first, setting.second);
  }
}

void FinalEngine::render(Depsgraph *depsgraph)
{
  Scene *scene = DEG_get_input_scene(depsgraph);
  ViewLayer *view_layer = DEG_get_input_view_layer(depsgraph);

  string scene_name(MAX_ID_FULL_NAME, 0);
  BKE_id_full_name_get(scene_name.data(), (ID *)scene, 0);
  string layer_name = view_layer->name;

  RenderData &r = scene->r;
  GfVec4f border(0, 0, 1, 1);
  if (r.mode & R_BORDER) {
    border = GfVec4f(r.border.xmin, r.border.ymin,
                     r.border.xmax - r.border.xmin, r.border.ymax - r.border.ymin);
  }
  GfVec2i image_res(r.xsch * r.size / 100, r.ysch * r.size / 100);
  GfVec2i res(int(image_res[0] * border[2]), int(image_res[1] * border[3]));
  GfCamera camera = CameraData(scene->camera, image_res,  GfVec4f(0, 0, 1, 1)).gf_camera(border);

  free_camera_delegate->SetCamera(camera);
  render_task_delegate->SetCameraAndViewport(free_camera_delegate->GetCameraId(), GfVec4d(0, 0, res[0], res[1]));
  render_task_delegate->SetRendererAov(HdAovTokens->color);
  if (simple_light_task_delegate) {
    simple_light_task_delegate->SetCameraPath(free_camera_delegate->GetCameraId());
  }

  HdTaskSharedPtrVector tasks;
  if (simple_light_task_delegate) {
    tasks.push_back(simple_light_task_delegate->GetTask());
  }
  tasks.push_back(render_task_delegate->GetTask());

  chrono::time_point<chrono::steady_clock> timeBegin = chrono::steady_clock::now(), timeCurrent;
  chrono::milliseconds elapsedTime;

  float percentDone = 0.0;

  map<string, vector<float>> renderImages{
    {"Combined", vector<float>(res[0] * res[1] * 4)}};  // 4 - number of channels
  vector<float> &pixels = renderImages["Combined"];

  {
    // Release the GIL before calling into hydra, in case any hydra plugins call into python.
    TF_PY_ALLOW_THREADS_IN_SCOPE();
    engine->Execute(render_index.get(), &tasks);
  }

  while (true) {
    if (RE_engine_test_break(bl_engine)) {
      break;
    }

    percentDone = renderer_percent_done();
    timeCurrent = chrono::steady_clock::now();
    elapsedTime = chrono::duration_cast<chrono::milliseconds>(timeCurrent - timeBegin);

    notify_status(percentDone / 100.0, scene_name + ": " + layer_name,
      "Render Time: " + format_duration(elapsedTime) + " | Done: " + to_string(int(percentDone)) + "%");

    if (render_task_delegate->IsConverged()) {
      break;
    }

    render_task_delegate->GetRendererAovData(HdAovTokens->color, pixels.data());
    updateRenderResult(renderImages, layer_name, res[0], res[1]);
  }

  render_task_delegate->GetRendererAovData(HdAovTokens->color, pixels.data());
  updateRenderResult(renderImages, layer_name, res[0], res[1]);
}

GfVec2i FinalEngine::get_resolution(Scene *scene)
{
  RenderData &r = scene->r;
  float border_w = 1.0, border_h = 1.0;
  if (r.mode & R_BORDER) {
    border_w = r.border.xmax - r.border.xmin;
    border_h = r.border.ymax - r.border.ymin;
  }
  return GfVec2i(int(r.xsch * border_w * r.size / 100), int(r.ysch * border_h * r.size / 100));
}

void FinalEngine::updateRenderResult(map<string, vector<float>>& renderImages, const string &layerName, int width, int height)
{
  RenderResult *result = RE_engine_begin_result(bl_engine, 0, 0, width, height, layerName.c_str(), nullptr);

  /* TODO: only for the first render layer */
  RenderLayer *layer = (RenderLayer *)result->layers.first;
  for (RenderPass *pass = (RenderPass *)layer->passes.first; pass != nullptr; pass = pass->next) {
    auto it_image = renderImages.find(pass->name);
    if (it_image == renderImages.end()) {
      continue;
    }
    memcpy(pass->rect, it_image->second.data(),
           sizeof(float) * pass->rectx * pass->recty * pass->channels);
  }

  RE_engine_end_result(bl_engine, result, false, false, false);
}

void FinalEngine::notify_status(float progress, const string &title, const string &info)
{
  RE_engine_update_progress(bl_engine, progress);
  RE_engine_update_stats(bl_engine, title.c_str(), info.c_str());
}

void FinalEngineGL::render(Depsgraph *depsgraph)
{
  Scene *scene = DEG_get_input_scene(depsgraph);
  ViewLayer *view_layer = DEG_get_input_view_layer(depsgraph);
  
  string scene_name(MAX_ID_FULL_NAME, 0);
  BKE_id_full_name_get(scene_name.data(), (ID *)scene, 0);
  string layer_name = view_layer->name;

  RenderData &r = scene->r;
  GfVec4f border(0, 0, 1, 1);
  if (r.mode & R_BORDER) {
    border = GfVec4f(r.border.xmin, r.border.ymin,
                     r.border.xmax - r.border.xmin, r.border.ymax - r.border.ymin);
  }
  GfVec2i image_res = {r.xsch * r.size / 100, r.ysch * r.size / 100};
  GfVec2i res = {int(image_res[0] * border[2]), int(image_res[1] * border[3])};
  GfCamera camera = CameraData(scene->camera, image_res,  GfVec4f(0, 0, 1, 1)).gf_camera(border);


  free_camera_delegate->SetCamera(camera);
  render_task_delegate->SetCameraAndViewport(free_camera_delegate->GetCameraId(),
                                           GfVec4d(0, 0, res[0], res[1]));
  if (simple_light_task_delegate) {
    simple_light_task_delegate->SetCameraPath(free_camera_delegate->GetCameraId());
  }

  HdTaskSharedPtrVector tasks;
  if (simple_light_task_delegate) {
    /* TODO: Uncomment this and fix GL error:
         invalid operation, reported from void __cdecl pxrInternal_v0_22__pxrReserved__::HgiGLResourceBindings::BindResources(void) */
    // tasks.push_back(simple_light_task_delegate->GetTask());
  }
  tasks.push_back(render_task_delegate->GetTask());

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
    engine->Execute(render_index.get(), &tasks);
  }

  while (true) {
    if (RE_engine_test_break(bl_engine)) {
      break;
    }

    percentDone = renderer_percent_done();
    timeCurrent = chrono::steady_clock::now();
    elapsedTime = chrono::duration_cast<chrono::milliseconds>(timeCurrent - timeBegin);

    notify_status(percentDone / 100.0,
                 scene_name + ": " + layer_name,
                 "Render Time: " + format_duration(elapsedTime) +
                     " | Done: " + to_string(int(percentDone)) + "%");

    if (render_task_delegate->IsConverged()) {
      break;
    }

    glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_FLOAT, pixels.data());
    updateRenderResult(renderImages, layer_name, res[0], res[1]);
  }

  glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_FLOAT, pixels.data());
  updateRenderResult(renderImages, layer_name, res[0], res[1]);
}

}   // namespace blender::render::hydra
