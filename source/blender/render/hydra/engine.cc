/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include <pxr/imaging/hd/rendererPluginRegistry.h>
#include <pxr/imaging/hdSt/renderDelegate.h>
#include <pxr/imaging/hgi/tokens.h>
#include <pxr/base/plug/plugin.h>
#include <pxr/base/plug/registry.h>
#include <pxr/usd/usdGeom/tokens.h>

#include "glog/logging.h"

#include "engine.h"

using namespace pxr;

namespace blender::render::hydra {

Engine::Engine(RenderEngine *bl_engine, const std::string &delegateId)
    : bl_engine(bl_engine)
{
  HdRendererPluginRegistry& registry = HdRendererPluginRegistry::GetInstance();

  TF_PY_ALLOW_THREADS_IN_SCOPE();
  render_delegate = registry.CreateRenderDelegate(TfToken(delegateId));

  HdDriverVector hd_drivers;
  if (bl_engine->type->flag & RE_USE_GPU_CONTEXT) {
    hgi = Hgi::CreatePlatformDefaultHgi();
    hgi_driver.name = HgiTokens->renderDriver; 
    hgi_driver.driver = VtValue(hgi.get());

    hd_drivers.push_back(&hgi_driver);
  }

  render_index.reset(HdRenderIndex::New(render_delegate.Get(), hd_drivers));
  free_camera_delegate = std::make_unique<HdxFreeCameraSceneDelegate>(
    render_index.get(), SdfPath::AbsoluteRootPath().AppendElementString("freeCamera"));
  render_task_delegate = std::make_unique<RenderTaskDelegate>(
    render_index.get(), SdfPath::AbsoluteRootPath().AppendElementString("renderTask"));
  if (render_delegate->GetRendererDisplayName() == "GL") {
    simple_light_task_delegate = std::make_unique<SimpleLightTaskDelegate>(
        render_index.get(), SdfPath::AbsoluteRootPath().AppendElementString("simpleLightTask"));
  }

  engine = std::make_unique<HdEngine>();
}

Engine::~Engine()
{
  scene_delegate = nullptr;
  render_task_delegate = nullptr;
  free_camera_delegate = nullptr;
  simple_light_task_delegate = nullptr;
  render_index = nullptr;
  render_delegate = nullptr;
  engine = nullptr;
  hgi = nullptr;
}

float Engine::renderer_percent_done()
{
  VtDictionary render_stats = render_delegate->GetRenderStats();
  auto it = render_stats.find("percentDone");
  if (it == render_stats.end()) {
    return 0.0;
  }
  return (float)it->second.UncheckedGet<double>();
}

}   // namespace blender::render::hydra
