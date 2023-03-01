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

Engine::Engine(BL::RenderEngine &b_engine, const std::string &delegateId)
  : b_engine(b_engine)
{
  HdRendererPluginRegistry& registry = HdRendererPluginRegistry::GetInstance();

  TF_PY_ALLOW_THREADS_IN_SCOPE();
  renderDelegate = registry.CreateRenderDelegate(TfToken(delegateId));

  HdDriverVector hdDrivers;

  if (b_engine.bl_use_gpu_context()) {
    hgi = Hgi::CreatePlatformDefaultHgi();
    hgiDriver.name = HgiTokens->renderDriver; 
    hgiDriver.driver = VtValue(hgi.get());

    hdDrivers.push_back(&hgiDriver);
  }

  renderIndex.reset(HdRenderIndex::New(renderDelegate.Get(), hdDrivers));
  freeCameraDelegate = std::make_unique<HdxFreeCameraSceneDelegate>(
    renderIndex.get(), SdfPath::AbsoluteRootPath().AppendElementString("freeCamera"));
  renderTaskDelegate = std::make_unique<RenderTaskDelegate>(
    renderIndex.get(), SdfPath::AbsoluteRootPath().AppendElementString("renderTask"));
  if (renderDelegate->GetRendererDisplayName() == "GL") {
    simpleLightTaskDelegate = std::make_unique<SimpleLightTaskDelegate>(
        renderIndex.get(), SdfPath::AbsoluteRootPath().AppendElementString("simpleLightTask"));
  }

  engine = std::make_unique<HdEngine>();
}

Engine::~Engine()
{
  sceneDelegate = nullptr;
  renderTaskDelegate = nullptr;
  freeCameraDelegate = nullptr;
  simpleLightTaskDelegate = nullptr;
  renderIndex = nullptr;
  renderDelegate = nullptr;
  engine = nullptr;
  hgi = nullptr;
}

float Engine::getRendererPercentDone()
{
  VtDictionary render_stats = renderDelegate->GetRenderStats();
  auto it = render_stats.find("percentDone");
  if (it == render_stats.end()) {
    return 0.0;
  }
  return (float)it->second.UncheckedGet<double>();
}

}   // namespace blender::render::hydra
