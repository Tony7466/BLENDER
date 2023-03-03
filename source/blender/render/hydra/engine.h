/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include <chrono>

#include <Python.h>

#include <pxr/imaging/hd/engine.h>
#include <pxr/imaging/hd/pluginRenderDelegateUniqueHandle.h>
#include <pxr/imaging/hd/driver.h>
#include <pxr/imaging/hdx/freeCameraSceneDelegate.h>
#include <pxr/imaging/hgi/hgi.h>

#include "MEM_guardedalloc.h"
#include "RNA_blender_cpp.h"

#include "sceneDelegate/blenderSceneDelegate.h"
#include "renderTaskDelegate.h"
#include "simpleLightTaskDelegate.h"

namespace blender::render::hydra {

class Engine {
public:
  Engine(BL::RenderEngine &b_engine, const std::string &delegateId);
  virtual ~Engine();

  virtual void sync(BL::Depsgraph &b_depsgraph, BL::Context &b_context, pxr::HdRenderSettingsMap &renderSettings) = 0;
  virtual void render(BL::Depsgraph &b_depsgraph) = 0;

protected:
  float getRendererPercentDone();

protected:
  BL::RenderEngine b_engine;

  HdPluginRenderDelegateUniqueHandle renderDelegate;
  std::unique_ptr<HdRenderIndex> renderIndex;
  std::unique_ptr<BlenderSceneDelegate> sceneDelegate;
  std::unique_ptr<RenderTaskDelegate> renderTaskDelegate;
  std::unique_ptr<HdxFreeCameraSceneDelegate> freeCameraDelegate;
  std::unique_ptr<SimpleLightTaskDelegate> simpleLightTaskDelegate;
  std::unique_ptr<HdEngine> engine;

  HgiUniquePtr hgi;
  HdDriver hgiDriver;
};

}   // namespace blender::render::hydra
