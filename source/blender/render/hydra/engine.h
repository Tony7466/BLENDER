/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include <chrono>

#include <Python.h>

#include <pxr/imaging/hd/driver.h>
#include <pxr/imaging/hd/engine.h>
#include <pxr/imaging/hd/pluginRenderDelegateUniqueHandle.h>
#include <pxr/imaging/hdx/freeCameraSceneDelegate.h>
#include <pxr/imaging/hgi/hgi.h>

#include "RE_engine.h"

#include "CLG_log.h"

#include "render_task_delegate.h"
#include "scene_delegate/blender_scene_delegate.h"
#include "simple_light_task_delegate.h"

namespace blender::render::hydra {

extern struct CLG_LogRef *LOG_EN; /* EN - Engine */

class Engine {
 public:
  Engine(RenderEngine *bl_engine, const std::string &render_delegate_id);
  virtual ~Engine();

  virtual void sync(Depsgraph *depsgraph,
                    bContext *context,
                    pxr::HdRenderSettingsMap &render_settings) = 0;
  virtual void render(Depsgraph *depsgraph) = 0;

 protected:
  float renderer_percent_done();

 protected:
  RenderEngine *bl_engine;

  pxr::HdPluginRenderDelegateUniqueHandle render_delegate;
  std::unique_ptr<pxr::HdRenderIndex> render_index;
  std::unique_ptr<BlenderSceneDelegate> scene_delegate;
  std::unique_ptr<RenderTaskDelegate> render_task_delegate;
  std::unique_ptr<pxr::HdxFreeCameraSceneDelegate> free_camera_delegate;
  std::unique_ptr<SimpleLightTaskDelegate> simple_light_task_delegate;
  std::unique_ptr<pxr::HdEngine> engine;

  pxr::HgiUniquePtr hgi;
  pxr::HdDriver hgi_driver;
};

}  // namespace blender::render::hydra
