/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include <pxr/imaging/hd/sceneDelegate.h>
#include <pxr/imaging/hdx/renderSetupTask.h>
#include <pxr/imaging/hdx/simpleLightTask.h>

namespace blender::render::hydra {

class LightTasksDelegate : public pxr::HdSceneDelegate {
 public:
  LightTasksDelegate(pxr::HdRenderIndex *parentIndex, pxr::SdfPath const &delegate_id);
  ~LightTasksDelegate() override = default;

  pxr::HdTaskSharedPtrVector get_tasks(const bool isTransparent);
  void set_camera_and_viewport(pxr::SdfPath const &camera_id, pxr::GfVec4d const &viewport);

  /* Delegate methods */
  pxr::VtValue Get(pxr::SdfPath const &id, pxr::TfToken const &key) override;

 private:
  pxr::SdfPath simple_task_id_;
  pxr::SdfPath skydome_task_id_;
  pxr::HdxSimpleLightTaskParams simple_task_params_;
  pxr::HdxRenderTaskParams skydome_task_params_;
};

}  // namespace blender::render::hydra
