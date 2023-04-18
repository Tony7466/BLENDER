/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include <pxr/imaging/hd/sceneDelegate.h>
#include <pxr/imaging/hdx/simpleLightTask.h>

namespace blender::render::hydra {

class SimpleLightTaskDelegate : public pxr::HdSceneDelegate {
 public:
  SimpleLightTaskDelegate(pxr::HdRenderIndex *parentIndex, pxr::SdfPath const &delegate_id);
  ~SimpleLightTaskDelegate() override = default;

  pxr::SdfPath get_task_id() const;

  pxr::HdTaskSharedPtr get_task();
  void set_camera_path(pxr::SdfPath const &);

  /* Delegate methods */
  pxr::VtValue Get(pxr::SdfPath const &id, pxr::TfToken const &key) override;

 private:
  pxr::HdxSimpleLightTaskParams task_params_;
};

}  // namespace blender::render::hydra
