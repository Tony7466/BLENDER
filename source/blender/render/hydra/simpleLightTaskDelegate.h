/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include <pxr/imaging/hd/sceneDelegate.h>
#include <pxr/imaging/hdx/simpleLightTask.h>

using namespace pxr;

namespace blender::render::hydra {

class SimpleLightTaskDelegate : public HdSceneDelegate {
 public:
  SimpleLightTaskDelegate(HdRenderIndex *parentIndex, SdfPath const &delegateID);
  ~SimpleLightTaskDelegate() override = default;

  SdfPath GetTaskID() const;

  HdTaskSharedPtr GetTask();
  void SetCameraPath(SdfPath const &);

  VtValue Get(SdfPath const &id, TfToken const &key) override;

 private:
  HdxSimpleLightTaskParams taskParams;
};

}  // namespace blender::render::hydra
