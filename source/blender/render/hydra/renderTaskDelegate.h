/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include <pxr/imaging/hdx/renderSetupTask.h>
#include <pxr/imaging/hd/sceneDelegate.h>

using namespace pxr;

namespace blender::render::hydra {

class RenderTaskDelegate : public HdSceneDelegate
{
public:
  RenderTaskDelegate(HdRenderIndex* parentIndex, SdfPath const &delegateID);
  ~RenderTaskDelegate() override = default;

  SdfPath GetTaskID() const;
  SdfPath GetAovID(TfToken const &aov) const;

  VtValue Get(SdfPath const &id, TfToken const &key) override;
  HdRenderBufferDescriptor GetRenderBufferDescriptor(SdfPath const &id) override;
  TfTokenVector GetTaskRenderTags(SdfPath const &taskId) override;

  bool IsConverged();
  void SetRendererAov(TfToken const &aovId);
  HdRenderBuffer *GetRendererAov(TfToken const &id);
  void GetRendererAovData(TfToken const &id, void *buf);

  HdTaskSharedPtr GetTask();
  void SetCameraAndViewport(SdfPath const &cameraId, GfVec4d const &viewport);

private:
  HdxRenderTaskParams taskParams;
  TfHashMap<SdfPath, HdRenderBufferDescriptor, SdfPath::Hash> bufferDescriptors;
};

} // namespace blender::render::hydra
