/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include <pxr/imaging/hd/sceneDelegate.h>
#include <pxr/imaging/hdx/renderSetupTask.h>

namespace blender::render::hydra {

class RenderTaskDelegate : public pxr::HdSceneDelegate {
 public:
  RenderTaskDelegate(pxr::HdRenderIndex *parent_index, pxr::SdfPath const &delegate_id);
  ~RenderTaskDelegate() override = default;

  pxr::SdfPath get_task_id() const;
  pxr::SdfPath get_aov_id(pxr::TfToken const &aov) const;

  bool is_converged();
  void set_renderer_aov(pxr::TfToken const &aovId);
  pxr::HdRenderBuffer *get_renderer_aov(pxr::TfToken const &id);
  void get_renderer_aov_data(pxr::TfToken const &id, void *buf);

  pxr::HdTaskSharedPtr get_task();
  void set_camera_and_viewport(pxr::SdfPath const &cameraId, pxr::GfVec4d const &viewport);

  /* Delegate methods */
  pxr::VtValue Get(pxr::SdfPath const &id, pxr::TfToken const &key) override;
  pxr::HdRenderBufferDescriptor GetRenderBufferDescriptor(pxr::SdfPath const &id) override;
  pxr::TfTokenVector GetTaskRenderTags(pxr::SdfPath const &taskId) override;

 private:
  pxr::HdxRenderTaskParams task_params_;
  pxr::TfHashMap<pxr::SdfPath, pxr::HdRenderBufferDescriptor, pxr::SdfPath::Hash>
      buffer_descriptors_;
};

}  // namespace blender::render::hydra
