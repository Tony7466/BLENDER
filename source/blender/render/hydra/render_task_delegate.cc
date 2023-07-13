/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include <iostream>

#include <pxr/imaging/hd/renderBuffer.h>
#include <pxr/imaging/hd/renderDelegate.h>
#include <pxr/imaging/hdx/renderTask.h>

#include "render_task_delegate.h"

namespace blender::render::hydra {

RenderTaskDelegate::RenderTaskDelegate(pxr::HdRenderIndex *parent_index,
                                       pxr::SdfPath const &delegate_id)
    : pxr::HdSceneDelegate(parent_index, delegate_id)
{
  pxr::SdfPath render_task_id = get_task_id();
  GetRenderIndex().InsertTask<pxr::HdxRenderTask>(this, render_task_id);
  GetRenderIndex().GetChangeTracker().MarkTaskDirty(render_task_id,
                                                    pxr::HdChangeTracker::DirtyCollection);
  GetRenderIndex().GetChangeTracker().MarkTaskDirty(render_task_id,
                                                    pxr::HdChangeTracker::DirtyRenderTags);

  task_params_.enableLighting = true;
  task_params_.alphaThreshold = 0.1f;
}

pxr::SdfPath RenderTaskDelegate::get_task_id() const
{
  return GetDelegateID().AppendElementString("task");
}

pxr::SdfPath RenderTaskDelegate::get_aov_id(pxr::TfToken const &aov) const
{
  return GetDelegateID().AppendElementString("aov_" + aov.GetString());
}

bool RenderTaskDelegate::is_converged()
{
  pxr::HdTaskSharedPtr renderTask = GetRenderIndex().GetTask(get_task_id());
  return ((pxr::HdxRenderTask &)*renderTask).IsConverged();
}

void RenderTaskDelegate::set_renderer_aov(pxr::TfToken const &aov)
{
  pxr::HdAovDescriptor aov_desc = GetRenderIndex().GetRenderDelegate()->GetDefaultAovDescriptor(
      aov);
  pxr::HdRenderBufferDescriptor desc(
      pxr::GfVec3i(task_params_.viewport[2], task_params_.viewport[3], 1),
      aov_desc.format,
      aov_desc.multiSampled);
  pxr::SdfPath buffer_id = get_aov_id(aov);

  if (buffer_descriptors_.find(buffer_id) == buffer_descriptors_.end()) {
    GetRenderIndex().InsertBprim(pxr::HdPrimTypeTokens->renderBuffer, this, buffer_id);
    buffer_descriptors_[buffer_id] = desc;
    GetRenderIndex().GetChangeTracker().MarkBprimDirty(buffer_id,
                                                       pxr::HdRenderBuffer::DirtyDescription);

    pxr::HdRenderPassAovBinding binding;
    binding.aovName = aov;
    binding.renderBufferId = buffer_id;
    binding.aovSettings = aov_desc.aovSettings;
    task_params_.aovBindings.push_back(binding);

    GetRenderIndex().GetChangeTracker().MarkTaskDirty(get_task_id(),
                                                      pxr::HdChangeTracker::DirtyParams);
  }
  else if (buffer_descriptors_[buffer_id] != desc) {
    buffer_descriptors_[buffer_id] = desc;
    GetRenderIndex().GetChangeTracker().MarkBprimDirty(buffer_id,
                                                       pxr::HdRenderBuffer::DirtyDescription);
  }
}

pxr::HdRenderBuffer *RenderTaskDelegate::get_renderer_aov(pxr::TfToken const &aov)
{
  return (pxr::HdRenderBuffer *)(GetRenderIndex().GetBprim(pxr::HdPrimTypeTokens->renderBuffer,
                                                           get_aov_id(aov)));
}

void RenderTaskDelegate::get_renderer_aov_data(pxr::TfToken const &aov, void *data)
{
  pxr::HdRenderBuffer *buffer = get_renderer_aov(aov);
  void *buf_data = buffer->Map();
  memcpy(data,
         buf_data,
         buffer->GetWidth() * buffer->GetHeight() * pxr::HdDataSizeOfFormat(buffer->GetFormat()));
  buffer->Unmap();
}

pxr::HdTaskSharedPtr RenderTaskDelegate::get_task()
{
  return GetRenderIndex().GetTask(get_task_id());
}

void RenderTaskDelegate::set_camera_and_viewport(pxr::SdfPath const &camera_id,
                                                 pxr::GfVec4d const &viewport)
{
  if (task_params_.viewport != viewport || task_params_.camera != camera_id) {
    task_params_.viewport = viewport;
    task_params_.camera = camera_id;
    GetRenderIndex().GetChangeTracker().MarkTaskDirty(get_task_id(),
                                                      pxr::HdChangeTracker::DirtyParams);
  }
}

pxr::VtValue RenderTaskDelegate::Get(pxr::SdfPath const & /*id*/, pxr::TfToken const &key)
{
  if (key == pxr::HdTokens->params) {
    return pxr::VtValue(task_params_);
  }
  if (key == pxr::HdTokens->collection) {
    pxr::HdRprimCollection rprim_collection(pxr::HdTokens->geometry,
                                            pxr::HdReprSelector(pxr::HdReprTokens->smoothHull),
                                            false,
                                            pxr::TfToken());
    rprim_collection.SetRootPath(pxr::SdfPath::AbsoluteRootPath());
    return pxr::VtValue(rprim_collection);
  }
  return pxr::VtValue();
}

pxr::HdRenderBufferDescriptor RenderTaskDelegate::GetRenderBufferDescriptor(pxr::SdfPath const &id)
{
  return buffer_descriptors_[id];
}

pxr::TfTokenVector RenderTaskDelegate::GetTaskRenderTags(pxr::SdfPath const & /*task_id*/)
{
  return {pxr::HdRenderTagTokens->geometry};
}

}  // namespace blender::render::hydra
