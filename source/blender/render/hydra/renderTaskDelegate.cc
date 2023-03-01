/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include <iostream>

#include <pxr/imaging/hd/renderBuffer.h>
#include <pxr/imaging/hd/renderDelegate.h>
#include <pxr/imaging/hdx/renderTask.h>

#include "glog/logging.h"

#include "renderTaskDelegate.h"

namespace blender::render::hydra {

RenderTaskDelegate::RenderTaskDelegate(HdRenderIndex* parentIndex, SdfPath const& delegateID)
    : HdSceneDelegate(parentIndex, delegateID)
{
  SdfPath renderTaskId = GetTaskID();
  GetRenderIndex().InsertTask<HdxRenderTask>(this, renderTaskId);
  GetRenderIndex().GetChangeTracker().MarkTaskDirty(renderTaskId, HdChangeTracker::DirtyCollection);
  GetRenderIndex().GetChangeTracker().MarkTaskDirty(renderTaskId, HdChangeTracker::DirtyRenderTags);

  taskParams.enableLighting = true;
  taskParams.alphaThreshold = 0.1f;
}

SdfPath RenderTaskDelegate::GetTaskID() const
{
  return GetDelegateID().AppendElementString("task");
}

SdfPath RenderTaskDelegate::GetAovID(TfToken const &aov) const
{
  return GetDelegateID().AppendElementString("aov_" + aov.GetString());
}

VtValue RenderTaskDelegate::Get(SdfPath const& id, TfToken const& key)
{
  LOG(INFO) << "RenderTaskDelegate::Get - " << id.GetAsString() << " " << key.GetString() << "\n";
  if (key == HdTokens->params) {
    return VtValue(taskParams);
  }
  if (key == HdTokens->collection) {
    HdRprimCollection rprimCollection(HdTokens->geometry, HdReprSelector(HdReprTokens->smoothHull), false, TfToken());
    rprimCollection.SetRootPath(SdfPath::AbsoluteRootPath());
    return VtValue(rprimCollection);
  }
  return VtValue();
}

HdRenderBufferDescriptor RenderTaskDelegate::GetRenderBufferDescriptor(SdfPath const &id)
{
  LOG(INFO) << "RenderTaskDelegate::GetRenderBufferDescriptor - " << id.GetAsString() << "\n";
  return bufferDescriptors[id];
}

TfTokenVector RenderTaskDelegate::GetTaskRenderTags(SdfPath const &taskId)
{
  LOG(INFO) << "RenderTaskDelegate::GetTaskRenderTags - " << taskId.GetAsString() << "\n";
  return { HdRenderTagTokens->geometry };
}

bool RenderTaskDelegate::IsConverged()
{
  HdTaskSharedPtr renderTask = GetRenderIndex().GetTask(GetTaskID());
  return ((HdxRenderTask &)*renderTask).IsConverged();
}

void RenderTaskDelegate::SetRendererAov(TfToken const &aov)
{
  HdAovDescriptor aovDesc = GetRenderIndex().GetRenderDelegate()->GetDefaultAovDescriptor(aov);
  HdRenderBufferDescriptor desc(GfVec3i(taskParams.viewport[2], taskParams.viewport[3], 1),
    aovDesc.format, aovDesc.multiSampled);
  SdfPath bufferId = GetAovID(aov);

  if (bufferDescriptors.find(bufferId) == bufferDescriptors.end()) {
    GetRenderIndex().InsertBprim(HdPrimTypeTokens->renderBuffer, this, bufferId);
    bufferDescriptors[bufferId] = desc;
    GetRenderIndex().GetChangeTracker().MarkBprimDirty(bufferId, HdRenderBuffer::DirtyDescription);

    HdRenderPassAovBinding binding;
    binding.aovName = aov;
    binding.renderBufferId = bufferId;
    binding.aovSettings = aovDesc.aovSettings;
    taskParams.aovBindings.push_back(binding);

    GetRenderIndex().GetChangeTracker().MarkTaskDirty(GetTaskID(), HdChangeTracker::DirtyParams);
  }
  else if (bufferDescriptors[bufferId] != desc) {
    bufferDescriptors[bufferId] = desc;
    GetRenderIndex().GetChangeTracker().MarkBprimDirty(bufferId, HdRenderBuffer::DirtyDescription);
  }
}

HdRenderBuffer *RenderTaskDelegate::GetRendererAov(TfToken const &aov)
{
  return (HdRenderBuffer *)(GetRenderIndex().GetBprim(HdPrimTypeTokens->renderBuffer, GetAovID(aov)));
}

void RenderTaskDelegate::GetRendererAovData(TfToken const &aov, void *data)
{
  HdRenderBuffer *buffer = GetRendererAov(aov);
  void *bufData = buffer->Map();
  memcpy(data, bufData, buffer->GetWidth() * buffer->GetHeight() * HdDataSizeOfFormat(buffer->GetFormat()));
  buffer->Unmap();
}

HdTaskSharedPtr RenderTaskDelegate::GetTask()
{
  return GetRenderIndex().GetTask(GetTaskID());
}

void RenderTaskDelegate::SetCameraAndViewport(SdfPath const &cameraId, GfVec4d const &viewport)
{
  if (taskParams.viewport != viewport || taskParams.camera != cameraId) {
    taskParams.viewport = viewport;
    taskParams.camera = cameraId;
    GetRenderIndex().GetChangeTracker().MarkTaskDirty(GetTaskID(), HdChangeTracker::DirtyParams);
  }
}


} // namespace blender::render::hydra
