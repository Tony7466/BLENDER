/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include <pxr/imaging/hdx/simpleLightTask.h>

#include "simpleLightTaskDelegate.h"

namespace blender::render::hydra {

SimpleLightTaskDelegate::SimpleLightTaskDelegate(HdRenderIndex *parentIndex,
                                                 SdfPath const &delegateID)
    : HdSceneDelegate(parentIndex, delegateID)
{
  SdfPath taskId = GetTaskID();
  GetRenderIndex().InsertTask<HdxSimpleLightTask>(this, taskId);
}

SdfPath SimpleLightTaskDelegate::GetTaskID() const
{
  return GetDelegateID().AppendElementString("task");
}

HdTaskSharedPtr SimpleLightTaskDelegate::GetTask()
{
  return GetRenderIndex().GetTask(GetTaskID());
}

void SimpleLightTaskDelegate::SetCameraPath(SdfPath const &cameraPath)
{
  taskParams.cameraPath = cameraPath;
}

VtValue SimpleLightTaskDelegate::Get(SdfPath const &id, TfToken const &key)
{
  if (key == HdTokens->params) {
    return VtValue(taskParams);
  }
  return VtValue();
}

}  // namespace blender::render::hydra
