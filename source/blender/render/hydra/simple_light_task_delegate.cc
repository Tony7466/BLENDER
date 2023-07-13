/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include <pxr/imaging/hdx/simpleLightTask.h>

#include "simple_light_task_delegate.h"

namespace blender::render::hydra {

SimpleLightTaskDelegate::SimpleLightTaskDelegate(pxr::HdRenderIndex *parent_index,
                                                 pxr::SdfPath const &delegate_id)
    : pxr::HdSceneDelegate(parent_index, delegate_id)
{
  pxr::SdfPath task_id = get_task_id();
  GetRenderIndex().InsertTask<pxr::HdxSimpleLightTask>(this, task_id);
}

pxr::SdfPath SimpleLightTaskDelegate::get_task_id() const
{
  return GetDelegateID().AppendElementString("task");
}

pxr::HdTaskSharedPtr SimpleLightTaskDelegate::get_task()
{
  return GetRenderIndex().GetTask(get_task_id());
}

void SimpleLightTaskDelegate::set_camera_path(pxr::SdfPath const &camera_path)
{
  task_params_.cameraPath = camera_path;
}

pxr::VtValue SimpleLightTaskDelegate::Get(pxr::SdfPath const & /*id*/, pxr::TfToken const &key)
{
  if (key == pxr::HdTokens->params) {
    return pxr::VtValue(task_params_);
  }
  return pxr::VtValue();
}

}  // namespace blender::render::hydra
