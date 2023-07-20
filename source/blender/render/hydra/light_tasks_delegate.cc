/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include <pxr/imaging/hdx/simpleLightTask.h>
#include <pxr/imaging/hdx/skydomeTask.h>

#include "light_tasks_delegate.h"

namespace blender::render::hydra {

LightTasksDelegate::LightTasksDelegate(pxr::HdRenderIndex *parent_index,
                                       pxr::SdfPath const &delegate_id)
    : pxr::HdSceneDelegate(parent_index, delegate_id)
{
  skydome_task_id_ = GetDelegateID().AppendElementString("simpleLightTask");
  simple_task_id_ = GetDelegateID().AppendElementString("skydomeTask");
  GetRenderIndex().InsertTask<pxr::HdxSkydomeTask>(this, skydome_task_id_);
  GetRenderIndex().InsertTask<pxr::HdxSimpleLightTask>(this, simple_task_id_);
}

pxr::HdTaskSharedPtrVector LightTasksDelegate::get_tasks()
{
  /*Note that this task is intended to be the first "Render Task",
  so that the AOV's are properly cleared, however it
  does not spawn a HdRenderPass.*/
  return {GetRenderIndex().GetTask(skydome_task_id_), GetRenderIndex().GetTask(simple_task_id_)};
}

void LightTasksDelegate::set_camera_and_viewport(pxr::SdfPath const &camera_id,
                                                 pxr::GfVec4d const &viewport)
{
  if (simple_task_params_.cameraPath != camera_id) {
    simple_task_params_.cameraPath = camera_id;
    GetRenderIndex().GetChangeTracker().MarkTaskDirty(simple_task_id_,
                                                      pxr::HdChangeTracker::DirtyParams);
  }
  if (skydome_task_params_.viewport != viewport || skydome_task_params_.camera != camera_id) {
    skydome_task_params_.viewport = viewport;
    skydome_task_params_.camera = camera_id;
    GetRenderIndex().GetChangeTracker().MarkTaskDirty(skydome_task_id_,
                                                      pxr::HdChangeTracker::DirtyParams);
  }
}

pxr::VtValue LightTasksDelegate::Get(pxr::SdfPath const &id, pxr::TfToken const &key)
{
  if (key == pxr::HdTokens->params) {
    if (id == simple_task_id_) {
      return pxr::VtValue(simple_task_params_);
    }
    else if (id == skydome_task_id_) {
      return pxr::VtValue(skydome_task_params_);
    }
  }
  return pxr::VtValue();
}

}  // namespace blender::render::hydra
