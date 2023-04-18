/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include <map>

#include <pxr/base/gf/camera.h>
#include <pxr/base/gf/vec2f.h>

#include "BKE_context.h"

#include "DNA_object_types.h"

namespace blender::render::hydra {

class CameraData {
 public:
  CameraData(bContext *context);
  CameraData(Object *camera_obj, pxr::GfVec2i res, pxr::GfVec4f tile);

  pxr::GfCamera gf_camera();
  pxr::GfCamera gf_camera(pxr::GfVec4f tile);

 private:
  int mode_;
  pxr::GfRange1f clip_range_;
  float focal_length_;
  pxr::GfVec2f sensor_size_;
  pxr::GfMatrix4d transform_;
  pxr::GfVec2f lens_shift_;
  pxr::GfVec2f ortho_size_;
  std::tuple<float, float, int> dof_data_;
};

}  // namespace blender::render::hydra
