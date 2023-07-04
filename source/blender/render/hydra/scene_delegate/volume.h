/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include <pxr/imaging/hd/sceneDelegate.h>

#include "object.h"

namespace blender::render::hydra {

class VolumeData : public ObjectData {

 public:
  VolumeData(BlenderSceneDelegate *scene_delegate, Object *object, pxr::SdfPath const &prim_id);

  void init() override;
  void insert() override;
  void remove() override;
  void update() override;

  pxr::VtValue get_data(pxr::SdfPath const &id, pxr::TfToken const &key) const;
  bool update_visibility() override;

  pxr::HdVolumeFieldDescriptorVector field_descriptors() const;
  pxr::SdfPath material_id() const;
  void available_materials(Set<pxr::SdfPath> &paths) const;

 private:
  void write_materials();

  std::string filepath_;
  pxr::HdVolumeFieldDescriptorVector field_descriptors_;
  MaterialData *mat_data_ = nullptr;
};

}  // namespace blender::render::hydra
