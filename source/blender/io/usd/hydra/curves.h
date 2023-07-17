/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include <pxr/base/vt/array.h>
#include <pxr/imaging/hd/sceneDelegate.h>

#include "BKE_duplilist.h"
#include "DNA_curves_types.h"

#include "BLI_set.hh"
#include "material.h"
#include "object.h"

namespace blender::io::hydra {

class CurvesData : public ObjectData {
 public:
  CurvesData(HydraSceneDelegate *scene_delegate, Object *object, pxr::SdfPath const &prim_id);

  void init() override;
  void insert() override;
  void remove() override;
  void update() override;

  pxr::VtValue get_data(pxr::TfToken const &key) const override;
  pxr::SdfPath material_id() const override;
  void available_materials(Set<pxr::SdfPath> &paths) const override;

  pxr::HdBasisCurvesTopology topology() const;
  pxr::HdPrimvarDescriptorVector primvar_descriptors(pxr::HdInterpolation interpolation) const;

 protected:
  void write_materials() override;

 private:
  void write_curves(Curves *curves);
  void write_uv_maps(Curves *curves);

  pxr::VtIntArray curve_vertex_counts_;
  pxr::VtVec3fArray vertices_;
  pxr::VtVec2fArray uvs_;
  pxr::VtFloatArray widths_;

  MaterialData *mat_data_ = nullptr;
};

}  // namespace blender::io::hydra
