/* SPDX-FileCopyrightText: 2011-2022 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <pxr/base/vt/array.h>
#include <pxr/imaging/hd/sceneDelegate.h>

#include "material.h"
#include "particle_system.h"

class ParticleSystem;

namespace blender::io::hydra {

class HairData : public ParticleSystemData {

 private:
  pxr::VtIntArray curve_vertex_counts_;
  pxr::VtVec3fArray vertices_;

  MaterialData *mat_data_ = nullptr;

 public:
  HairData(HydraSceneDelegate *scene_delegate,
           const Object *object,
           pxr::SdfPath const &prim_id,
           ParticleSystem *particle_system);

  void init() override;
  void insert() override;
  void remove() override;
  void update() override;

  pxr::VtValue get_data(pxr::TfToken const &key) const override;
  pxr::SdfPath material_id(pxr::SdfPath const &id) const override;

  pxr::HdBasisCurvesTopology topology() const;
  pxr::HdPrimvarDescriptorVector primvar_descriptors(pxr::HdInterpolation interpolation) const;

  ParticleSystem *particle_system;

 protected:
  void write_materials() override;

 private:
  void write_hair();
};

}  // namespace blender::io::hydra
