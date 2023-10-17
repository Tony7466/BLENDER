/* SPDX-FileCopyrightText: 2011-2022 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <pxr/base/vt/array.h>
#include <pxr/imaging/hd/sceneDelegate.h>

#include "BLI_set.hh"

#include "BKE_duplilist.h"
#include "BKE_particle.h"

#include "DNA_particle_types.h"

#include "material.h"
#include "object.h"

namespace blender::io::hydra {

class ParticleSystemData : public ObjectData {

 private:
  pxr::VtIntArray curve_vertex_counts_;
  pxr::VtVec3fArray vertices_;

  MaterialData *mat_data_ = nullptr;

 public:
  ParticleSystemData(HydraSceneDelegate *scene_delegate,
                   const Object *object,
                   pxr::SdfPath const &prim_id,
                   ParticleSystem *particle_system);

  static bool is_supported(const ParticleSystem *particle_system);
  static bool is_visible(HydraSceneDelegate *scene_delegate,
                         Object *object,
                         ParticleSystem *particle_system,
                         int object_mode = OB_VISIBLE_SELF);

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
  void write_curves();
  void write_hair();
};

using ParticleSystemMap = Map<pxr::SdfPath, std::unique_ptr<ParticleSystemData>>;

}  // namespace blender::io::hydra
