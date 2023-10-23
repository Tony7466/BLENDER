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
 public:
  ParticleSystemData(HydraSceneDelegate *scene_delegate,
                     const Object *object,
                     pxr::SdfPath const &prim_id);

  static std::unique_ptr<ParticleSystemData> create(HydraSceneDelegate *scene_delegate,
                                                    const Object *object,
                                                    pxr::SdfPath const &prim_id,
                                                    ParticleSystem *particle_system);

  static bool is_supported(const ParticleSystem *particle_system);
  static bool is_visible(HydraSceneDelegate *scene_delegate,
                         Object *object,
                         ParticleSystem *particle_system,
                         int object_mode = OB_VISIBLE_SELF);

  virtual pxr::VtValue get_data(pxr::TfToken const &key) const override;
  virtual void init() override;
  virtual void insert() override;
  virtual void remove() override;
  virtual void update() override;
};

using ParticleSystemMap = Map<pxr::SdfPath, std::unique_ptr<ParticleSystemData>>;

}  // namespace blender::io::hydra
