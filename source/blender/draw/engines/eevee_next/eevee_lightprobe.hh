/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup eevee
 *
 * Module that handles light probe update tagging.
 * Lighting data is contained in their respective module `IrradianceCache` and `ReflectionProbes`.
 */

#pragma once

#include "BLI_map.hh"

#include "eevee_sync.hh"

namespace blender::eevee {

class Instance;

struct IrradianceGrid {
  float4x4 transform;
  int3 resolution;
};

struct ReflectionCube {};

struct LightProbe {
  bool used = false;
  bool initialized = false;
};

class LightProbeModule {
 public:
  /** Synced probe data. Only valid if the `eevee::Instance` is a baking instance. */
  Vector<IrradianceGrid> grids;
  Vector<ReflectionCube> cubes;

 private:
  Instance &inst_;

  /** Light Probe map to detect deletion. */
  Map<ObjectKey, LightProbe> grid_map_, cube_map_;
  /** True if a grid update was detected. It will trigger a bake if auto bake is enabled. */
  bool grid_update_;
  /** True if a grid update was detected. It will trigger a bake if auto bake is enabled. */
  bool cube_update_;
  /** True if the auto bake feature is enabled & available in this context. */
  bool auto_bake_enabled_;

 public:
  LightProbeModule(Instance &inst) : inst_(inst){};
  ~LightProbeModule(){};

  void begin_sync();

  void sync_cube(ObjectHandle &handle);
  void sync_grid(const Object *ob, ObjectHandle &handle);

  void sync_probe(const Object *ob, ObjectHandle &handle);

  void end_sync();
};

}  // namespace blender::eevee
