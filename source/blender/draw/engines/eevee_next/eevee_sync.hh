/* SPDX-FileCopyrightText: 2021 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup eevee
 *
 * Structures to identify unique data blocks. The keys are unique so we are able to
 * match ids across frame updates.
 */

#pragma once

#include "BKE_duplilist.h"
#include "BLI_ghash.h"
#include "BLI_map.hh"
#include "DEG_depsgraph_query.h"
#include "DNA_object_types.h"
#include "DRW_render.h"
#include "GPU_material.h"

#include "eevee_shader_shared.hh"

namespace blender::eevee {

class Instance;

/* -------------------------------------------------------------------- */
/** \name ObjectKey
 *
 * Unique key to identify each object in the hash-map.
 * Note that we get a unique key for each object component.
 * \{ */

struct ObjectKey {
  /** Hash value of the key. */
  uint64_t hash_value = 0;

  ObjectKey() = default;

  ObjectKey(Object *ob, int sub_key_ = 0)
  {
    hash_value = BLI_ghashutil_ptrhash(DEG_get_original_object(ob));

    if (DupliObject *dupli = DRW_object_get_dupli(ob)) {
      hash_value = BLI_ghashutil_combine_hash(hash_value, dupli->random_id);
    }
    if (sub_key_ != 0) {
      hash_value = BLI_ghashutil_combine_hash(hash_value, sub_key_);
    }
  };

  uint64_t hash() const
  {
    return hash_value;
  }

  bool operator<(const ObjectKey &k) const
  {
    return hash_value < k.hash_value;
  }

  bool operator==(const ObjectKey &k) const
  {
    return hash_value == k.hash_value;
  }
};

/** \} */

/* -------------------------------------------------------------------- */
/** \name Sync Module
 *
 * \{ */

struct BaseHandle {
  /* Accumulated recalc flags, which corresponds to ID->recalc flags. */
  unsigned int recalc;
  void reset_recalc_flag()
  {
    if (recalc != 0) {
      recalc = 0;
    }
  }
};

struct ObjectHandle : public BaseHandle {
  ObjectKey object_key;
};

struct WorldHandle : public DrawData {
  void reset_recalc_flag()
  {
    if (recalc != 0) {
      recalc = 0;
    }
  }
};

struct SceneHandle : public DrawData {
  void reset_recalc_flag()
  {
    if (recalc != 0) {
      recalc = 0;
    }
  }
};

class SyncModule {
 private:
  Instance &inst_;

  Map<uint64_t, ObjectHandle> ob_handles = {};

 public:
  SyncModule(Instance &inst) : inst_(inst){};
  ~SyncModule(){};

  ObjectHandle &sync_object(Object *ob);
  WorldHandle &sync_world(::World *world);
  SceneHandle &sync_scene(::Scene *scene);

  void sync_mesh(Object *ob,
                 ObjectHandle &ob_handle,
                 ResourceHandle res_handle,
                 const ObjectRef &ob_ref);
  bool sync_sculpt(Object *ob,
                   ObjectHandle &ob_handle,
                   ResourceHandle res_handle,
                   const ObjectRef &ob_ref);
  void sync_point_cloud(Object *ob,
                        ObjectHandle &ob_handle,
                        ResourceHandle res_handle,
                        const ObjectRef &ob_ref);
  void sync_gpencil(Object *ob, ObjectHandle &ob_handle, ResourceHandle res_handle);
  void sync_curves(Object *ob,
                   ObjectHandle &ob_handle,
                   ResourceHandle res_handle,
                   ModifierData *modifier_data = nullptr,
                   ParticleSystem *particle_sys = nullptr);
  void sync_light_probe(Object *ob, ObjectHandle &ob_handle);
};

using HairHandleCallback = FunctionRef<void(ObjectHandle, ModifierData &, ParticleSystem &)>;
void foreach_hair_particle_handle(Object *ob, ObjectHandle ob_handle, HairHandleCallback callback);

/** \} */

}  // namespace blender::eevee
