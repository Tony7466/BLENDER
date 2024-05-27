/* SPDX-FileCopyrightText: Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sim
 */

#pragma once

#include "BLI_index_mask_fwd.hh"
#include "BLI_map.hh"
#include "BLI_math_quaternion_types.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_set.hh"
#include "BLI_virtual_array_fwd.hh"

namespace blender::simulation {

using RigidBodyID = int;

class CollisionShape;
class RigidBody;
class RigidBodyWorld;

class PhysicsGeometry {
 private:
  Array<RigidBody *> rigid_bodies_;

  RigidBodyWorld *world_ = nullptr;

 public:
  bool has_world() const;
  RigidBodyWorld *world() const;

  RigidBodyWorld *ensure_world();

  Span<const RigidBody *> rigid_bodies() const;
  Span<RigidBody *> rigid_bodies_for_write();

  Span<RigidBody *> add_rigid_bodies(const Span<const CollisionShape *> shapes,
                                     const VArray<int> &shape_indices,
                                     const VArray<float> &masses,
                                     const VArray<float3> &inertiae,
                                     const VArray<bool> &simulated);
  void remove_rigid_bodies(const IndexMask &mask);
  void clear_rigid_bodies();

  void set_bodies_simulated(const IndexMask &selection, bool enable);
  void set_all_bodies_simulated(bool enable);

  VArray<RigidBodyID> body_ids() const;
};

}  // namespace blender::simulation
