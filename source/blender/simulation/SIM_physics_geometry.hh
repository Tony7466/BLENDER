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
struct PhysicsImpl;

class PhysicsGeometry {
 private:
  PhysicsImpl *impl_;

 public:
  using OverlapFilterFn = std::function<bool(const RigidBodyID a, const RigidBodyID b)>;

  PhysicsGeometry();
  PhysicsGeometry(const PhysicsGeometry &other);
  ~PhysicsGeometry();

  PhysicsGeometry copy() const;

  bool has_world() const;
  void set_world(bool enable);

  void set_overlap_filter(OverlapFilterFn fn);
  void clear_overlap_filter();
  
  float3 gravity() const;
  void set_gravity(const float3 &gravity);
  void set_solver_iterations(int num_solver_iterations);
  void set_split_impulse(bool split_impulse);
  
  int rigid_bodies_num() const;
  int constraints_num() const;
  int shapes_num() const;

  IndexRange add_rigid_bodies(const Span<const CollisionShape *> shapes,
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
