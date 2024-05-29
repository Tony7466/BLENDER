/* SPDX-FileCopyrightText: Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sim
 */

#pragma once

#include "BKE_attribute.hh"
#include "BLI_index_mask_fwd.hh"
#include "BLI_index_range.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_virtual_array_fwd.hh"

#include <functional>

namespace blender::bke {
class AttributeAccessor;
class MutableAttributeAccessor;
}  // namespace blender::bke

namespace blender::bke {

using RigidBodyID = int;
using CollisionShapeID = int;

class CollisionShape;
struct PhysicsImpl;

class PhysicsGeometry {
 private:
  PhysicsImpl *impl_;

 public:
  using OverlapFilterFn = std::function<bool(const RigidBodyID a, const RigidBodyID b)>;

  static const struct BuiltinAttributes {
    std::string id;
    std::string mass;
    std::string inertia;
    std::string position;
    std::string rotation;
  } builtin_attributes;

  PhysicsGeometry();
  explicit PhysicsGeometry(int rigid_bodies_num);
  PhysicsGeometry(const PhysicsGeometry &other);
  ~PhysicsGeometry();

  PhysicsImpl &impl();
  const PhysicsImpl &impl() const;

  //PhysicsGeometry copy() const;

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

  IndexRange rigid_bodies_range() const;
  IndexRange constraints_range() const;
  IndexRange shapes_range() const;

  //void set_bodies_simulated(const IndexMask &selection, bool enable);
  //void set_all_bodies_simulated(bool enable);

  VArray<const CollisionShape *> body_collision_shapes() const;
  VMutableArray<CollisionShape *> body_collision_shapes_for_write();

  VArray<RigidBodyID> body_ids() const;

  VArray<float> body_masses() const;
  VMutableArray<float> body_masses_for_write();

  VArray<float3> body_inertias() const;
  VMutableArray<float3> body_inertias_for_write();

  VArray<float3> body_positions() const;
  VMutableArray<float3> body_positions_for_write();

  VArray<math::Quaternion> body_rotations() const;
  VMutableArray<math::Quaternion> body_rotations_for_write();

  void tag_collision_shapes_changed();
  void tag_body_transforms_changed();

  bke::AttributeAccessor attributes() const;
  bke::MutableAttributeAccessor attributes_for_write();
};

}  // namespace blender::bke
