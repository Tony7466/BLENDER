/* SPDX-FileCopyrightText: Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sim
 */

#pragma once

#include "BLI_implicit_sharing.hh"
#include "BLI_index_mask_fwd.hh"
#include "BLI_index_range.hh"
#include "BLI_math_quaternion_types.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_virtual_array_fwd.hh"

#include "BKE_attribute.hh"

#include <functional>
#include <mutex>

namespace blender::bke {
class AttributeAccessor;
class MutableAttributeAccessor;
}  // namespace blender::bke

namespace blender::bke {

class CollisionShape;
struct PhysicsGeometryImpl;

class PhysicsGeometry {
 public:
  using OverlapFilterFn = std::function<bool(const int a, const int b)>;

 private:
  /* Implementation of the physics world and rigid bodies.
   * This is an implicit shared pointer, multiple users can read the physics data. Requesting write
   * access will move the data to a new component. This is somewhat different from other components
   * which do a full copy of the data, but necessary for efficiently handling physics state. Moving
   * the physics state will create a read-only cache, so older state still be accessed. */
  const PhysicsGeometryImpl *impl_ = nullptr;

 public:
  static const struct BuiltinAttributes {
    std::string id;
    std::string is_simulated;
    std::string is_static;
    std::string is_kinematic;
    std::string mass;
    std::string inertia;
    std::string position;
    std::string rotation;
    std::string velocity;
    std::string angular_velocity;
  } builtin_attributes;

  PhysicsGeometry();
  explicit PhysicsGeometry(int rigid_bodies_num, int constraints_num, int shapes_num);
  PhysicsGeometry(const PhysicsGeometry &other);
  ~PhysicsGeometry();

  const PhysicsGeometryImpl &impl() const;
  PhysicsGeometryImpl &impl_for_write();

  bool has_world() const;
  void set_world_enabled(bool enable);

  int bodies_num() const;
  int constraints_num() const;
  int shapes_num() const;

  IndexRange bodies_range() const;
  IndexRange constraints_range() const;
  IndexRange shapes_range() const;

  void set_overlap_filter(OverlapFilterFn fn);
  void clear_overlap_filter();

  float3 gravity() const;
  void set_gravity(const float3 &gravity);
  void set_solver_iterations(int num_solver_iterations);
  void set_split_impulse(bool split_impulse);

  void step_simulation(float delta_time);

  VArray<const CollisionShape *> body_collision_shapes() const;
  VMutableArray<CollisionShape *> body_collision_shapes_for_write();

  VArray<int> body_ids() const;
  AttributeWriter<int> body_ids_for_write();

  VArray<bool> body_is_simulated() const;
  AttributeWriter<bool> body_is_simulated_for_write();

  VArray<bool> body_is_static() const;

  /* Set to zero to make static bodies. */
  VArray<float> body_masses() const;
  AttributeWriter<float> body_masses_for_write();

  VArray<float3> body_inertias() const;
  AttributeWriter<float3> body_inertias_for_write();

  VArray<float3> body_positions() const;
  AttributeWriter<float3> body_positions_for_write();

  VArray<math::Quaternion> body_rotations() const;
  AttributeWriter<math::Quaternion> body_rotations_for_write();

  VArray<float3> body_velocities() const;
  AttributeWriter<float3> body_velocities_for_write();

  VArray<float3> body_angular_velocities() const;
  AttributeWriter<float3> body_angular_velocities_for_write();

  void tag_collision_shapes_changed();
  void tag_body_transforms_changed();
  void tag_physics_changed();

  bke::AttributeAccessor attributes() const;
  bke::MutableAttributeAccessor attributes_for_write();
};

void move_physics_data(const PhysicsGeometry &from,
                       PhysicsGeometry &to,
                       bool use_world,
                       int rigid_bodies_offset,
                       int constraints_offset,
                       int shapes_offset);

}  // namespace blender::bke
