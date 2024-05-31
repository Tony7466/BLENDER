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

namespace blender::bke {
class AttributeAccessor;
class MutableAttributeAccessor;
}  // namespace blender::bke

namespace blender::bke {

class CollisionShape;
struct PhysicsGeometryImpl;
struct PhysicsWorldImpl;

class PhysicsWorld : NonCopyable, NonMovable {
 public:
  using OverlapFilterFn = std::function<bool(const int a, const int b)>;

 private:
  PhysicsWorldImpl *impl_ = nullptr;

 public:
  PhysicsWorldImpl &impl();
  const PhysicsWorldImpl &impl() const;

  void set_overlap_filter(OverlapFilterFn fn);
  void clear_overlap_filter();

  float3 gravity() const;
  void set_gravity(const float3 &gravity);
  void set_solver_iterations(int num_solver_iterations);
  void set_split_impulse(bool split_impulse);

  void step_simulation(float delta_time);

 private:
  PhysicsWorld(PhysicsWorldImpl *impl);
  ~PhysicsWorld();

  friend class PhysicsGeometry;
};

using PhysicsWorldPtr = ImplicitSharingPtr<PhysicsWorld>;

class PhysicsGeometry {
 private:
  const PhysicsGeometryImpl *impl_ = nullptr;
  PhysicsWorld *world_ = nullptr;

 public:
  static const struct BuiltinAttributes {
    std::string id;
    std::string simulated;
    std::string mass;
    std::string inertia;
    std::string position;
    std::string rotation;
    std::string velocity;
    std::string angular_velocity;
  } builtin_attributes;

  PhysicsGeometry();
  explicit PhysicsGeometry(int rigid_bodies_num);
  PhysicsGeometry(const PhysicsGeometry &other);
  ~PhysicsGeometry();

  PhysicsGeometryImpl &impl_for_write();
  const PhysicsGeometryImpl &impl() const;

  PhysicsWorld *world_for_write();
  const PhysicsWorld *world() const;
  void set_world_enabled(bool enable);

  int rigid_bodies_num() const;
  int constraints_num() const;
  int shapes_num() const;

  IndexRange rigid_bodies_range() const;
  IndexRange constraints_range() const;
  IndexRange shapes_range() const;

  VArray<const CollisionShape *> body_collision_shapes() const;
  VMutableArray<CollisionShape *> body_collision_shapes_for_write();

  VArray<int> body_ids() const;
  AttributeWriter<int> body_ids_for_write();

  VArray<bool> body_is_simulated() const;
  AttributeWriter<bool> body_is_simulated_for_write();

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

  bke::AttributeAccessor attributes() const;
  bke::MutableAttributeAccessor attributes_for_write();

  /* Special functions to efficiently transfer data to another geometry.
   * Physics worlds have state data that needs to be rebuilt when bodies are added and removed, so
   * keeping bodies registered with the world and transfer everything at once is more efficient
   * than copying attribute arrays. */
  void transfer_world_from(PhysicsGeometry &src_physics);
  void transfer_rigid_bodies_from(PhysicsGeometry &src_physics, IndexRange dst_bodies_range);
};

}  // namespace blender::bke
