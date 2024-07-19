/* SPDX-FileCopyrightText: Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sim
 */

#pragma once

#include "BLI_index_mask_fwd.hh"
#include "BLI_index_range.hh"
#include "BLI_math_quaternion_types.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_utility_mixins.hh"
#include "BLI_virtual_array_fwd.hh"

#include "BKE_attribute.hh"
#include "BKE_geometry_fields.hh"

#include "DNA_customdata_types.h"

#include <functional>

namespace blender::bke {
class AttributeAccessor;
class MutableAttributeAccessor;
}  // namespace blender::bke

namespace blender::bke {

class CollisionShape;
struct PhysicsGeometryImpl;

using CollisionShapePtr = ImplicitSharingPtr<CollisionShape>;

class PhysicsGeometry {
 public:
  using OverlapFilterFn = std::function<bool(const int a, const int b)>;

  enum class BodyActivationState {
    AlwaysActive = 0,
    Active,
    WantsSleep,
    Sleeping,
    AlwaysSleeping,
  };

  enum class ConstraintType {
    None = -1,
    Fixed = 0,
    Point,
    Hinge,
    Slider,
    ConeTwist,
    SixDoF,
    SixDoFSpring,
    SixDoFSpring2,
    Contact,
    Gear,
  };

 private:
  /* Implementation of the physics world and rigid bodies.
   * This is an implicit shared pointer, multiple users can read the physics data. Requesting write
   * access will move the data to a new component. This is somewhat different from other components
   * which do a full copy of the data, but necessary for efficiently handling physics state. Moving
   * the physics state will create a read-only cache, so older state still be accessed. */
  const PhysicsGeometryImpl *impl_ = nullptr;

 public:
  static const struct BuiltinAttributes {
    /* Increment this when adding attributes! */
    const int num_builtin_attributes = 36;
    std::array<std::string, 36> all;
    /* Attributes that should not be copied when there is a physics world. */
    std::array<std::string, 36> skip_copy;

    /* Body attributes. */
    std::string id;
    std::string collision_shape;
    std::string is_static;
    std::string is_kinematic;
    std::string mass;
    std::string inertia;
    std::string center_of_mass;
    std::string position;
    std::string rotation;
    std::string velocity;
    std::string angular_velocity;
    std::string activation_state;
    std::string friction;
    std::string rolling_friction;
    std::string spinning_friction;
    std::string restitution;
    std::string linear_damping;
    std::string angular_damping;
    std::string linear_sleeping_threshold;
    std::string angular_sleeping_threshold;
    std::string total_force;
    std::string total_torque;

    /* Constraint attributes. */
    std::string constraint_type;
    std::string constraint_body1;
    std::string constraint_body2;
    std::string constraint_enabled;
    std::string constraint_frame1;
    std::string constraint_frame2;
    std::string applied_impulse;
    std::string applied_force1;
    std::string applied_force2;
    std::string applied_torque1;
    std::string applied_torque2;
    std::string breaking_impulse_threshold;
    std::string disable_collision;
  } builtin_attributes;

  PhysicsGeometry();
  explicit PhysicsGeometry(int bodies_num, int constraints_num, int shapes_num);
  PhysicsGeometry(const PhysicsGeometry &other);
  ~PhysicsGeometry();

  const PhysicsGeometryImpl &impl() const;
  PhysicsGeometryImpl &impl_for_write();

  bool has_world() const;
  void create_world(bool copy_attributes = true);
  void destroy_world(bool copy_attributes = true);

  int bodies_num() const;
  int constraints_num() const;
  int shapes_num() const;

  IndexRange bodies_range() const;
  IndexRange constraints_range() const;
  IndexRange shapes_range() const;

  void resize(int bodies_num, int constraints_num);

  void move_world_data(const PhysicsGeometry &from,
                       bool move_world,
                       const IndexMask &body_mask,
                       const IndexMask &constraint_mask,
                       const IndexMask &shape_mask,
                       int body_offset,
                       int constraint_offset,
                       int shape_offset);

  void move_or_copy_selection(const PhysicsGeometry &from,
                              const IndexMask &body_mask,
                              const IndexMask &constraint_mask,
                              const IndexMask &shape_mask,
                              const bke::AnonymousAttributePropagationInfo &propagation_info);

  void set_overlap_filter(OverlapFilterFn fn);
  void clear_overlap_filter();

  float3 gravity() const;
  void set_gravity(const float3 &gravity);
  void set_solver_iterations(int num_solver_iterations);
  void set_split_impulse(bool split_impulse);

  void step_simulation(float delta_time);

  Span<CollisionShapePtr> shapes() const;
  MutableSpan<CollisionShapePtr> shapes_for_write();
  void set_body_shapes(const IndexMask &selection,
                       Span<int> shape_handles,
                       bool update_local_inertia);

  VArray<int> body_ids() const;
  AttributeWriter<int> body_ids_for_write();

  VArray<int> body_shapes() const;

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

  /* Type is #BodyActivationState, can't return enum as VArray (no CPPType). */
  VArray<int> body_activation_states() const;
  AttributeWriter<int> body_activation_states_for_write();

  VArray<float3> body_total_force() const;
  VArray<float3> body_total_torque() const;
  void apply_force(const IndexMask &selection,
                   const VArray<float3> &forces,
                   const VArray<float3> &relative_positions = {});
  void apply_torque(const IndexMask &selection, const VArray<float3> &torques);

  void apply_impulse(const IndexMask &selection,
                     const VArray<float3> &impulses,
                     const VArray<float3> &relative_positions = {});
  void apply_angular_impulse(const IndexMask &selection, const VArray<float3> &angular_impulses);
  void clear_forces(const IndexMask &selection);

  void create_constraints(const IndexMask &selection,
                          const VArray<int> &types,
                          const VArray<int> &bodies1,
                          const VArray<int> &bodies2);

  VArray<int> constraint_types() const;
  VArray<int> constraint_body1() const;
  VArray<int> constraint_body2() const;

  VArray<bool> constraint_enabled() const;
  AttributeWriter<bool> constraint_enabled_for_write();

  VArray<float4x4> constraint_frame1() const;
  AttributeWriter<float4x4> constraint_frame1_for_write();

  VArray<float4x4> constraint_frame2() const;
  AttributeWriter<float4x4> constraint_frame2_for_write();

  VArray<float> constraint_applied_impulse() const;

  VArray<float> constraint_breaking_impulse_threshold_impulse() const;
  AttributeWriter<float> constraint_breaking_impulse_threshold_for_write();

  VArray<bool> constraint_disable_collision() const;
  AttributeWriter<bool> constraint_disable_collision_for_write();

  void tag_collision_shapes_changed();
  void tag_body_transforms_changed();
  void tag_topology_changed();
  void tag_physics_changed();

  bke::AttributeAccessor attributes(bool force_cache = false) const;
  bke::MutableAttributeAccessor attributes_for_write(bool force_cache = false);

  friend class BuiltinPhysicsAttributeBase;
};

// XXX Below: possible separation of world data and cache.

// struct PhysicsWorldData;

// class PhysicsWorldState {
//  private:
//   CustomData body_data_;
//   CustomData constraint_data_;

//  public:
//   bke::AttributeAccessor attributes() const;
//   bke::MutableAttributeAccessor attributes_for_write();
// };

// class PhysicsWorld : NonCopyable, NonMovable {
//  private:
//   std::unique_ptr<PhysicsWorldData> data_;

//  public:
//   PhysicsWorldState copy_state() const;
//   PhysicsWorldState copy_state(const IndexMask &body_mask, const IndexMask &constraint_mask)
//   const;

//   void apply_state(const PhysicsWorldState &state);
//   void apply_state(const PhysicsWorldState &state, const IndexMask &body_mask, const IndexMask
//   &constraint_mask);

//   bke::AttributeAccessor attributes() const;
//   bke::MutableAttributeAccessor attributes_for_write();
// };

}  // namespace blender::bke
