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
  enum class BodyAttribute {
    id,
    collision_shape,
    is_static,
    is_kinematic,
    mass,
    inertia,
    center_of_mass,
    position,
    rotation,
    velocity,
    angular_velocity,
    activation_state,
    friction,
    rolling_friction,
    spinning_friction,
    restitution,
    linear_damping,
    angular_damping,
    linear_sleeping_threshold,
    angular_sleeping_threshold,
    total_force,
    total_torque,
  };

  enum class ConstraintAttribute {
    constraint_type,
    constraint_body1,
    constraint_body2,
    constraint_enabled,
    constraint_frame1,
    constraint_frame2,
    applied_impulse,
    applied_force1,
    applied_force2,
    applied_torque1,
    applied_torque2,
    breaking_impulse_threshold,
    disable_collision,
  };

  PhysicsGeometry();
  explicit PhysicsGeometry(int bodies_num, int constraints_num, int shapes_num);
  PhysicsGeometry(const PhysicsGeometry &other);
  ~PhysicsGeometry();

  const PhysicsGeometryImpl &impl() const;
  PhysicsGeometryImpl &impl_for_write();

  bool has_world() const;
  void create_world();
  void destroy_world();

  int bodies_num() const;
  int constraints_num() const;
  int shapes_num() const;

  IndexRange bodies_range() const;
  IndexRange constraints_range() const;
  IndexRange shapes_range() const;

  void move_or_copy_selection(const PhysicsGeometry &from,
                              const IndexMask &body_mask,
                              const IndexMask &constraint_mask,
                              const bke::AnonymousAttributePropagationInfo &propagation_info);
  bool try_move_data(const PhysicsGeometry &src,
                     int body_num,
                     int constraint_num,
                     const IndexMask &src_body_mask,
                     const IndexMask &src_constraint_mask,
                     int dst_body_offset,
                     int dst_constraint_offset);

  void set_overlap_filter(OverlapFilterFn fn);
  void clear_overlap_filter();

  float3 gravity() const;
  void set_gravity(const float3 &gravity);
  void set_solver_iterations(int num_solver_iterations);
  void set_split_impulse(bool split_impulse);

  void step_simulation(float delta_time);

  void ensure_read_cache() const;
  void ensure_custom_data_attribute(BodyAttribute attribute) const;
  void ensure_custom_data_attribute(ConstraintAttribute attribute) const;

  Span<CollisionShapePtr> shapes() const;
  MutableSpan<CollisionShapePtr> shapes_for_write();

  VArray<int> body_ids() const;
  AttributeWriter<int> body_ids_for_write();

  VArray<int> body_shapes() const;
  AttributeWriter<int> body_shapes_for_write();

  VArray<bool> body_is_static() const;
  AttributeWriter<bool> body_is_static_for_write();

  VArray<bool> body_is_kinematic() const;
  AttributeWriter<bool> body_is_kinematic_for_write();

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

  void compute_local_inertia(const IndexMask &selection);

  // void create_constraints(const IndexMask &selection,
  //                         const VArray<int> &types,
  //                         const VArray<int> &bodies1,
  //                         const VArray<int> &bodies2);

  VArray<int> constraint_types() const;
  AttributeWriter<int> constraint_types_for_write();
  VArray<int> constraint_body1() const;
  AttributeWriter<int> constraint_body1_for_write();
  VArray<int> constraint_body2() const;
  AttributeWriter<int> constraint_body2_for_write();

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

  static StringRef body_attribute_name(BodyAttribute attribute);
  static StringRef constraint_attribute_name(ConstraintAttribute attribute);

  bke::AttributeAccessor attributes() const;
  bke::MutableAttributeAccessor attributes_for_write();

  /* XXX Attributes functions on components can nominally return a nullopt, but some parts of the
   * code don't check for that return value case
   * (realize_instances.cc::gather_attributes_for_propagation).
   * These dummy functions return a placeholder accessor that can be used as a fallback to avoid
   * this bug */
  static AttributeAccessor dummy_attributes();
  static MutableAttributeAccessor dummy_attributes_for_write();

  /* Validate internal world data.
   * Should only be used in tests. */
  bool validate_world_data();

  friend class BuiltinPhysicsAttributeBase;
};

}  // namespace blender::bke
