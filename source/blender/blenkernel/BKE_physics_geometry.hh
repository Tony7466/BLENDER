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
class PhysicsWorldData;
struct CustomDataAccessInfo;
struct PhysicsWorldDataAccessInfo;

using CollisionShapePtr = ImplicitSharingPtr<CollisionShape>;

enum class PhysicsBodyAttribute {
  id,
  collision_shape,
  is_static,
  is_kinematic,
  mass,
  inertia,
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

enum class PhysicsConstraintAttribute {
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

enum class PhysicsConstraintType {
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

enum class PhysicsBodyActivationState {
  AlwaysActive = 0,
  Active,
  WantsSleep,
  Sleeping,
  AlwaysSleeping,
};

/**
 * A physics world state is a snapshot of the physics world.
 * The physics state acts as a bridge between geometry nodes and the physics engine. Physics
 * world data cannot be copied easily, there can only be one owner of the physics data. One
 * "active" physics state owns the world data.
 *
 * At the same time a physics state is also implicitly shared data. This way it represents physics
 * data in the geometry nodes system of components and attributes. In order to make a physics state
 * mutable it gets copied. Importantly the physics world data is moved to the new state, making the
 * new state the active one.
 *
 * In order to adhere to the rules of copy-on-write the physics state must be able to move the
 * world data without requiring a lock. To make this possible the state always stores a copy of the
 * physics state for use by attribute readers. This cache can safely be accessed while the world
 * data pointer ownership changes.
 */
class PhysicsWorldState : public ImplicitSharingMixin {
  using BodyAttribute = PhysicsBodyAttribute;
  using ConstraintAttribute = PhysicsConstraintAttribute;
  using ConstraintType = PhysicsConstraintType;
  using BodyActivationState = PhysicsBodyActivationState;

  using OverlapFilterFn = std::function<bool(const int a, const int b)>;

  using CacheFlag = std::atomic<bool>;

  /* Cache for readers storing copies of physics data in custom data. */
  mutable CacheFlag custom_data_read_cache_valid_ = false;
  /* Valid when body collision shape pointers match pointers from the
   * shapes list, as stored in the body shapes index attribute. */
  mutable CacheFlag body_collision_shapes_valid_ = false;
  /* Valid when is_static flags match the world data motion type for each body. */
  mutable CacheFlag body_is_static_valid_ = false;
  /* Valid when mass matches the world data motion type for each body. */
  mutable CacheFlag body_mass_valid_ = false;
  /* Valid when internal constraints have been updated to specified types and bodies. */
  mutable CacheFlag constraints_valid_ = false;
  /* Valid when constraint references to disable collisions have been updated. */
  mutable CacheFlag constraint_disable_collision_valid_ = false;

  int body_num_;
  int constraint_num_;
  CustomData body_data_;
  CustomData constraint_data_;

  Array<CollisionShapePtr> shapes_;

  mutable PhysicsWorldData *world_data_ = nullptr;
  /* Protects shared read/write access to world data. */
  mutable std::mutex world_data_mutex_;

 public:
  PhysicsWorldState();
  PhysicsWorldState(int body_num, int constraint_num, int shape_num);
  PhysicsWorldState(const PhysicsWorldState &other);
  ~PhysicsWorldState();

  PhysicsWorldState &operator=(const PhysicsWorldState &other);

  void delete_self() override;

  int bodies_num() const;
  int constraints_num() const;
  int shapes_num() const;

  IndexRange bodies_range() const;
  IndexRange constraints_range() const;
  IndexRange shapes_range() const;

  bool has_world_data() const;

  void tag_read_cache_changed();
  void tag_body_topology_changed();
  void tag_body_collision_shape_changed();
  void tag_body_is_static_changed();
  void tag_body_mass_changed();
  void tag_constraints_changed();
  void tag_constraint_disable_collision_changed();
  void tag_shapes_changed();

  bool has_builtin_attribute_custom_data_layer(BodyAttribute attribute) const;
  bool has_builtin_attribute_custom_data_layer(ConstraintAttribute attribute) const;

  /* Make sure all world data has been copied to the custom data read cache. */
  void ensure_read_cache() const;
  /* Make sure attributes with write caches have been transferred to world data. */
  void ensure_motion_type();
  void ensure_constraints();
  void ensure_constraint_disable_collision();
  void ensure_custom_data_attribute(BodyAttribute attribute) const;
  void ensure_custom_data_attribute(ConstraintAttribute attribute) const;
  void remove_attributes_from_customdata();

  void create_world();
  void destroy_world();
  void move_or_copy_selection(const PhysicsWorldState &src,
                              const IndexMask &src_body_mask,
                              const IndexMask &src_constraint_mask,
                              const bke::AnonymousAttributePropagationInfo &propagation_info);
  bool try_move_data(const PhysicsWorldState &src,
                     int body_num,
                     int constraint_num,
                     const IndexMask &src_body_mask,
                     const IndexMask &src_constraint_mask,
                     int dst_body_offset,
                     int dst_constraint_offset);

  Span<CollisionShapePtr> shapes() const;
  MutableSpan<CollisionShapePtr> shapes_for_write();

  void set_overlap_filter(OverlapFilterFn fn);
  void clear_overlap_filter();

  float3 gravity() const;
  void set_gravity(const float3 &gravity);
  void set_solver_iterations(int num_solver_iterations);
  void set_split_impulse(bool split_impulse);

  void compute_local_inertia(const IndexMask &selection);

  void step_simulation(float delta_time);

  void apply_force(const IndexMask &selection,
                   const VArray<float3> &forces,
                   const VArray<float3> &relative_positions = {});
  void apply_torque(const IndexMask &selection, const VArray<float3> &torques);

  void apply_impulse(const IndexMask &selection,
                     const VArray<float3> &impulses,
                     const VArray<float3> &relative_positions = {});
  void apply_angular_impulse(const IndexMask &selection, const VArray<float3> &angular_impulses);
  void clear_forces(const IndexMask &selection);

  /* Validate internal relations, for debugging and testing purposes.
   * Should only be used in tests or debug mode. */
  bool validate_world_data();

  bke::AttributeAccessor attributes() const;
  bke::MutableAttributeAccessor attributes_for_write();

  static const PhysicsWorldDataAccessInfo &world_data_access_info();
  static const CustomDataAccessInfo &body_custom_data_access_info();
  static const CustomDataAccessInfo &constraint_custom_data_access_info();

 private:
  void ensure_read_cache_no_lock() const;
  void ensure_motion_type_no_lock();
  void ensure_body_collision_shapes_no_lock();
  void ensure_body_is_static_no_lock();
  void ensure_body_masses_no_lock();
  void ensure_constraints_no_lock();
  void ensure_constraint_disable_collision_no_lock();
  void ensure_custom_data_attribute_no_lock(BodyAttribute attribute);
  void ensure_custom_data_attribute_no_lock(ConstraintAttribute attribute);

  bke::AttributeAccessor custom_data_attributes() const;
  bke::MutableAttributeAccessor custom_data_attributes_for_write();
  bke::AttributeAccessor world_data_attributes() const;
  bke::MutableAttributeAccessor world_data_attributes_for_write();
};

class PhysicsGeometry {
 public:
  using OverlapFilterFn = std::function<bool(const int a, const int b)>;

  using BodyAttribute = PhysicsBodyAttribute;
  using ConstraintAttribute = PhysicsConstraintAttribute;
  using ConstraintType = PhysicsConstraintType;
  using BodyActivationState = PhysicsBodyActivationState;

 private:
  /* Implementation of the physics world and rigid bodies.
   * This is an implicit shared pointer, multiple users can read the physics data. Requesting write
   * access will move the data to a new component. This is somewhat different from other components
   * which do a full copy of the data, but necessary for efficiently handling physics state. Moving
   * the physics state will create a read-only cache, so older state still be accessed. */
  const PhysicsWorldState *world_state_ = nullptr;

 public:
  PhysicsGeometry();
  explicit PhysicsGeometry(int bodies_num, int constraints_num, int shapes_num);
  PhysicsGeometry(const PhysicsGeometry &other);
  ~PhysicsGeometry();

  const PhysicsWorldState &state() const;
  PhysicsWorldState &state_for_write();

  int bodies_num() const;
  int constraints_num() const;
  int shapes_num() const;

  IndexRange bodies_range() const;
  IndexRange constraints_range() const;
  IndexRange shapes_range() const;

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

  static StringRef body_attribute_name(BodyAttribute attribute);
  static StringRef constraint_attribute_name(ConstraintAttribute attribute);

  bke::AttributeAccessor attributes() const;
  bke::MutableAttributeAccessor attributes_for_write();

  /* XXX Attributes functions on components can nominally return a nullopt, but some parts of
   * the code don't check for that return value case
   * (realize_instances.cc::gather_attributes_for_propagation).
   * These dummy functions return a placeholder accessor that can be used as a fallback to
   * avoid this bug */
  static AttributeAccessor dummy_attributes();
  static MutableAttributeAccessor dummy_attributes_for_write();

  /* Validate internal world data.
   * Should only be used in tests. */
  bool validate_world_data();
};

}  // namespace blender::bke
