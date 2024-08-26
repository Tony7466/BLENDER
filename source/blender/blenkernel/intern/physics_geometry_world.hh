/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#pragma once

#include <functional>

#include "BLI_array.hh"
#include "BLI_cache_mutex.hh"
#include "BLI_index_mask_fwd.hh"
#include "BLI_utility_mixins.hh"
#include "BLI_virtual_array_fwd.hh"

#include "BKE_collision_shape.hh"
#include "BKE_physics_geometry.hh"

class btRigidBody;
class btMotionState;
class btCollisionShape;
class btTypedConstraint;
class btDiscreteDynamicsWorld;
class btDynamicsWorld;
class btCollisionConfiguration;
class btCollisionDispatcher;
class btBroadphaseInterface;
class btConstraintSolver;
struct btOverlapFilterCallback;
struct btJointFeedback;

#ifdef WITH_BULLET
#  include <BulletDynamics/ConstraintSolver/btTypedConstraint.h>
#else
struct btJointFeedback {};
#endif

namespace blender::bke {

using CollisionShapePtr = ImplicitSharingPtr<CollisionShape>;

class PhysicsWorldData : NonCopyable, NonMovable {
 public:
  using OverlapFilterFn = std::function<bool(const int a, const int b)>;

  using BodyAttribute = PhysicsBodyAttribute;
  using ConstraintAttribute = PhysicsConstraintAttribute;
  using BodyActivationState = PhysicsBodyActivationState;
  using ConstraintType = PhysicsConstraintType;

 private:
  btDiscreteDynamicsWorld *world_ = nullptr;
  btCollisionConfiguration *config_ = nullptr;
  btCollisionDispatcher *dispatcher_ = nullptr;
  btBroadphaseInterface *broadphase_ = nullptr;
  btConstraintSolver *constraint_solver_ = nullptr;
  btOverlapFilterCallback *overlap_filter_ = nullptr;

  Array<btRigidBody *> rigid_bodies_;
  Array<btMotionState *> motion_states_;
  Array<btTypedConstraint *> constraints_;
  Array<btJointFeedback> constraint_feedback_;

  mutable CacheMutex body_index_cache_;
  mutable CacheMutex constraint_index_cache_;
  mutable CacheMutex bodies_in_world_cache_;
  mutable CacheMutex constraints_in_world_cache_;

 public:
  PhysicsWorldData();
  PhysicsWorldData(int body_num, int constraint_num);
  ~PhysicsWorldData();

  void resize(int body_num, int constraint_num);
  void resize(int body_num,
              int constraint_num,
              const IndexMask &src_body_mask,
              const IndexMask &src_constraint_mask,
              int dst_body_offset,
              int dst_constraint_offset);
  void move(const IndexMask &src_body_mask,
            const IndexMask &src_constraint_mask,
            int dst_body_offset,
            int dst_constraint_offset);

  void tag_bodies_in_world() const;
  void tag_constraints_in_world() const;

  void ensure_body_and_constraint_indices() const;
  void ensure_bodies_and_constraints_in_world();

  /* Compute the indices of body collision shape pointers in the span. */
  void compute_body_shape_indices(Span<CollisionShapePtr> shapes,
                                  MutableSpan<int> r_indices) const;

  void create_constraints(const IndexMask &selection,
                          const VArray<int> &types,
                          const VArray<int> &bodies1,
                          const VArray<int> &bodies2);

  void set_disable_collision(const IndexMask &selection, const VArray<bool> &disable_collision);

  void set_overlap_filter(OverlapFilterFn fn);
  void clear_overlap_filter();

  float3 gravity() const;
  void set_gravity(const float3 &gravity);
  void set_solver_iterations(int num_solver_iterations);
  void set_split_impulse(bool split_impulse);

  void step_simulation(float delta_time);

  void set_body_shapes(const IndexMask &selection,
                       const Span<CollisionShapePtr> shapes,
                       const Span<int> shape_handles);
  void set_body_static(const IndexMask &selection, const Span<bool> is_static);
  void set_body_mass(const IndexMask &selection, const Span<float> masses);

  void apply_force(const IndexMask &selection,
                   const VArray<float3> &forces,
                   const VArray<float3> &relative_positions = {});
  void apply_torque(const IndexMask &selection, const VArray<float3> &torques);

  void apply_impulse(const IndexMask &selection,
                     const VArray<float3> &impulses,
                     const VArray<float3> &relative_positions = {});
  void apply_angular_impulse(const IndexMask &selection, const VArray<float3> &angular_impulses);
  void clear_forces(const IndexMask &selection);

  bke::AttributeAccessor attributes() const;
  bke::MutableAttributeAccessor attributes_for_write();

  /* For VArray access. */
  Span<btRigidBody *> bodies() const;
  Span<btTypedConstraint *> constraints() const;
  const btDynamicsWorld &world() const;

 private:
  void create_world();
  void destroy_world();
};

// XXX These are used by the validation function currently, should not be needed.
int get_body_index(const btRigidBody &body);
int get_constraint_index(const btTypedConstraint &constraint);
bool is_constraint_in_world(const btTypedConstraint &constraint);
bool validate_bullet_body_shape(const btRigidBody &body, const CollisionShapePtr &collision_shape);

}  // namespace blender::bke
