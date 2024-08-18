/* SPDX-FileCopyrightText: 2004-2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sim
 */

#include "BKE_attribute.hh"
#include "BKE_collision_shape.hh"
#include "BKE_customdata.hh"
#include "BKE_geometry_set.hh"
#include "BKE_physics_geometry.hh"
#include "BKE_type_conversions.hh"

#include "BLI_array_utils.hh"
#include "BLI_assert.h"
#include "BLI_cpp_type.hh"
#include "BLI_index_mask.hh"
#include "BLI_math_matrix.hh"
#include "BLI_math_quaternion_types.hh"
#include "BLI_utildefines.h"
#include "BLI_virtual_array.hh"

#include "DNA_customdata_types.h"
#include "physics_geometry_attributes.hh"
#include "physics_geometry_impl.hh"

#include <functional>
#include <mutex>

#ifdef WITH_BULLET
#  include <BulletCollision/CollisionDispatch/btCollisionObject.h>
#  include <BulletCollision/CollisionShapes/btBoxShape.h>
#  include <BulletCollision/CollisionShapes/btCollisionShape.h>
#  include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>
#  include <BulletCollision/Gimpact/btGImpactShape.h>
#  include <BulletDynamics/ConstraintSolver/btConeTwistConstraint.h>
#  include <BulletDynamics/ConstraintSolver/btFixedConstraint.h>
#  include <BulletDynamics/ConstraintSolver/btGearConstraint.h>
#  include <BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h>
#  include <BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h>
#  include <BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.h>
#  include <BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h>
#  include <BulletDynamics/ConstraintSolver/btTypedConstraint.h>
#  include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
#  include <BulletDynamics/Dynamics/btRigidBody.h>
#  include <LinearMath/btDefaultMotionState.h>
#  include <LinearMath/btMotionState.h>
#  include <LinearMath/btTransform.h>
#  include <btBulletDynamicsCommon.h>
#endif

namespace blender::bke {

#ifdef WITH_BULLET

struct DefaultOverlapFilter : public btOverlapFilterCallback {
  virtual bool needBroadphaseCollision(btBroadphaseProxy *proxy0, btBroadphaseProxy *proxy1) const
  {
    return (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) &&
           (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask);
  }
};

struct OverlapFilterWrapper : public btOverlapFilterCallback {
  using OverlapFilterFn = std::function<bool(const int a, const int b)>;

  OverlapFilterFn fn;

  OverlapFilterWrapper(OverlapFilterFn fn) : fn(fn) {}

  virtual bool needBroadphaseCollision(btBroadphaseProxy *proxy0, btBroadphaseProxy *proxy1) const
  {
    const int64_t body0 = int64_t(proxy0->m_clientObject);
    const int64_t body1 = int64_t(proxy1->m_clientObject);
    return (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) &&
           (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask) && fn(body0, body1);
  }
};

static int get_body_index(const btRigidBody &body)
{
  /* Bullet uses the btTypedConstraint static fixed body for unilateral constraints.
   * This is represented by index -1. */
  if (&body == &btTypedConstraint::getFixedBody()) {
    return -1;
  }

  return body.getUserIndex3();
}

static void set_body_index(btRigidBody &body, const int index)
{
  /* Bullet uses the btTypedConstraint static fixed body for unilateral constraints.
   * This body should not be modified. */
  if (&body == &btTypedConstraint::getFixedBody()) {
    return;
  }

  body.setUserIndex3(index);
}

/* Note: Bullet does not keep track of which constraints have been added to the world. We use one
 * of the ID bits to store this flag. */

static constexpr int BulletConstraintFlagBits = 1;
static constexpr int BulletConstraintFlagMask = (1 << BulletConstraintFlagBits) - 1;
static constexpr int BulletConstraintIdMask = 0xffffffff >> BulletConstraintFlagBits;

enum BulletConstraintFlags {
  InWorld = 1,
};

static int get_constraint_index(const btTypedConstraint &constraint)
{
  return constraint.getUserConstraintId() >> BulletConstraintFlagBits;
}

static void set_constraint_index(btTypedConstraint &constraint, const int index)
{
  constraint.setUserConstraintId((index & BulletConstraintIdMask) << BulletConstraintFlagBits);
}

static bool is_constraint_in_world(const btTypedConstraint &constraint)
{
  return (constraint.getUserConstraintId() & BulletConstraintFlags::InWorld) != 0;
}

static void set_constraint_in_world(btTypedConstraint &constraint, const bool in_world)
{
  const int user_id = constraint.getUserConstraintId();
  if (in_world) {
    constraint.setUserConstraintId(user_id | BulletConstraintFlags::InWorld);
  }
  else {
    constraint.setUserConstraintId(user_id & ~BulletConstraintFlags::InWorld);
  }
}

/* -------------------------------------------------------------------- */
/** \name Physics Geometry
 * \{ */

/* Similar to CacheMutex, but with supplied mutex and flag. */
static void ensure_cache(std::mutex &mutex,
                         std::atomic<bool> &flag,
                         const FunctionRef<void()> compute_cache)
{
  if (flag.load(std::memory_order_acquire)) {
    return;
  }
  std::scoped_lock lock{mutex};
  /* Double checked lock. */
  if (flag.load(std::memory_order_relaxed)) {
    return;
  }
  /* Use task isolation because a mutex is locked and the cache computation might use
   * multi-threading. */
  threading::isolate_task(compute_cache);

  flag.store(true, std::memory_order_release);
}

/* Same as ensure_cache but updates if any flag is dirty. */
static void ensure_cache_any(std::mutex &mutex,
                             Span<std::atomic<bool> *> flags,
                             Span<const FunctionRef<void()>> compute_caches)
{
  bool all_valid = true;
  for (const int i : flags.index_range()) {
    if (!flags[i]->load(std::memory_order_acquire)) {
      all_valid = false;
      break;
    }
  }
  if (all_valid) {
    return;
  }
  std::scoped_lock lock{mutex};
  /* Double checked lock. */
  for (const int i : flags.index_range()) {
    if (!flags[i]->load(std::memory_order_acquire)) {
      /* Use task isolation because a mutex is locked and the cache computation might use
       * multi-threading. */
      threading::isolate_task(compute_caches[i]);

      flags[i]->store(true, std::memory_order_release);
    }
  }
}

static void tag_cache_dirty(std::atomic<bool> &flag)
{
  flag.store(false);
}

static bool is_cache_dirty(const std::atomic<bool> &flag)
{
  return !flag.load(std::memory_order_relaxed);
}

static void create_bodies(MutableSpan<btRigidBody *> rigid_bodies,
                          MutableSpan<btMotionState *> motion_states,
                          const IndexMask &mask)
{
  mask.foreach_index([&](const int index) {
    const float mass = 1.0f;
    const float3 local_inertia = float3(0.0f);
    btMotionState *motion_state = motion_states[index] = new btDefaultMotionState();
    btCollisionShape *collision_shape = nullptr;
    rigid_bodies[index] = new btRigidBody(
        mass, motion_state, collision_shape, to_bullet(local_inertia));
    rigid_bodies[index]->updateInertiaTensor();
  });
}

static void create_bodies(MutableSpan<btRigidBody *> rigid_bodies,
                          MutableSpan<btMotionState *> motion_states)
{
  return create_bodies(rigid_bodies, motion_states, rigid_bodies.index_range());
}

static bool validate_bullet_constraint_type(const PhysicsGeometry::ConstraintType type,
                                            const btTypedConstraint *constraint)
{
  using ConstraintType = PhysicsGeometry::ConstraintType;

  switch (type) {
    case ConstraintType::Fixed:
      return dynamic_cast<const btFixedConstraint *>(constraint) != nullptr;
    case ConstraintType::Point:
      return dynamic_cast<const btPoint2PointConstraint *>(constraint) != nullptr;
    case ConstraintType::Hinge:
      return dynamic_cast<const btHinge2Constraint *>(constraint) != nullptr;
    case ConstraintType::Slider:
      return dynamic_cast<const btSliderConstraint *>(constraint) != nullptr;
    case ConstraintType::ConeTwist:
      return dynamic_cast<const btConeTwistConstraint *>(constraint) != nullptr;
    case ConstraintType::SixDoF:
      return dynamic_cast<const btGeneric6DofConstraint *>(constraint) != nullptr;
    case ConstraintType::SixDoFSpring:
      return dynamic_cast<const btGeneric6DofSpringConstraint *>(constraint) != nullptr;
    case ConstraintType::SixDoFSpring2:
      return dynamic_cast<const btGeneric6DofSpring2Constraint *>(constraint) != nullptr;
    case ConstraintType::Contact:
      /* XXX Currently unsupported. */
      return false;
    case ConstraintType::Gear:
      return dynamic_cast<const btGearConstraint *>(constraint) != nullptr;
  }
  BLI_assert_unreachable();
  return false;
}

static btTypedConstraint *make_bullet_constraint_type(const PhysicsGeometry::ConstraintType type,
                                                      btRigidBody &body1,
                                                      btRigidBody &body2)
{
  using ConstraintType = PhysicsGeometry::ConstraintType;

  [[maybe_unused]] btTransform zero_mat = btTransform::getIdentity();
  [[maybe_unused]] btVector3 zero_vec = btVector3(0, 0, 0);
  [[maybe_unused]] btVector3 axis_x = btVector3(1, 0, 0);
  [[maybe_unused]] btVector3 axis_y = btVector3(0, 1, 0);
  [[maybe_unused]] btVector3 axis_z = btVector3(0, 0, 1);

  switch (type) {
    case ConstraintType::Fixed:
      return new btFixedConstraint(body1, body2, zero_mat, zero_mat);
    case ConstraintType::Point:
      return new btPoint2PointConstraint(body1, body2, zero_vec, zero_vec);
    case ConstraintType::Hinge:
      return new btHinge2Constraint(body1, body2, zero_vec, axis_x, axis_y);
    case ConstraintType::Slider:
      return new btSliderConstraint(body1, body2, zero_mat, zero_mat, true);
    case ConstraintType::ConeTwist:
      return new btConeTwistConstraint(body1, body2, zero_mat, zero_mat);
    case ConstraintType::SixDoF:
      return new btGeneric6DofConstraint(body1, body2, zero_mat, zero_mat, true);
    case ConstraintType::SixDoFSpring:
      return new btGeneric6DofSpringConstraint(body1, body2, zero_mat, zero_mat, true);
    case ConstraintType::SixDoFSpring2:
      return new btGeneric6DofSpring2Constraint(body1, body2, zero_mat, zero_mat);
    case ConstraintType::Contact:
      /* XXX Currently unsupported. */
      BLI_assert_unreachable();
      return nullptr;
    case ConstraintType::Gear:
      return new btGearConstraint(body1, body2, zero_vec, zero_vec);
  }
  BLI_assert_unreachable();
  return nullptr;
}

static btTypedConstraint *make_constraint_type(const PhysicsGeometry::ConstraintType type,
                                               btRigidBody &body1,
                                               btRigidBody &body2,
                                               btJointFeedback *feedback)
{
  btTypedConstraint *constraint = make_bullet_constraint_type(type, body1, body2);
  BLI_assert(constraint != nullptr);
  constraint->setUserConstraintType(int(type));
  /* The user ID value is used to store the index and some flags in the lower bits. Initialize
   * this to zero to clear those bits initially (index is lazy-initialized). */
  constraint->setUserConstraintId(0);
  // constraint->enableFeedback(true);
  constraint->setJointFeedback(feedback);
  return constraint;
}

PhysicsWorldData::PhysicsWorldData()
{
  this->create_world();
}

PhysicsWorldData::PhysicsWorldData(int body_num, int constraint_num)
{
  this->create_world();
  this->resize(body_num, constraint_num);
}

PhysicsWorldData::~PhysicsWorldData()
{
  this->destroy_world();
  for (const int i : rigid_bodies_.index_range()) {
    delete rigid_bodies_[i];
  }
  for (const int i : motion_states_.index_range()) {
    delete motion_states_[i];
  }
  for (const int i : constraints_.index_range()) {
    delete constraints_[i];
  }
}

void PhysicsWorldData::resize(const int body_num, const int constraint_num)
{
  const IndexRange src_body_range = rigid_bodies_.index_range().take_front(body_num);
  const IndexRange src_constraint_range = constraints_.index_range().take_front(constraint_num);
  this->resize(body_num, constraint_num, src_body_range, src_constraint_range, 0, 0);
}

void PhysicsWorldData::resize(const int body_num,
                              const int constraint_num,
                              const IndexMask &src_body_mask,
                              const IndexMask &src_constraint_mask,
                              const int dst_body_offset,
                              const int dst_constraint_offset)
{
  const IndexRange dst_body_range = src_body_mask.index_range().shift(dst_body_offset);
  const IndexRange dst_constraint_range = src_constraint_mask.index_range().shift(
      dst_constraint_offset);
  BLI_assert(dst_body_range.is_empty() || dst_body_range.last() < body_num);
  BLI_assert(dst_constraint_range.is_empty() || dst_constraint_range.last() < constraint_num);

  /* Check both target range and overall size, in case the size is growing. */
  const bool bodies_changed = (body_num != rigid_bodies_.size() ||
                               dst_body_range != rigid_bodies_.index_range());
  const bool constraints_changed = (constraint_num != constraints_.size() ||
                                    dst_constraint_range != constraints_.index_range());
  /* If the full range is copied the offset can only be zero. */
  BLI_assert(bodies_changed || dst_body_offset == 0);
  BLI_assert(constraints_changed || dst_constraint_offset == 0);
  if (!bodies_changed && !constraints_changed) {
    return;
  }

  if (bodies_changed) {
    Array<btRigidBody *> new_rigid_bodies(body_num);
    Array<btMotionState *> new_motion_states(body_num);
    src_body_mask.foreach_index([&](const int src_i, const int i) {
      const int dst_i = dst_body_range[i];
      new_rigid_bodies[dst_i] = rigid_bodies_[src_i];
      new_motion_states[dst_i] = motion_states_[src_i];
      /* Clear to avoid deleting. */
      rigid_bodies_[src_i] = nullptr;
      motion_states_[src_i] = nullptr;
    });
    /* Delete unused. */
    for (const int src_i : rigid_bodies_.index_range()) {
      delete rigid_bodies_[src_i];
      delete motion_states_[src_i];
    }
    /* Create new bodies if growing. */
    create_bodies(new_rigid_bodies.as_mutable_span().drop_front(rigid_bodies_.size()),
                  new_motion_states.as_mutable_span().drop_front(rigid_bodies_.size()));
    rigid_bodies_ = std::move(new_rigid_bodies);
    motion_states_ = std::move(new_motion_states);

    body_index_cache_.tag_dirty();
  }

  if (constraints_changed) {
    Array<btTypedConstraint *> new_constraints(constraint_num, nullptr);
    Array<btJointFeedback> new_constraint_feedback(
        constraint_num,
        {btVector3(0, 0, 0), btVector3(0, 0, 0), btVector3(0, 0, 0), btVector3(0, 0, 0)});
    src_constraint_mask.foreach_index([&](const int src_i, const int i) {
      const int dst_i = dst_constraint_range[i];
      new_constraints[dst_i] = constraints_[src_i];
      new_constraint_feedback[dst_i] = constraint_feedback_[src_i];
      /* Clear to avoid deleting. */
      constraints_[src_i] = nullptr;
    });
    /* Delete unused. */
    for (const int src_i : constraints_.index_range()) {
      delete constraints_[src_i];
    }
    constraints_ = std::move(new_constraints);
    constraint_feedback_ = std::move(new_constraint_feedback);

    /* Initialize new constraints. */
    const VArray<int> default_types = VArray<int>::ForSingle(
        int(PhysicsGeometry::ConstraintType::Fixed), constraint_num);
    const VArray<int> default_bodies = VArray<int>::ForSingle(-1, constraint_num);
    create_constraints(IndexRange::from_begin_end(0, dst_constraint_range.start()),
                       default_types,
                       default_bodies,
                       default_bodies);
    create_constraints(
        IndexRange::from_begin_end(dst_constraint_range.one_after_last(), constraint_num),
        default_types,
        default_bodies,
        default_bodies);

    constraint_index_cache_.tag_dirty();
  }

  bodies_in_world_cache_.tag_dirty();
}

void PhysicsWorldData::move(const IndexMask &src_body_mask,
                            const IndexMask &src_constraint_mask,
                            int dst_body_offset,
                            int dst_constraint_offset)
{
  this->resize(rigid_bodies_.size(),
               constraints_.size(),
               src_body_mask,
               src_constraint_mask,
               dst_body_offset,
               dst_constraint_offset);
}

void PhysicsWorldData::tag_bodies_in_world() const
{
  bodies_in_world_cache_.tag_dirty();
}

void PhysicsWorldData::tag_constraints_in_world() const
{
  constraints_in_world_cache_.tag_dirty();
}

void PhysicsWorldData::ensure_body_and_constraint_indices() const
{
  body_index_cache_.ensure([&]() {
    for (const int i : rigid_bodies_.index_range()) {
      /* Note: Technically the btRigidBody is not mutable here! We're just using it as a cache with
       * exclusive write access, so it's fine. */
      btRigidBody &body = const_cast<btRigidBody &>(*rigid_bodies_[i]);
      set_body_index(body, i);
    }
  });
  constraint_index_cache_.ensure([&]() {
    for (const int i : constraints_.index_range()) {
      /* Note: Technically the btTypedConstraint is not mutable here! We're just using it as a
       * cache with exclusive write access, so it's fine. */
      btTypedConstraint &constraint = *const_cast<btTypedConstraint *>(this->constraints_[i]);
      set_constraint_index(constraint, i);
    }
  });
}

void PhysicsWorldData::ensure_bodies_and_constraints_in_world()
{
  bodies_in_world_cache_.ensure([&]() {
    for (const int i : rigid_bodies_.index_range()) {
      btRigidBody &body = *rigid_bodies_[i];
      if (!body.isInWorld()) {
        world_->addRigidBody(&body);
      }
    }
  });

  /* Note: Bullet does a linear search for every single constraint removal, not great. */
  constraints_in_world_cache_.ensure([&]() {
    for (const int i : constraints_.index_range()) {
      btTypedConstraint &constraint = *constraints_[i];
      if (!is_constraint_in_world(constraint)) {
        if (&constraint.getRigidBodyA() != &constraint.getRigidBodyB()) {
          world_->addConstraint(&constraint);
        }
        set_constraint_in_world(constraint, true);
      }
    }
  });
}

void PhysicsWorldData::compute_body_shape_indices(Span<CollisionShapePtr> shapes,
                                                  MutableSpan<int> r_indices) const
{
  BLI_assert(r_indices.size() == rigid_bodies_.size());

  /* Map shape pointers to indices in the local shapes array. */
  Map<const btCollisionShape *, int> shapes_map;
  shapes_map.reserve(shapes.size());
  for (const int index : shapes.index_range()) {
    const bke::CollisionShapePtr &shape_ptr = shapes[index];
    if (!shape_ptr) {
      continue;
    }
    const btCollisionShape *bt_shape = &shape_ptr->impl().as_bullet_shape();
    /* Note: duplicates are possible here, nothing preventing a shape pointer to be in the
    shapes
     * list twice, this is fine. */
    shapes_map.add(bt_shape, index);
  }

  for (const int i : rigid_bodies_.index_range()) {
    const btRigidBody *bt_body = rigid_bodies_[i];
    const btCollisionShape *bt_shape = bt_body->getCollisionShape();
    /* Every body's shape must be in the shapes list, can use asserting lookup here. */
    r_indices[i] = (bt_shape != nullptr ? shapes_map.lookup(bt_shape) : -1);
  }
}

void PhysicsWorldData::compute_disable_collision_flags(MutableSpan<bool> r_flags)
{
  BLI_assert(r_flags.size() == constraints_.size());

  /* Technically only need constraint indices here. */
  ensure_body_and_constraint_indices();

  r_flags.fill(false);
  for (const int i_body : rigid_bodies_.index_range()) {
    /* Note: Technically the btRigidBody is not mutable here, Bullet is just very incorrect with
     * const usage ... */
    btRigidBody &body = const_cast<btRigidBody &>(*rigid_bodies_[i_body]);
    for (const int i_ref : IndexRange(body.getNumConstraintRefs())) {
      const btTypedConstraint *constraint = body.getConstraintRef(i_ref);
      BLI_assert(constraint != nullptr);
      const int i_constraint = get_constraint_index(*constraint);
      r_flags[i_constraint] = true;
    }
  }
}

void PhysicsWorldData::create_constraints(const IndexMask &selection,
                                          const VArray<int> &types,
                                          const VArray<int> &bodies1,
                                          const VArray<int> &bodies2)
{
  using ConstraintType = PhysicsGeometry::ConstraintType;

  bool changed = false;

  const IndexRange bodies_range = rigid_bodies_.index_range();
  selection.foreach_index([&](const int index) {
    const ConstraintType type = ConstraintType(types[index]);
    const int body_index1 = bodies1[index];
    const int body_index2 = bodies2[index];
    btRigidBody *fixed_body = &btTypedConstraint::getFixedBody();
    btRigidBody *body1 = bodies_range.contains(body_index1) ? rigid_bodies_[body_index1] :
                                                              fixed_body;
    btRigidBody *body2 = bodies_range.contains(body_index2) ? rigid_bodies_[body_index2] :
                                                              fixed_body;

    /* This function may be used to initialize constraint pointers, they can still be null here. */
    const btTypedConstraint *old_constraint = constraints_[index];
    if (old_constraint) {
      const ConstraintType old_type = ConstraintType(old_constraint->getUserConstraintType());
      const btRigidBody *old_body1 = &old_constraint->getRigidBodyA();
      const btRigidBody *old_body2 = &old_constraint->getRigidBodyB();
      if (old_type == type && old_body1 == body1 && old_body2 == body2) {
        /* All valid, no changes. */
        return;
      }

      world_->removeConstraint(const_cast<btTypedConstraint *>(old_constraint));
    }

    constraints_[index] = make_constraint_type(type, *body1, *body2, &constraint_feedback_[index]);
    changed = true;

    if (old_constraint) {
      /* Copy properties from the old constraint so attributes are persistent.
       * Notes:
       * - Applied impulse/force/torque is not copied, these are results of the solver and
       *   resetting on type changes is ok.
       * - Disabled collision isn't directly stored on constraints but as a constraint reference on
       *   the constrained bodies. We reconstruct this from the custom data flag instead of trying
       *   to copy here.
       */
      constraints_[index]->setEnabled(old_constraint->isEnabled());
      set_constraint_frame1(*constraints_[index], get_constraint_frame1(*old_constraint));
      set_constraint_frame2(*constraints_[index], get_constraint_frame2(*old_constraint));
      constraints_[index]->setBreakingImpulseThreshold(
          old_constraint->getBreakingImpulseThreshold());

      delete old_constraint;
    }
  });

  if (changed) {
    constraint_index_cache_.tag_dirty();
    tag_constraints_in_world();
  }
}

void PhysicsWorldData::set_overlap_filter(OverlapFilterFn fn)
{
  overlap_filter_ = new OverlapFilterWrapper(std::move(fn));
  broadphase_->getOverlappingPairCache()->setOverlapFilterCallback(overlap_filter_);
}

void PhysicsWorldData::clear_overlap_filter()
{
  if (overlap_filter_) {
    delete overlap_filter_;
    overlap_filter_ = new DefaultOverlapFilter();
  }
  broadphase_->getOverlappingPairCache()->setOverlapFilterCallback(overlap_filter_);
}

float3 PhysicsWorldData::gravity() const
{
  return to_blender(world_->getGravity());
}

void PhysicsWorldData::set_gravity(const float3 &gravity)
{
  world_->setGravity(to_bullet(gravity));
}

void PhysicsWorldData::set_solver_iterations(const int num_solver_iterations)
{
  btContactSolverInfo &info = world_->getSolverInfo();
  info.m_numIterations = num_solver_iterations;
}

void PhysicsWorldData::set_split_impulse(const bool split_impulse)
{
  btContactSolverInfo &info = world_->getSolverInfo();
  /* Note: Bullet stores this as int, but it's used as a bool. */
  info.m_splitImpulse = int(split_impulse);
}

void PhysicsWorldData::step_simulation(float delta_time)
{
  constexpr const float fixed_time_step = 1.0f / 60.0f;

  this->ensure_bodies_and_constraints_in_world();

  world_->stepSimulation(delta_time, 20, fixed_time_step);
}

void PhysicsWorldData::set_body_shapes(const IndexMask &selection,
                                       const Span<CollisionShapePtr> shapes,
                                       const Span<int> shape_handles)
{
  bool removed_body = false;
  selection.foreach_index([&](const int index) {
    const int handle = shape_handles[index];
    if (!shapes.index_range().contains(handle)) {
      return;
    }
    const CollisionShapePtr &shape_ptr = shapes[handle];
    const btCollisionShape *bt_shape = shape_ptr ? &shape_ptr->impl().as_bullet_shape() : nullptr;

    btRigidBody *body = this->rigid_bodies_[index];
    if (body->getCollisionShape() == bt_shape) {
      /* Shape is correct, nothing to do. */
      return;
    }
    const bool was_static = body->isStaticObject();

    // XXX is const_cast safe here? not sure why Bullet wants a mutable shape.
    if (bt_shape == nullptr) {
      world_->removeRigidBody(body);
      body->setCollisionShape(nullptr);
      removed_body = true;
    }
    else {
      /* Motion type and mass must be compatible with the shape. */
      const bool set_static = bt_shape->isNonMoving();
      if (set_static != was_static) {
        world_->removeRigidBody(body);
        // this->broadphase_->getOverlappingPairCache()->cleanProxyFromPairs(
        //    body->getBroadphaseProxy(), this->dispatcher_);
        removed_body = true;
      }

      if (set_static) {
        body->setMassProps(0.0f, btVector3(0.0f, 0.0f, 0.0f));
        body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT);
      }

      body->setCollisionShape(const_cast<btCollisionShape *>(bt_shape));
    }
  });
  if (removed_body) {
    this->tag_bodies_in_world();
  }
}

void PhysicsWorldData::set_body_static(const IndexMask &selection, const Span<bool> is_static)
{
  bool removed_body = false;
  selection.foreach_index([&](const int index) {
    btRigidBody *body = this->rigid_bodies_[index];

    /* Body must also be static if the collision shape is non-moveable. */
    const bool was_static = body->isStaticObject();
    const bool set_static = is_static[index] || (body->getCollisionShape() &&
                                                 body->getCollisionShape()->isNonMoving());
    if (set_static != was_static) {
      world_->removeRigidBody(body);
      // this->broadphase_->getOverlappingPairCache()->cleanProxyFromPairs(
      //    body->getBroadphaseProxy(), this->dispatcher_);
      removed_body = true;
    }

    if (set_static) {
      /* Static body must have zero mass. */
      body->setMassProps(0.0f, btVector3(0.0f, 0.0f, 0.0f));

      body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT);
    }
    else {
      body->setCollisionFlags(body->getCollisionFlags() & ~btCollisionObject::CF_STATIC_OBJECT);
    }
  });
  if (removed_body) {
    this->tag_bodies_in_world();
  }
}

void PhysicsWorldData::set_body_mass(const IndexMask &selection, const Span<float> masses)
{
  bool removed_body = false;
  selection.foreach_index([&](const int index) {
    btRigidBody *body = this->rigid_bodies_[index];

    /* Body must have zero mass if static or the collision shape is non-moveable. */
    const bool was_static = body->isStaticObject();
    const bool set_zero_mass = (body->getCollisionFlags() & btCollisionObject::CF_STATIC_OBJECT) ||
                               (body->getCollisionShape() &&
                                body->getCollisionShape()->isNonMoving());
    if (set_zero_mass != was_static) {
      world_->removeRigidBody(body);
      // this->broadphase_->getOverlappingPairCache()->cleanProxyFromPairs(
      //    body->getBroadphaseProxy(), this->dispatcher_);
      removed_body = true;
    }

    if (set_zero_mass) {
      /* Static body must have zero mass. */
      body->setMassProps(0.0f, btVector3(0.0f, 0.0f, 0.0f));
    }
    else {
      body->setMassProps(masses[index], body->getLocalInertia());
    }
  });
  if (removed_body) {
    this->tag_bodies_in_world();
  }
}

void PhysicsWorldData::apply_force(const IndexMask &selection,
                                   const VArray<float3> &forces,
                                   const VArray<float3> &relative_positions)
{
  if (!relative_positions) {
    selection.foreach_index([&](const int index) {
      const float3 force = forces[index];
      rigid_bodies_[index]->applyCentralForce(to_bullet(force));
    });
  }

  selection.foreach_index([&](const int index) {
    const float3 force = forces[index];
    const float3 relative_position = relative_positions[index];
    rigid_bodies_[index]->applyForce(to_bullet(force), to_bullet(relative_position));
  });
}

void PhysicsWorldData::apply_torque(const IndexMask &selection, const VArray<float3> &torques)
{
  selection.foreach_index([&](const int index) {
    const float3 torque = torques[index];
    rigid_bodies_[index]->applyTorque(to_bullet(torque));
  });
}

void PhysicsWorldData::apply_impulse(const IndexMask &selection,
                                     const VArray<float3> &impulses,
                                     const VArray<float3> &relative_positions)
{
  if (!relative_positions) {
    selection.foreach_index([&](const int index) {
      const float3 impulse = impulses[index];
      rigid_bodies_[index]->applyCentralImpulse(to_bullet(impulse));
    });
  }

  selection.foreach_index([&](const int index) {
    const float3 impulse = impulses[index];
    const float3 relative_position = relative_positions[index];
    rigid_bodies_[index]->applyImpulse(to_bullet(impulse), to_bullet(relative_position));
  });
}

void PhysicsWorldData::apply_angular_impulse(const IndexMask &selection,
                                             const VArray<float3> &angular_impulses)
{
  selection.foreach_index([&](const int index) {
    const float3 angular_impulse = angular_impulses[index];
    rigid_bodies_[index]->applyTorqueImpulse(to_bullet(angular_impulse));
  });
}

void PhysicsWorldData::clear_forces(const IndexMask &selection)
{
  selection.foreach_index([&](const int index) { rigid_bodies_[index]->clearForces(); });
}

Span<btRigidBody *> PhysicsWorldData::bodies() const
{
  return rigid_bodies_;
}

Span<btTypedConstraint *> PhysicsWorldData::constraints() const
{
  return constraints_;
}

const btDynamicsWorld &PhysicsWorldData::world() const
{
  return *world_;
}

void PhysicsWorldData::create_world()
{ /* Add all bodies and constraints to the world. */
  config_ = new btDefaultCollisionConfiguration();
  dispatcher_ = new btCollisionDispatcher(config_);
  btGImpactCollisionAlgorithm::registerAlgorithm((btCollisionDispatcher *)dispatcher_);

  broadphase_ = new btDbvtBroadphase();
  overlap_filter_ = new DefaultOverlapFilter();
  broadphase_->getOverlappingPairCache()->setOverlapFilterCallback(overlap_filter_);

  constraint_solver_ = new btSequentialImpulseConstraintSolver();

  world_ = new btDiscreteDynamicsWorld(dispatcher_, broadphase_, constraint_solver_, config_);
}

void PhysicsWorldData::destroy_world()
{
  for (btRigidBody *body : rigid_bodies_) {
    world_->removeRigidBody(body);
  }
  for (btTypedConstraint *constraint : constraints_) {
    if (!constraint) {
      continue;
    }
    world_->removeConstraint(constraint);
  }

  delete world_;
  delete constraint_solver_;
  delete broadphase_;
  delete dispatcher_;
  delete config_;
  delete overlap_filter_;

  world_ = nullptr;
  constraint_solver_ = nullptr;
  broadphase_ = nullptr;
  dispatcher_ = nullptr;
  config_ = nullptr;
  overlap_filter_ = nullptr;
}

PhysicsGeometryImpl::PhysicsGeometryImpl()
    : body_num_(0), constraint_num_(0), body_data_({}), constraint_data_({})
{
  CustomData_reset(&body_data_);
  CustomData_reset(&constraint_data_);
  this->tag_read_cache_changed();
}

PhysicsGeometryImpl::PhysicsGeometryImpl(int body_num, int constraint_num, int shape_num)
    : body_num_(body_num), constraint_num_(constraint_num), body_data_({}), constraint_data_({})
{
  CustomData_reset(&body_data_);
  CustomData_reset(&constraint_data_);
  CustomData_realloc(&body_data_, 0, body_num);
  CustomData_realloc(&constraint_data_, 0, constraint_num);
  shapes.reinitialize(shape_num);
  this->tag_read_cache_changed();
}

PhysicsGeometryImpl::PhysicsGeometryImpl(const PhysicsGeometryImpl &other)
{
  *this = other;
  this->tag_read_cache_changed();
}

PhysicsGeometryImpl::~PhysicsGeometryImpl()
{
  CustomData_free(&body_data_, body_num_);
  CustomData_free(&constraint_data_, constraint_num_);

  /* World data is owned by the geometry when it's mutable (always the case on destruction). */
  delete this->world_data;
}

PhysicsGeometryImpl &PhysicsGeometryImpl::operator=(const PhysicsGeometryImpl &other)
{
  CustomData_reset(&body_data_);
  CustomData_reset(&constraint_data_);
  body_num_ = other.body_num_;
  constraint_num_ = other.constraint_num_;
  CustomData_copy(&other.body_data_, &body_data_, CD_MASK_ALL, other.body_num_);
  CustomData_copy(&other.constraint_data_, &constraint_data_, CD_MASK_ALL, other.constraint_num_);

  if (other.world_data) {
    try_move_data(other,
                  body_num_,
                  constraint_num_,
                  IndexRange(body_num_),
                  IndexRange(constraint_num_),
                  0,
                  0);
  }

  this->tag_read_cache_changed();
  return *this;
}

void PhysicsGeometryImpl::delete_self()
{
  delete this;
}

void PhysicsGeometryImpl::tag_read_cache_changed()
{
  /* Cache only becomes invalid if there is world data that can change. */
  if (this->world_data != nullptr) {
    tag_cache_dirty(this->custom_data_read_cache_valid);
  }
}

void PhysicsGeometryImpl::tag_body_topology_changed()
{
  this->tag_constraint_disable_collision_changed();
}

void PhysicsGeometryImpl::tag_body_collision_shape_changed()
{
  tag_cache_dirty(this->body_collision_shapes_valid);
}

void PhysicsGeometryImpl::tag_body_is_static_changed()
{
  tag_cache_dirty(this->body_is_static_valid);
}

void PhysicsGeometryImpl::tag_body_mass_changed()
{
  tag_cache_dirty(this->body_mass_valid);
}

void PhysicsGeometryImpl::tag_constraints_changed()
{
  tag_cache_dirty(this->constraints_valid);
}

void PhysicsGeometryImpl::tag_constraint_disable_collision_changed()
{
  tag_cache_dirty(this->constraint_disable_collision_valid);
  this->tag_read_cache_changed();
}

bool PhysicsGeometryImpl::has_builtin_attribute_custom_data_layer(
    PhysicsGeometryImpl::BodyAttribute attribute) const
{
  AttributeAccessor dst_attributes = this->custom_data_attributes();
  StringRef name = physics_attribute_name(attribute);
  return bool(dst_attributes.lookup_meta_data(name));
}

bool PhysicsGeometryImpl::has_builtin_attribute_custom_data_layer(
    PhysicsGeometryImpl::ConstraintAttribute attribute) const
{
  AttributeAccessor dst_attributes = this->custom_data_attributes();
  StringRef name = physics_attribute_name(attribute);
  return bool(dst_attributes.lookup_meta_data(name));
}

void PhysicsGeometryImpl::ensure_read_cache() const
{
  using BodyAttribute = PhysicsGeometry::BodyAttribute;
  using ConstraintAttribute = PhysicsGeometry::ConstraintAttribute;

  const static StringRef collision_shape_id = PhysicsGeometry::body_attribute_name(
      BodyAttribute::collision_shape);
  const static StringRef is_static_id = PhysicsGeometry::body_attribute_name(
      BodyAttribute::is_static);
  const static StringRef mass_id = PhysicsGeometry::body_attribute_name(BodyAttribute::mass);
  const static StringRef constraint_type_id = PhysicsGeometry::constraint_attribute_name(
      ConstraintAttribute::constraint_type);
  const static StringRef constraint_body1_id = PhysicsGeometry::constraint_attribute_name(
      ConstraintAttribute::constraint_body1);
  const static StringRef constraint_body2_id = PhysicsGeometry::constraint_attribute_name(
      ConstraintAttribute::constraint_body2);
  const static StringRef disable_collision_id = PhysicsGeometry::constraint_attribute_name(
      ConstraintAttribute::disable_collision);

  ensure_cache(this->data_mutex, this->custom_data_read_cache_valid, [&]() {
    PhysicsGeometryImpl &dst = *const_cast<PhysicsGeometryImpl *>(this);

    if (this->world_data == nullptr) {
      for (const BodyAttribute attribute : all_body_attributes()) {
        dst.ensure_custom_data_attribute_no_lock(attribute);
      }
      for (const ConstraintAttribute attribute : all_constraint_attributes()) {
        dst.ensure_custom_data_attribute_no_lock(attribute);
      }
      return;
    }

    /* Some attributes require other updates before valid world data can be read. */
    dst.ensure_motion_type_no_lock();
    dst.ensure_constraints_no_lock();

    /* Write to cache attributes. */
    MutableAttributeAccessor dst_attributes = dst.custom_data_attributes_for_write();
    /* Disable collision flags for constraints are computed explicitly. */
    SpanAttributeWriter<bool> disable_collision_flags =
        dst_attributes.lookup_or_add_for_write_only_span<bool>(disable_collision_id,
                                                               AttrDomain::Edge);
    this->world_data->compute_disable_collision_flags(disable_collision_flags.span);
    disable_collision_flags.finish();

    /* Read from world data and ignore the cache.
     * Important! This also prevents deadlock caused by re-entering this function. */
    const AttributeAccessor src_attributes = this->world_data_attributes();
    Set<std::string> skip_attributes = {collision_shape_id,
                                        is_static_id,
                                        mass_id,
                                        constraint_type_id,
                                        constraint_body1_id,
                                        constraint_body2_id,
                                        disable_collision_id};
    /* Only use builtin attributes, dynamic attributes are already in custom data. */
    src_attributes.for_all(
        [&](const AttributeIDRef &id, const AttributeMetaData & /*meta_data*/) -> bool {
          if (!src_attributes.is_builtin(id)) {
            skip_attributes.add(id.name());
          }
          return true;
        });

    gather_attributes(src_attributes,
                      bke::AttrDomain::Point,
                      {},
                      skip_attributes,
                      IndexRange(body_num_),
                      dst_attributes);
    gather_attributes(src_attributes,
                      bke::AttrDomain::Edge,
                      {},
                      skip_attributes,
                      IndexRange(constraint_num_),
                      dst_attributes);
  });
}

void PhysicsGeometryImpl::ensure_motion_type()
{
  ensure_cache_any(
      this->data_mutex,
      {&this->body_collision_shapes_valid, &this->body_is_static_valid, &this->body_mass_valid},
      {[&]() { this->ensure_body_collision_shapes_no_lock(); },
       [&]() { this->ensure_body_is_static_no_lock(); },
       [&]() { this->ensure_body_masses_no_lock(); }});
}

void PhysicsGeometryImpl::ensure_motion_type_no_lock()
{
  this->ensure_body_collision_shapes_no_lock();
  this->ensure_body_is_static_no_lock();
  this->ensure_body_masses_no_lock();
}

void PhysicsGeometryImpl::ensure_body_collision_shapes_no_lock()
{
  const static StringRef collision_shape_id = PhysicsGeometry::body_attribute_name(
      PhysicsGeometry::BodyAttribute::collision_shape);

  if (!is_cache_dirty(this->body_collision_shapes_valid)) {
    return;
  }
  if (this->world_data == nullptr) {
    return;
  }

  AttributeAccessor custom_data_attributes = this->custom_data_attributes();
  const VArraySpan<int> body_shapes = *custom_data_attributes.lookup_or_default(
      collision_shape_id, AttrDomain::Point, -1);

  const IndexMask selection = this->world_data->bodies().index_range();
  this->world_data->set_body_shapes(selection, this->shapes, body_shapes);
}

void PhysicsGeometryImpl::ensure_body_is_static_no_lock()
{
  const static StringRef is_static_id = PhysicsGeometry::body_attribute_name(
      PhysicsGeometry::BodyAttribute::is_static);

  if (!is_cache_dirty(this->body_is_static_valid)) {
    return;
  }
  if (this->world_data == nullptr) {
    return;
  }

  AttributeAccessor custom_data_attributes = this->custom_data_attributes();
  const VArraySpan<bool> is_static = *custom_data_attributes.lookup_or_default<bool>(
      is_static_id, AttrDomain::Point, -1);

  const IndexMask selection = this->world_data->bodies().index_range();
  this->world_data->set_body_static(selection, is_static);
}

void PhysicsGeometryImpl::ensure_body_masses_no_lock()
{
  const static StringRef mass_id = PhysicsGeometry::body_attribute_name(
      PhysicsGeometry::BodyAttribute::mass);

  if (!is_cache_dirty(this->body_mass_valid)) {
    return;
  }
  if (this->world_data == nullptr) {
    return;
  }

  AttributeAccessor custom_data_attributes = this->custom_data_attributes();
  const VArraySpan<float> masses = *custom_data_attributes.lookup_or_default<float>(
      mass_id, AttrDomain::Point, 0.0f);

  const IndexMask selection = this->world_data->bodies().index_range();
  this->world_data->set_body_mass(selection, masses);
}

void PhysicsGeometryImpl::ensure_constraints()
{
  ensure_cache(
      this->data_mutex, this->constraints_valid, [&]() { this->ensure_constraints_no_lock(); });
}

void PhysicsGeometryImpl::ensure_constraints_no_lock()
{
  using ConstraintType = PhysicsGeometry::ConstraintType;

  const static StringRef constraint_type_id = PhysicsGeometry::constraint_attribute_name(
      PhysicsGeometry::ConstraintAttribute::constraint_type);
  const static StringRef constraint_body1_id = PhysicsGeometry::constraint_attribute_name(
      PhysicsGeometry::ConstraintAttribute::constraint_body1);
  const static StringRef constraint_body2_id = PhysicsGeometry::constraint_attribute_name(
      PhysicsGeometry::ConstraintAttribute::constraint_body2);

  if (!is_cache_dirty(this->constraints_valid)) {
    return;
  }
  if (this->world_data == nullptr) {
    return;
  }

  AttributeAccessor custom_data_attributes = this->custom_data_attributes();
  const VArray<int> types = *custom_data_attributes.lookup_or_default<int>(
      constraint_type_id, AttrDomain::Edge, int(ConstraintType::Fixed));
  const VArray<int> body1 = *custom_data_attributes.lookup_or_default<int>(
      constraint_body1_id, AttrDomain::Edge, -1);
  const VArray<int> body2 = *custom_data_attributes.lookup_or_default<int>(
      constraint_body2_id, AttrDomain::Edge, -1);

  const IndexMask selection = this->world_data->constraints().index_range();
  this->world_data->create_constraints(selection, types, body1, body2);
}

void PhysicsGeometryImpl::ensure_custom_data_attribute(
    PhysicsGeometryImpl::BodyAttribute attribute) const
{
  if (this->world_data != nullptr || has_builtin_attribute_custom_data_layer(attribute)) {
    return;
  }
  std::scoped_lock lock(this->data_mutex);
  if (this->world_data != nullptr || has_builtin_attribute_custom_data_layer(attribute)) {
    return;
  }

  PhysicsGeometryImpl &dst = *const_cast<PhysicsGeometryImpl *>(this);
  dst.ensure_custom_data_attribute_no_lock(attribute);
}

void PhysicsGeometryImpl::ensure_custom_data_attribute(
    PhysicsGeometryImpl::ConstraintAttribute attribute) const
{
  if (this->world_data != nullptr || has_builtin_attribute_custom_data_layer(attribute)) {
    return;
  }
  std::scoped_lock lock(this->data_mutex);
  if (this->world_data != nullptr || has_builtin_attribute_custom_data_layer(attribute)) {
    return;
  }

  PhysicsGeometryImpl &dst = *const_cast<PhysicsGeometryImpl *>(this);
  dst.ensure_custom_data_attribute_no_lock(attribute);
}

void PhysicsGeometryImpl::ensure_custom_data_attribute_no_lock(BodyAttribute attribute)
{
  MutableAttributeAccessor dst_attributes = this->custom_data_attributes_for_write();
  StringRef name = physics_attribute_name(attribute);
  const CPPType &type = physics_attribute_type(attribute);
  GVArray varray = GVArray::ForSingle(
      type, this->body_num_, physics_attribute_default_value(attribute));
  eCustomDataType data_type = cpp_type_to_custom_data_type(type);
  dst_attributes.add(name, AttrDomain::Point, data_type, AttributeInitVArray(varray));
}

void PhysicsGeometryImpl::ensure_custom_data_attribute_no_lock(ConstraintAttribute attribute)
{
  MutableAttributeAccessor dst_attributes = this->custom_data_attributes_for_write();
  StringRef name = physics_attribute_name(attribute);
  const CPPType &type = physics_attribute_type(attribute);
  GVArray varray = GVArray::ForSingle(
      type, constraint_num_, physics_attribute_default_value(attribute));
  eCustomDataType data_type = cpp_type_to_custom_data_type(type);
  dst_attributes.add(name, AttrDomain::Edge, data_type, AttributeInitVArray(varray));
}

void PhysicsGeometryImpl::compute_local_inertia(const IndexMask &selection)
{
  using BodyAttribute = PhysicsGeometry::BodyAttribute;

  MutableAttributeAccessor attributes = this->attributes_for_write();
  const VArray<int> body_shapes = *attributes.lookup_or_default<int>(
      physics_attribute_name(BodyAttribute::collision_shape), AttrDomain::Point, -1);
  const VArray<float> masses = *attributes.lookup_or_default<float>(
      physics_attribute_name(BodyAttribute::mass), AttrDomain::Point, 1.0f);
  AttributeWriter<float3> local_inertias = attributes.lookup_or_add_for_write<float3>(
      physics_attribute_name(BodyAttribute::inertia), AttrDomain::Point);

  selection.foreach_index([&](const int body_i) {
    const int shape_index = body_shapes[body_i];
    if (!this->shapes.index_range().contains(shape_index)) {
      local_inertias.varray.set(body_i, float3(1.0f));
      return;
    }
    const CollisionShapePtr &shape_ptr = shapes[shape_index];
    if (shape_ptr == nullptr) {
      local_inertias.varray.set(body_i, float3(1.0f));
      return;
    }
    const btCollisionShape &bt_shape = shape_ptr->impl().as_bullet_shape();

    btVector3 bt_local_inertia;
    if (bt_shape.isNonMoving()) {
      bt_local_inertia = btVector3(0, 0, 0);
    }
    else {
      bt_shape.calculateLocalInertia(masses[body_i], bt_local_inertia);
    }

    local_inertias.varray.set(body_i, to_blender(bt_local_inertia));
  });

  local_inertias.finish();
}

void PhysicsGeometryImpl::apply_force(const IndexMask &selection,
                                      const VArray<float3> &forces,
                                      const VArray<float3> &relative_positions)
{
  if (this->world_data == nullptr) {
    return;
  }
  std::scoped_lock lock(this->data_mutex);
  if (this->world_data == nullptr) {
    return;
  }
  world_data->apply_force(selection, forces, relative_positions);

  this->tag_read_cache_changed();
}

void PhysicsGeometryImpl::apply_torque(const IndexMask &selection, const VArray<float3> &torques)
{
  if (this->world_data == nullptr) {
    return;
  }
  std::scoped_lock lock(this->data_mutex);
  if (this->world_data == nullptr) {
    return;
  }
  world_data->apply_torque(selection, torques);

  this->tag_read_cache_changed();
}

void PhysicsGeometryImpl::apply_impulse(const IndexMask &selection,
                                        const VArray<float3> &impulses,
                                        const VArray<float3> &relative_positions)
{
  if (this->world_data == nullptr) {
    return;
  }
  std::scoped_lock lock(this->data_mutex);
  if (this->world_data == nullptr) {
    return;
  }
  world_data->apply_impulse(selection, impulses, relative_positions);

  this->tag_read_cache_changed();
}

void PhysicsGeometryImpl::apply_angular_impulse(const IndexMask &selection,
                                                const VArray<float3> &angular_impulses)
{
  if (this->world_data == nullptr) {
    return;
  }
  std::scoped_lock lock(this->data_mutex);
  if (this->world_data == nullptr) {
    return;
  }
  world_data->apply_angular_impulse(selection, angular_impulses);

  this->tag_read_cache_changed();
}

void PhysicsGeometryImpl::clear_forces(const IndexMask &selection)
{
  if (this->world_data == nullptr) {
    return;
  }
  std::scoped_lock lock(this->data_mutex);
  if (this->world_data == nullptr) {
    return;
  }
  world_data->clear_forces(selection);

  this->tag_read_cache_changed();
}

// void PhysicsGeometryImpl::create_constraints(const IndexMask &selection,
//                                              const VArray<int> &types,
//                                              const VArray<int> &bodies1,
//                                              const VArray<int> &bodies2)
//{
//   const static StringRef constraint_type_id = PhysicsGeometry::constraint_attribute_name(
//       PhysicsGeometry::ConstraintAttribute::constraint_type);
//   const static StringRef constraint_body1_id = PhysicsGeometry::constraint_attribute_name(
//       PhysicsGeometry::ConstraintAttribute::constraint_body1);
//   const static StringRef constraint_body2_id = PhysicsGeometry::constraint_attribute_name(
//       PhysicsGeometry::ConstraintAttribute::constraint_body2);
//
//   auto write_to_custom_data = [&]() {
//     this->ensure_custom_data_attribute(PhysicsGeometry::ConstraintAttribute::constraint_type);
//     this->ensure_custom_data_attribute(PhysicsGeometry::ConstraintAttribute::constraint_body1);
//     this->ensure_custom_data_attribute(PhysicsGeometry::ConstraintAttribute::constraint_body2);
//     MutableAttributeAccessor attributes = this->custom_data_attributes_for_write();
//     SpanAttributeWriter<int> dst_types =
//     attributes.lookup_for_write_span<int>(constraint_type_id); SpanAttributeWriter<int>
//     dst_body1 = attributes.lookup_for_write_span<int>(
//         constraint_body1_id);
//     SpanAttributeWriter<int> dst_body2 = attributes.lookup_for_write_span<int>(
//         constraint_body2_id);
//     types.materialize_to_uninitialized(selection, dst_types.span);
//     bodies1.materialize_to_uninitialized(selection, dst_body1.span);
//     bodies2.materialize_to_uninitialized(selection, dst_body2.span);
//     dst_types.finish();
//     dst_body1.finish();
//     dst_body2.finish();
//   };
//
//   if (world_data == nullptr) {
//     write_to_custom_data();
//     return;
//   }
//   std::scoped_lock lock(this->data_mutex);
//   if (world_data == nullptr) {
//     write_to_custom_data();
//     return;
//   }
//
//   world_data->create_constraints(selection, types, bodies1, bodies2);
//
//   this->tag_read_cache_changed();
// }

void PhysicsGeometryImpl::create_world()
{
  // using BodyAttribute = PhysicsGeometry::BodyAttribute;
  using ConstraintAttribute = PhysicsGeometry::ConstraintAttribute;
  using ConstraintType = PhysicsGeometry::ConstraintType;

  /* Avoid locking later on. */
  this->ensure_read_cache();

  if (this->world_data) {
    return;
  }
  std::scoped_lock lock(this->data_mutex);
  if (this->world_data) {
    return;
  }

  /* Read from custom data, write to world data. */
  const AttributeAccessor src_attributes = this->custom_data_attributes();

  const IndexRange body_range = IndexRange(this->body_num_);
  const IndexRange constraint_range = IndexRange(this->constraint_num_);

  const VArraySpan<int> body_shapes = *src_attributes.lookup_or_default(
      physics_attribute_name(BodyAttribute::collision_shape), AttrDomain::Point, -1);
  const VArray<int> constraint_types = *src_attributes.lookup_or_default(
      physics_attribute_name(ConstraintAttribute::constraint_type),
      AttrDomain::Edge,
      int(ConstraintType::Fixed));
  const VArray<int> constraint_bodies1 = *src_attributes.lookup_or_default(
      physics_attribute_name(ConstraintAttribute::constraint_body1), AttrDomain::Edge, -1);
  const VArray<int> constraint_bodies2 = *src_attributes.lookup_or_default(
      physics_attribute_name(ConstraintAttribute::constraint_body2), AttrDomain::Edge, -1);

  this->world_data = new PhysicsWorldData(this->body_num_, this->constraint_num_);
  this->world_data->set_body_shapes(body_range, this->shapes, body_shapes);
  this->world_data->create_constraints(
      constraint_range, constraint_types, constraint_bodies1, constraint_bodies2);
}

void PhysicsGeometryImpl::destroy_world()
{
  if (this->world_data == nullptr) {
    return;
  }
  std::scoped_lock lock(this->data_mutex);
  if (this->world_data == nullptr) {
    return;
  }
  delete this->world_data;
  this->world_data = nullptr;
}

void PhysicsGeometryImpl::set_overlap_filter(OverlapFilterFn fn)
{
  if (this->world_data == nullptr) {
    return;
  }
  std::scoped_lock lock(this->data_mutex);
  if (this->world_data == nullptr) {
    return;
  }

  world_data->set_overlap_filter(fn);
}

void PhysicsGeometryImpl::clear_overlap_filter()
{
  if (this->world_data == nullptr) {
    return;
  }
  std::scoped_lock lock(this->data_mutex);
  if (this->world_data == nullptr) {
    return;
  }

  world_data->clear_overlap_filter();
}

float3 PhysicsGeometryImpl::gravity() const
{
  /* TODO add output caches for single values too, to avoid locking. */
  if (this->world_data == nullptr) {
    return float3(0.0f);
  }
  std::scoped_lock lock(this->data_mutex);
  if (this->world_data == nullptr) {
    return float3(0.0f);
  }

  return world_data->gravity();
}

void PhysicsGeometryImpl::set_gravity(const float3 &gravity)
{
  if (this->world_data == nullptr) {
    return;
  }
  std::scoped_lock lock(this->data_mutex);
  if (this->world_data == nullptr) {
    return;
  }

  world_data->set_gravity(gravity);
}

void PhysicsGeometryImpl::set_solver_iterations(const int num_solver_iterations)
{
  if (this->world_data == nullptr) {
    return;
  }
  std::scoped_lock lock(this->data_mutex);
  if (this->world_data == nullptr) {
    return;
  }

  world_data->set_solver_iterations(num_solver_iterations);
}

void PhysicsGeometryImpl::set_split_impulse(const bool split_impulse)
{
  if (this->world_data == nullptr) {
    return;
  }
  std::scoped_lock lock(this->data_mutex);
  if (this->world_data == nullptr) {
    return;
  }

  world_data->set_split_impulse(split_impulse);
}

void PhysicsGeometryImpl::step_simulation(float delta_time)
{
  if (this->world_data == nullptr) {
    return;
  }
  std::scoped_lock lock(this->data_mutex);
  if (this->world_data == nullptr) {
    return;
  }

  this->ensure_motion_type_no_lock();
  this->ensure_constraints_no_lock();
  world_data->step_simulation(delta_time);

  this->tag_read_cache_changed();
}

void PhysicsGeometryImpl::remove_attributes_from_customdata()
{
  /* Force use of cache for writing. */
  MutableAttributeAccessor attributes = this->custom_data_attributes_for_write();
  attributes.for_all(
      [&](const AttributeIDRef &attribute_id, const AttributeMetaData &meta_data) -> bool {
        CustomData *custom_data = nullptr;
        int totelem = 0;
        switch (meta_data.domain) {
          case AttrDomain::Point:
            custom_data = &body_data_;
            totelem = body_num_;
            break;
          case AttrDomain::Edge:
            custom_data = &constraint_data_;
            totelem = constraint_num_;
            break;
          case AttrDomain::Instance:
            break;
          default:
            BLI_assert_unreachable();
            break;
        }
        if (custom_data == nullptr) {
          return true;
        }
        CustomData_free_layer_named(custom_data, attribute_id.name(), totelem);
        return true;
      });
}

static void remap_bodies(const int src_bodies_num,
                         const IndexMask &bodies_mask,
                         const IndexMask &constraints_mask,
                         const Span<int> src_constraint_types,
                         const Span<int> src_constraint_body1,
                         const Span<int> src_constraint_body2,
                         MutableSpan<int> dst_constraint_types,
                         MutableSpan<int> dst_constraint_body1,
                         MutableSpan<int> dst_constraint_body2)
{
  Array<int> map(src_bodies_num);
  const IndexRange body_range = map.index_range();
  index_mask::build_reverse_map<int>(bodies_mask, map);
  constraints_mask.foreach_index(GrainSize(512), [&](const int64_t src_i, const int64_t dst_i) {
    dst_constraint_types[dst_i] = src_constraint_types[src_i];

    const int body1 = src_constraint_body1[src_i];
    const int body2 = src_constraint_body2[src_i];
    dst_constraint_body1[dst_i] = body_range.contains(body1) ? map[body1] : -1;
    dst_constraint_body2[dst_i] = body_range.contains(body2) ? map[body2] : -1;
  });
}

void PhysicsGeometryImpl::move_or_copy_selection(
    const PhysicsGeometryImpl &src,
    const IndexMask &src_body_mask,
    const IndexMask &src_constraint_mask,
    const bke::AnonymousAttributePropagationInfo &propagation_info)
{
  this->try_move_data(src, body_num_, constraint_num_, src_body_mask, src_constraint_mask, 0, 0);

  const bke::AttributeAccessor src_attributes = src.attributes();
  bke::MutableAttributeAccessor dst_attributes = this->attributes_for_write();

  Set<std::string> ignored_attributes = {};
  if (this->world_data) {
    /* Don't copy builtin attributes when there is world data. */
    ignored_attributes.add_multiple(all_body_attribute_names());
    ignored_attributes.add_multiple(all_constraint_attribute_names());
  }
  else {
    /* Map index references in custom data that is no longer based on world data. */
    static const StringRef constraint_type_id = PhysicsGeometry::constraint_attribute_name(
        PhysicsGeometry::ConstraintAttribute::constraint_type);
    static const StringRef constraint_body1_id = PhysicsGeometry::constraint_attribute_name(
        PhysicsGeometry::ConstraintAttribute::constraint_body1);
    static const StringRef constraint_body2_id = PhysicsGeometry::constraint_attribute_name(
        PhysicsGeometry::ConstraintAttribute::constraint_body2);

    ignored_attributes.add_multiple(
        {constraint_type_id, constraint_body1_id, constraint_body2_id});

    const VArraySpan<int> src_types = *src_attributes.lookup<int>(constraint_type_id);
    const VArraySpan<int> src_body1 = *src_attributes.lookup<int>(constraint_body1_id);
    const VArraySpan<int> src_body2 = *src_attributes.lookup<int>(constraint_body2_id);

    SpanAttributeWriter<int> dst_types = dst_attributes.lookup_for_write_span<int>(
        constraint_type_id);
    SpanAttributeWriter<int> dst_body1 = dst_attributes.lookup_for_write_span<int>(
        constraint_body1_id);
    SpanAttributeWriter<int> dst_body2 = dst_attributes.lookup_for_write_span<int>(
        constraint_body2_id);

    remap_bodies(src.body_num_,
                 src_body_mask,
                 src_constraint_mask,
                 src_types,
                 src_body1,
                 src_body2,
                 dst_types.span,
                 dst_body1.span,
                 dst_body2.span);
  }

  bke::gather_attributes(src_attributes,
                         bke::AttrDomain::Point,
                         propagation_info,
                         ignored_attributes,
                         src_body_mask,
                         dst_attributes);
  bke::gather_attributes(src_attributes,
                         bke::AttrDomain::Edge,
                         propagation_info,
                         ignored_attributes,
                         src_constraint_mask,
                         dst_attributes);
}

bool PhysicsGeometryImpl::try_move_data(const PhysicsGeometryImpl &src,
                                        const int body_num,
                                        const int constraint_num,
                                        const IndexMask &src_body_mask,
                                        const IndexMask &src_constraint_mask,
                                        int dst_body_offset,
                                        int dst_constraint_offset)
{
  BLI_assert(this->is_mutable());

  if (src.world_data == nullptr) {
    return false;
  }
  std::scoped_lock src_lock(src.data_mutex);
  if (src.world_data == nullptr) {
    return false;
  }

  std::scoped_lock dst_lock(this->data_mutex);
  if (this->world_data) {
    delete this->world_data;
    this->world_data = nullptr;
  }

  this->world_data = src.world_data;
  src.world_data = nullptr;

  this->world_data->resize(body_num,
                           constraint_num,
                           src_body_mask,
                           src_constraint_mask,
                           dst_body_offset,
                           dst_constraint_offset);

  return true;
}

bool PhysicsGeometryImpl::validate_world_data()
{
  bool ok = true;

  if (world_data == nullptr) {
    return ok;
  }

  this->ensure_motion_type();
  this->ensure_constraints();
  this->world_data->ensure_body_and_constraint_indices();
  this->world_data->ensure_bodies_and_constraints_in_world();

  AttributeAccessor cached_attributes = custom_data_attributes();
  const Span<btRigidBody *> rigid_bodies = this->world_data->bodies();
  const Span<btTypedConstraint *> constraints = this->world_data->constraints();
  const btCollisionObjectArray &bt_collision_objects =
      this->world_data->world().getCollisionObjectArray();

  for (const int i : rigid_bodies.index_range()) {
    const btRigidBody *body = rigid_bodies[i];

    /* Bodies should always be allocated. */
    if (body == nullptr) {
      BLI_assert_unreachable();
      ok = false;
    }
    if (get_body_index(*body) != i) {
      BLI_assert_unreachable();
      ok = false;
    }

    /* All bodies must be in the world, except if they don't have a collision shape. */
    if (body->getCollisionShape() != nullptr &&
        (!body->isInWorld() || bt_collision_objects.findLinearSearch(const_cast<btRigidBody *>(
                                   body)) >= bt_collision_objects.size()))
    {
      BLI_assert_unreachable();
      ok = false;
    }

    /* Bodies with a non-moving collision shape must be static and zero-mass. */
    if (body->getCollisionShape() != nullptr && body->getCollisionShape()->isNonMoving()) {
      if (!body->isStaticObject() || body->getMass() != 0.0f) {
        BLI_assert_unreachable();
        ok = false;
      }
    }
    /* Static bodies must have zero mass. */
    if (body->isStaticObject()) {
      if (body->getMass() != 0.0f) {
        BLI_assert_unreachable();
        ok = false;
      }
    }
    /* Zero mass bodies must be static. */
    if (body->getMass() == 0.0f) {
      if (!body->isStaticObject()) {
        BLI_assert_unreachable();
        ok = false;
      }
    }
  }

  const VArray<int> cached_constraint_types = *cached_attributes.lookup<int>(
      physics_attribute_name(ConstraintAttribute::constraint_type), AttrDomain::Edge);
  const VArray<int> cached_constraint_body1 = *cached_attributes.lookup<int>(
      physics_attribute_name(ConstraintAttribute::constraint_body1), AttrDomain::Edge);
  const VArray<int> cached_constraint_body2 = *cached_attributes.lookup<int>(
      physics_attribute_name(ConstraintAttribute::constraint_body2), AttrDomain::Edge);
  for (const int i : constraints.index_range()) {
    const btTypedConstraint *constraint = constraints[i];
    if (constraint == nullptr) {
      BLI_assert_unreachable();
      ok = false;
    }
    /* Constraint class should match the type enum. */
    if (!validate_bullet_constraint_type(
            PhysicsGeometry::ConstraintType(cached_constraint_types[i]), constraint))
    {
      BLI_assert_unreachable();
      ok = false;
    }
    if (get_constraint_index(*constraint) != i) {
      BLI_assert_unreachable();
      ok = false;
    }

    const int cached_body1 = cached_constraint_body1[i];
    const int cached_body2 = cached_constraint_body2[i];
    const btRigidBody *bt_body1_expected = (rigid_bodies.index_range().contains(cached_body1) ?
                                                rigid_bodies[cached_body1] :
                                                &btTypedConstraint::getFixedBody());
    const btRigidBody *bt_body2_expected = (rigid_bodies.index_range().contains(cached_body2) ?
                                                rigid_bodies[cached_body2] :
                                                &btTypedConstraint::getFixedBody());
    if (&constraint->getRigidBodyA() != bt_body1_expected) {
      BLI_assert_unreachable();
      ok = false;
    }
    if (&constraint->getRigidBodyB() != bt_body2_expected) {
      BLI_assert_unreachable();
      ok = false;
    }

    /* Bullet does not keep track of which constraints have been added to the world. This flag is
     * set by us to know when to add or remove a constraint. */
    if (!is_constraint_in_world(*constraint)) {
      BLI_assert_unreachable();
      ok = false;
    }
    /* Constraints with body1 == body2 are not actually added to the world (Bullet crashes
     * otherwise). */
    if (&constraint->getRigidBodyA() != &constraint->getRigidBodyB()) {
      bool world_has_constraint_ref = false;
      for (const int i : IndexRange(this->world_data->world().getNumConstraints())) {
        if (this->world_data->world().getConstraint(i) == constraint) {
          world_has_constraint_ref = true;
        }
      }
      if (!world_has_constraint_ref) {
        BLI_assert_unreachable();
        ok = false;
      }
    }
  }

  const VArray<int> cached_body_shapes = *cached_attributes.lookup<int>(
      physics_attribute_name(BodyAttribute::collision_shape), AttrDomain::Point);
  const IndexRange shape_range = this->shapes.index_range();
  if (rigid_bodies.size() != cached_body_shapes.size()) {
    BLI_assert_unreachable();
    ok = false;
  }
  for (const int i : cached_body_shapes.index_range()) {
    /* Internal shape pointers must match the body shape index attribute. */
    const int shape_index = cached_body_shapes[i];
    if (shape_range.contains(shape_index)) {
      /* Shape pointer must match indicated shape. */
      const CollisionShapePtr shape_ptr = this->shapes[shape_index];
      const btCollisionShape *bt_indicated_shape = shape_ptr ?
                                                       &shape_ptr->impl().as_bullet_shape() :
                                                       nullptr;
      const btCollisionShape *bt_shape = rigid_bodies[i]->getCollisionShape();
      if (bt_shape != bt_indicated_shape) {
        BLI_assert_unreachable();
        ok = false;
      }
    }
    else {
      const btCollisionShape *bt_shape = rigid_bodies[i]->getCollisionShape();
      if (bt_shape != nullptr) {
        BLI_assert_unreachable();
        ok = false;
      }
    }
  }

  return ok;
}

AttributeAccessor PhysicsGeometryImpl::attributes() const
{
  return AttributeAccessor(this, bke::get_physics_accessor_functions_ref());
}

MutableAttributeAccessor PhysicsGeometryImpl::attributes_for_write()
{
  return MutableAttributeAccessor(this, bke::get_physics_accessor_functions_ref());
}

AttributeAccessor PhysicsGeometryImpl::custom_data_attributes() const
{
  return AttributeAccessor(this, bke::get_physics_custom_data_accessor_functions_ref());
}

MutableAttributeAccessor PhysicsGeometryImpl::custom_data_attributes_for_write()
{
  return MutableAttributeAccessor(this, bke::get_physics_custom_data_accessor_functions_ref());
}

AttributeAccessor PhysicsGeometryImpl::world_data_attributes() const
{
  return AttributeAccessor(this, bke::get_physics_world_data_accessor_functions_ref());
}

MutableAttributeAccessor PhysicsGeometryImpl::world_data_attributes_for_write()
{
  return MutableAttributeAccessor(this, bke::get_physics_world_data_accessor_functions_ref());
}

PhysicsGeometry::PhysicsGeometry()
{
  impl_ = new PhysicsGeometryImpl();
}

PhysicsGeometry::PhysicsGeometry(int bodies_num, int constraints_num, int shapes_num)
{
  impl_ = new PhysicsGeometryImpl(bodies_num, constraints_num, shapes_num);
  this->tag_topology_changed();
}

PhysicsGeometry::PhysicsGeometry(const PhysicsGeometry &other)
{
  impl_ = other.impl_;
  impl_->add_user();
}

PhysicsGeometry::~PhysicsGeometry()
{
  BLI_assert(impl_ && impl_->strong_users() > 0);
  impl_->remove_user_and_delete_if_last();
}

const PhysicsGeometryImpl &PhysicsGeometry::impl() const
{
  return *impl_;
}

PhysicsGeometryImpl &PhysicsGeometry::impl_for_write()
{
  if (impl_->is_mutable()) {
    return *const_cast<PhysicsGeometryImpl *>(impl_);
  }

  PhysicsGeometryImpl *new_impl = new PhysicsGeometryImpl(
      impl_->body_num_, impl_->constraint_num_, impl_->shapes.size());

  new_impl->try_move_data(*impl_,
                          impl_->body_num_,
                          impl_->constraint_num_,
                          IndexRange(impl_->body_num_),
                          IndexRange(impl_->constraint_num_),
                          0,
                          0);

  new_impl->custom_data_read_cache_valid.store(impl_->custom_data_read_cache_valid,
                                               std::memory_order_relaxed);
  new_impl->body_collision_shapes_valid.store(impl_->body_collision_shapes_valid,
                                              std::memory_order_relaxed);
  new_impl->constraint_disable_collision_valid.store(impl_->constraint_disable_collision_valid,
                                                     std::memory_order_relaxed);

  CustomData_copy(&impl_->body_data_, &new_impl->body_data_, CD_MASK_ALL, impl_->body_num_);
  CustomData_copy(
      &impl_->constraint_data_, &new_impl->constraint_data_, CD_MASK_ALL, impl_->constraint_num_);
  new_impl->shapes = impl_->shapes;

  impl_->remove_user_and_delete_if_last();
  impl_ = new_impl;

  return *const_cast<PhysicsGeometryImpl *>(impl_);
}

bool PhysicsGeometry::has_world() const
{
  return this->impl().world_data != nullptr;
}

void PhysicsGeometry::create_world()
{
  PhysicsGeometryImpl &impl = impl_for_write();
  impl.create_world();
}

void PhysicsGeometry::destroy_world()
{
  PhysicsGeometryImpl &impl = impl_for_write();
  impl.ensure_read_cache();
  impl.destroy_world();
}

void PhysicsGeometry::set_overlap_filter(OverlapFilterFn fn)
{
  this->impl_for_write().set_overlap_filter(fn);
}

void PhysicsGeometry::clear_overlap_filter()
{
  this->impl_for_write().clear_overlap_filter();
}

float3 PhysicsGeometry::gravity() const
{
  return this->impl().gravity();
}

void PhysicsGeometry::set_gravity(const float3 &gravity)
{
  this->impl_for_write().set_gravity(gravity);
}

void PhysicsGeometry::set_solver_iterations(const int num_solver_iterations)
{
  this->impl_for_write().set_solver_iterations(num_solver_iterations);
}

void PhysicsGeometry::set_split_impulse(const bool split_impulse)
{
  this->impl_for_write().set_split_impulse(split_impulse);
}

void PhysicsGeometry::step_simulation(float delta_time)
{
  this->impl_for_write().step_simulation(delta_time);
}

void PhysicsGeometry::ensure_read_cache() const
{
  this->impl().ensure_read_cache();
}

void PhysicsGeometry::ensure_custom_data_attribute(BodyAttribute attribute) const
{
  this->impl().ensure_custom_data_attribute(attribute);
}

void PhysicsGeometry::ensure_custom_data_attribute(ConstraintAttribute attribute) const
{
  this->impl().ensure_custom_data_attribute(attribute);
}

int PhysicsGeometry::bodies_num() const
{
  return impl_->body_num_;
}

int PhysicsGeometry::constraints_num() const
{
  return impl_->constraint_num_;
}

int PhysicsGeometry::shapes_num() const
{
  return impl_->shapes.size();
}

IndexRange PhysicsGeometry::bodies_range() const
{
  return IndexRange(impl_->body_num_);
}

IndexRange PhysicsGeometry::constraints_range() const
{
  return IndexRange(impl_->constraint_num_);
}

IndexRange PhysicsGeometry::shapes_range() const
{
  return impl_->shapes.index_range();
}

bool PhysicsGeometry::try_move_data(const PhysicsGeometry &src,
                                    const int body_num,
                                    const int constraint_num,
                                    const IndexMask &src_body_mask,
                                    const IndexMask &src_constraint_mask,
                                    const int dst_body_offset,
                                    const int dst_constraint_offset)
{
  return this->impl_for_write().try_move_data(src.impl(),
                                              body_num,
                                              constraint_num,
                                              src_body_mask,
                                              src_constraint_mask,
                                              dst_body_offset,
                                              dst_constraint_offset);
}

void PhysicsGeometry::move_or_copy_selection(
    const PhysicsGeometry &from,
    const IndexMask &body_mask,
    const IndexMask &constraint_mask,
    const bke::AnonymousAttributePropagationInfo &propagation_info)
{
  this->impl_for_write().move_or_copy_selection(
      from.impl(), body_mask, constraint_mask, propagation_info);
}

void PhysicsGeometry::tag_collision_shapes_changed() {}

void PhysicsGeometry::tag_body_transforms_changed() {}

void PhysicsGeometry::tag_topology_changed()
{
  this->impl_for_write().tag_body_topology_changed();
}

void PhysicsGeometry::tag_physics_changed()
{
  this->tag_topology_changed();
}

Span<CollisionShape::Ptr> PhysicsGeometry::shapes() const
{
  return impl_->shapes;
}

MutableSpan<CollisionShapePtr> PhysicsGeometry::shapes_for_write()
{
  return impl_for_write().shapes;
}

VArray<int> PhysicsGeometry::body_ids() const
{
  return attributes().lookup(body_attribute_name(BodyAttribute::id)).varray.typed<int>();
}

AttributeWriter<int> PhysicsGeometry::body_ids_for_write()
{
  return attributes_for_write().lookup_for_write<int>(body_attribute_name(BodyAttribute::id));
}

VArray<int> PhysicsGeometry::body_shapes() const
{
  return *attributes().lookup_or_default<int>(
      body_attribute_name(BodyAttribute::collision_shape), AttrDomain::Point, -1);
}

AttributeWriter<int> PhysicsGeometry::body_shapes_for_write()
{
  const GVArray init_varray = VArray<int>::ForSingle(-1, this->bodies_num());
  return attributes_for_write().lookup_or_add_for_write<int>(
      body_attribute_name(BodyAttribute::collision_shape),
      AttrDomain::Point,
      AttributeInitVArray(init_varray));
}

VArray<bool> PhysicsGeometry::body_is_static() const
{
  return attributes().lookup(body_attribute_name(BodyAttribute::is_static)).varray.typed<bool>();
}

AttributeWriter<bool> PhysicsGeometry::body_is_static_for_write()
{
  return attributes_for_write().lookup_or_add_for_write<bool>(
      body_attribute_name(BodyAttribute::is_static), AttrDomain::Point);
}

VArray<bool> PhysicsGeometry::body_is_kinematic() const
{
  return attributes()
      .lookup(body_attribute_name(BodyAttribute::is_kinematic))
      .varray.typed<bool>();
}

AttributeWriter<bool> PhysicsGeometry::body_is_kinematic_for_write()
{
  return attributes_for_write().lookup_for_write<bool>(
      body_attribute_name(BodyAttribute::is_kinematic));
}

VArray<float> PhysicsGeometry::body_masses() const
{
  return attributes().lookup(body_attribute_name(BodyAttribute::mass)).varray.typed<float>();
}

AttributeWriter<float> PhysicsGeometry::body_masses_for_write()
{
  return attributes_for_write().lookup_or_add_for_write<float>(
      body_attribute_name(BodyAttribute::mass), AttrDomain::Point);
}

VArray<float3> PhysicsGeometry::body_inertias() const
{
  return attributes().lookup(body_attribute_name(BodyAttribute::inertia)).varray.typed<float3>();
}

AttributeWriter<float3> PhysicsGeometry::body_inertias_for_write()
{
  return attributes_for_write().lookup_for_write<float3>(
      body_attribute_name(BodyAttribute::inertia));
}

VArray<float3> PhysicsGeometry::body_positions() const
{
  return attributes().lookup(body_attribute_name(BodyAttribute::position)).varray.typed<float3>();
}

AttributeWriter<float3> PhysicsGeometry::body_positions_for_write()
{
  return attributes_for_write().lookup_for_write<float3>(
      body_attribute_name(BodyAttribute::position));
}

VArray<math::Quaternion> PhysicsGeometry::body_rotations() const
{
  return attributes()
      .lookup(body_attribute_name(BodyAttribute::rotation))
      .varray.typed<math::Quaternion>();
}

AttributeWriter<math::Quaternion> PhysicsGeometry::body_rotations_for_write()
{
  return attributes_for_write().lookup_for_write<math::Quaternion>(
      body_attribute_name(BodyAttribute::rotation));
}

VArray<float3> PhysicsGeometry::body_velocities() const
{
  return attributes().lookup(body_attribute_name(BodyAttribute::velocity)).varray.typed<float3>();
}

AttributeWriter<float3> PhysicsGeometry::body_velocities_for_write()
{
  return attributes_for_write().lookup_for_write<float3>(
      body_attribute_name(BodyAttribute::velocity));
}

VArray<float3> PhysicsGeometry::body_angular_velocities() const
{
  return attributes()
      .lookup(body_attribute_name(BodyAttribute::angular_velocity))
      .varray.typed<float3>();
}

AttributeWriter<float3> PhysicsGeometry::body_angular_velocities_for_write()
{
  return attributes_for_write().lookup_for_write<float3>(
      body_attribute_name(BodyAttribute::angular_velocity));
}

VArray<int> PhysicsGeometry::body_activation_states() const
{
  return attributes().lookup<int>(body_attribute_name(BodyAttribute::activation_state)).varray;
}

AttributeWriter<int> PhysicsGeometry::body_activation_states_for_write()
{
  return attributes_for_write().lookup_for_write<int>(
      body_attribute_name(BodyAttribute::activation_state));
}

VArray<float3> PhysicsGeometry::body_total_force() const
{
  return attributes().lookup<float3>(body_attribute_name(BodyAttribute::total_force)).varray;
}

VArray<float3> PhysicsGeometry::body_total_torque() const
{
  return attributes().lookup<float3>(body_attribute_name(BodyAttribute::total_torque)).varray;
}

void PhysicsGeometry::apply_force(const IndexMask &selection,
                                  const VArray<float3> &forces,
                                  const VArray<float3> &relative_positions)
{
  this->impl_for_write().apply_force(selection, forces, relative_positions);
}

void PhysicsGeometry::apply_torque(const IndexMask &selection, const VArray<float3> &torques)
{
  this->impl_for_write().apply_torque(selection, torques);
}

void PhysicsGeometry::apply_impulse(const IndexMask &selection,
                                    const VArray<float3> &impulses,
                                    const VArray<float3> &relative_positions)
{
  this->impl_for_write().apply_impulse(selection, impulses, relative_positions);
}

void PhysicsGeometry::apply_angular_impulse(const IndexMask &selection,
                                            const VArray<float3> &angular_impulses)
{
  this->impl_for_write().apply_angular_impulse(selection, angular_impulses);
}

void PhysicsGeometry::clear_forces(const IndexMask &selection)
{
  this->impl_for_write().clear_forces(selection);
}

void PhysicsGeometry::compute_local_inertia(const IndexMask &selection)
{
  this->impl_for_write().compute_local_inertia(selection);
}

// void PhysicsGeometry::create_constraints(const IndexMask &selection,
//                                          const VArray<int> &types,
//                                          const VArray<int> &bodies1,
//                                          const VArray<int> &bodies2)
//{
//   this->impl_for_write().create_constraints(selection, types, bodies1, bodies2);
// }

VArray<bool> PhysicsGeometry::constraint_enabled() const
{
  return attributes()
      .lookup<bool>(constraint_attribute_name(ConstraintAttribute::constraint_enabled))
      .varray;
}

AttributeWriter<bool> PhysicsGeometry::constraint_enabled_for_write()
{
  return attributes_for_write().lookup_for_write<bool>(
      constraint_attribute_name(ConstraintAttribute::constraint_enabled));
}

VArray<int> PhysicsGeometry::constraint_types() const
{
  return attributes()
      .lookup<int>(constraint_attribute_name(ConstraintAttribute::constraint_type))
      .varray;
}

AttributeWriter<int> PhysicsGeometry::constraint_types_for_write()
{
  const VArray<int> default_varray = VArray<int>::ForSingle(
      int(PhysicsGeometry::ConstraintType::Fixed), this->constraints_num());
  return attributes_for_write().lookup_or_add_for_write<int>(
      constraint_attribute_name(ConstraintAttribute::constraint_type),
      AttrDomain::Edge,
      AttributeInitVArray(default_varray));
}

VArray<int> PhysicsGeometry::constraint_body1() const
{
  return attributes()
      .lookup<int>(constraint_attribute_name(ConstraintAttribute::constraint_body1))
      .varray;
}

AttributeWriter<int> PhysicsGeometry::constraint_body1_for_write()
{
  const VArray<int> default_varray = VArray<int>::ForSingle(-1, this->constraints_num());
  return attributes_for_write().lookup_or_add_for_write<int>(
      constraint_attribute_name(ConstraintAttribute::constraint_body1),
      AttrDomain::Edge,
      AttributeInitVArray(default_varray));
}

VArray<int> PhysicsGeometry::constraint_body2() const
{
  return attributes()
      .lookup<int>(constraint_attribute_name(ConstraintAttribute::constraint_body2))
      .varray;
}

AttributeWriter<int> PhysicsGeometry::constraint_body2_for_write()
{
  const VArray<int> default_varray = VArray<int>::ForSingle(-1, this->constraints_num());
  return attributes_for_write().lookup_or_add_for_write<int>(
      constraint_attribute_name(ConstraintAttribute::constraint_body2),
      AttrDomain::Edge,
      AttributeInitVArray(default_varray));
}

VArray<float4x4> PhysicsGeometry::constraint_frame1() const
{
  return attributes()
      .lookup<float4x4>(constraint_attribute_name(ConstraintAttribute::constraint_frame1))
      .varray;
}

AttributeWriter<float4x4> PhysicsGeometry::constraint_frame1_for_write()
{
  return attributes_for_write().lookup_for_write<float4x4>(
      constraint_attribute_name(ConstraintAttribute::constraint_frame1));
}

VArray<float4x4> PhysicsGeometry::constraint_frame2() const
{
  return attributes()
      .lookup<float4x4>(constraint_attribute_name(ConstraintAttribute::constraint_frame2))
      .varray;
}

AttributeWriter<float4x4> PhysicsGeometry::constraint_frame2_for_write()
{
  return attributes_for_write().lookup_for_write<float4x4>(
      constraint_attribute_name(ConstraintAttribute::constraint_frame2));
}

VArray<float> PhysicsGeometry::constraint_applied_impulse() const
{
  return attributes()
      .lookup<float>(constraint_attribute_name(ConstraintAttribute::applied_impulse))
      .varray;
}

VArray<float> PhysicsGeometry::constraint_breaking_impulse_threshold_impulse() const
{
  return attributes()
      .lookup<float>(constraint_attribute_name(ConstraintAttribute::breaking_impulse_threshold))
      .varray;
}

AttributeWriter<float> PhysicsGeometry::constraint_breaking_impulse_threshold_for_write()
{
  return attributes_for_write().lookup_for_write<float>(
      constraint_attribute_name(ConstraintAttribute::breaking_impulse_threshold));
}

VArray<bool> PhysicsGeometry::constraint_disable_collision() const
{
  return attributes()
      .lookup<bool>(constraint_attribute_name(ConstraintAttribute::disable_collision))
      .varray;
}

AttributeWriter<bool> PhysicsGeometry::constraint_disable_collision_for_write()
{
  return attributes_for_write().lookup_for_write<bool>(
      constraint_attribute_name(ConstraintAttribute::disable_collision));
}

StringRef PhysicsGeometry::body_attribute_name(BodyAttribute attribute)
{
  return bke::physics_attribute_name(attribute);
}

StringRef PhysicsGeometry::constraint_attribute_name(ConstraintAttribute attribute)
{
  return bke::physics_attribute_name(attribute);
}

bool PhysicsGeometry::validate_world_data()
{
  return impl_for_write().validate_world_data();
}

AttributeAccessor PhysicsGeometry::attributes() const
{
  return AttributeAccessor(&this->impl(), bke::get_physics_accessor_functions_ref());
}

MutableAttributeAccessor PhysicsGeometry::attributes_for_write()
{
  return MutableAttributeAccessor(&this->impl_for_write(),
                                  bke::get_physics_accessor_functions_ref());
}

AttributeAccessor PhysicsGeometry::dummy_attributes()
{
  return AttributeAccessor(nullptr, bke::get_physics_accessor_functions_ref());
}

MutableAttributeAccessor PhysicsGeometry::dummy_attributes_for_write()
{
  return MutableAttributeAccessor(nullptr, bke::get_physics_accessor_functions_ref());
}

/** \} */

float4x4 get_constraint_frame1(const btTypedConstraint &constraint)
{
  using ConstraintType = PhysicsGeometry::ConstraintType;

  switch (ConstraintType(constraint.getUserConstraintType())) {
    case ConstraintType::Fixed: {
      const auto &typed_constraint = static_cast<const btFixedConstraint &>(constraint);
      return to_blender(typed_constraint.getFrameOffsetA());
    }
    case ConstraintType::Point: {
      const auto &typed_constraint = static_cast<const btPoint2PointConstraint &>(constraint);
      return math::from_location<float4x4>(to_blender(typed_constraint.getPivotInA()));
    }
    case ConstraintType::Hinge: {
      const auto &typed_constraint = static_cast<const btHingeConstraint &>(constraint);
      return to_blender(typed_constraint.getAFrame());
    }
    case ConstraintType::Slider: {
      const auto &typed_constraint = static_cast<const btSliderConstraint &>(constraint);
      return to_blender(typed_constraint.getFrameOffsetA());
    }
    case ConstraintType::ConeTwist: {
      const auto &typed_constraint = static_cast<const btConeTwistConstraint &>(constraint);
      return to_blender(typed_constraint.getFrameOffsetA());
    }
    case ConstraintType::SixDoF: {
      const auto &typed_constraint = static_cast<const btGeneric6DofConstraint &>(constraint);
      return to_blender(typed_constraint.getFrameOffsetA());
    }
    case ConstraintType::SixDoFSpring: {
      const auto &typed_constraint = static_cast<const btGeneric6DofSpringConstraint &>(
          constraint);
      return to_blender(typed_constraint.getFrameOffsetA());
    }
    case ConstraintType::SixDoFSpring2: {
      const auto &typed_constraint = static_cast<const btGeneric6DofSpring2Constraint &>(
          constraint);
      return to_blender(typed_constraint.getFrameOffsetA());
    }
    case ConstraintType::Gear: {
      const auto &typed_constraint = static_cast<const btGearConstraint &>(constraint);
      return math::from_up_axis<float4x4>(to_blender(typed_constraint.getAxisA()));
    }
    case ConstraintType::Contact:
      return float4x4::identity();
  }
  BLI_assert_unreachable();
  return float4x4::identity();
}

void set_constraint_frame1(btTypedConstraint &constraint, float4x4 value)
{
  using ConstraintType = PhysicsGeometry::ConstraintType;

  switch (ConstraintType(constraint.getUserConstraintType())) {
    case ConstraintType::Fixed: {
      auto &typed_constraint = static_cast<btFixedConstraint &>(constraint);
      typed_constraint.setFrames(to_bullet(value), typed_constraint.getFrameOffsetB());
      break;
    }
    case ConstraintType::Point: {
      auto &typed_constraint = static_cast<btPoint2PointConstraint &>(constraint);
      typed_constraint.setPivotA(to_bullet(value.location()));
      break;
    }
    case ConstraintType::Hinge: {
      auto &typed_constraint = static_cast<btHingeConstraint &>(constraint);
      typed_constraint.setFrames(to_bullet(value), typed_constraint.getBFrame());
      break;
    }
    case ConstraintType::Slider: {
      auto &typed_constraint = static_cast<btSliderConstraint &>(constraint);
      typed_constraint.setFrames(to_bullet(value), typed_constraint.getFrameOffsetB());
      break;
    }
    case ConstraintType::ConeTwist: {
      auto &typed_constraint = static_cast<btConeTwistConstraint &>(constraint);
      typed_constraint.setFrames(to_bullet(value), typed_constraint.getFrameOffsetB());
      break;
    }
    case ConstraintType::SixDoF: {
      auto &typed_constraint = static_cast<btGeneric6DofConstraint &>(constraint);
      typed_constraint.setFrames(to_bullet(value), typed_constraint.getFrameOffsetB());
      break;
    }
    case ConstraintType::SixDoFSpring: {
      auto &typed_constraint = static_cast<btGeneric6DofSpringConstraint &>(constraint);
      typed_constraint.setFrames(to_bullet(value), typed_constraint.getFrameOffsetB());
      break;
    }
    case ConstraintType::SixDoFSpring2: {
      auto &typed_constraint = static_cast<btGeneric6DofSpring2Constraint &>(constraint);
      typed_constraint.setFrames(to_bullet(value), typed_constraint.getFrameOffsetB());
      break;
    }
    case ConstraintType::Gear: {
      auto &typed_constraint = static_cast<btGearConstraint &>(constraint);
      btVector3 bt_axis = to_bullet(value.z_axis());
      typed_constraint.setAxisA(bt_axis);
      break;
    }
    case ConstraintType::Contact:
      break;
  }
}

float4x4 get_constraint_frame2(const btTypedConstraint &constraint)
{
  using ConstraintType = PhysicsGeometry::ConstraintType;

  switch (ConstraintType(constraint.getUserConstraintType())) {
    case ConstraintType::Fixed: {
      const auto &typed_constraint = static_cast<const btFixedConstraint &>(constraint);
      return to_blender(typed_constraint.getFrameOffsetB());
    }
    case ConstraintType::Point: {
      const auto &typed_constraint = static_cast<const btPoint2PointConstraint &>(constraint);
      return math::from_location<float4x4>(to_blender(typed_constraint.getPivotInB()));
    }
    case ConstraintType::Hinge: {
      const auto &typed_constraint = static_cast<const btHingeConstraint &>(constraint);
      return to_blender(typed_constraint.getBFrame());
    }
    case ConstraintType::Slider: {
      const auto &typed_constraint = static_cast<const btSliderConstraint &>(constraint);
      return to_blender(typed_constraint.getFrameOffsetB());
    }
    case ConstraintType::ConeTwist: {
      const auto &typed_constraint = static_cast<const btConeTwistConstraint &>(constraint);
      return to_blender(typed_constraint.getFrameOffsetB());
    }
    case ConstraintType::SixDoF: {
      const auto &typed_constraint = static_cast<const btGeneric6DofConstraint &>(constraint);
      return to_blender(typed_constraint.getFrameOffsetB());
    }
    case ConstraintType::SixDoFSpring: {
      const auto &typed_constraint = static_cast<const btGeneric6DofSpringConstraint &>(
          constraint);
      return to_blender(typed_constraint.getFrameOffsetB());
    }
    case ConstraintType::SixDoFSpring2: {
      const auto &typed_constraint = static_cast<const btGeneric6DofSpring2Constraint &>(
          constraint);
      return to_blender(typed_constraint.getFrameOffsetB());
    }
    case ConstraintType::Gear: {
      const auto &typed_constraint = static_cast<const btGearConstraint &>(constraint);
      return math::from_up_axis<float4x4>(to_blender(typed_constraint.getAxisB()));
    }
    case ConstraintType::Contact:
      return float4x4::identity();
  }
  BLI_assert_unreachable();
  return float4x4::identity();
}

void set_constraint_frame2(btTypedConstraint &constraint, float4x4 value)
{
  using ConstraintType = PhysicsGeometry::ConstraintType;

  switch (ConstraintType(constraint.getUserConstraintType())) {
    case ConstraintType::Fixed: {
      auto &typed_constraint = static_cast<btFixedConstraint &>(constraint);
      typed_constraint.setFrames(typed_constraint.getFrameOffsetA(), to_bullet(value));
      break;
    }
    case ConstraintType::Point: {
      auto &typed_constraint = static_cast<btPoint2PointConstraint &>(constraint);
      typed_constraint.setPivotB(to_bullet(value.location()));
      break;
    }
    case ConstraintType::Hinge: {
      auto &typed_constraint = static_cast<btHingeConstraint &>(constraint);
      typed_constraint.setFrames(typed_constraint.getAFrame(), to_bullet(value));
      break;
    }
    case ConstraintType::Slider: {
      auto &typed_constraint = static_cast<btSliderConstraint &>(constraint);
      typed_constraint.setFrames(typed_constraint.getFrameOffsetA(), to_bullet(value));
      break;
    }
    case ConstraintType::ConeTwist: {
      auto &typed_constraint = static_cast<btConeTwistConstraint &>(constraint);
      typed_constraint.setFrames(typed_constraint.getFrameOffsetA(), to_bullet(value));
      break;
    }
    case ConstraintType::SixDoF: {
      auto &typed_constraint = static_cast<btGeneric6DofConstraint &>(constraint);
      typed_constraint.setFrames(typed_constraint.getFrameOffsetA(), to_bullet(value));
      break;
    }
    case ConstraintType::SixDoFSpring: {
      auto &typed_constraint = static_cast<btGeneric6DofSpringConstraint &>(constraint);
      typed_constraint.setFrames(typed_constraint.getFrameOffsetA(), to_bullet(value));
      break;
    }
    case ConstraintType::SixDoFSpring2: {
      auto &typed_constraint = static_cast<btGeneric6DofSpring2Constraint &>(constraint);
      typed_constraint.setFrames(typed_constraint.getFrameOffsetA(), to_bullet(value));
      break;
    }
    case ConstraintType::Gear: {
      auto &typed_constraint = static_cast<btGearConstraint &>(constraint);
      btVector3 bt_axis = to_bullet(value.z_axis());
      typed_constraint.setAxisB(bt_axis);
      break;
    }
    case ConstraintType::Contact:
      break;
  }
}

#else

// TODO add stub functions for when Bullet is disabled

#endif

}  // namespace blender::bke
