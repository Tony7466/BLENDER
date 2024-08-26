/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#include <functional>

#include "BLI_array.hh"
#include "BLI_generic_virtual_array.hh"
#include "BLI_math_matrix.hh"
#include "BLI_math_matrix_types.hh"

#include "BKE_attribute.hh"
#include "BKE_collision_shape.hh"

#include "attribute_access_intern.hh"
#include "physics_geometry_attributes.hh"
#include "physics_geometry_intern.hh"
#include "physics_geometry_world.hh"

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

static const AttributeAccessorFunctions &get_accessor_functions_ref();

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

int get_body_index(const btRigidBody &body)
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

int get_constraint_index(const btTypedConstraint &constraint)
{
  return constraint.getUserConstraintId() >> BulletConstraintFlagBits;
}

static void set_constraint_index(btTypedConstraint &constraint, const int index)
{
  constraint.setUserConstraintId((index & BulletConstraintIdMask) << BulletConstraintFlagBits);
}

bool is_constraint_in_world(const btTypedConstraint &constraint)
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

bool validate_bullet_body_shape(const btRigidBody &body, const CollisionShapePtr &collision_shape)
{
  const btCollisionShape *bt_indicated_shape = collision_shape ?
                                                   &collision_shape->impl().as_bullet_shape() :
                                                   nullptr;
  const btCollisionShape *bt_shape = body.getCollisionShape();
  return bt_shape == bt_indicated_shape;
}

static btTypedConstraint *make_bullet_constraint_type(const PhysicsConstraintType type,
                                                      btRigidBody &body1,
                                                      btRigidBody &body2)
{
  using ConstraintType = PhysicsConstraintType;

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

static btTypedConstraint *make_constraint_type(const PhysicsConstraintType type,
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

/* -------------------------------------------------------------------- */
/** \name Physics World Data
 * \{ */

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
    const VArray<int> default_types = VArray<int>::ForSingle(int(PhysicsConstraintType::Fixed),
                                                             constraint_num);
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

void PhysicsWorldData::create_constraints(const IndexMask &selection,
                                          const VArray<int> &types,
                                          const VArray<int> &bodies1,
                                          const VArray<int> &bodies2)
{
  using ConstraintType = PhysicsConstraintType;

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
#

  if (changed) {
    constraint_index_cache_.tag_dirty();
    tag_constraints_in_world();
  }
}

void PhysicsWorldData::set_disable_collision(const IndexMask &selection,
                                             const VArray<bool> &disable_collision)
{
  selection.foreach_index([&](const int index) {
    btTypedConstraint *constraint = constraints_[index];
    const bool disable = disable_collision[index];

    btRigidBody &body1 = constraint->getRigidBodyA();
    btRigidBody &body2 = constraint->getRigidBodyB();
    if (&body1 != &btTypedConstraint::getFixedBody()) {
      if (disable) {
        body1.addConstraintRef(constraint);
      }
      else {
        body1.removeConstraintRef(constraint);
      }
    }
    if (&body2 != &btTypedConstraint::getFixedBody()) {
      if (disable) {
        body2.addConstraintRef(constraint);
      }
      else {
        body2.removeConstraintRef(constraint);
      }
    }
  });
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
  bool added_or_removed_body = false;
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
      added_or_removed_body = true;
    }
    else {
      if (body->getCollisionShape() == nullptr) {
        /* Tag the cache to make sure the body is added to the world. */
        added_or_removed_body = true;
      }
      /* Motion type and mass must be compatible with the shape. */
      const bool set_static = bt_shape->isNonMoving();
      if (set_static != was_static) {
        world_->removeRigidBody(body);
        added_or_removed_body = true;
      }
      else {
        /* No need to remove the body entirely, just clean up the pair cache to force an update. */
        this->broadphase_->getOverlappingPairCache()->cleanProxyFromPairs(
            body->getBroadphaseProxy(), this->dispatcher_);
      }

      if (set_static) {
        body->setMassProps(0.0f, btVector3(0.0f, 0.0f, 0.0f));
        body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT);
      }

      body->setCollisionShape(const_cast<btCollisionShape *>(bt_shape));
    }
  });
  if (added_or_removed_body) {
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

AttributeAccessor PhysicsWorldData::attributes() const
{
  return AttributeAccessor(this, bke::get_accessor_functions_ref());
}

MutableAttributeAccessor PhysicsWorldData::attributes_for_write()
{
  return MutableAttributeAccessor(this, bke::get_accessor_functions_ref());
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Concrete Attribute Functions
 * \{ */

namespace physics_world_attribute_functions {

static void physics_attribute_finish(PhysicsWorldData & /*world_data*/,
                                     const PhysicsBodyAttribute attribute)
{
  using BodyAttribute = PhysicsBodyAttribute;

  switch (attribute) {
    case BodyAttribute::id:
    case BodyAttribute::is_static:
    case BodyAttribute::is_kinematic:
    case BodyAttribute::collision_shape:
    case BodyAttribute::mass:
    case BodyAttribute::inertia:
    case BodyAttribute::position:
    case BodyAttribute::rotation:
    case BodyAttribute::velocity:
    case BodyAttribute::angular_velocity:
    case BodyAttribute::activation_state:
    case BodyAttribute::friction:
    case BodyAttribute::rolling_friction:
    case BodyAttribute::spinning_friction:
    case BodyAttribute::restitution:
    case BodyAttribute::linear_damping:
    case BodyAttribute::angular_damping:
    case BodyAttribute::linear_sleeping_threshold:
    case BodyAttribute::angular_sleeping_threshold:
    case BodyAttribute::total_force:
    case BodyAttribute::total_torque:
      break;
  }
}

static void physics_attribute_finish(PhysicsWorldData & /*world_data*/,
                                     const PhysicsWorldData::ConstraintAttribute attribute)
{
  using ConstraintAttribute = PhysicsWorldData::ConstraintAttribute;

  switch (attribute) {
    case ConstraintAttribute::constraint_type:
    case ConstraintAttribute::constraint_body1:
    case ConstraintAttribute::constraint_body2:
    case ConstraintAttribute::constraint_frame1:
    case ConstraintAttribute::constraint_frame2:
    case ConstraintAttribute::constraint_enabled:
    case ConstraintAttribute::disable_collision:
    case ConstraintAttribute::breaking_impulse_threshold:
    case ConstraintAttribute::applied_impulse:
    case ConstraintAttribute::applied_force1:
    case ConstraintAttribute::applied_force2:
    case ConstraintAttribute::applied_torque1:
    case ConstraintAttribute::applied_torque2:
      break;
  }
}

static int id_get_fn(const btRigidBody &body)
{
  return body.getUserIndex();
}
static void id_set_fn(btRigidBody &body, int value)
{
  body.setUserIndex(value);
}
static bool static_get_fn(const btRigidBody &body)
{
  return body.isStaticObject();
}
static void static_set_fn(btRigidBody &body, bool value)
{
  int bt_collision_flags = body.getCollisionFlags();
  SET_FLAG_FROM_TEST(bt_collision_flags, value, btCollisionObject::CF_STATIC_OBJECT);
  body.setCollisionFlags(bt_collision_flags);
}
static bool kinematic_get_fn(const btRigidBody &body)
{
  return body.isKinematicObject();
}
static void kinematic_set_fn(btRigidBody &body, bool value)
{
  int bt_collision_flags = body.getCollisionFlags();
  SET_FLAG_FROM_TEST(bt_collision_flags, value, btCollisionObject::CF_KINEMATIC_OBJECT);
  body.setCollisionFlags(bt_collision_flags);
}
static float mass_get_fn(const btRigidBody &body)
{
  return body.getMass();
}
static void mass_set_fn(btRigidBody &body, float value)
{
  const bool is_moveable_shape = !body.getCollisionShape() ||
                                 !body.getCollisionShape()->isNonMoving();
  if (is_moveable_shape) {
    body.setMassProps(value, body.getLocalInertia());
    body.updateInertiaTensor();
  }
}
static float3 inertia_get_fn(const btRigidBody &body)
{
  return to_blender(body.getLocalInertia());
}
static void inertia_set_fn(btRigidBody &body, float3 value)
{
  const bool is_moveable_shape = !body.getCollisionShape() ||
                                 !body.getCollisionShape()->isNonMoving();
  if (is_moveable_shape) {
    if (math::is_zero(value) && body.getCollisionShape()) {
      btVector3 bt_inertia;
      body.getCollisionShape()->calculateLocalInertia(body.getMass(), bt_inertia);
      body.setMassProps(body.getMass(), bt_inertia);
    }
    else {
      body.setMassProps(body.getMass(), to_bullet(value));
    }
    body.updateInertiaTensor();
  }
}
static float3 position_get_fn(const btRigidBody &body)
{
  return to_blender(body.getWorldTransform().getOrigin());
}
static void position_set_fn(btRigidBody &body, float3 value)
{
  body.getWorldTransform().setOrigin(to_bullet(value));
}
static math::Quaternion rotation_get_fn(const btRigidBody &body)
{
  return to_blender(body.getWorldTransform().getRotation());
}
static void rotation_set_fn(btRigidBody &body, math::Quaternion value)
{
  body.getWorldTransform().setRotation(to_bullet(value));
}
static float3 velocity_get_fn(const btRigidBody &body)
{
  return to_blender(body.getLinearVelocity());
}
static void velocity_set_fn(btRigidBody &body, float3 value)
{
  body.setLinearVelocity(to_bullet(value));
}
static float3 angular_velocity_get_fn(const btRigidBody &body)
{
  return to_blender(body.getAngularVelocity());
}
static void angular_velocity_set_fn(btRigidBody &body, float3 value)
{
  body.setAngularVelocity(to_bullet(value));
}
static int activation_state_get_fn(const btRigidBody &body)
{
  return int(activation_state_to_blender(body.getActivationState()));
}
static void activation_state_set_fn(btRigidBody &body, int value)
{
  /* Note: there is also setActivationState, but that only sets if the state is not
   * always-active or always-sleeping. This check can be performed on the caller side if the
   * "always-x" state must be retained. */
  body.forceActivationState(
      activation_state_to_bullet(bke::PhysicsWorldData::BodyActivationState(value)));
}
static float friction_get_fn(const btRigidBody &body)
{
  return body.getFriction();
}
static void friction_set_fn(btRigidBody &body, float value)
{
  body.setFriction(value);
}
static float rolling_friction_get_fn(const btRigidBody &body)
{
  return body.getRollingFriction();
}
static void rolling_friction_set_fn(btRigidBody &body, float value)
{
  body.setRollingFriction(value);
}
static float spinning_friction_get_fn(const btRigidBody &body)
{
  return body.getSpinningFriction();
}
static void spinning_friction_set_fn(btRigidBody &body, float value)
{
  body.setSpinningFriction(value);
}
static float restitution_get_fn(const btRigidBody &body)
{
  return body.getRestitution();
}
static void restitution_set_fn(btRigidBody &body, float value)
{
  body.setRestitution(value);
}
static float linear_damping_get_fn(const btRigidBody &body)
{
  return body.getLinearSleepingThreshold();
}
static void linear_damping_set_fn(btRigidBody &body, float value)
{
  body.setSleepingThresholds(value, body.getAngularSleepingThreshold());
}
static float angular_damping_get_fn(const btRigidBody &body)
{
  return body.getAngularSleepingThreshold();
}
static void angular_damping_set_fn(btRigidBody &body, float value)
{
  body.setSleepingThresholds(value, body.getAngularSleepingThreshold());
}
static float linear_sleeping_threshold_get_fn(const btRigidBody &body)
{
  return body.getLinearSleepingThreshold();
}
static void linear_sleeping_threshold_set_fn(btRigidBody &body, float value)
{
  body.setSleepingThresholds(value, body.getAngularSleepingThreshold());
}

static float angular_sleeping_threshold_get_fn(const btRigidBody &body)
{
  return body.getAngularSleepingThreshold();
}
static void angular_sleeping_threshold_set_fn(btRigidBody &body, float value)
{
  body.setSleepingThresholds(body.getLinearSleepingThreshold(), value);
}

static float3 total_force_get_fn(const btRigidBody &body)
{
  return to_blender(body.getTotalForce());
}
static void total_force_set_fn(btRigidBody & /*body*/, float3 /*value*/) {}

static float3 total_torque_get_fn(const btRigidBody &body)
{
  return to_blender(body.getTotalTorque());
}
static void total_torque_set_fn(btRigidBody & /*body*/, float3 /*value*/) {}

// static int constraint_type_get_fn(const btTypedConstraint &constraint)
//{
//   return int(bke::PhysicsConstraintType(constraint.getUserConstraintType()));
// }
// static void constraint_type_set_fn(btTypedConstraint & /*constraint*/, int /*value*/) {}

static bool constraint_enabled_get_fn(const btTypedConstraint &constraint)
{
  return constraint.isEnabled();
}
static void constraint_enabled_set_fn(btTypedConstraint &constraint, const bool value)
{
  constraint.setEnabled(value);
}

// static int constraint_body1_get_fn(const btTypedConstraint &constraint)
//{
//   return get_body_index(constraint.getRigidBodyA());
// }
// static void constraint_body1_set_fn(btTypedConstraint &constraint, int /*value*/) {}
//
// static int constraint_body2_get_fn(const btTypedConstraint &constraint)
//{
//   return get_body_index(constraint.getRigidBodyB());
// }
// static void constraint_body2_set_fn(btTypedConstraint & /*constraint*/, int /*value*/) {}

static float4x4 constraint_frame1_get_fn(const btTypedConstraint &constraint)
{
  return get_constraint_frame1(constraint);
}
static void constraint_frame1_set_fn(btTypedConstraint &constraint, float4x4 value)
{
  set_constraint_frame1(constraint, value);
}

static float4x4 constraint_frame2_get_fn(const btTypedConstraint &constraint)
{
  return get_constraint_frame2(constraint);
}
static void constraint_frame2_set_fn(btTypedConstraint &constraint, float4x4 value)
{
  set_constraint_frame2(constraint, value);
}

static float constraint_applied_impulse_get_fn(const btTypedConstraint &constraint)
{
  /* Note: applied transform requires that needsFeedback is set first. */
  return constraint.needsFeedback() ? to_blender(constraint.getAppliedImpulse()) : float(0.0f);
}
static void constraint_applied_impulse_set_fn(btTypedConstraint & /*constraint*/, float /*value*/)
{
}

static float3 applied_force1_get_fn(const btTypedConstraint &constraint)
{
  const btJointFeedback *feedback = constraint.getJointFeedback();
  if (!feedback) {
    return float3(0.0f);
  }
  /* Note: applied transform requires that needsFeedback is set first. */
  return to_blender(feedback->m_appliedForceBodyA);
}
static void applied_force1_set_fn(btTypedConstraint & /*constraint*/, float3 /*value*/) {}

static float3 applied_force2_get_fn(const btTypedConstraint &constraint)
{
  const btJointFeedback *feedback = constraint.getJointFeedback();
  if (!feedback) {
    return float3(0.0f);
  }
  /* Note: applied transform requires that needsFeedback is set first. */
  return to_blender(feedback->m_appliedForceBodyB);
}
static void applied_force2_set_fn(btTypedConstraint & /*constraint*/, float3 /*value*/) {}

static float3 applied_torque1_get_fn(const btTypedConstraint &constraint)
{
  const btJointFeedback *feedback = constraint.getJointFeedback();
  if (!feedback) {
    return float3(0.0f);
  }
  /* Note: applied transform requires that needsFeedback is set first. */
  return to_blender(feedback->m_appliedTorqueBodyA);
}
static void applied_torque1_set_fn(btTypedConstraint & /*constraint*/, float3 /*value*/) {}

static float3 applied_torque2_get_fn(const btTypedConstraint &constraint)
{
  const btJointFeedback *feedback = constraint.getJointFeedback();
  if (!feedback) {
    return float3(0.0f);
  }
  /* Note: applied transform requires that needsFeedback is set first. */
  return to_blender(feedback->m_appliedForceBodyA);
}
static void applied_torque2_set_fn(btTypedConstraint & /*constraint*/, float3 /*value*/) {}

static float constraint_breaking_impulse_threshold_get_fn(const btTypedConstraint &constraint)
{
  return to_blender(constraint.getBreakingImpulseThreshold());
}
static void constraint_breaking_impulse_threshold_set_fn(btTypedConstraint &constraint,
                                                         const float value)
{
  constraint.setBreakingImpulseThreshold(value);
}

}  // namespace physics_world_attribute_functions

/** \} */

/* -------------------------------------------------------------------- */
/** \name VArrays for Physics World Data
 * \{ */

template<typename T> using RigidBodyGetFn = T (*)(const btRigidBody &body);
template<typename T> using RigidBodySetFn = void (*)(btRigidBody &body, T value);

template<typename T> using ConstraintGetFn = T (*)(const btTypedConstraint &constraint);
template<typename T> using ConstraintSetFn = void (*)(btTypedConstraint &constraint, T value);

template<typename ElemT, RigidBodyGetFn<ElemT> GetFn, RigidBodySetFn<ElemT> SetFn>
class VMutableArrayImpl_For_PhysicsBodies final : public VMutableArrayImpl<ElemT> {
 private:
  PhysicsWorldData *world_data_;

 public:
  VMutableArrayImpl_For_PhysicsBodies(PhysicsWorldData &world_data)
      : VMutableArrayImpl<ElemT>(world_data.bodies().size()),
        world_data_(&world_data) /*, lock_(impl_->data_mutex)*/
  {
  }

  /* Construct from constant world data.
   * This avoids the need for a separate VArrayImpl class.
   */
  VMutableArrayImpl_For_PhysicsBodies(const PhysicsWorldData &world_data)
      : VMutableArrayImpl<ElemT>(world_data.bodies().size()),
        world_data_(const_cast<PhysicsWorldData *>(&world_data)) /*, lock_(impl_->data_mutex)*/
  {
  }

  ~VMutableArrayImpl_For_PhysicsBodies() {}

  template<typename OtherElemT,
           RigidBodyGetFn<OtherElemT> OtherGetFn,
           RigidBodySetFn<OtherElemT> OtherSetFn>
  friend class VMutableArrayImpl_For_PhysicsBodies;

 private:
  ElemT get(const int64_t index) const override
  {
    return GetFn(*world_data_->bodies()[index]);
  }

  void set(const int64_t index, ElemT value) override
  {
    SetFn(*world_data_->bodies()[index], value);
  }

  void materialize(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>(
        [&](const int64_t i) { dst[i] = GetFn(*world_data_->bodies()[i]); });
  }

  void materialize_to_uninitialized(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>(
        [&](const int64_t i) { new (dst + i) ElemT(GetFn(*world_data_->bodies()[i])); });
  }

  void materialize_compressed(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>(
        [&](const int64_t i, const int64_t pos) { dst[pos] = GetFn(*world_data_->bodies()[i]); });
  }

  void materialize_compressed_to_uninitialized(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>([&](const int64_t i, const int64_t pos) {
      new (dst + pos) ElemT(GetFn(*world_data_->bodies()[i]));
    });
  }
};

template<typename ElemT, ConstraintGetFn<ElemT> GetFn, ConstraintSetFn<ElemT> SetFn>
class VMutableArrayImpl_For_PhysicsConstraints final : public VMutableArrayImpl<ElemT> {
 private:
  PhysicsWorldData *world_data_;

 public:
  VMutableArrayImpl_For_PhysicsConstraints(PhysicsWorldData &world_data)
      : VMutableArrayImpl<ElemT>(world_data.constraints().size()), world_data_(&world_data)
  {
  }

  /* Construct from constant world data.
   * This avoids the need for a separate VArrayImpl class.
   */
  VMutableArrayImpl_For_PhysicsConstraints(const PhysicsWorldData &world_data)
      : VMutableArrayImpl<ElemT>(world_data.constraints().size()),
        world_data_(const_cast<PhysicsWorldData *>(&world_data))
  {
  }

  ~VMutableArrayImpl_For_PhysicsConstraints() {}

  template<typename OtherElemT,
           ConstraintGetFn<OtherElemT> OtherGetFn,
           ConstraintSetFn<OtherElemT> OtherSetFn>
  friend class VMutableArrayImpl_For_PhysicsConstraints;

 private:
  ElemT get(const int64_t index) const override
  {
    return GetFn(*world_data_->constraints()[index]);
  }

  void set(const int64_t index, ElemT value) override
  {
    SetFn(*world_data_->constraints()[index], value);
  }

  void materialize(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>(
        [&](const int64_t i) { dst[i] = GetFn(*world_data_->constraints()[i]); });
  }

  void materialize_to_uninitialized(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>(
        [&](const int64_t i) { new (dst + i) ElemT(GetFn(*world_data_->constraints()[i])); });
  }

  void materialize_compressed(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>([&](const int64_t i, const int64_t pos) {
      dst[pos] = GetFn(*world_data_->constraints()[i]);
    });
  }

  void materialize_compressed_to_uninitialized(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>([&](const int64_t i, const int64_t pos) {
      new (dst + pos) ElemT(GetFn(*world_data_->constraints()[i]));
    });
  }
};

static GVMutableArray physics_attribute_vmutablearray(PhysicsBodyAttribute attribute,
                                                      PhysicsWorldData &world_data)
{
  using BodyAttribute = PhysicsBodyAttribute;
  using namespace physics_world_attribute_functions;

  switch (attribute) {
    case BodyAttribute::id:
      return VMutableArray<int>::For<
          VMutableArrayImpl_For_PhysicsBodies<int, id_get_fn, id_set_fn>>(world_data);
    case BodyAttribute::collision_shape:
      /* Shape index is defined externally. */
      return {};
    case BodyAttribute::is_static:
      return VMutableArray<bool>::For<
          VMutableArrayImpl_For_PhysicsBodies<bool, static_get_fn, static_set_fn>>(world_data);
    case BodyAttribute::is_kinematic:
      return VMutableArray<bool>::For<
          VMutableArrayImpl_For_PhysicsBodies<bool, kinematic_get_fn, kinematic_set_fn>>(
          world_data);
    case BodyAttribute::mass:
      return VMutableArray<float>::For<
          VMutableArrayImpl_For_PhysicsBodies<float, mass_get_fn, mass_set_fn>>(world_data);
    case BodyAttribute::inertia:
      return VMutableArray<float3>::For<
          VMutableArrayImpl_For_PhysicsBodies<float3, inertia_get_fn, inertia_set_fn>>(world_data);
    case BodyAttribute::position:
      return VMutableArray<float3>::For<
          VMutableArrayImpl_For_PhysicsBodies<float3, position_get_fn, position_set_fn>>(
          world_data);
    case BodyAttribute::rotation:
      return VMutableArray<math::Quaternion>::For<
          VMutableArrayImpl_For_PhysicsBodies<math::Quaternion, rotation_get_fn, rotation_set_fn>>(
          world_data);
    case BodyAttribute::velocity:
      return VMutableArray<float3>::For<
          VMutableArrayImpl_For_PhysicsBodies<float3, velocity_get_fn, velocity_set_fn>>(
          world_data);
    case BodyAttribute::angular_velocity:
      return VMutableArray<float3>::For<
          VMutableArrayImpl_For_PhysicsBodies<float3,
                                              angular_velocity_get_fn,
                                              angular_velocity_set_fn>>(world_data);
    case BodyAttribute::activation_state:
      return VMutableArray<int>::For<VMutableArrayImpl_For_PhysicsBodies<int,
                                                                         activation_state_get_fn,
                                                                         activation_state_set_fn>>(
          world_data);
    case BodyAttribute::friction:
      return VMutableArray<float>::For<
          VMutableArrayImpl_For_PhysicsBodies<float, friction_get_fn, friction_set_fn>>(
          world_data);
    case BodyAttribute::rolling_friction:
      return VMutableArray<float>::For<
          VMutableArrayImpl_For_PhysicsBodies<float,
                                              rolling_friction_get_fn,
                                              rolling_friction_set_fn>>(world_data);
    case BodyAttribute::spinning_friction:
      return VMutableArray<float>::For<
          VMutableArrayImpl_For_PhysicsBodies<float,
                                              spinning_friction_get_fn,
                                              spinning_friction_set_fn>>(world_data);
    case BodyAttribute::restitution:
      return VMutableArray<float>::For<
          VMutableArrayImpl_For_PhysicsBodies<float, restitution_get_fn, restitution_set_fn>>(
          world_data);
    case BodyAttribute::linear_damping:
      return VMutableArray<float>::For<VMutableArrayImpl_For_PhysicsBodies<float,
                                                                           linear_damping_get_fn,
                                                                           linear_damping_set_fn>>(
          world_data);
    case BodyAttribute::angular_damping:
      return VMutableArray<float>::For<
          VMutableArrayImpl_For_PhysicsBodies<float,
                                              angular_damping_get_fn,
                                              angular_damping_set_fn>>(world_data);
    case BodyAttribute::linear_sleeping_threshold:
      return VMutableArray<float>::For<
          VMutableArrayImpl_For_PhysicsBodies<float,
                                              linear_sleeping_threshold_get_fn,
                                              linear_sleeping_threshold_set_fn>>(world_data);
    case BodyAttribute::angular_sleeping_threshold:
      return VMutableArray<float>::For<
          VMutableArrayImpl_For_PhysicsBodies<float,
                                              angular_sleeping_threshold_get_fn,
                                              angular_sleeping_threshold_set_fn>>(world_data);
    case BodyAttribute::total_force:
      return VMutableArray<float3>::For<
          VMutableArrayImpl_For_PhysicsBodies<float3, total_force_get_fn, total_force_set_fn>>(
          world_data);
    case BodyAttribute::total_torque:
      return VMutableArray<float3>::For<
          VMutableArrayImpl_For_PhysicsBodies<float3, total_torque_get_fn, total_torque_set_fn>>(
          world_data);
  }
  BLI_assert_unreachable();
  return {};
}

static GVArray physics_attribute_varray(PhysicsBodyAttribute attribute,
                                        const PhysicsWorldData &world_data)
{
  return physics_attribute_vmutablearray(attribute, const_cast<PhysicsWorldData &>(world_data));
}

static GVMutableArray physics_attribute_vmutablearray(PhysicsConstraintAttribute attribute,
                                                      PhysicsWorldData &world_data)
{
  using ConstraintAttribute = PhysicsConstraintAttribute;
  using namespace physics_world_attribute_functions;

  switch (attribute) {
    case ConstraintAttribute::constraint_type:
      return {};
    case ConstraintAttribute::constraint_body1:
      return {};
    case ConstraintAttribute::constraint_body2:
      return {};
    case ConstraintAttribute::constraint_enabled:
      return VMutableArray<bool>::For<
          VMutableArrayImpl_For_PhysicsConstraints<bool,
                                                   constraint_enabled_get_fn,
                                                   constraint_enabled_set_fn>>(world_data);
    case ConstraintAttribute::constraint_frame1:
      return VMutableArray<float4x4>::For<
          VMutableArrayImpl_For_PhysicsConstraints<float4x4,
                                                   constraint_frame1_get_fn,
                                                   constraint_frame1_set_fn>>(world_data);
    case ConstraintAttribute::constraint_frame2:
      return VMutableArray<float4x4>::For<
          VMutableArrayImpl_For_PhysicsConstraints<float4x4,
                                                   constraint_frame2_get_fn,
                                                   constraint_frame2_set_fn>>(world_data);
    case ConstraintAttribute::applied_impulse:
      return VMutableArray<float>::For<
          VMutableArrayImpl_For_PhysicsConstraints<float,
                                                   constraint_applied_impulse_get_fn,
                                                   constraint_applied_impulse_set_fn>>(world_data);
    case ConstraintAttribute::applied_force1:
      return VMutableArray<float3>::For<
          VMutableArrayImpl_For_PhysicsConstraints<float3,
                                                   applied_force1_get_fn,
                                                   applied_force1_set_fn>>(world_data);
    case ConstraintAttribute::applied_force2:
      return VMutableArray<float3>::For<
          VMutableArrayImpl_For_PhysicsConstraints<float3,
                                                   applied_force2_get_fn,
                                                   applied_force2_set_fn>>(world_data);
    case ConstraintAttribute::applied_torque1:
      return VMutableArray<float3>::For<
          VMutableArrayImpl_For_PhysicsConstraints<float3,
                                                   applied_torque1_get_fn,
                                                   applied_torque1_set_fn>>(world_data);
    case ConstraintAttribute::applied_torque2:
      return VMutableArray<float3>::For<
          VMutableArrayImpl_For_PhysicsConstraints<float3,
                                                   applied_torque2_get_fn,
                                                   applied_torque2_set_fn>>(world_data);
    case ConstraintAttribute::breaking_impulse_threshold:
      return VMutableArray<float>::For<
          VMutableArrayImpl_For_PhysicsConstraints<float,
                                                   constraint_breaking_impulse_threshold_get_fn,
                                                   constraint_breaking_impulse_threshold_set_fn>>(
          world_data);
    case ConstraintAttribute::disable_collision:
      return {};
  }
  BLI_assert_unreachable();
  return {};
}

static GVArray physics_attribute_varray(PhysicsConstraintAttribute attribute,
                                        const PhysicsWorldData &world_data)
{
  return physics_attribute_vmutablearray(attribute, const_cast<PhysicsWorldData &>(world_data));
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Attribute Providers for World Data
 * \{ */

using namespace physics_world_attribute_functions;

PhysicsWorldBodyAttributeProvider::PhysicsWorldBodyAttributeProvider(
    const BodyAttribute attribute,
    const PhysicsWorldDataAccessInfo access_info,
    const UpdateOnChange update_on_change,
    const AttributeValidator validator)
    : BuiltinAttributeProvider(physics_attribute_name(attribute),
                               bke::AttrDomain::Point,
                               cpp_type_to_custom_data_type(physics_attribute_type(attribute)),
                               NonDeletable,
                               validator),
      attribute_(attribute),
      access_info_(access_info),
      update_on_change_(update_on_change)
{
}

GAttributeReader PhysicsWorldBodyAttributeProvider::try_get_for_read(const void *owner) const
{
  const PhysicsWorldData &world_data = *access_info_.get_const_world_data(owner);
  GVArray varray = physics_attribute_varray(attribute_, world_data);
  return {std::move(varray), domain_, nullptr};
}

GAttributeWriter PhysicsWorldBodyAttributeProvider::try_get_for_write(void *owner) const
{
  PhysicsWorldData &world_data = *access_info_.get_world_data(owner);
  GVMutableArray varray = physics_attribute_vmutablearray(attribute_, world_data);
  std::function<void()> tag_modified_fn = [attribute = attribute_,
                                           access_info = access_info_,
                                           update_on_change = update_on_change_,
                                           owner]() {
    PhysicsWorldData &world_data = *access_info.get_world_data(owner);
    physics_attribute_finish(world_data, attribute);
    if (update_on_change) {
      update_on_change(owner);
    }
  };
  return {std::move(varray), domain_, std::move(tag_modified_fn)};
}

bool PhysicsWorldBodyAttributeProvider::try_delete(void * /*owner*/) const
{
  return false;
}

bool PhysicsWorldBodyAttributeProvider::try_create(void * /*owner*/,
                                                   const AttributeInit & /*initializer*/) const
{
  return false;
}

bool PhysicsWorldBodyAttributeProvider::exists(const void * /*owner*/) const
{
  return true;
}

PhysicsWorldConstraintAttributeProvider::PhysicsWorldConstraintAttributeProvider(
    const ConstraintAttribute attribute,
    const PhysicsWorldDataAccessInfo access_info,
    const UpdateOnChange update_on_change,
    const AttributeValidator validator)
    : BuiltinAttributeProvider(physics_attribute_name(attribute),
                               bke::AttrDomain::Edge,
                               cpp_type_to_custom_data_type(physics_attribute_type(attribute)),
                               NonDeletable,
                               validator),
      attribute_(attribute),
      access_info_(access_info),
      update_on_change_(update_on_change)
{
}

GAttributeReader PhysicsWorldConstraintAttributeProvider::try_get_for_read(const void *owner) const
{
  const PhysicsWorldData &world_data = *access_info_.get_const_world_data(owner);
  GVArray varray = physics_attribute_varray(attribute_, world_data);
  return {std::move(varray), domain_, nullptr};
}

GAttributeWriter PhysicsWorldConstraintAttributeProvider::try_get_for_write(void *owner) const
{
  PhysicsWorldData &world_data = *access_info_.get_world_data(owner);
  GVMutableArray varray = physics_attribute_vmutablearray(attribute_, world_data);
  std::function<void()> tag_modified_fn = [attribute = attribute_,
                                           access_info = access_info_,
                                           update_on_change = update_on_change_,
                                           owner]() {
    PhysicsWorldData &world_data = *access_info.get_world_data(owner);
    physics_attribute_finish(world_data, attribute);
    if (update_on_change) {
      update_on_change(owner);
    }
  };
  return {std::move(varray), domain_, std::move(tag_modified_fn)};
}

bool PhysicsWorldConstraintAttributeProvider::try_delete(void * /*owner*/) const
{
  return false;
}

bool PhysicsWorldConstraintAttributeProvider::try_create(
    void * /*owner*/, const AttributeInit & /*initializer*/) const
{
  return false;
}

bool PhysicsWorldConstraintAttributeProvider::exists(const void * /*owner*/) const
{
  return true;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name World Data Attribute Accessor Functions
 * \{ */

static ComponentAttributeProviders create_attribute_providers()
{
  using BodyAttribute = PhysicsBodyAttribute;
  using ConstraintAttribute = PhysicsConstraintAttribute;

  static PhysicsWorldDataAccessInfo world_data_access_info = {
      [](void *owner) -> PhysicsWorldData * { return static_cast<PhysicsWorldData *>(owner); },
      [](const void *owner) -> const PhysicsWorldData * {
        return static_cast<const PhysicsWorldData *>(owner);
      },
  };

  const Span<BodyAttribute> body_attributes = all_body_attributes();
  const Span<ConstraintAttribute> constraint_attributes = all_constraint_attributes();
  static Vector<PhysicsWorldBodyAttributeProvider> body_attribute_providers_data;
  static Vector<PhysicsWorldConstraintAttributeProvider> constraint_attribute_providers_data;
  body_attribute_providers_data.reserve(body_attributes.size());
  constraint_attribute_providers_data.reserve(constraint_attributes.size());
  for (const BodyAttribute attribute : body_attributes) {
    PhysicsWorldBodyAttributeProvider provider(attribute, world_data_access_info, {});
    body_attribute_providers_data.append(std::move(provider));
  }
  for (const ConstraintAttribute attribute : constraint_attributes) {
    PhysicsWorldConstraintAttributeProvider provider(attribute, world_data_access_info, {});
    constraint_attribute_providers_data.append(std::move(provider));
  }

  Array<const BuiltinAttributeProvider *> builtin_providers(
      body_attribute_providers_data.size() + constraint_attribute_providers_data.size());
  const IndexRange body_attribute_provider_range = body_attribute_providers_data.index_range();
  const IndexRange constraint_attribute_provider_range = body_attribute_provider_range.after(
      constraint_attribute_providers_data.size());
  for (const int i : builtin_providers.index_range()) {
    builtin_providers[body_attribute_provider_range[i]] = &body_attribute_providers_data[i];
  }
  for (const int i : builtin_providers.index_range()) {
    builtin_providers[constraint_attribute_provider_range[i]] =
        &constraint_attribute_providers_data[i];
  }

  return ComponentAttributeProviders(builtin_providers, {});
}

static GVArray adapt_physics_attribute_domain(const PhysicsWorldState & /*state*/,
                                              const GVArray &varray,
                                              const AttrDomain from,
                                              const AttrDomain to)
{
  if (from == to) {
    return varray;
  }
  return {};
}

static AttributeAccessorFunctions get_accessor_functions()
{
  static const ComponentAttributeProviders providers = create_attribute_providers();
  AttributeAccessorFunctions fn =
      attribute_accessor_functions::accessor_functions_for_providers<providers>();
  fn.domain_size = [](const void *owner, const AttrDomain domain) {
    if (owner == nullptr) {
      return 0;
    }
    const PhysicsWorldData &world_data = *static_cast<const PhysicsWorldData *>(owner);
    switch (domain) {
      case AttrDomain::Point:
        return int(world_data.bodies().size());
      case AttrDomain::Edge:
        return int(world_data.constraints().size());
      default:
        return 0;
    }
  };
  fn.domain_supported = [](const void * /*owner*/, const AttrDomain domain) {
    return ELEM(domain, AttrDomain::Point, AttrDomain::Edge);
  };
  fn.adapt_domain = [](const void *owner,
                       const GVArray &varray,
                       const AttrDomain from_domain,
                       const AttrDomain to_domain) -> GVArray {
    if (owner == nullptr) {
      return {};
    }
    const PhysicsWorldState &impl = *static_cast<const PhysicsWorldState *>(owner);
    return adapt_physics_attribute_domain(impl, varray, from_domain, to_domain);
  };
  return fn;
}

static const AttributeAccessorFunctions &get_accessor_functions_ref()
{
  static const AttributeAccessorFunctions fn = get_accessor_functions();
  return fn;
}

/** \} */

float4x4 get_constraint_frame1(const btTypedConstraint &constraint)
{
  using ConstraintType = PhysicsConstraintType;

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
      const auto &typed_constraint = static_cast<const btHinge2Constraint &>(constraint);
      return to_blender(typed_constraint.getFrameOffsetA());
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
  using ConstraintType = PhysicsConstraintType;

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
      auto &typed_constraint = static_cast<btHinge2Constraint &>(constraint);
      typed_constraint.setFrames(to_bullet(value), typed_constraint.getFrameOffsetB());
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
  using ConstraintType = PhysicsConstraintType;

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
      const auto &typed_constraint = static_cast<const btHinge2Constraint &>(constraint);
      return to_blender(typed_constraint.getFrameOffsetB());
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
  using ConstraintType = PhysicsConstraintType;

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
      auto &typed_constraint = static_cast<btHinge2Constraint &>(constraint);
      typed_constraint.setFrames(typed_constraint.getFrameOffsetA(), to_bullet(value));
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

#endif

}  // namespace blender::bke
