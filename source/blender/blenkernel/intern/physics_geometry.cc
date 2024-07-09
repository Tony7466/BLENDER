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

#include "BLI_array_utils.hh"
#include "BLI_assert.h"
#include "BLI_index_mask.hh"
#include "BLI_math_matrix.hh"
#include "BLI_math_quaternion_types.hh"
#include "BLI_utildefines.h"
#include "BLI_virtual_array.hh"

#include "physics_geometry_attributes.hh"
#include "physics_geometry_impl.hh"

#include <functional>
#include <mutex>

#ifdef WITH_BULLET
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

// /* Extra flags stored in btRigidBody. */
// enum class RigidBodyUserFlag : int {};
// ENUM_OPERATORS(RigidBodyUserFlag, 0)

// static RigidBodyUserFlag get_body_user_flags(const btRigidBody &body)
// {
//   return RigidBodyUserFlag(body.getUserIndex2());
// }

// static void set_body_user_flags(btRigidBody &body, const RigidBodyUserFlag flag, bool enable)
// {
//   RigidBodyUserFlag current = RigidBodyUserFlag(body.getUserIndex2());
//   SET_FLAG_FROM_TEST(current, enable, flag);
//   return body.setUserIndex2(int(current));
// }

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

static int get_constraint_index(const btTypedConstraint &constraint)
{
  return constraint.getUserConstraintId();
}

static void set_constraint_index(btTypedConstraint &constraint, const int index)
{
  constraint.setUserConstraintId(index);
}

/* -------------------------------------------------------------------- */
/** \name Physics Geometry
 * \{ */

static void create_world(PhysicsGeometryImpl &impl)
{
  impl.config = new btDefaultCollisionConfiguration();
  impl.dispatcher = new btCollisionDispatcher(impl.config);
  btGImpactCollisionAlgorithm::registerAlgorithm((btCollisionDispatcher *)impl.dispatcher);

  impl.broadphase = new btDbvtBroadphase();
  impl.overlap_filter = new DefaultOverlapFilter();
  impl.broadphase->getOverlappingPairCache()->setOverlapFilterCallback(impl.overlap_filter);

  impl.constraint_solver = new btSequentialImpulseConstraintSolver();

  impl.world = new btDiscreteDynamicsWorld(
      impl.dispatcher, impl.broadphase, impl.constraint_solver, impl.config);
}

static void destroy_world(PhysicsGeometryImpl &impl)
{
  delete impl.world;
  delete impl.constraint_solver;
  delete impl.broadphase;
  delete impl.dispatcher;
  delete impl.config;
  delete impl.overlap_filter;

  impl.world = nullptr;
  impl.constraint_solver = nullptr;
  impl.broadphase = nullptr;
  impl.dispatcher = nullptr;
  impl.config = nullptr;
  impl.overlap_filter = nullptr;
}

static void move_world(PhysicsGeometryImpl &from, PhysicsGeometryImpl &to)
{
  to.world = from.world;
  to.constraint_solver = from.constraint_solver;
  to.broadphase = from.broadphase;
  to.dispatcher = from.dispatcher;
  to.config = from.config;
  to.overlap_filter = from.overlap_filter;

  from.world = nullptr;
  from.constraint_solver = nullptr;
  from.broadphase = nullptr;
  from.dispatcher = nullptr;
  from.config = nullptr;
  from.overlap_filter = nullptr;
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
    case ConstraintType::None:
      return nullptr;
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
      /* Can't be created manually. */
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
  if (constraint) {
    // constraint->enableFeedback(true);
    constraint->setJointFeedback(feedback);
  }
  return constraint;
}

static void create_constraints(MutableSpan<btTypedConstraint *> constraints,
                               MutableSpan<btJointFeedback> constraint_feedback,
                               const Span<btRigidBody *> rigid_bodies,
                               const IndexMask &selection,
                               const VArray<int> &types,
                               const VArray<int> &bodies1,
                               const VArray<int> &bodies2)
{
  using ConstraintType = PhysicsGeometry::ConstraintType;

  const IndexRange bodies_range = rigid_bodies.index_range();
  selection.foreach_index([&](const int index) {
    const ConstraintType type = ConstraintType(types[index]);
    const int body_index1 = bodies1[index];
    const int body_index2 = bodies2[index];
    btRigidBody *fixed_body = &btTypedConstraint::getFixedBody();
    btRigidBody *body1 = bodies_range.contains(body_index1) ? rigid_bodies[body_index1] :
                                                              fixed_body;
    btRigidBody *body2 = bodies_range.contains(body_index2) ? rigid_bodies[body_index2] :
                                                              fixed_body;

    if (constraints[index]) {
      const btTypedConstraint &current_constraint = *constraints[index];
      const ConstraintType current_type = to_blender(current_constraint.getConstraintType());
      const btRigidBody *current_body1 = &current_constraint.getRigidBodyA();
      const btRigidBody *current_body2 = &current_constraint.getRigidBodyB();
      if (current_type == type && current_body1 == body1 && current_body2 == body2) {
        return;
      }

      delete constraints[index];
    }

    constraints[index] = make_constraint_type(type, *body1, *body2, &constraint_feedback[index]);
  });
}

/* Various checks on constraints to ensure Bullet doesn't crash. */
static bool is_constraint_valid(const btTypedConstraint &constraint)
{
  if (&constraint.getRigidBodyA() == &constraint.getRigidBodyB()) {
    return false;
  }
  return true;
}

static void add_to_world(btDynamicsWorld *world,
                         Span<btRigidBody *> bodies,
                         Span<btTypedConstraint *> constraints)
{
  if (!world) {
    return;
  }
  for (btRigidBody *body : bodies) {
    world->addRigidBody(body);
  }
  for (btTypedConstraint *constraint : constraints) {
    if (!constraint || !is_constraint_valid(*constraint)) {
      continue;
    }
    world->addConstraint(constraint);
  }
}

static void remove_from_world(btDynamicsWorld *world,
                              Span<btRigidBody *> bodies,
                              Span<btTypedConstraint *> constraints)
{
  if (!world) {
    return;
  }
  for (btRigidBody *body : bodies) {
    world->removeRigidBody(body);
  }
  for (btTypedConstraint *constraint : constraints) {
    if (!constraint) {
      continue;
    }
    world->removeConstraint(constraint);
  }
}

PhysicsGeometryImpl::PhysicsGeometryImpl()
    : body_num_(0), constraint_num_(0), body_data_({}), constraint_data_({})
{
  CustomData_reset(&body_data_);
  CustomData_reset(&constraint_data_);
}

PhysicsGeometryImpl::~PhysicsGeometryImpl()
{
  CustomData_free(&body_data_, body_num_);
  CustomData_free(&constraint_data_, constraint_num_);

  if (this->world) {
    remove_from_world(this->world, this->rigid_bodies, this->constraints);
    destroy_world(*this);
  }
  for (const int i : this->rigid_bodies.index_range()) {
    delete this->rigid_bodies[i];
  }
  for (const int i : this->motion_states.index_range()) {
    delete this->motion_states[i];
  }
  for (const int i : this->constraints.index_range()) {
    delete this->constraints[i];
  }
}

void PhysicsGeometryImpl::delete_self()
{
  delete this;
}

void PhysicsGeometryImpl::tag_constraint_disable_collision_changed()
{
  this->constraint_disable_collision_cache.tag_dirty();
}

void PhysicsGeometryImpl::tag_body_topology_changed()
{
  this->body_index_cache.tag_dirty();
  this->tag_constraint_disable_collision_changed();
}

void PhysicsGeometryImpl::ensure_body_indices() const
{
  this->body_index_cache.ensure([&]() {
    for (const int i : this->rigid_bodies.index_range()) {
      /* Note: Technically the btRigidBody is not mutable here! We're just using it as a cache with
       * exclusive write access, so it's fine. */
      btRigidBody &body = const_cast<btRigidBody &>(*this->rigid_bodies[i]);
      set_body_index(body, i);
    }
  });
}

void PhysicsGeometryImpl::ensure_constraint_indices() const
{
  this->constraint_index_cache.ensure([&]() {
    for (const int i : this->constraints.index_range()) {
      /* Note: Technically the btTypedConstraint is not mutable here! We're just using it as a
       * cache with exclusive write access, so it's fine. */
      btTypedConstraint *constraint = const_cast<btTypedConstraint *>(this->constraints[i]);
      if (constraint) {
        set_constraint_index(*constraint, i);
      }
    }
  });
}

void PhysicsGeometryImpl::ensure_constraint_disable_collision() const
{
  this->constraint_disable_collision_cache.ensure([&]() {
    ensure_constraint_indices();

    this->constraint_disable_collision.reinitialize(this->constraints.size());
    this->constraint_disable_collision.fill(false);
    for (const int i_body : this->rigid_bodies.index_range()) {
      /* Note: Technically the btRigidBody is not mutable here, Bullet is just very incorrect with
       * const usage ... */
      btRigidBody &body = const_cast<btRigidBody &>(*this->rigid_bodies[i_body]);
      for (const int i_ref : IndexRange(body.getNumConstraintRefs())) {
        const btTypedConstraint *constraint = body.getConstraintRef(i_ref);
        BLI_assert(constraint != nullptr);
        const int i_constraint = get_constraint_index(*constraint);
        this->constraint_disable_collision[i_constraint] = true;
      }
    }
  });
}

void PhysicsGeometryImpl::realize(const PhysicsGeometryImpl &from,
                                  const AttributeAccessor from_attributes,
                                  const IndexMask &body_mask,
                                  const IndexMask &constraint_mask,
                                  int bodies_offset,
                                  int constraints_offset,
                                  const AnonymousAttributePropagationInfo &propagation_info,
                                  MutableAttributeAccessor to_attributes)
{
  using ConstraintType = PhysicsGeometry::ConstraintType;

  BLI_assert(!this->is_empty);
  BLI_assert(from.is_empty);

  btDynamicsWorld *to_world = this->world;

  const IndexRange body_range = IndexRange(bodies_offset, body_mask.size());
  const IndexRange constraint_range = IndexRange(constraints_offset, constraint_mask.size());
  /* Make sure target has enough space. */
  BLI_assert(body_range.intersect(this->rigid_bodies.index_range()) == body_range);
  BLI_assert(constraint_range.intersect(this->constraints.index_range()) == constraint_range);

  create_bodies(this->rigid_bodies.as_mutable_span().slice(body_range),
                this->motion_states.as_mutable_span().slice(body_range),
                body_mask);
  this->tag_body_topology_changed();

  const VArray<int> constraint_types = *from_attributes.lookup_or_default(
      PhysicsGeometry::builtin_attributes.constraint_type,
      AttrDomain::Edge,
      int(ConstraintType::None));
  const VArray<int> constraint_bodies1 = *from_attributes.lookup_or_default(
      PhysicsGeometry::builtin_attributes.constraint_body1, AttrDomain::Edge, -1);
  const VArray<int> constraint_bodies2 = *from_attributes.lookup_or_default(
      PhysicsGeometry::builtin_attributes.constraint_body2, AttrDomain::Edge, -1);
  create_constraints(this->constraints.as_mutable_span().slice(constraint_range),
                     this->constraint_feedback.as_mutable_span().slice(constraint_range),
                     this->rigid_bodies.as_span().slice(body_range),
                     constraint_mask,
                     constraint_types,
                     constraint_bodies1,
                     constraint_bodies2);

  Set<std::string> skip_attributes = {PhysicsGeometry::builtin_attributes.skip_copy};
  skip_attributes.add_multiple({PhysicsGeometry::builtin_attributes.constraint_type,
                                PhysicsGeometry::builtin_attributes.constraint_body1,
                                PhysicsGeometry::builtin_attributes.constraint_body2});
  gather_attributes(from_attributes,
                    AttrDomain::Point,
                    propagation_info,
                    skip_attributes,
                    body_mask,
                    to_attributes);
  gather_attributes(from_attributes,
                    AttrDomain::Edge,
                    propagation_info,
                    skip_attributes,
                    constraint_mask,
                    to_attributes);

  /* Add all bodies and constraints to the world. */
  if (to_world != nullptr) {
    add_to_world(to_world,
                 this->rigid_bodies.as_span().slice(body_range),
                 this->constraints.as_span().slice(constraint_range));
  }
}

bool PhysicsGeometryImpl::try_copy_to_customdata(const PhysicsGeometryImpl &from,
                                                 const AttributeAccessor from_attributes,
                                                 const IndexMask &body_mask,
                                                 const IndexMask &constraint_mask,
                                                 const int bodies_offset,
                                                 const int constraints_offset)
{
  if (this->is_empty) {
    return false;
  }

  from_attributes.for_all(
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
          default:
            BLI_assert_unreachable();
            break;
        }
        const eCustomDataType data_type = meta_data.data_type;
        void *data = CustomData_add_layer_named(
            custom_data, data_type, CD_CONSTRUCT, totelem, attribute_id.name());

        GAttributeReader reader = from_attributes.lookup(attribute_id);
        reader.varray.materialize_to_uninitialized(data);

        return true;
      });

  return true;
}

bool PhysicsGeometryImpl::try_move(const PhysicsGeometryImpl &from,
                                   const AttributeAccessor from_attributes,
                                   const bool use_world,
                                   const IndexMask &body_mask,
                                   const IndexMask &constraint_mask,
                                   const int bodies_offset,
                                   const int constraints_offset)
{
  BLI_assert(this->is_mutable());
  BLI_assert(!this->is_empty);

  /* Early check before locking. */
  if (from.is_empty) {
    return false;
  }
  std::unique_lock lock(from.data_mutex);
  if (from.is_empty) {
    /* This may have changed before locking the mutex. */
    return false;
  }
  PhysicsGeometryImpl &from_mutable = const_cast<PhysicsGeometryImpl &>(from);

  /* Cache the source before moving physics data. */
  from_mutable.try_copy_to_customdata(
      from, from_attributes, body_mask, constraint_mask, bodies_offset, constraints_offset);

  btDynamicsWorld *from_world = from_mutable.world;
  if (use_world) {
    if (this->world) {
      destroy_world(*this);
    }
    move_world(from_mutable, *this);
  }
  btDynamicsWorld *to_world = this->world;

  const IndexRange body_range = IndexRange(bodies_offset, body_mask.size());
  const IndexRange constraint_range = IndexRange(constraints_offset, constraint_mask.size());
  /* Make sure target has enough space. */
  BLI_assert(body_range.intersect(this->rigid_bodies.index_range()) == body_range);
  BLI_assert(constraint_range.intersect(this->constraints.index_range()) == constraint_range);

  /* No need to update topology caches on full copy.
   * Note: empty index ranges will always compare equal! */
  if (!body_range.is_empty() && body_range == from_mutable.rigid_bodies.index_range()) {
    this->rigid_bodies.as_mutable_span().slice(body_range).copy_from(from_mutable.rigid_bodies);
    this->motion_states.as_mutable_span().slice(body_range).copy_from(from_mutable.motion_states);
  }
  else {
    body_mask.foreach_index([&](const int src_i, const int dst_i) {
      this->rigid_bodies[body_range[dst_i]] = from_mutable.rigid_bodies[src_i];
      this->motion_states[body_range[dst_i]] = from_mutable.motion_states[src_i];
      from_mutable.rigid_bodies[src_i] = nullptr;
      from_mutable.motion_states[src_i] = nullptr;
    });
    for (const int src_i : from_mutable.rigid_bodies.index_range()) {
      delete from_mutable.rigid_bodies[src_i];
      delete from_mutable.motion_states[src_i];
    }
    this->tag_body_topology_changed();
  }
  if (!constraint_range.is_empty() && constraint_range == from_mutable.constraints.index_range()) {
    this->constraints.as_mutable_span()
        .slice(constraint_range)
        .copy_from(from_mutable.constraints);
  }
  else {
    constraint_mask.foreach_index([&](const int src_i, const int dst_i) {
      this->constraints[constraint_range[dst_i]] = from_mutable.constraints[src_i];
    });
    for (const int src_i : from_mutable.constraints.index_range()) {
      delete from_mutable.constraints[src_i];
    }
  }

  /* Move all bodies and constraints to the new world. */
  if (to_world != from_world) {
    remove_from_world(from_world,
                      this->rigid_bodies.as_span().slice(body_range),
                      this->constraints.as_span().slice(constraint_range));
    add_to_world(to_world,
                 this->rigid_bodies.as_span().slice(body_range),
                 this->constraints.as_span().slice(constraint_range));
  }

  /* Clear source pointers. */
  from_mutable.world = nullptr;
  from_mutable.rigid_bodies.reinitialize(0);
  from_mutable.motion_states.reinitialize(0);
  from_mutable.constraints.reinitialize(0);
  from_mutable.constraint_feedback.reinitialize(0);
  from_mutable.is_empty.store(true);

  return true;
}

const PhysicsGeometry::BuiltinAttributes PhysicsGeometry::builtin_attributes = []() {
  PhysicsGeometry::BuiltinAttributes attributes;

  int num_all = 0;
  int num_skip = 0;
  auto register_attribute = [&](const StringRef name, const bool skip_copy = true) {
    BLI_assert(num_all < attributes.num_builtin_attributes);
    BLI_assert(num_skip < attributes.num_builtin_attributes);
    attributes.all[num_all++] = name;
    if (skip_copy) {
      attributes.skip_copy[num_skip++] = name;
    }
    return name;
  };

  attributes.id = register_attribute("id");
  attributes.is_static = register_attribute("static");
  attributes.is_kinematic = register_attribute("kinematic");
  attributes.mass = register_attribute("mass");
  attributes.inertia = register_attribute("inertia");
  attributes.center_of_mass = register_attribute("center_of_mass");
  attributes.position = register_attribute("position");
  attributes.rotation = register_attribute("rotation");
  attributes.velocity = register_attribute("velocity");
  attributes.angular_velocity = register_attribute("angular_velocity");
  attributes.activation_state = register_attribute("activation_state");
  attributes.friction = register_attribute("friction");
  attributes.rolling_friction = register_attribute("rolling_friction");
  attributes.spinning_friction = register_attribute("spinning_friction");
  attributes.restitution = register_attribute("restitution");
  attributes.linear_damping = register_attribute("linear_damping");
  attributes.angular_damping = register_attribute("angular_damping");
  attributes.linear_sleeping_threshold = register_attribute("linear_sleeping_threshold");
  attributes.angular_sleeping_threshold = register_attribute("angular_sleeping_threshold");
  attributes.constraint_type = register_attribute("type");
  attributes.constraint_body1 = register_attribute("constraint_body1");
  attributes.constraint_body2 = register_attribute("constraint_body2");
  attributes.constraint_enabled = register_attribute("enabled");
  attributes.constraint_frame1 = register_attribute("constraint_frame1");
  attributes.constraint_frame2 = register_attribute("constraint_frame2");
  attributes.constraint_enabled = register_attribute("constraint_enabled");
  attributes.applied_impulse = register_attribute("applied_impulse");
  attributes.applied_force1 = register_attribute("applied_force1");
  attributes.applied_force2 = register_attribute("applied_force2");
  attributes.applied_torque1 = register_attribute("applied_torque1");
  attributes.applied_torque2 = register_attribute("applied_torque2");
  attributes.breaking_impulse_threshold = register_attribute("breaking_impulse_threshold");
  attributes.disable_collision = register_attribute("disable_collision");
  attributes.total_force = register_attribute("total_force");
  attributes.total_torque = register_attribute("total_torque");

  return attributes;
}();

PhysicsGeometry::PhysicsGeometry()
{
  impl_ = new PhysicsGeometryImpl();
}

PhysicsGeometry::PhysicsGeometry(const PhysicsGeometry &other)
{
  impl_ = other.impl_;
  impl_->add_user();
  shapes_ = other.shapes_;
}

PhysicsGeometry::PhysicsGeometry(int bodies_num, int constraints_num)
{
  PhysicsGeometryImpl *impl = new PhysicsGeometryImpl();
  impl->body_num_ = bodies_num;
  impl->constraint_num_ = constraints_num;
  CustomData_reset(&impl->body_data_);
  CustomData_reset(&impl->constraint_data_);
  CustomData_realloc(&impl->body_data_, 0, impl->body_num_);
  CustomData_realloc(&impl->constraint_data_, 0, impl->constraint_num_);

  impl->rigid_bodies.reinitialize(bodies_num);
  impl->motion_states.reinitialize(bodies_num);
  create_bodies(impl->rigid_bodies, impl->motion_states);
  impl->constraints.reinitialize(constraints_num);
  impl->constraint_feedback.reinitialize(constraints_num);
  impl->constraint_feedback.fill(
      {btVector3(0, 0, 0), btVector3(0, 0, 0), btVector3(0, 0, 0), btVector3(0, 0, 0)});
  impl->constraints.fill(nullptr);
  impl_ = impl;

  this->tag_topology_changed();
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

  PhysicsGeometryImpl *new_impl = new PhysicsGeometryImpl();
  CustomData_reset(&new_impl->body_data_);
  CustomData_reset(&new_impl->constraint_data_);
  new_impl->body_num_ = impl_->body_num_;
  new_impl->constraint_num_ = impl_->constraint_num_;
  CustomData_copy(&impl_->body_data_, &new_impl->body_data_, CD_MASK_ALL, impl_->body_num_);
  CustomData_copy(
      &impl_->constraint_data_, &new_impl->constraint_data_, CD_MASK_ALL, impl_->constraint_num_);

  if (!impl_->is_empty) {
    new_impl->rigid_bodies.reinitialize(impl_->rigid_bodies.size());
    new_impl->motion_states.reinitialize(impl_->motion_states.size());
    new_impl->constraints.reinitialize(impl_->constraints.size());
    new_impl->constraint_feedback.reinitialize(impl_->constraints.size());
    new_impl->constraint_feedback.fill(
        {btVector3(0, 0, 0), btVector3(0, 0, 0), btVector3(0, 0, 0), btVector3(0, 0, 0)});
    new_impl->constraints.fill(nullptr);
    new_impl->try_move(
        *impl_, this->attributes(), true, this->bodies_range(), this->constraints_range(), 0, 0);
  }
  else {
    new_impl->is_empty.store(true);
  }

  impl_->remove_user_and_delete_if_last();
  impl_ = new_impl;

  return *const_cast<PhysicsGeometryImpl *>(impl_);
}

bool PhysicsGeometry::has_world() const
{
  return this->impl().world != nullptr;
}

void PhysicsGeometry::set_world_enabled(const bool enabled)
{
  PhysicsGeometryImpl &impl = this->impl_for_write();
  if (enabled) {
    if (this->impl().world == nullptr) {
      create_world(impl);
    }
  }
  else {
    if (this->impl().world != nullptr) {
      destroy_world(impl);
    }
  }
  // ensure_bodies_simulated(*this);
}

void PhysicsGeometry::set_overlap_filter(OverlapFilterFn fn)
{
  PhysicsGeometryImpl &impl = this->impl_for_write();
  impl.overlap_filter = new OverlapFilterWrapper(std::move(fn));
  impl.broadphase->getOverlappingPairCache()->setOverlapFilterCallback(impl.overlap_filter);
}

void PhysicsGeometry::clear_overlap_filter()
{
  PhysicsGeometryImpl &impl = this->impl_for_write();
  if (impl.overlap_filter) {
    delete impl.overlap_filter;
    impl.overlap_filter = new DefaultOverlapFilter();
  }
  impl.broadphase->getOverlappingPairCache()->setOverlapFilterCallback(impl.overlap_filter);
}

float3 PhysicsGeometry::gravity() const
{
  if (this->impl().world) {
    return to_blender(this->impl().world->getGravity());
  }
  return float3(0.0f);
}

void PhysicsGeometry::set_gravity(const float3 &gravity)
{
  PhysicsGeometryImpl &impl = this->impl_for_write();
  impl.world->setGravity(to_bullet(gravity));
}

void PhysicsGeometry::set_solver_iterations(const int num_solver_iterations)
{
  PhysicsGeometryImpl &impl = this->impl_for_write();
  btContactSolverInfo &info = impl.world->getSolverInfo();
  info.m_numIterations = num_solver_iterations;
}

void PhysicsGeometry::set_split_impulse(const bool split_impulse)
{
  PhysicsGeometryImpl &impl = this->impl_for_write();
  btContactSolverInfo &info = impl.world->getSolverInfo();
  /* Note: Bullet stores this as int, but it's used as a bool. */
  info.m_splitImpulse = int(split_impulse);
}

void PhysicsGeometry::step_simulation(float delta_time)
{
  constexpr const float fixed_time_step = 1.0f / 120.0f;

  PhysicsGeometryImpl &impl = this->impl_for_write();
  if (impl.world) {
    impl.world->stepSimulation(delta_time, 100, fixed_time_step);
  }
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
  return shapes_.size();
}

IndexRange PhysicsGeometry::bodies_range() const
{
  return impl().rigid_bodies.index_range();
}

IndexRange PhysicsGeometry::constraints_range() const
{
  return impl().constraints.index_range();
}

IndexRange PhysicsGeometry::shapes_range() const
{
  return shapes_.index_range();
}

/* Returns an index mask of all constraints affecting the bodies. */
static IndexMask get_constraints_mask_for_points(const PhysicsGeometryImpl &impl,
                                                 const IndexMask &body_selection,
                                                 IndexMaskMemory &memory)
{
  BLI_assert(impl.body_index_cache.is_cached());

  return IndexMask::from_predicate(
      impl.constraints.index_range(), GrainSize(1024), memory, [&](const int index) {
        const btTypedConstraint *constraint = impl.constraints[index];
        if (constraint == nullptr) {
          return false;
        }
        const int index1 = get_body_index(constraint->getRigidBodyA());
        const int index2 = get_body_index(constraint->getRigidBodyB());
        return body_selection.contains(index1) || body_selection.contains(index2);
      });
}

void PhysicsGeometry::resize(int bodies_num, int constraints_num)
{
  PhysicsGeometryImpl &impl = this->impl_for_write();
  impl.ensure_body_indices();

  CustomData_realloc(&impl.body_data_, impl.body_num_, bodies_num);
  CustomData_realloc(&impl.constraint_data_, impl.constraint_num_, constraints_num);
  impl.body_num_ = bodies_num;
  impl.constraint_num_ = constraints_num;

  const IndexMask bodies_to_delete = impl.rigid_bodies.index_range().drop_front(bodies_num);
  const IndexMask constraints_to_delete = impl.constraints.index_range().drop_front(
      constraints_num);
  IndexMaskMemory constraint_memory;
  const IndexMask extra_constraints_to_delete = get_constraints_mask_for_points(
      impl, bodies_to_delete, constraint_memory);

  bodies_to_delete.foreach_index([&](const int index) {
    delete impl.rigid_bodies[index];
    impl.rigid_bodies[index] = nullptr;
    delete impl.motion_states[index];
    impl.motion_states[index] = nullptr;
  });
  constraints_to_delete.foreach_index([&](const int index) {
    delete impl.constraints[index];
    impl.constraints[index] = nullptr;
  });
  extra_constraints_to_delete.foreach_index([&](const int index) {
    delete impl.constraints[index];
    impl.constraints[index] = nullptr;
  });

  if (!impl.is_empty) {
    Array<btRigidBody *> new_bodies(bodies_num);
    Array<btMotionState *> new_motion_states(bodies_num);
    Array<btTypedConstraint *> new_constraints(constraints_num);
    Array<btJointFeedback> new_constraint_feedback(constraints_num);
    new_bodies.as_mutable_span().take_front(impl.rigid_bodies.size()).copy_from(impl.rigid_bodies);
    new_motion_states.as_mutable_span()
        .take_front(impl.motion_states.size())
        .copy_from(impl.motion_states);
    new_constraints.as_mutable_span()
        .take_front(impl.constraints.size())
        .copy_from(impl.constraints);
    new_constraint_feedback.as_mutable_span()
        .take_front(impl.constraints.size())
        .copy_from(impl.constraint_feedback);
    /* Bodies and motion states must always exist. */
    const IndexRange bodies_to_initialize = new_bodies.index_range().drop_front(
        impl.rigid_bodies.size());
    const IndexRange constraints_to_initialize = new_constraints.index_range().drop_front(
        impl.constraints.size());
    create_bodies(new_bodies.as_mutable_span().slice(bodies_to_initialize),
                  new_motion_states.as_mutable_span().slice(bodies_to_initialize));
    new_constraints.as_mutable_span().slice(constraints_to_initialize).fill(nullptr);
    new_constraint_feedback.as_mutable_span()
        .slice(constraints_to_initialize)
        .fill({btVector3(0, 0, 0), btVector3(0, 0, 0), btVector3(0, 0, 0), btVector3(0, 0, 0)});

    impl.rigid_bodies = std::move(new_bodies);
    impl.motion_states = std::move(new_motion_states);
    impl.constraints = std::move(new_constraints);
    impl.constraint_feedback = std::move(new_constraint_feedback);
  }

  this->tag_topology_changed();
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

void PhysicsGeometry::cache_or_copy_selection(
    const PhysicsGeometry &from,
    const IndexMask &body_mask,
    const IndexMask &constraint_mask,
    int bodies_offset,
    int constraints_offset,
    const bke::AnonymousAttributePropagationInfo &propagation_info)
{
  if (!impl_->is_empty) {
    return;
  }

  PhysicsGeometryImpl &impl = this->impl_for_write();
  const bool is_cached = impl.try_copy_to_customdata(from.impl(),
                                                     from.attributes(),
                                                     body_mask,
                                                     constraint_mask,
                                                     bodies_offset,
                                                     constraints_offset);

  const Set<std::string> skip_attributes = is_cached ?
                                               Set<std::string>{builtin_attributes.skip_copy} :
                                               Set<std::string>{};

  /* Physics data is empty, copy attributes instead. */

  const VArraySpan<int> src_types = from.constraint_types();
  const VArraySpan<int> src_body1 = from.constraint_body1();
  const VArraySpan<int> src_body2 = from.constraint_body2();
  const bke::AttributeAccessor src_attributes = from.attributes();

  // BKE_physics_copy_parameters_for_eval(dst_physics, &src_physics);
  bke::MutableAttributeAccessor dst_attributes = this->attributes_for_write();
  Array<int> dst_types(this->constraints_num());
  Array<int> dst_body1(this->constraints_num());
  Array<int> dst_body2(this->constraints_num());

  remap_bodies(from.bodies_num(),
               body_mask,
               constraint_mask,
               src_types,
               src_body1,
               src_body2,
               dst_types,
               dst_body1,
               dst_body2);

  this->create_constraints(this->constraints_range(),
                           VArray<int>::ForSpan(dst_types),
                           VArray<int>::ForSpan(dst_body1),
                           VArray<int>::ForSpan(dst_body2));

  bke::gather_attributes(src_attributes,
                         bke::AttrDomain::Point,
                         propagation_info,
                         skip_attributes,
                         body_mask,
                         dst_attributes);
  bke::gather_attributes(src_attributes,
                         bke::AttrDomain::Edge,
                         propagation_info,
                         skip_attributes,
                         constraint_mask,
                         dst_attributes);
}

void PhysicsGeometry::move_or_copy_selection(
    const PhysicsGeometry &from,
    const bool use_world,
    const IndexMask &body_mask,
    const IndexMask &constraint_mask,
    int bodies_offset,
    int constraints_offset,
    const bke::AnonymousAttributePropagationInfo &propagation_info)
{
  if (impl_->is_empty) {
    return;
  }

  PhysicsGeometryImpl &impl = this->impl_for_write();
  const bool was_moved = impl.try_move(from.impl(),
                                       from.attributes(),
                                       use_world,
                                       body_mask,
                                       constraint_mask,
                                       bodies_offset,
                                       constraints_offset);

  const Set<std::string> skip_attributes = was_moved ?
                                               Set<std::string>{builtin_attributes.skip_copy} :
                                               Set<std::string>{};

  /* Physics data is empty, copy attributes instead. */

  const VArraySpan<int> src_types = from.constraint_types();
  const VArraySpan<int> src_body1 = from.constraint_body1();
  const VArraySpan<int> src_body2 = from.constraint_body2();
  const bke::AttributeAccessor src_attributes = from.attributes();

  // BKE_physics_copy_parameters_for_eval(dst_physics, &src_physics);
  bke::MutableAttributeAccessor dst_attributes = this->attributes_for_write();
  Array<int> dst_types(this->constraints_num());
  Array<int> dst_body1(this->constraints_num());
  Array<int> dst_body2(this->constraints_num());

  remap_bodies(from.bodies_num(),
               body_mask,
               constraint_mask,
               src_types,
               src_body1,
               src_body2,
               dst_types,
               dst_body1,
               dst_body2);

  this->create_constraints(this->constraints_range(),
                           VArray<int>::ForSpan(dst_types),
                           VArray<int>::ForSpan(dst_body1),
                           VArray<int>::ForSpan(dst_body2));

  bke::gather_attributes(src_attributes,
                         bke::AttrDomain::Point,
                         propagation_info,
                         skip_attributes,
                         body_mask,
                         dst_attributes);
  bke::gather_attributes(src_attributes,
                         bke::AttrDomain::Edge,
                         propagation_info,
                         skip_attributes,
                         constraint_mask,
                         dst_attributes);
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
  return shapes_.as_span();
}

std::optional<int> PhysicsGeometry::find_shape_handle(const CollisionShape &shape)
{
  for (const int i : shapes_.index_range()) {
    const CollisionShapePtr &ptr = shapes_[i];
    if (ptr.get() == &shape) {
      return i;
    }
  }
  return std::nullopt;
}

int PhysicsGeometry::add_shape(const CollisionShapePtr &shape)
{
  if (const std::optional<int> handle = find_shape_handle(*shape)) {
    return *handle;
  }

  return shapes_.append_and_get_index(shape);
}

void PhysicsGeometry::set_body_shapes(const IndexMask &selection,
                                      const Span<int> shape_handles,
                                      const bool update_local_inertia)
{
  BLI_assert(selection.bounds().intersect(impl_->rigid_bodies.index_range()) ==
             selection.bounds());
  selection.foreach_index([&](const int index) {
    const int handle = shape_handles[index];
    if (!shapes_.index_range().contains(handle)) {
      return;
    }
    const CollisionShapePtr &shape_ptr = shapes_[handle];
    const btCollisionShape *bt_shape = &shape_ptr->impl().as_bullet_shape();
    const bool is_moveable_shape = !bt_shape->isNonMoving();

    btRigidBody *body = impl_->rigid_bodies[index];
    if (body->getCollisionShape() == bt_shape) {
      /* Shape is correct, nothing to do. */
      return;
    }

    // XXX is const_cast safe here? not sure why Bullet wants a mutable shape.
    body->setCollisionShape(const_cast<btCollisionShape *>(bt_shape));
    if (impl_->broadphase && body->isInWorld()) {
      impl_->broadphase->getOverlappingPairCache()->cleanProxyFromPairs(body->getBroadphaseProxy(),
                                                                        impl_->dispatcher);
    }
    if (is_moveable_shape) {
      if (update_local_inertia) {
        btVector3 bt_local_inertia;
        bt_shape->calculateLocalInertia(body->getMass(), bt_local_inertia);
        body->setMassProps(body->getMass(), bt_local_inertia);
        body->updateInertiaTensor();
      }
    }
    else {
      body->setMassProps(0.0f, btVector3(0.0f, 0.0f, 0.0f));
      body->updateInertiaTensor();
    }
  });
}

VArray<int> PhysicsGeometry::body_ids() const
{
  return attributes().lookup(builtin_attributes.id).varray.typed<int>();
}

AttributeWriter<int> PhysicsGeometry::body_ids_for_write()
{
  return attributes_for_write().lookup_for_write<int>(builtin_attributes.id);
}

VArray<bool> PhysicsGeometry::body_is_static() const
{
  return attributes().lookup(builtin_attributes.is_static).varray.typed<bool>();
}

VArray<float> PhysicsGeometry::body_masses() const
{
  return attributes().lookup(builtin_attributes.mass).varray.typed<float>();
}

AttributeWriter<float> PhysicsGeometry::body_masses_for_write()
{
  return attributes_for_write().lookup_for_write<float>(builtin_attributes.mass);
}

VArray<float3> PhysicsGeometry::body_inertias() const
{
  return attributes().lookup(builtin_attributes.inertia).varray.typed<float3>();
}

AttributeWriter<float3> PhysicsGeometry::body_inertias_for_write()
{
  return attributes_for_write().lookup_for_write<float3>(builtin_attributes.inertia);
}

VArray<float3> PhysicsGeometry::body_positions() const
{
  return attributes().lookup(builtin_attributes.position).varray.typed<float3>();
}

AttributeWriter<float3> PhysicsGeometry::body_positions_for_write()
{
  return attributes_for_write().lookup_for_write<float3>(builtin_attributes.position);
}

VArray<math::Quaternion> PhysicsGeometry::body_rotations() const
{
  return attributes().lookup(builtin_attributes.rotation).varray.typed<math::Quaternion>();
}

AttributeWriter<math::Quaternion> PhysicsGeometry::body_rotations_for_write()
{
  return attributes_for_write().lookup_for_write<math::Quaternion>(builtin_attributes.rotation);
}

VArray<float3> PhysicsGeometry::body_velocities() const
{
  return attributes().lookup(builtin_attributes.velocity).varray.typed<float3>();
}

AttributeWriter<float3> PhysicsGeometry::body_velocities_for_write()
{
  return attributes_for_write().lookup_for_write<float3>(builtin_attributes.velocity);
}

VArray<float3> PhysicsGeometry::body_angular_velocities() const
{
  return attributes().lookup(builtin_attributes.angular_velocity).varray.typed<float3>();
}

AttributeWriter<float3> PhysicsGeometry::body_angular_velocities_for_write()
{
  return attributes_for_write().lookup_for_write<float3>(builtin_attributes.angular_velocity);
}

VArray<int> PhysicsGeometry::body_activation_states() const
{
  return attributes().lookup<int>(builtin_attributes.activation_state).varray;
}

AttributeWriter<int> PhysicsGeometry::body_activation_states_for_write()
{
  return attributes_for_write().lookup_for_write<int>(builtin_attributes.activation_state);
}

VArray<float3> PhysicsGeometry::body_total_force() const
{
  return attributes().lookup<float3>(builtin_attributes.total_force).varray;
}

VArray<float3> PhysicsGeometry::body_total_torque() const
{
  return attributes().lookup<float3>(builtin_attributes.total_torque).varray;
}

void PhysicsGeometry::apply_force(const IndexMask &selection,
                                  const VArray<float3> &forces,
                                  const VArray<float3> &relative_positions)
{
  PhysicsGeometryImpl &impl = impl_for_write();

  if (!relative_positions) {
    selection.foreach_index([&](const int index) {
      const float3 force = forces[index];
      impl.rigid_bodies[index]->applyCentralForce(to_bullet(force));
    });
  }

  selection.foreach_index([&](const int index) {
    const float3 force = forces[index];
    const float3 relative_position = relative_positions[index];
    impl.rigid_bodies[index]->applyForce(to_bullet(force), to_bullet(relative_position));
  });
}

void PhysicsGeometry::apply_torque(const IndexMask &selection, const VArray<float3> &torques)
{
  PhysicsGeometryImpl &impl = impl_for_write();
  selection.foreach_index([&](const int index) {
    const float3 torque = torques[index];
    impl.rigid_bodies[index]->applyTorque(to_bullet(torque));
  });
}

void PhysicsGeometry::apply_impulse(const IndexMask &selection,
                                    const VArray<float3> &impulses,
                                    const VArray<float3> &relative_positions)
{
  PhysicsGeometryImpl &impl = impl_for_write();

  if (!relative_positions) {
    selection.foreach_index([&](const int index) {
      const float3 impulse = impulses[index];
      impl.rigid_bodies[index]->applyCentralImpulse(to_bullet(impulse));
    });
  }

  selection.foreach_index([&](const int index) {
    const float3 impulse = impulses[index];
    const float3 relative_position = relative_positions[index];
    impl.rigid_bodies[index]->applyImpulse(to_bullet(impulse), to_bullet(relative_position));
  });
}

void PhysicsGeometry::apply_angular_impulse(const IndexMask &selection,
                                            const VArray<float3> &angular_impulses)
{
  PhysicsGeometryImpl &impl = impl_for_write();
  selection.foreach_index([&](const int index) {
    const float3 angular_impulse = angular_impulses[index];
    impl.rigid_bodies[index]->applyTorqueImpulse(to_bullet(angular_impulse));
  });
}

void PhysicsGeometry::clear_forces(const IndexMask &selection)
{
  PhysicsGeometryImpl &impl = impl_for_write();
  selection.foreach_index([&](const int index) { impl.rigid_bodies[index]->clearForces(); });
}

void PhysicsGeometry::create_constraints(const IndexMask &selection,
                                         const VArray<int> &types,
                                         const VArray<int> &bodies1,
                                         const VArray<int> &bodies2)
{
  PhysicsGeometryImpl &impl = this->impl_for_write();

  if (impl.is_empty) {
    void *types_data = CustomData_add_layer_named(&impl.constraint_data_,
                                                  CD_PROP_INT32,
                                                  CD_CONSTRUCT,
                                                  impl.constraint_num_,
                                                  builtin_attributes.constraint_type);
    void *body1_data = CustomData_add_layer_named(&impl.constraint_data_,
                                                  CD_PROP_INT32,
                                                  CD_CONSTRUCT,
                                                  impl.constraint_num_,
                                                  builtin_attributes.constraint_body1);
    void *body2_data = CustomData_add_layer_named(&impl.constraint_data_,
                                                  CD_PROP_INT32,
                                                  CD_CONSTRUCT,
                                                  impl.constraint_num_,
                                                  builtin_attributes.constraint_body2);
    types.materialize_to_uninitialized(
        MutableSpan(static_cast<int *>(types_data), impl.constraint_num_));
    bodies1.materialize_to_uninitialized(
        MutableSpan(static_cast<int *>(body1_data), impl.constraint_num_));
    bodies2.materialize_to_uninitialized(
        MutableSpan(static_cast<int *>(body2_data), impl.constraint_num_));

    return;
  }

  bke::create_constraints(impl.constraints,
                          impl.constraint_feedback,
                          impl.rigid_bodies,
                          selection,
                          types,
                          bodies1,
                          bodies2);
}

VArray<bool> PhysicsGeometry::constraint_enabled() const
{
  return attributes().lookup<bool>(builtin_attributes.constraint_enabled).varray;
}

AttributeWriter<bool> PhysicsGeometry::constraint_enabled_for_write()
{
  return attributes_for_write().lookup_for_write<bool>(builtin_attributes.constraint_enabled);
}

VArray<int> PhysicsGeometry::constraint_types() const
{
  return attributes().lookup<int>(builtin_attributes.constraint_type).varray;
}

VArray<int> PhysicsGeometry::constraint_body1() const
{
  return attributes().lookup<int>(builtin_attributes.constraint_body1).varray;
}

VArray<int> PhysicsGeometry::constraint_body2() const
{
  return attributes().lookup<int>(builtin_attributes.constraint_body2).varray;
}

VArray<float4x4> PhysicsGeometry::constraint_frame1() const
{
  return attributes().lookup<float4x4>(builtin_attributes.constraint_frame1).varray;
}

AttributeWriter<float4x4> PhysicsGeometry::constraint_frame1_for_write()
{
  return attributes_for_write().lookup_for_write<float4x4>(builtin_attributes.constraint_frame1);
}

VArray<float4x4> PhysicsGeometry::constraint_frame2() const
{
  return attributes().lookup<float4x4>(builtin_attributes.constraint_frame2).varray;
}

AttributeWriter<float4x4> PhysicsGeometry::constraint_frame2_for_write()
{
  return attributes_for_write().lookup_for_write<float4x4>(builtin_attributes.constraint_frame2);
}

VArray<float> PhysicsGeometry::constraint_applied_impulse() const
{
  return attributes().lookup<float>(builtin_attributes.applied_impulse).varray;
}

VArray<float> PhysicsGeometry::constraint_breaking_impulse_threshold_impulse() const
{
  return attributes().lookup<float>(builtin_attributes.breaking_impulse_threshold).varray;
}

AttributeWriter<float> PhysicsGeometry::constraint_breaking_impulse_threshold_for_write()
{
  return attributes_for_write().lookup_for_write<float>(
      builtin_attributes.breaking_impulse_threshold);
}

VArray<bool> PhysicsGeometry::constraint_disable_collision() const
{
  return attributes().lookup<bool>(builtin_attributes.disable_collision).varray;
}

AttributeWriter<bool> PhysicsGeometry::constraint_disable_collision_for_write()
{
  return attributes_for_write().lookup_for_write<bool>(builtin_attributes.disable_collision);
}

// static int id_get_fn(const btRigidBody &body)
// {
//   return body.getUserIndex();
// }
// static void id_set_fn(btRigidBody &body, int value)
// {
//   body.setUserIndex(value);
// }
// static bool static_get_fn(const btRigidBody &body)
// {
//   return body.isStaticObject();
// }
// static void static_set_fn(btRigidBody &body, bool value)
// {
//   const bool is_moveable_shape = !body.getCollisionShape() ||
//                                  !body.getCollisionShape()->isNonMoving();
//   if (is_moveable_shape) {
//     if (value) {
//       body.setMassProps(0.0f, to_bullet(float3(0.0f)));
//       body.updateInertiaTensor();
//     }
//     else if (body.isStaticObject()) {
//       body.setMassProps(1.0f, to_bullet(float3(1.0f)));
//       body.updateInertiaTensor();
//     }
//   }
// }
// static bool kinematic_get_fn(const btRigidBody &body)
// {
//   return body.isKinematicObject();
// }
// static void kinematic_set_fn(btRigidBody &body, bool value)
// {
//   int bt_collision_flags = body.getCollisionFlags();
//   SET_FLAG_FROM_TEST(bt_collision_flags, value, btCollisionObject::CF_KINEMATIC_OBJECT);
//   body.setCollisionFlags(bt_collision_flags);
// }
// static float mass_get_fn(const btRigidBody &body)
// {
//   return body.getMass();
// }
// static void mass_set_fn(btRigidBody &body, float value)
// {
//   const bool is_moveable_shape = !body.getCollisionShape() ||
//                                  !body.getCollisionShape()->isNonMoving();
//   if (is_moveable_shape) {
//     body.setMassProps(value, body.getLocalInertia());
//     body.updateInertiaTensor();
//   }
// }
// static float3 inertia_get_fn(const btRigidBody &body)
// {
//   return to_blender(body.getLocalInertia());
// }
// static void inertia_set_fn(btRigidBody &body, float3 value)
// {
//   const bool is_moveable_shape = !body.getCollisionShape() ||
//                                  !body.getCollisionShape()->isNonMoving();
//   if (is_moveable_shape) {
//     if (math::is_zero(value) && body.getCollisionShape()) {
//       btVector3 bt_inertia;
//       body.getCollisionShape()->calculateLocalInertia(body.getMass(), bt_inertia);
//       body.setMassProps(body.getMass(), bt_inertia);
//     }
//     else {
//       body.setMassProps(body.getMass(), to_bullet(value));
//     }
//     body.updateInertiaTensor();
//   }
// }
// static float4x4 center_of_mass_get_fn(const btRigidBody &body)
// {
//   return to_blender(body.getCenterOfMassTransform());
// }
// static void center_of_mass_set_fn(btRigidBody &body, float4x4 value)
// {
//   body.setCenterOfMassTransform(to_bullet(value));
// }
// static float3 position_get_fn(const btRigidBody &body)
// {
//   return to_blender(body.getWorldTransform().getOrigin());
// }
// static void position_set_fn(btRigidBody &body, float3 value)
// {
//   body.getWorldTransform().setOrigin(to_bullet(value));
// }
// static math::Quaternion rotation_get_fn(const btRigidBody &body)
// {
//   return to_blender(body.getWorldTransform().getRotation());
// }
// static void rotation_set_fn(btRigidBody &body, math::Quaternion value)
// {
//   body.getWorldTransform().setRotation(to_bullet(value));
// }
// static float3 velocity_get_fn(const btRigidBody &body)
// {
//   return to_blender(body.getLinearVelocity());
// }
// static void velocity_set_fn(btRigidBody &body, float3 value)
// {
//   body.setLinearVelocity(to_bullet(value));
// }
// static float3 angular_velocity_get_fn(const btRigidBody &body)
// {
//   return to_blender(body.getAngularVelocity());
// }
// static void angular_velocity_set_fn(btRigidBody &body, float3 value)
// {
//   body.setAngularVelocity(to_bullet(value));
// }
// static int activation_state_get_fn(const btRigidBody &body)
// {
//   return int(activation_state_to_blender(body.getActivationState()));
// }
// static void activation_state_set_fn(btRigidBody &body, int value)
// {
//   /* Note: there is also setActivationState, but that only sets if the state is not
//    * always-active or always-sleeping. This check can be performed on the caller side if the
//    * "always-x" state must be retained. */
//   body.forceActivationState(
//       activation_state_to_bullet(bke::PhysicsGeometry::BodyActivationState(value)));
// }
// static float friction_get_fn(const btRigidBody &body)
// {
//   return body.getFriction();
// }
// static void friction_set_fn(btRigidBody &body, float value)
// {
//   body.setFriction(value);
// }
// static float rolling_friction_get_fn(const btRigidBody &body)
// {
//   return body.getRollingFriction();
// }
// static void rolling_friction_set_fn(btRigidBody &body, float value)
// {
//   body.setRollingFriction(value);
// }
// static float spinning_friction_get_fn(const btRigidBody &body)
// {
//   return body.getSpinningFriction();
// }
// static void spinning_friction_set_fn(btRigidBody &body, float value)
// {
//   body.setSpinningFriction(value);
// }
// static float restitution_get_fn(const btRigidBody &body)
// {
//   return body.getRestitution();
// }
// static void restitution_set_fn(btRigidBody &body, float value)
// {
//   body.setRestitution(value);
// }
// static float linear_damping_get_fn(const btRigidBody &body)
// {
//   return body.getLinearSleepingThreshold();
// }
// static void linear_damping_set_fn(btRigidBody &body, float value)
// {
//   body.setSleepingThresholds(value, body.getAngularSleepingThreshold());
// }
// static float angular_damping_get_fn(const btRigidBody &body)
// {
//   return body.getAngularSleepingThreshold();
// }
// static void angular_damping_set_fn(btRigidBody &body, float value)
// {
//   body.setSleepingThresholds(value, body.getAngularSleepingThreshold());
// }
// static float linear_sleeping_threshold_get_fn(const btRigidBody &body)
// {
//   return body.getLinearSleepingThreshold();
// }
// static void linear_sleeping_threshold_set_fn(btRigidBody &body, float value)
// {
//   body.setSleepingThresholds(value, body.getAngularSleepingThreshold());
// }

// static float angular_sleeping_threshold_get_fn(const btRigidBody &body)
// {
//   return body.getAngularSleepingThreshold();
// }
// static void angular_sleeping_threshold_set_fn(btRigidBody &body, float value)
// {
//   body.setSleepingThresholds(body.getLinearSleepingThreshold(), value);
// }

// static float3 total_force_get_fn(const btRigidBody &body)
// {
//   return to_blender(body.getTotalForce());
// }

// static float3 total_torque_get_fn(const btRigidBody &body)
// {
//   return to_blender(body.getTotalTorque());
// }

// static int constraint_type_get_fn(const btTypedConstraint *constraint)
// {
//   return int(constraint ? to_blender(constraint->getConstraintType()) :
//                           bke::PhysicsGeometry::ConstraintType::None);
// }

// static bool constraint_enabled_get_fn(const btTypedConstraint *constraint)
// {
//   return constraint ? constraint->isEnabled() : false;
// }
// static void constraint_enabled_set_fn(btTypedConstraint *constraint, const bool value)
// {
//   if (constraint) {
//     constraint->setEnabled(value);
//   }
// }

// static int constraint_body1_get_fn(const btTypedConstraint *constraint)
// {
//   return constraint ? get_body_index(constraint->getRigidBodyA()) : -1;
// }

// static int constraint_body2_get_fn(const btTypedConstraint *constraint)
// {
//   return constraint ? get_body_index(constraint->getRigidBodyB()) : -1;
// }

// static float4x4 constraint_frame1_get_fn(const btTypedConstraint *constraint)
// {
//   if (!constraint) {
//     return float4x4::identity();
//   }
//   switch (constraint->getConstraintType()) {
//     case FIXED_CONSTRAINT_TYPE: {
//       const auto *typed_constraint = static_cast<const btFixedConstraint *>(constraint);
//       return to_blender(typed_constraint->getFrameOffsetA());
//     }
//     case POINT2POINT_CONSTRAINT_TYPE: {
//       const auto *typed_constraint = static_cast<const btPoint2PointConstraint *>(constraint);
//       return math::from_location<float4x4>(to_blender(typed_constraint->getPivotInA()));
//     }
//     case HINGE_CONSTRAINT_TYPE: {
//       const auto *typed_constraint = static_cast<const btHingeConstraint *>(constraint);
//       return to_blender(typed_constraint->getAFrame());
//     }
//     case SLIDER_CONSTRAINT_TYPE: {
//       const auto *typed_constraint = static_cast<const btSliderConstraint *>(constraint);
//       return to_blender(typed_constraint->getFrameOffsetA());
//     }
//     case CONETWIST_CONSTRAINT_TYPE: {
//       const auto *typed_constraint = static_cast<const btConeTwistConstraint *>(constraint);
//       return to_blender(typed_constraint->getFrameOffsetA());
//     }
//     case D6_CONSTRAINT_TYPE: {
//       const auto *typed_constraint = static_cast<const btGeneric6DofConstraint *>(constraint);
//       return to_blender(typed_constraint->getFrameOffsetA());
//     }
//     case D6_SPRING_CONSTRAINT_TYPE: {
//       const auto *typed_constraint = static_cast<const btGeneric6DofSpringConstraint *>(
//           constraint);
//       return to_blender(typed_constraint->getFrameOffsetA());
//     }
//     case D6_SPRING_2_CONSTRAINT_TYPE: {
//       const auto *typed_constraint = static_cast<const btGeneric6DofSpring2Constraint *>(
//           constraint);
//       return to_blender(typed_constraint->getFrameOffsetA());
//     }
//     case GEAR_CONSTRAINT_TYPE: {
//       const auto *typed_constraint = static_cast<const btGearConstraint *>(constraint);
//       return math::from_up_axis<float4x4>(to_blender(typed_constraint->getAxisA()));
//     }
//     default:
//       BLI_assert_unreachable();
//       return float4x4::identity();
//   }
// }
// static void constraint_frame1_set_fn(btTypedConstraint *constraint, float4x4 value)
// {
//   if (!constraint) {
//     return;
//   }
//   switch (constraint->getConstraintType()) {
//     case FIXED_CONSTRAINT_TYPE: {
//       auto *typed_constraint = static_cast<btFixedConstraint *>(constraint);
//       typed_constraint->setFrames(to_bullet(value), typed_constraint->getFrameOffsetB());
//       break;
//     }
//     case POINT2POINT_CONSTRAINT_TYPE: {
//       auto *typed_constraint = static_cast<btPoint2PointConstraint *>(constraint);
//       typed_constraint->setPivotA(to_bullet(value.location()));
//       break;
//     }
//     case HINGE_CONSTRAINT_TYPE: {
//       auto *typed_constraint = static_cast<btHingeConstraint *>(constraint);
//       typed_constraint->setFrames(to_bullet(value), typed_constraint->getBFrame());
//       break;
//     }
//     case SLIDER_CONSTRAINT_TYPE: {
//       auto *typed_constraint = static_cast<btSliderConstraint *>(constraint);
//       typed_constraint->setFrames(to_bullet(value), typed_constraint->getFrameOffsetB());
//       break;
//     }
//     case CONETWIST_CONSTRAINT_TYPE: {
//       auto *typed_constraint = static_cast<btConeTwistConstraint *>(constraint);
//       typed_constraint->setFrames(to_bullet(value), typed_constraint->getFrameOffsetB());
//       break;
//     }
//     case D6_CONSTRAINT_TYPE: {
//       auto *typed_constraint = static_cast<btGeneric6DofConstraint *>(constraint);
//       typed_constraint->setFrames(to_bullet(value), typed_constraint->getFrameOffsetB());
//       break;
//     }
//     case D6_SPRING_CONSTRAINT_TYPE: {
//       auto *typed_constraint = static_cast<btGeneric6DofSpringConstraint *>(constraint);
//       typed_constraint->setFrames(to_bullet(value), typed_constraint->getFrameOffsetB());
//       break;
//     }
//     case D6_SPRING_2_CONSTRAINT_TYPE: {
//       auto *typed_constraint = static_cast<btGeneric6DofSpring2Constraint *>(constraint);
//       typed_constraint->setFrames(to_bullet(value), typed_constraint->getFrameOffsetB());
//       break;
//     }
//     case GEAR_CONSTRAINT_TYPE: {
//       auto *typed_constraint = static_cast<btGearConstraint *>(constraint);
//       btVector3 bt_axis = to_bullet(value.z_axis());
//       typed_constraint->setAxisA(bt_axis);
//       break;
//     }
//     default:
//       BLI_assert_unreachable();
//       break;
//   }
// }

// static float4x4 constraint_frame2_get_fn(const btTypedConstraint *constraint)
// {
//   if (!constraint) {
//     return float4x4::identity();
//   }
//   switch (constraint->getConstraintType()) {
//     case FIXED_CONSTRAINT_TYPE: {
//       const auto *typed_constraint = static_cast<const btFixedConstraint *>(constraint);
//       return to_blender(typed_constraint->getFrameOffsetB());
//     }
//     case POINT2POINT_CONSTRAINT_TYPE: {
//       const auto *typed_constraint = static_cast<const btPoint2PointConstraint *>(constraint);
//       return math::from_location<float4x4>(to_blender(typed_constraint->getPivotInB()));
//     }
//     case HINGE_CONSTRAINT_TYPE: {
//       const auto *typed_constraint = static_cast<const btHingeConstraint *>(constraint);
//       return to_blender(typed_constraint->getBFrame());
//     }
//     case SLIDER_CONSTRAINT_TYPE: {
//       const auto *typed_constraint = static_cast<const btSliderConstraint *>(constraint);
//       return to_blender(typed_constraint->getFrameOffsetB());
//     }
//     case CONETWIST_CONSTRAINT_TYPE: {
//       const auto *typed_constraint = static_cast<const btConeTwistConstraint *>(constraint);
//       return to_blender(typed_constraint->getFrameOffsetB());
//     }
//     case D6_CONSTRAINT_TYPE: {
//       const auto *typed_constraint = static_cast<const btGeneric6DofConstraint *>(constraint);
//       return to_blender(typed_constraint->getFrameOffsetB());
//     }
//     case D6_SPRING_CONSTRAINT_TYPE: {
//       const auto *typed_constraint = static_cast<const btGeneric6DofSpringConstraint *>(
//           constraint);
//       return to_blender(typed_constraint->getFrameOffsetB());
//     }
//     case D6_SPRING_2_CONSTRAINT_TYPE: {
//       const auto *typed_constraint = static_cast<const btGeneric6DofSpring2Constraint *>(
//           constraint);
//       return to_blender(typed_constraint->getFrameOffsetB());
//     }
//     case GEAR_CONSTRAINT_TYPE: {
//       const auto *typed_constraint = static_cast<const btGearConstraint *>(constraint);
//       return math::from_up_axis<float4x4>(to_blender(typed_constraint->getAxisB()));
//     }
//     default:
//       BLI_assert_unreachable();
//       return float4x4::identity();
//   }
// }
// static void constraint_frame2_set_fn(btTypedConstraint *constraint, float4x4 value)
// {
//   if (!constraint) {
//     return;
//   }
//   switch (constraint->getConstraintType()) {
//     case FIXED_CONSTRAINT_TYPE: {
//       auto *typed_constraint = static_cast<btFixedConstraint *>(constraint);
//       typed_constraint->setFrames(typed_constraint->getFrameOffsetA(), to_bullet(value));
//       break;
//     }
//     case POINT2POINT_CONSTRAINT_TYPE: {
//       auto *typed_constraint = static_cast<btPoint2PointConstraint *>(constraint);
//       typed_constraint->setPivotB(to_bullet(value.location()));
//       break;
//     }
//     case HINGE_CONSTRAINT_TYPE: {
//       auto *typed_constraint = static_cast<btHingeConstraint *>(constraint);
//       typed_constraint->setFrames(typed_constraint->getAFrame(), to_bullet(value));
//       break;
//     }
//     case SLIDER_CONSTRAINT_TYPE: {
//       auto *typed_constraint = static_cast<btSliderConstraint *>(constraint);
//       typed_constraint->setFrames(typed_constraint->getFrameOffsetA(), to_bullet(value));
//       break;
//     }
//     case CONETWIST_CONSTRAINT_TYPE: {
//       auto *typed_constraint = static_cast<btConeTwistConstraint *>(constraint);
//       typed_constraint->setFrames(typed_constraint->getFrameOffsetA(), to_bullet(value));
//       break;
//     }
//     case D6_CONSTRAINT_TYPE: {
//       auto *typed_constraint = static_cast<btGeneric6DofConstraint *>(constraint);
//       typed_constraint->setFrames(typed_constraint->getFrameOffsetA(), to_bullet(value));
//       break;
//     }
//     case D6_SPRING_CONSTRAINT_TYPE: {
//       auto *typed_constraint = static_cast<btGeneric6DofSpringConstraint *>(constraint);
//       typed_constraint->setFrames(typed_constraint->getFrameOffsetA(), to_bullet(value));
//       break;
//     }
//     case D6_SPRING_2_CONSTRAINT_TYPE: {
//       auto *typed_constraint = static_cast<btGeneric6DofSpring2Constraint *>(constraint);
//       typed_constraint->setFrames(typed_constraint->getFrameOffsetA(), to_bullet(value));
//       break;
//     }
//     case GEAR_CONSTRAINT_TYPE: {
//       auto *typed_constraint = static_cast<btGearConstraint *>(constraint);
//       btVector3 bt_axis = to_bullet(value.z_axis());
//       typed_constraint->setAxisB(bt_axis);
//       break;
//     }
//     default:
//       BLI_assert_unreachable();
//       break;
//   }
// }

// static float constraint_applied_impulse_get_fn(const btTypedConstraint *constraint)
// {
//   /* Note: applied transform requires that needsFeedback is set first. */
//   return constraint && constraint->needsFeedback() ? to_blender(constraint->getAppliedImpulse())
//   :
//                                                      float(0.0f);
// }

// static float3 applied_force1_get_fn(const btTypedConstraint *constraint)
// {
//   if (!constraint) {
//     return float3(0.0f);
//   }
//   const btJointFeedback *feedback = constraint->getJointFeedback();
//   if (!feedback) {
//     return float3(0.0f);
//   }
//   /* Note: applied transform requires that needsFeedback is set first. */
//   return to_blender(feedback->m_appliedForceBodyA);
// }

// static float3 applied_force2_get_fn(const btTypedConstraint *constraint)
// {
//   if (!constraint) {
//     return float3(0.0f);
//   }
//   const btJointFeedback *feedback = constraint->getJointFeedback();
//   if (!feedback) {
//     return float3(0.0f);
//   }
//   /* Note: applied transform requires that needsFeedback is set first. */
//   return to_blender(feedback->m_appliedForceBodyB);
// }

// static float3 applied_torque1_get_fn(const btTypedConstraint *constraint)
// {
//   if (!constraint) {
//     return float3(0.0f);
//   }
//   const btJointFeedback *feedback = constraint->getJointFeedback();
//   if (!feedback) {
//     return float3(0.0f);
//   }
//   /* Note: applied transform requires that needsFeedback is set first. */
//   return to_blender(feedback->m_appliedTorqueBodyA);
// }

// static float3 applied_torque2_get_fn(const btTypedConstraint *constraint)
// {
//   if (!constraint) {
//     return float3(0.0f);
//   }
//   const btJointFeedback *feedback = constraint->getJointFeedback();
//   if (!feedback) {
//     return float3(0.0f);
//   }
//   /* Note: applied transform requires that needsFeedback is set first. */
//   return to_blender(feedback->m_appliedForceBodyA);
// }

// static float constraint_breaking_impulse_threshold_get_fn(const btTypedConstraint *constraint)
// {
//   return constraint ? to_blender(constraint->getBreakingImpulseThreshold()) : float(0.0f);
// }
// static void constraint_breaking_impulse_threshold_set_fn(btTypedConstraint *constraint,
//                                                          const float value)
// {
//   if (constraint) {
//     constraint->setBreakingImpulseThreshold(value);
//   }
// }

// static bool constraint_disable_collision_get_fn(const btTypedConstraint * /*constraint*/)
// {
//   return false;
// }
// static Span<bool> constraint_disable_collision_get_cache_fn(const PhysicsGeometryImpl &impl)
// {
//   return impl.constraint_disable_collision;
// }
// static void constraint_disable_collision_set_fn(btTypedConstraint *constraint, bool value)
// {
//   if (constraint) {
//     if (value) {
//       constraint->getRigidBodyA().addConstraintRef(constraint);
//       constraint->getRigidBodyB().addConstraintRef(constraint);
//     }
//     else {
//       constraint->getRigidBodyA().removeConstraintRef(constraint);
//       constraint->getRigidBodyB().removeConstraintRef(constraint);
//     }
//   }
// }

// static ComponentAttributeProviders create_attribute_providers_for_physics()
// {
//   static PhysicsAccessInfo physics_access = {
//       [](void *owner) -> PhysicsGeometry * { return static_cast<PhysicsGeometry *>(owner); },
//       [](const void *owner) -> const PhysicsGeometry * {
//         return static_cast<const PhysicsGeometry *>(owner);
//       }};
//   static BuiltinRigidBodyAttributeProvider<int, id_get_fn, id_set_fn> body_id(
//       PhysicsGeometry::builtin_attributes.id,
//       AttrDomain::Point,
//       BuiltinAttributeProvider::NonDeletable,
//       physics_access,
//       nullptr);
//   static BuiltinRigidBodyAttributeProvider<bool, static_get_fn, static_set_fn> body_static(
//       PhysicsGeometry::builtin_attributes.is_static,
//       AttrDomain::Point,
//       BuiltinAttributeProvider::NonDeletable,
//       physics_access,
//       nullptr);
//   static BuiltinRigidBodyAttributeProvider<bool, kinematic_get_fn, kinematic_set_fn>
//       body_kinematic(PhysicsGeometry::builtin_attributes.is_kinematic,
//                      AttrDomain::Point,
//                      BuiltinAttributeProvider::NonDeletable,
//                      physics_access,
//                      nullptr);
//   static BuiltinRigidBodyAttributeProvider<float, mass_get_fn, mass_set_fn> body_mass(
//       PhysicsGeometry::builtin_attributes.mass,
//       AttrDomain::Point,
//       BuiltinAttributeProvider::NonDeletable,
//       physics_access,
//       nullptr);
//   static BuiltinRigidBodyAttributeProvider<float3, inertia_get_fn, inertia_set_fn> body_inertia(
//       PhysicsGeometry::builtin_attributes.inertia,
//       AttrDomain::Point,
//       BuiltinAttributeProvider::NonDeletable,
//       physics_access,
//       nullptr);
//   static BuiltinRigidBodyAttributeProvider<float4x4, center_of_mass_get_fn,
//   center_of_mass_set_fn>
//       body_center_of_mass(PhysicsGeometry::builtin_attributes.center_of_mass,
//                           AttrDomain::Point,
//                           BuiltinAttributeProvider::NonDeletable,
//                           physics_access,
//                           nullptr);
//   static BuiltinRigidBodyAttributeProvider<float3, position_get_fn, position_set_fn>
//   body_position(
//       PhysicsGeometry::builtin_attributes.position,
//       AttrDomain::Point,
//       BuiltinAttributeProvider::NonDeletable,
//       physics_access,
//       nullptr);
//   static BuiltinRigidBodyAttributeProvider<math::Quaternion, rotation_get_fn, rotation_set_fn>
//       body_rotation(PhysicsGeometry::builtin_attributes.rotation,
//                     AttrDomain::Point,
//                     BuiltinAttributeProvider::NonDeletable,
//                     physics_access,
//                     nullptr);
//   static BuiltinRigidBodyAttributeProvider<float3, velocity_get_fn, velocity_set_fn>
//   body_velocity(
//       PhysicsGeometry::builtin_attributes.velocity,
//       AttrDomain::Point,
//       BuiltinAttributeProvider::NonDeletable,
//       physics_access,
//       nullptr);
//   static BuiltinRigidBodyAttributeProvider<float3,
//                                            angular_velocity_get_fn,
//                                            angular_velocity_set_fn>
//       body_angular_velocity(PhysicsGeometry::builtin_attributes.angular_velocity,
//                             AttrDomain::Point,
//                             BuiltinAttributeProvider::NonDeletable,
//                             physics_access,
//                             nullptr);
//   static BuiltinRigidBodyAttributeProvider<int, activation_state_get_fn,
//   activation_state_set_fn>
//       body_activation_state(PhysicsGeometry::builtin_attributes.activation_state,
//                             AttrDomain::Point,
//                             BuiltinAttributeProvider::NonDeletable,
//                             physics_access,
//                             nullptr);
//   static BuiltinRigidBodyAttributeProvider<float, friction_get_fn, friction_set_fn>
//   body_friction(
//       PhysicsGeometry::builtin_attributes.friction,
//       AttrDomain::Point,
//       BuiltinAttributeProvider::NonDeletable,
//       physics_access,
//       nullptr);
//   static BuiltinRigidBodyAttributeProvider<float, rolling_friction_get_fn,
//   rolling_friction_set_fn>
//       body_rolling_friction(PhysicsGeometry::builtin_attributes.rolling_friction,
//                             AttrDomain::Point,
//                             BuiltinAttributeProvider::NonDeletable,
//                             physics_access,
//                             nullptr);
//   static BuiltinRigidBodyAttributeProvider<float,
//                                            spinning_friction_get_fn,
//                                            spinning_friction_set_fn>
//       body_spinning_friction(PhysicsGeometry::builtin_attributes.spinning_friction,
//                              AttrDomain::Point,
//                              BuiltinAttributeProvider::NonDeletable,
//                              physics_access,
//                              nullptr);
//   static BuiltinRigidBodyAttributeProvider<float, restitution_get_fn, restitution_set_fn>
//       body_restitution(PhysicsGeometry::builtin_attributes.restitution,
//                        AttrDomain::Point,
//                        BuiltinAttributeProvider::NonDeletable,
//                        physics_access,
//                        nullptr);
//   static BuiltinRigidBodyAttributeProvider<float, linear_damping_get_fn, linear_damping_set_fn>
//       body_linear_damping(PhysicsGeometry::builtin_attributes.linear_damping,
//                           AttrDomain::Point,
//                           BuiltinAttributeProvider::NonDeletable,
//                           physics_access,
//                           nullptr);
//   static BuiltinRigidBodyAttributeProvider<float, angular_damping_get_fn,
//   angular_damping_set_fn>
//       body_angular_damping(PhysicsGeometry::builtin_attributes.angular_damping,
//                            AttrDomain::Point,
//                            BuiltinAttributeProvider::NonDeletable,
//                            physics_access,
//                            nullptr);
//   static BuiltinRigidBodyAttributeProvider<float,
//                                            linear_sleeping_threshold_get_fn,
//                                            linear_sleeping_threshold_set_fn>
//       body_linear_sleeping_threshold(PhysicsGeometry::builtin_attributes.linear_sleeping_threshold,
//                                      AttrDomain::Point,
//                                      BuiltinAttributeProvider::NonDeletable,
//                                      physics_access,
//                                      nullptr);
//   static BuiltinRigidBodyAttributeProvider<float,
//                                            angular_sleeping_threshold_get_fn,
//                                            angular_sleeping_threshold_set_fn>
//       body_angular_sleeping_threshold(
//           PhysicsGeometry::builtin_attributes.angular_sleeping_threshold,
//           AttrDomain::Point,
//           BuiltinAttributeProvider::NonDeletable,
//           physics_access,
//           nullptr);
//   static BuiltinRigidBodyAttributeProvider<float3, total_force_get_fn> body_total_force(
//       PhysicsGeometry::builtin_attributes.total_force,
//       AttrDomain::Point,
//       BuiltinAttributeProvider::NonDeletable,
//       physics_access,
//       nullptr);
//   static BuiltinRigidBodyAttributeProvider<float3, total_torque_get_fn> body_total_torque(
//       PhysicsGeometry::builtin_attributes.total_torque,
//       AttrDomain::Point,
//       BuiltinAttributeProvider::NonDeletable,
//       physics_access,
//       nullptr);
//   static BuiltinConstraintAttributeProvider<int, constraint_type_get_fn> constraint_type(
//       PhysicsGeometry::builtin_attributes.constraint_type,
//       AttrDomain::Edge,
//       BuiltinAttributeProvider::NonDeletable,
//       physics_access,
//       nullptr,
//       {});
//   static BuiltinConstraintAttributeProvider<bool,
//                                             constraint_enabled_get_fn,
//                                             constraint_enabled_set_fn>
//       constraint_enabled(PhysicsGeometry::builtin_attributes.constraint_enabled,
//                          AttrDomain::Edge,
//                          BuiltinAttributeProvider::NonDeletable,
//                          physics_access,
//                          nullptr,
//                          {});
//   static BuiltinConstraintAttributeProvider<int, constraint_body1_get_fn> constraint_body1(
//       PhysicsGeometry::builtin_attributes.constraint_body1,
//       AttrDomain::Edge,
//       BuiltinAttributeProvider::NonDeletable,
//       physics_access,
//       nullptr,
//       {},
//       [](const void *owner) {
//         const PhysicsGeometry *physics = static_cast<const PhysicsGeometry *>(owner);
//         physics->impl().ensure_body_indices();
//       });
//   static BuiltinConstraintAttributeProvider<int, constraint_body2_get_fn> constraint_body2(
//       PhysicsGeometry::builtin_attributes.constraint_body2,
//       AttrDomain::Edge,
//       BuiltinAttributeProvider::NonDeletable,
//       physics_access,
//       nullptr,
//       {},
//       [](const void *owner) {
//         const PhysicsGeometry *physics = static_cast<const PhysicsGeometry *>(owner);
//         physics->impl().ensure_body_indices();
//       });
//   static BuiltinConstraintAttributeProvider<float4x4,
//                                             constraint_frame1_get_fn,
//                                             constraint_frame1_set_fn>
//       constraint_frame1(PhysicsGeometry::builtin_attributes.constraint_frame1,
//                         AttrDomain::Edge,
//                         BuiltinAttributeProvider::NonDeletable,
//                         physics_access,
//                         nullptr,
//                         {});
//   static BuiltinConstraintAttributeProvider<float4x4,
//                                             constraint_frame2_get_fn,
//                                             constraint_frame2_set_fn>
//       constraint_frame2(PhysicsGeometry::builtin_attributes.constraint_frame2,
//                         AttrDomain::Edge,
//                         BuiltinAttributeProvider::NonDeletable,
//                         physics_access,
//                         nullptr,
//                         {});
//   static BuiltinConstraintAttributeProvider<float, constraint_applied_impulse_get_fn>
//       constraint_applied_impulse(PhysicsGeometry::builtin_attributes.applied_impulse,
//                                  AttrDomain::Edge,
//                                  BuiltinAttributeProvider::NonDeletable,
//                                  physics_access,
//                                  nullptr,
//                                  {});
//   static BuiltinConstraintAttributeProvider<float3, applied_force1_get_fn> applied_force1(
//       PhysicsGeometry::builtin_attributes.applied_force1,
//       AttrDomain::Edge,
//       BuiltinAttributeProvider::NonDeletable,
//       physics_access,
//       nullptr,
//       {});
//   static BuiltinConstraintAttributeProvider<float3, applied_force2_get_fn> applied_force2(
//       PhysicsGeometry::builtin_attributes.applied_force2,
//       AttrDomain::Edge,
//       BuiltinAttributeProvider::NonDeletable,
//       physics_access,
//       nullptr,
//       {});
//   static BuiltinConstraintAttributeProvider<float3, applied_torque1_get_fn> applied_torque1(
//       PhysicsGeometry::builtin_attributes.applied_torque1,
//       AttrDomain::Edge,
//       BuiltinAttributeProvider::NonDeletable,
//       physics_access,
//       nullptr,
//       {});
//   static BuiltinConstraintAttributeProvider<float3, applied_torque2_get_fn> applied_torque2(
//       PhysicsGeometry::builtin_attributes.applied_torque2,
//       AttrDomain::Edge,
//       BuiltinAttributeProvider::NonDeletable,
//       physics_access,
//       nullptr,
//       {});
//   static BuiltinConstraintAttributeProvider<float,
//                                             constraint_breaking_impulse_threshold_get_fn,
//                                             constraint_breaking_impulse_threshold_set_fn>
//       constraint_breaking_impulse_threshold(
//           PhysicsGeometry::builtin_attributes.breaking_impulse_threshold,
//           AttrDomain::Edge,
//           BuiltinAttributeProvider::NonDeletable,
//           physics_access,
//           nullptr,
//           {});
//   static BuiltinConstraintAttributeProvider<bool,
//                                             constraint_disable_collision_get_fn,
//                                             constraint_disable_collision_set_fn,
//                                             constraint_disable_collision_get_cache_fn>
//       constraint_disable_collision(
//           PhysicsGeometry::builtin_attributes.disable_collision,
//           AttrDomain::Edge,
//           BuiltinAttributeProvider::NonDeletable,
//           physics_access,
//           [](void *owner) {
//             static_cast<PhysicsGeometry *>(owner)
//                 ->impl_for_write()
//                 .tag_constraint_disable_collision_changed();
//           },
//           {},
//           [](const void *owner) {
//             static_cast<const PhysicsGeometry *>(owner)
//                 ->impl()
//                 .ensure_constraint_disable_collision();
//           });

//   static CustomDataAccessInfo body_custom_data_access = {
//       [](void *owner) -> CustomData * {
//         PhysicsGeometry *physics = static_cast<PhysicsGeometry *>(owner);
//         return &physics->impl_for_write().body_data_;
//       },
//       [](const void *owner) -> const CustomData * {
//         const PhysicsGeometry *physics = static_cast<const PhysicsGeometry *>(owner);
//         return &physics->impl().body_data_;
//       },
//       [](const void *owner) -> int {
//         const PhysicsGeometry *physics = static_cast<const PhysicsGeometry *>(owner);
//         return physics->impl().body_num_;
//       }};
//   static CustomDataAccessInfo constraint_custom_data_access = {
//       [](void *owner) -> CustomData * {
//         PhysicsGeometry *physics = static_cast<PhysicsGeometry *>(owner);
//         return &physics->impl_for_write().constraint_data_;
//       },
//       [](const void *owner) -> const CustomData * {
//         const PhysicsGeometry *physics = static_cast<const PhysicsGeometry *>(owner);
//         return &physics->impl().constraint_data_;
//       },
//       [](const void *owner) -> int {
//         const PhysicsGeometry *physics = static_cast<const PhysicsGeometry *>(owner);
//         return physics->impl().constraint_num_;
//       }};
//   static CustomDataAttributeProvider body_custom_data(AttrDomain::Point,
//   body_custom_data_access); static CustomDataAttributeProvider
//   constraint_custom_data(AttrDomain::Point,
//                                                             constraint_custom_data_access);

//   return ComponentAttributeProviders({&body_id,
//                                       &body_static,
//                                       &body_kinematic,
//                                       &body_mass,
//                                       &body_inertia,
//                                       &body_center_of_mass,
//                                       &body_position,
//                                       &body_rotation,
//                                       &body_velocity,
//                                       &body_angular_velocity,
//                                       &body_activation_state,
//                                       &body_friction,
//                                       &body_rolling_friction,
//                                       &body_spinning_friction,
//                                       &body_restitution,
//                                       &body_linear_damping,
//                                       &body_angular_damping,
//                                       &body_linear_sleeping_threshold,
//                                       &body_angular_sleeping_threshold,
//                                       &body_total_force,
//                                       &body_total_torque,
//                                       &constraint_type,
//                                       &constraint_body1,
//                                       &constraint_body2,
//                                       &constraint_enabled,
//                                       &constraint_frame1,
//                                       &constraint_frame2,
//                                       &constraint_applied_impulse,
//                                       &applied_force1,
//                                       &applied_force2,
//                                       &applied_torque1,
//                                       &applied_torque2,
//                                       &constraint_breaking_impulse_threshold,
//                                       &constraint_disable_collision},
//                                      {&body_custom_data, &constraint_custom_data});
// }

// static GVArray adapt_physics_attribute_domain(const PhysicsGeometry & /*physics*/,
//                                               const GVArray &varray,
//                                               const AttrDomain from,
//                                               const AttrDomain to)
// {
//   if (from == to) {
//     return varray;
//   }
//   return {};
// }

// static AttributeAccessorFunctions get_physics_accessor_functions()
// {
//   static const ComponentAttributeProviders providers = create_attribute_providers_for_physics();
//   AttributeAccessorFunctions fn =
//       attribute_accessor_functions::accessor_functions_for_providers<providers>();
//   fn.domain_size = [](const void *owner, const AttrDomain domain) {
//     if (owner == nullptr) {
//       return 0;
//     }
//     const PhysicsGeometry &physics = *static_cast<const PhysicsGeometry *>(owner);
//     switch (domain) {
//       case AttrDomain::Point:
//         return int(physics.bodies_num());
//       case AttrDomain::Edge:
//         return int(physics.constraints_num());
//       default:
//         return 0;
//     }
//   };
//   fn.domain_supported = [](const void * /*owner*/, const AttrDomain domain) {
//     return ELEM(domain, AttrDomain::Point, AttrDomain::Edge);
//   };
//   fn.adapt_domain = [](const void *owner,
//                        const GVArray &varray,
//                        const AttrDomain from_domain,
//                        const AttrDomain to_domain) -> GVArray {
//     if (owner == nullptr) {
//       return {};
//     }
//     const PhysicsGeometry &physics = *static_cast<const PhysicsGeometry *>(owner);
//     return adapt_physics_attribute_domain(physics, varray, from_domain, to_domain);
//   };
//   return fn;
// }

// static const AttributeAccessorFunctions &get_physics_accessor_functions_ref()
// {
//   static const AttributeAccessorFunctions fn = get_physics_accessor_functions();
//   return fn;
// }

AttributeAccessor PhysicsGeometry::attributes() const
{
  return AttributeAccessor(&this->impl(), bke::get_physics_accessor_functions_ref(true));
}

MutableAttributeAccessor PhysicsGeometry::attributes_for_write()
{
  return MutableAttributeAccessor(&this->impl_for_write(), bke::get_physics_accessor_functions_ref(true));
}

/** \} */

#else

// TODO add stub functions for when Bullet is disabled

#endif

}  // namespace blender::bke
