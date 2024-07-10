/* SPDX-FileCopyrightText: 2004-2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sim
 */

#include "BKE_attribute.hh"

#include "BLI_assert.h"
#include "BLI_math_matrix.hh"
#include "BLI_math_quaternion_types.hh"
#include "BLI_utildefines.h"
#include "BLI_virtual_array.hh"

#include "physics_geometry_attributes.hh"
#include "physics_geometry_impl.hh"

#include <functional>

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

/* -------------------------------------------------------------------- */
/** \name Physics Geometry
 * \{ */

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
  const bool is_moveable_shape = !body.getCollisionShape() ||
                                 !body.getCollisionShape()->isNonMoving();
  if (is_moveable_shape) {
    if (value) {
      body.setMassProps(0.0f, to_bullet(float3(0.0f)));
      body.updateInertiaTensor();
    }
    else if (body.isStaticObject()) {
      body.setMassProps(1.0f, to_bullet(float3(1.0f)));
      body.updateInertiaTensor();
    }
  }
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
static float4x4 center_of_mass_get_fn(const btRigidBody &body)
{
  return to_blender(body.getCenterOfMassTransform());
}
static void center_of_mass_set_fn(btRigidBody &body, float4x4 value)
{
  body.setCenterOfMassTransform(to_bullet(value));
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
      activation_state_to_bullet(bke::PhysicsGeometry::BodyActivationState(value)));
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

static float3 total_torque_get_fn(const btRigidBody &body)
{
  return to_blender(body.getTotalTorque());
}

static int constraint_type_get_fn(const btTypedConstraint *constraint)
{
  return int(constraint ? to_blender(constraint->getConstraintType()) :
                          bke::PhysicsGeometry::ConstraintType::None);
}

static bool constraint_enabled_get_fn(const btTypedConstraint *constraint)
{
  return constraint ? constraint->isEnabled() : false;
}
static void constraint_enabled_set_fn(btTypedConstraint *constraint, const bool value)
{
  if (constraint) {
    constraint->setEnabled(value);
  }
}

static int constraint_body1_get_fn(const btTypedConstraint *constraint)
{
  return constraint ? get_body_index(constraint->getRigidBodyA()) : -1;
}

static int constraint_body2_get_fn(const btTypedConstraint *constraint)
{
  return constraint ? get_body_index(constraint->getRigidBodyB()) : -1;
}

static float4x4 constraint_frame1_get_fn(const btTypedConstraint *constraint)
{
  if (!constraint) {
    return float4x4::identity();
  }
  switch (constraint->getConstraintType()) {
    case FIXED_CONSTRAINT_TYPE: {
      const auto *typed_constraint = static_cast<const btFixedConstraint *>(constraint);
      return to_blender(typed_constraint->getFrameOffsetA());
    }
    case POINT2POINT_CONSTRAINT_TYPE: {
      const auto *typed_constraint = static_cast<const btPoint2PointConstraint *>(constraint);
      return math::from_location<float4x4>(to_blender(typed_constraint->getPivotInA()));
    }
    case HINGE_CONSTRAINT_TYPE: {
      const auto *typed_constraint = static_cast<const btHingeConstraint *>(constraint);
      return to_blender(typed_constraint->getAFrame());
    }
    case SLIDER_CONSTRAINT_TYPE: {
      const auto *typed_constraint = static_cast<const btSliderConstraint *>(constraint);
      return to_blender(typed_constraint->getFrameOffsetA());
    }
    case CONETWIST_CONSTRAINT_TYPE: {
      const auto *typed_constraint = static_cast<const btConeTwistConstraint *>(constraint);
      return to_blender(typed_constraint->getFrameOffsetA());
    }
    case D6_CONSTRAINT_TYPE: {
      const auto *typed_constraint = static_cast<const btGeneric6DofConstraint *>(constraint);
      return to_blender(typed_constraint->getFrameOffsetA());
    }
    case D6_SPRING_CONSTRAINT_TYPE: {
      const auto *typed_constraint = static_cast<const btGeneric6DofSpringConstraint *>(
          constraint);
      return to_blender(typed_constraint->getFrameOffsetA());
    }
    case D6_SPRING_2_CONSTRAINT_TYPE: {
      const auto *typed_constraint = static_cast<const btGeneric6DofSpring2Constraint *>(
          constraint);
      return to_blender(typed_constraint->getFrameOffsetA());
    }
    case GEAR_CONSTRAINT_TYPE: {
      const auto *typed_constraint = static_cast<const btGearConstraint *>(constraint);
      return math::from_up_axis<float4x4>(to_blender(typed_constraint->getAxisA()));
    }
    default:
      BLI_assert_unreachable();
      return float4x4::identity();
  }
}
static void constraint_frame1_set_fn(btTypedConstraint *constraint, float4x4 value)
{
  if (!constraint) {
    return;
  }
  switch (constraint->getConstraintType()) {
    case FIXED_CONSTRAINT_TYPE: {
      auto *typed_constraint = static_cast<btFixedConstraint *>(constraint);
      typed_constraint->setFrames(to_bullet(value), typed_constraint->getFrameOffsetB());
      break;
    }
    case POINT2POINT_CONSTRAINT_TYPE: {
      auto *typed_constraint = static_cast<btPoint2PointConstraint *>(constraint);
      typed_constraint->setPivotA(to_bullet(value.location()));
      break;
    }
    case HINGE_CONSTRAINT_TYPE: {
      auto *typed_constraint = static_cast<btHingeConstraint *>(constraint);
      typed_constraint->setFrames(to_bullet(value), typed_constraint->getBFrame());
      break;
    }
    case SLIDER_CONSTRAINT_TYPE: {
      auto *typed_constraint = static_cast<btSliderConstraint *>(constraint);
      typed_constraint->setFrames(to_bullet(value), typed_constraint->getFrameOffsetB());
      break;
    }
    case CONETWIST_CONSTRAINT_TYPE: {
      auto *typed_constraint = static_cast<btConeTwistConstraint *>(constraint);
      typed_constraint->setFrames(to_bullet(value), typed_constraint->getFrameOffsetB());
      break;
    }
    case D6_CONSTRAINT_TYPE: {
      auto *typed_constraint = static_cast<btGeneric6DofConstraint *>(constraint);
      typed_constraint->setFrames(to_bullet(value), typed_constraint->getFrameOffsetB());
      break;
    }
    case D6_SPRING_CONSTRAINT_TYPE: {
      auto *typed_constraint = static_cast<btGeneric6DofSpringConstraint *>(constraint);
      typed_constraint->setFrames(to_bullet(value), typed_constraint->getFrameOffsetB());
      break;
    }
    case D6_SPRING_2_CONSTRAINT_TYPE: {
      auto *typed_constraint = static_cast<btGeneric6DofSpring2Constraint *>(constraint);
      typed_constraint->setFrames(to_bullet(value), typed_constraint->getFrameOffsetB());
      break;
    }
    case GEAR_CONSTRAINT_TYPE: {
      auto *typed_constraint = static_cast<btGearConstraint *>(constraint);
      btVector3 bt_axis = to_bullet(value.z_axis());
      typed_constraint->setAxisA(bt_axis);
      break;
    }
    default:
      BLI_assert_unreachable();
      break;
  }
}

static float4x4 constraint_frame2_get_fn(const btTypedConstraint *constraint)
{
  if (!constraint) {
    return float4x4::identity();
  }
  switch (constraint->getConstraintType()) {
    case FIXED_CONSTRAINT_TYPE: {
      const auto *typed_constraint = static_cast<const btFixedConstraint *>(constraint);
      return to_blender(typed_constraint->getFrameOffsetB());
    }
    case POINT2POINT_CONSTRAINT_TYPE: {
      const auto *typed_constraint = static_cast<const btPoint2PointConstraint *>(constraint);
      return math::from_location<float4x4>(to_blender(typed_constraint->getPivotInB()));
    }
    case HINGE_CONSTRAINT_TYPE: {
      const auto *typed_constraint = static_cast<const btHingeConstraint *>(constraint);
      return to_blender(typed_constraint->getBFrame());
    }
    case SLIDER_CONSTRAINT_TYPE: {
      const auto *typed_constraint = static_cast<const btSliderConstraint *>(constraint);
      return to_blender(typed_constraint->getFrameOffsetB());
    }
    case CONETWIST_CONSTRAINT_TYPE: {
      const auto *typed_constraint = static_cast<const btConeTwistConstraint *>(constraint);
      return to_blender(typed_constraint->getFrameOffsetB());
    }
    case D6_CONSTRAINT_TYPE: {
      const auto *typed_constraint = static_cast<const btGeneric6DofConstraint *>(constraint);
      return to_blender(typed_constraint->getFrameOffsetB());
    }
    case D6_SPRING_CONSTRAINT_TYPE: {
      const auto *typed_constraint = static_cast<const btGeneric6DofSpringConstraint *>(
          constraint);
      return to_blender(typed_constraint->getFrameOffsetB());
    }
    case D6_SPRING_2_CONSTRAINT_TYPE: {
      const auto *typed_constraint = static_cast<const btGeneric6DofSpring2Constraint *>(
          constraint);
      return to_blender(typed_constraint->getFrameOffsetB());
    }
    case GEAR_CONSTRAINT_TYPE: {
      const auto *typed_constraint = static_cast<const btGearConstraint *>(constraint);
      return math::from_up_axis<float4x4>(to_blender(typed_constraint->getAxisB()));
    }
    default:
      BLI_assert_unreachable();
      return float4x4::identity();
  }
}
static void constraint_frame2_set_fn(btTypedConstraint *constraint, float4x4 value)
{
  if (!constraint) {
    return;
  }
  switch (constraint->getConstraintType()) {
    case FIXED_CONSTRAINT_TYPE: {
      auto *typed_constraint = static_cast<btFixedConstraint *>(constraint);
      typed_constraint->setFrames(typed_constraint->getFrameOffsetA(), to_bullet(value));
      break;
    }
    case POINT2POINT_CONSTRAINT_TYPE: {
      auto *typed_constraint = static_cast<btPoint2PointConstraint *>(constraint);
      typed_constraint->setPivotB(to_bullet(value.location()));
      break;
    }
    case HINGE_CONSTRAINT_TYPE: {
      auto *typed_constraint = static_cast<btHingeConstraint *>(constraint);
      typed_constraint->setFrames(typed_constraint->getAFrame(), to_bullet(value));
      break;
    }
    case SLIDER_CONSTRAINT_TYPE: {
      auto *typed_constraint = static_cast<btSliderConstraint *>(constraint);
      typed_constraint->setFrames(typed_constraint->getFrameOffsetA(), to_bullet(value));
      break;
    }
    case CONETWIST_CONSTRAINT_TYPE: {
      auto *typed_constraint = static_cast<btConeTwistConstraint *>(constraint);
      typed_constraint->setFrames(typed_constraint->getFrameOffsetA(), to_bullet(value));
      break;
    }
    case D6_CONSTRAINT_TYPE: {
      auto *typed_constraint = static_cast<btGeneric6DofConstraint *>(constraint);
      typed_constraint->setFrames(typed_constraint->getFrameOffsetA(), to_bullet(value));
      break;
    }
    case D6_SPRING_CONSTRAINT_TYPE: {
      auto *typed_constraint = static_cast<btGeneric6DofSpringConstraint *>(constraint);
      typed_constraint->setFrames(typed_constraint->getFrameOffsetA(), to_bullet(value));
      break;
    }
    case D6_SPRING_2_CONSTRAINT_TYPE: {
      auto *typed_constraint = static_cast<btGeneric6DofSpring2Constraint *>(constraint);
      typed_constraint->setFrames(typed_constraint->getFrameOffsetA(), to_bullet(value));
      break;
    }
    case GEAR_CONSTRAINT_TYPE: {
      auto *typed_constraint = static_cast<btGearConstraint *>(constraint);
      btVector3 bt_axis = to_bullet(value.z_axis());
      typed_constraint->setAxisB(bt_axis);
      break;
    }
    default:
      BLI_assert_unreachable();
      break;
  }
}

static float constraint_applied_impulse_get_fn(const btTypedConstraint *constraint)
{
  /* Note: applied transform requires that needsFeedback is set first. */
  return constraint && constraint->needsFeedback() ? to_blender(constraint->getAppliedImpulse()) :
                                                     float(0.0f);
}

static float3 applied_force1_get_fn(const btTypedConstraint *constraint)
{
  if (!constraint) {
    return float3(0.0f);
  }
  const btJointFeedback *feedback = constraint->getJointFeedback();
  if (!feedback) {
    return float3(0.0f);
  }
  /* Note: applied transform requires that needsFeedback is set first. */
  return to_blender(feedback->m_appliedForceBodyA);
}

static float3 applied_force2_get_fn(const btTypedConstraint *constraint)
{
  if (!constraint) {
    return float3(0.0f);
  }
  const btJointFeedback *feedback = constraint->getJointFeedback();
  if (!feedback) {
    return float3(0.0f);
  }
  /* Note: applied transform requires that needsFeedback is set first. */
  return to_blender(feedback->m_appliedForceBodyB);
}

static float3 applied_torque1_get_fn(const btTypedConstraint *constraint)
{
  if (!constraint) {
    return float3(0.0f);
  }
  const btJointFeedback *feedback = constraint->getJointFeedback();
  if (!feedback) {
    return float3(0.0f);
  }
  /* Note: applied transform requires that needsFeedback is set first. */
  return to_blender(feedback->m_appliedTorqueBodyA);
}

static float3 applied_torque2_get_fn(const btTypedConstraint *constraint)
{
  if (!constraint) {
    return float3(0.0f);
  }
  const btJointFeedback *feedback = constraint->getJointFeedback();
  if (!feedback) {
    return float3(0.0f);
  }
  /* Note: applied transform requires that needsFeedback is set first. */
  return to_blender(feedback->m_appliedForceBodyA);
}

static float constraint_breaking_impulse_threshold_get_fn(const btTypedConstraint *constraint)
{
  return constraint ? to_blender(constraint->getBreakingImpulseThreshold()) : float(0.0f);
}
static void constraint_breaking_impulse_threshold_set_fn(btTypedConstraint *constraint,
                                                         const float value)
{
  if (constraint) {
    constraint->setBreakingImpulseThreshold(value);
  }
}

static bool constraint_disable_collision_get_fn(const btTypedConstraint * /*constraint*/)
{
  return false;
}
static Span<bool> constraint_disable_collision_get_cache_fn(const PhysicsGeometryImpl &impl)
{
  return impl.constraint_disable_collision;
}
static void constraint_disable_collision_set_fn(btTypedConstraint *constraint, bool value)
{
  if (constraint) {
    if (value) {
      constraint->getRigidBodyA().addConstraintRef(constraint);
      constraint->getRigidBodyB().addConstraintRef(constraint);
    }
    else {
      constraint->getRigidBodyA().removeConstraintRef(constraint);
      constraint->getRigidBodyB().removeConstraintRef(constraint);
    }
  }
}

template<bool force_cache>
static ComponentAttributeProviders create_attribute_providers_for_physics()
{
  static PhysicsAccessInfo physics_access = {[](void *owner) -> PhysicsGeometryImpl * {
                                               return static_cast<PhysicsGeometryImpl *>(owner);
                                             },
                                             [](const void *owner) -> const PhysicsGeometryImpl * {
                                               return static_cast<const PhysicsGeometryImpl *>(
                                                   owner);
                                             }};

  static BuiltinRigidBodyAttributeProvider<int, force_cache, id_get_fn, id_set_fn> body_id(
      PhysicsGeometry::builtin_attributes.id,
      AttrDomain::Point,
      BuiltinAttributeProvider::NonDeletable,
      physics_access,
      nullptr);
  static BuiltinRigidBodyAttributeProvider<bool, force_cache, static_get_fn, static_set_fn>
      body_static(PhysicsGeometry::builtin_attributes.is_static,
                  AttrDomain::Point,
                  BuiltinAttributeProvider::NonDeletable,
                  physics_access,
                  nullptr);
  static BuiltinRigidBodyAttributeProvider<bool, force_cache, kinematic_get_fn, kinematic_set_fn>
      body_kinematic(PhysicsGeometry::builtin_attributes.is_kinematic,
                     AttrDomain::Point,
                     BuiltinAttributeProvider::NonDeletable,
                     physics_access,
                     nullptr);
  static BuiltinRigidBodyAttributeProvider<float, force_cache, mass_get_fn, mass_set_fn> body_mass(
      PhysicsGeometry::builtin_attributes.mass,
      AttrDomain::Point,
      BuiltinAttributeProvider::NonDeletable,
      physics_access,
      nullptr);
  static BuiltinRigidBodyAttributeProvider<float3, force_cache, inertia_get_fn, inertia_set_fn>
      body_inertia(PhysicsGeometry::builtin_attributes.inertia,
                   AttrDomain::Point,
                   BuiltinAttributeProvider::NonDeletable,
                   physics_access,
                   nullptr);
  static BuiltinRigidBodyAttributeProvider<float4x4,
                                           force_cache,
                                           center_of_mass_get_fn,
                                           center_of_mass_set_fn>
      body_center_of_mass(PhysicsGeometry::builtin_attributes.center_of_mass,
                          AttrDomain::Point,
                          BuiltinAttributeProvider::NonDeletable,
                          physics_access,
                          nullptr);
  static BuiltinRigidBodyAttributeProvider<float3, force_cache, position_get_fn, position_set_fn>
      body_position(PhysicsGeometry::builtin_attributes.position,
                    AttrDomain::Point,
                    BuiltinAttributeProvider::NonDeletable,
                    physics_access,
                    nullptr);
  static BuiltinRigidBodyAttributeProvider<math::Quaternion,
                                           force_cache,
                                           rotation_get_fn,
                                           rotation_set_fn>
      body_rotation(PhysicsGeometry::builtin_attributes.rotation,
                    AttrDomain::Point,
                    BuiltinAttributeProvider::NonDeletable,
                    physics_access,
                    nullptr);
  static BuiltinRigidBodyAttributeProvider<float3, force_cache, velocity_get_fn, velocity_set_fn>
      body_velocity(PhysicsGeometry::builtin_attributes.velocity,
                    AttrDomain::Point,
                    BuiltinAttributeProvider::NonDeletable,
                    physics_access,
                    nullptr);
  static BuiltinRigidBodyAttributeProvider<float3,
                                           force_cache,
                                           angular_velocity_get_fn,
                                           angular_velocity_set_fn>
      body_angular_velocity(PhysicsGeometry::builtin_attributes.angular_velocity,
                            AttrDomain::Point,
                            BuiltinAttributeProvider::NonDeletable,
                            physics_access,
                            nullptr);
  static BuiltinRigidBodyAttributeProvider<int,
                                           force_cache,
                                           activation_state_get_fn,
                                           activation_state_set_fn>
      body_activation_state(PhysicsGeometry::builtin_attributes.activation_state,
                            AttrDomain::Point,
                            BuiltinAttributeProvider::NonDeletable,
                            physics_access,
                            nullptr);
  static BuiltinRigidBodyAttributeProvider<float, force_cache, friction_get_fn, friction_set_fn>
      body_friction(PhysicsGeometry::builtin_attributes.friction,
                    AttrDomain::Point,
                    BuiltinAttributeProvider::NonDeletable,
                    physics_access,
                    nullptr);
  static BuiltinRigidBodyAttributeProvider<float,
                                           force_cache,
                                           rolling_friction_get_fn,
                                           rolling_friction_set_fn>
      body_rolling_friction(PhysicsGeometry::builtin_attributes.rolling_friction,
                            AttrDomain::Point,
                            BuiltinAttributeProvider::NonDeletable,
                            physics_access,
                            nullptr);
  static BuiltinRigidBodyAttributeProvider<float,
                                           force_cache,
                                           spinning_friction_get_fn,
                                           spinning_friction_set_fn>
      body_spinning_friction(PhysicsGeometry::builtin_attributes.spinning_friction,
                             AttrDomain::Point,
                             BuiltinAttributeProvider::NonDeletable,
                             physics_access,
                             nullptr);
  static BuiltinRigidBodyAttributeProvider<float,
                                           force_cache,
                                           restitution_get_fn,
                                           restitution_set_fn>
      body_restitution(PhysicsGeometry::builtin_attributes.restitution,
                       AttrDomain::Point,
                       BuiltinAttributeProvider::NonDeletable,
                       physics_access,
                       nullptr);
  static BuiltinRigidBodyAttributeProvider<float,
                                           force_cache,
                                           linear_damping_get_fn,
                                           linear_damping_set_fn>
      body_linear_damping(PhysicsGeometry::builtin_attributes.linear_damping,
                          AttrDomain::Point,
                          BuiltinAttributeProvider::NonDeletable,
                          physics_access,
                          nullptr);
  static BuiltinRigidBodyAttributeProvider<float,
                                           force_cache,
                                           angular_damping_get_fn,
                                           angular_damping_set_fn>
      body_angular_damping(PhysicsGeometry::builtin_attributes.angular_damping,
                           AttrDomain::Point,
                           BuiltinAttributeProvider::NonDeletable,
                           physics_access,
                           nullptr);
  static BuiltinRigidBodyAttributeProvider<float,
                                           force_cache,
                                           linear_sleeping_threshold_get_fn,
                                           linear_sleeping_threshold_set_fn>
      body_linear_sleeping_threshold(PhysicsGeometry::builtin_attributes.linear_sleeping_threshold,
                                     AttrDomain::Point,
                                     BuiltinAttributeProvider::NonDeletable,
                                     physics_access,
                                     nullptr);
  static BuiltinRigidBodyAttributeProvider<float,
                                           force_cache,
                                           angular_sleeping_threshold_get_fn,
                                           angular_sleeping_threshold_set_fn>
      body_angular_sleeping_threshold(
          PhysicsGeometry::builtin_attributes.angular_sleeping_threshold,
          AttrDomain::Point,
          BuiltinAttributeProvider::NonDeletable,
          physics_access,
          nullptr);
  static BuiltinRigidBodyAttributeProvider<float3, force_cache, total_force_get_fn>
      body_total_force(PhysicsGeometry::builtin_attributes.total_force,
                       AttrDomain::Point,
                       BuiltinAttributeProvider::NonDeletable,
                       physics_access,
                       nullptr);
  static BuiltinRigidBodyAttributeProvider<float3, force_cache, total_torque_get_fn>
      body_total_torque(PhysicsGeometry::builtin_attributes.total_torque,
                        AttrDomain::Point,
                        BuiltinAttributeProvider::NonDeletable,
                        physics_access,
                        nullptr);
  static BuiltinConstraintAttributeProvider<int, force_cache, constraint_type_get_fn>
      constraint_type(PhysicsGeometry::builtin_attributes.constraint_type,
                      AttrDomain::Edge,
                      BuiltinAttributeProvider::NonDeletable,
                      physics_access,
                      nullptr,
                      {});
  static BuiltinConstraintAttributeProvider<bool,
                                            force_cache,
                                            constraint_enabled_get_fn,
                                            constraint_enabled_set_fn>
      constraint_enabled(PhysicsGeometry::builtin_attributes.constraint_enabled,
                         AttrDomain::Edge,
                         BuiltinAttributeProvider::NonDeletable,
                         physics_access,
                         nullptr,
                         {});
  static BuiltinConstraintAttributeProvider<int, force_cache, constraint_body1_get_fn>
      constraint_body1(PhysicsGeometry::builtin_attributes.constraint_body1,
                       AttrDomain::Edge,
                       BuiltinAttributeProvider::NonDeletable,
                       physics_access,
                       nullptr,
                       {},
                       [](const void *owner) {
                         const PhysicsGeometry *physics = static_cast<const PhysicsGeometry *>(
                             owner);
                         physics->impl().ensure_body_indices();
                       });
  static BuiltinConstraintAttributeProvider<int, force_cache, constraint_body2_get_fn>
      constraint_body2(PhysicsGeometry::builtin_attributes.constraint_body2,
                       AttrDomain::Edge,
                       BuiltinAttributeProvider::NonDeletable,
                       physics_access,
                       nullptr,
                       {},
                       [](const void *owner) {
                         const PhysicsGeometry *physics = static_cast<const PhysicsGeometry *>(
                             owner);
                         physics->impl().ensure_body_indices();
                       });
  static BuiltinConstraintAttributeProvider<float4x4,
                                            force_cache,
                                            constraint_frame1_get_fn,
                                            constraint_frame1_set_fn>
      constraint_frame1(PhysicsGeometry::builtin_attributes.constraint_frame1,
                        AttrDomain::Edge,
                        BuiltinAttributeProvider::NonDeletable,
                        physics_access,
                        nullptr,
                        {});
  static BuiltinConstraintAttributeProvider<float4x4,
                                            force_cache,
                                            constraint_frame2_get_fn,
                                            constraint_frame2_set_fn>
      constraint_frame2(PhysicsGeometry::builtin_attributes.constraint_frame2,
                        AttrDomain::Edge,
                        BuiltinAttributeProvider::NonDeletable,
                        physics_access,
                        nullptr,
                        {});
  static BuiltinConstraintAttributeProvider<float, force_cache, constraint_applied_impulse_get_fn>
      constraint_applied_impulse(PhysicsGeometry::builtin_attributes.applied_impulse,
                                 AttrDomain::Edge,
                                 BuiltinAttributeProvider::NonDeletable,
                                 physics_access,
                                 nullptr,
                                 {});
  static BuiltinConstraintAttributeProvider<float3, force_cache, applied_force1_get_fn>
      applied_force1(PhysicsGeometry::builtin_attributes.applied_force1,
                     AttrDomain::Edge,
                     BuiltinAttributeProvider::NonDeletable,
                     physics_access,
                     nullptr,
                     {});
  static BuiltinConstraintAttributeProvider<float3, force_cache, applied_force2_get_fn>
      applied_force2(PhysicsGeometry::builtin_attributes.applied_force2,
                     AttrDomain::Edge,
                     BuiltinAttributeProvider::NonDeletable,
                     physics_access,
                     nullptr,
                     {});
  static BuiltinConstraintAttributeProvider<float3, force_cache, applied_torque1_get_fn>
      applied_torque1(PhysicsGeometry::builtin_attributes.applied_torque1,
                      AttrDomain::Edge,
                      BuiltinAttributeProvider::NonDeletable,
                      physics_access,
                      nullptr,
                      {});
  static BuiltinConstraintAttributeProvider<float3, force_cache, applied_torque2_get_fn>
      applied_torque2(PhysicsGeometry::builtin_attributes.applied_torque2,
                      AttrDomain::Edge,
                      BuiltinAttributeProvider::NonDeletable,
                      physics_access,
                      nullptr,
                      {});
  static BuiltinConstraintAttributeProvider<float,
                                            force_cache,
                                            constraint_breaking_impulse_threshold_get_fn,
                                            constraint_breaking_impulse_threshold_set_fn>
      constraint_breaking_impulse_threshold(
          PhysicsGeometry::builtin_attributes.breaking_impulse_threshold,
          AttrDomain::Edge,
          BuiltinAttributeProvider::NonDeletable,
          physics_access,
          nullptr,
          {});
  static BuiltinConstraintAttributeProvider<bool,
                                            force_cache,
                                            constraint_disable_collision_get_fn,
                                            constraint_disable_collision_set_fn,
                                            constraint_disable_collision_get_cache_fn>
      constraint_disable_collision(
          PhysicsGeometry::builtin_attributes.disable_collision,
          AttrDomain::Edge,
          BuiltinAttributeProvider::NonDeletable,
          physics_access,
          [](void *owner) {
            static_cast<PhysicsGeometry *>(owner)
                ->impl_for_write()
                .tag_constraint_disable_collision_changed();
          },
          {},
          [](const void *owner) {
            static_cast<const PhysicsGeometry *>(owner)
                ->impl()
                .ensure_constraint_disable_collision();
          });

  static CustomDataAccessInfo body_custom_data_access = {
      [](void *owner) -> CustomData * {
        PhysicsGeometry *physics = static_cast<PhysicsGeometry *>(owner);
        return &physics->impl_for_write().body_data_;
      },
      [](const void *owner) -> const CustomData * {
        const PhysicsGeometry *physics = static_cast<const PhysicsGeometry *>(owner);
        return &physics->impl().body_data_;
      },
      [](const void *owner) -> int {
        const PhysicsGeometry *physics = static_cast<const PhysicsGeometry *>(owner);
        return physics->impl().body_num_;
      }};
  static CustomDataAccessInfo constraint_custom_data_access = {
      [](void *owner) -> CustomData * {
        PhysicsGeometry *physics = static_cast<PhysicsGeometry *>(owner);
        return &physics->impl_for_write().constraint_data_;
      },
      [](const void *owner) -> const CustomData * {
        const PhysicsGeometry *physics = static_cast<const PhysicsGeometry *>(owner);
        return &physics->impl().constraint_data_;
      },
      [](const void *owner) -> int {
        const PhysicsGeometry *physics = static_cast<const PhysicsGeometry *>(owner);
        return physics->impl().constraint_num_;
      }};
  static CustomDataAttributeProvider body_custom_data(AttrDomain::Point, body_custom_data_access);
  static CustomDataAttributeProvider constraint_custom_data(AttrDomain::Point,
                                                            constraint_custom_data_access);

  return ComponentAttributeProviders({&body_id,
                                      &body_static,
                                      &body_kinematic,
                                      &body_mass,
                                      &body_inertia,
                                      &body_center_of_mass,
                                      &body_position,
                                      &body_rotation,
                                      &body_velocity,
                                      &body_angular_velocity,
                                      &body_activation_state,
                                      &body_friction,
                                      &body_rolling_friction,
                                      &body_spinning_friction,
                                      &body_restitution,
                                      &body_linear_damping,
                                      &body_angular_damping,
                                      &body_linear_sleeping_threshold,
                                      &body_angular_sleeping_threshold,
                                      &body_total_force,
                                      &body_total_torque,
                                      &constraint_type,
                                      &constraint_body1,
                                      &constraint_body2,
                                      &constraint_enabled,
                                      &constraint_frame1,
                                      &constraint_frame2,
                                      &constraint_applied_impulse,
                                      &applied_force1,
                                      &applied_force2,
                                      &applied_torque1,
                                      &applied_torque2,
                                      &constraint_breaking_impulse_threshold,
                                      &constraint_disable_collision},
                                     {&body_custom_data, &constraint_custom_data});
}

static GVArray adapt_physics_attribute_domain(const PhysicsGeometry & /*physics*/,
                                              const GVArray &varray,
                                              const AttrDomain from,
                                              const AttrDomain to)
{
  if (from == to) {
    return varray;
  }
  return {};
}

static AttributeAccessorFunctions get_physics_accessor_functions(const bool force_cache)
{
  static const ComponentAttributeProviders providers =
      create_attribute_providers_for_physics<false>();
  static const ComponentAttributeProviders providers_cached =
      create_attribute_providers_for_physics<true>();
  AttributeAccessorFunctions fn =
      force_cache ?
          attribute_accessor_functions::accessor_functions_for_providers<providers_cached>() :
          attribute_accessor_functions::accessor_functions_for_providers<providers>();
  fn.domain_size = [](const void *owner, const AttrDomain domain) {
    if (owner == nullptr) {
      return 0;
    }
    const PhysicsGeometry &physics = *static_cast<const PhysicsGeometry *>(owner);
    switch (domain) {
      case AttrDomain::Point:
        return int(physics.bodies_num());
      case AttrDomain::Edge:
        return int(physics.constraints_num());
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
    const PhysicsGeometry &physics = *static_cast<const PhysicsGeometry *>(owner);
    return adapt_physics_attribute_domain(physics, varray, from_domain, to_domain);
  };
  return fn;
}

const AttributeAccessorFunctions &get_physics_accessor_functions_ref(const bool force_cache)
{
  static const AttributeAccessorFunctions fn = get_physics_accessor_functions(force_cache);
  return fn;
}

/** \} */

#endif

}  // namespace blender::bke
