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

#include <BulletCollision/CollisionDispatch/btCollisionObject.h>
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
static void total_force_set_fn(btRigidBody & /*body*/, float3 /*value*/) {}

static float3 total_torque_get_fn(const btRigidBody &body)
{
  return to_blender(body.getTotalTorque());
}
static void total_torque_set_fn(btRigidBody & /*body*/, float3 /*value*/) {}

// static int constraint_type_get_fn(const btTypedConstraint &constraint)
//{
//   return int(bke::PhysicsGeometry::ConstraintType(constraint.getUserConstraintType()));
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

static bool constraint_disable_collision_get_fn(const btTypedConstraint & /*constraint*/)
{
  return false;
}
static void constraint_disable_collision_set_fn(btTypedConstraint &constraint, bool value)
{
  if (value) {
    constraint.getRigidBodyA().addConstraintRef(&constraint);
    constraint.getRigidBodyB().addConstraintRef(&constraint);
  }
  else {
    constraint.getRigidBodyA().removeConstraintRef(&constraint);
    constraint.getRigidBodyB().removeConstraintRef(&constraint);
  }
}

template<bool allow_cache>
static ComponentAttributeProviders create_attribute_providers_for_physics()
{
  using BodyAttribute = PhysicsGeometry::BodyAttribute;
  using ConstraintAttribute = PhysicsGeometry::ConstraintAttribute;

  static PhysicsAccessInfo physics_access = {[](void *owner) -> PhysicsGeometryImpl * {
                                               return static_cast<PhysicsGeometryImpl *>(owner);
                                             },
                                             [](const void *owner) -> const PhysicsGeometryImpl * {
                                               return static_cast<const PhysicsGeometryImpl *>(
                                                   owner);
                                             }};
  static CustomDataAccessInfo body_custom_data_access = {
      [](void *owner) -> CustomData * {
        PhysicsGeometryImpl *impl = static_cast<PhysicsGeometryImpl *>(owner);
        return &impl->body_data_;
      },
      [](const void *owner) -> const CustomData * {
        const PhysicsGeometryImpl *impl = static_cast<const PhysicsGeometryImpl *>(owner);
        return &impl->body_data_;
      },
      [](const void *owner) -> int {
        const PhysicsGeometryImpl *impl = static_cast<const PhysicsGeometryImpl *>(owner);
        return impl->body_num_;
      }};
  static CustomDataAccessInfo constraint_custom_data_access = {
      [](void *owner) -> CustomData * {
        PhysicsGeometryImpl *impl = static_cast<PhysicsGeometryImpl *>(owner);
        return &impl->constraint_data_;
      },
      [](const void *owner) -> const CustomData * {
        const PhysicsGeometryImpl *impl = static_cast<const PhysicsGeometryImpl *>(owner);
        return &impl->constraint_data_;
      },
      [](const void *owner) -> int {
        const PhysicsGeometryImpl *impl = static_cast<const PhysicsGeometryImpl *>(owner);
        return impl->constraint_num_;
      }};

  static BuiltinRigidBodyAttributeProvider<int, id_get_fn, id_set_fn> body_id(
      BodyAttribute::id, BuiltinAttributeProvider::NonDeletable, physics_access, allow_cache);
  /* Special case: body collision shapes primary storage is a custom data layer, and the internal
   * shape pointers are updated based on that. */
  static const int default_body_collision_shape = -1;
  static BuiltinCustomDataLayerProvider body_collision_shape(
      physics_attribute_name(BodyAttribute::collision_shape),
      AttrDomain::Point,
      CD_PROP_INT32,
      BuiltinAttributeProvider::NonDeletable,
      body_custom_data_access,
      [](void *owner) {
        PhysicsGeometryImpl &impl = *static_cast<PhysicsGeometryImpl *>(owner);
        physics_attribute_finish(impl, PhysicsGeometry::BodyAttribute::collision_shape);
      },
      {},
      &default_body_collision_shape);
  static BuiltinCustomDataLayerProvider body_static(
      physics_attribute_name(BodyAttribute::is_static),
      AttrDomain::Point,
      CD_PROP_BOOL,
      BuiltinAttributeProvider::NonDeletable,
      body_custom_data_access,
      [](void *owner) {
        PhysicsGeometryImpl &impl = *static_cast<PhysicsGeometryImpl *>(owner);
        physics_attribute_finish(impl, PhysicsGeometry::BodyAttribute::is_static);
      });
  static BuiltinRigidBodyAttributeProvider<bool, kinematic_get_fn, kinematic_set_fn>
      body_kinematic(BodyAttribute::is_kinematic,
                     BuiltinAttributeProvider::NonDeletable,
                     physics_access,
                     allow_cache);
  static BuiltinCustomDataLayerProvider body_mass(
      physics_attribute_name(BodyAttribute::mass),
      AttrDomain::Point,
      CD_PROP_FLOAT,
      BuiltinAttributeProvider::NonDeletable,
      body_custom_data_access,
      [](void *owner) {
        PhysicsGeometryImpl &impl = *static_cast<PhysicsGeometryImpl *>(owner);
        physics_attribute_finish(impl, PhysicsGeometry::BodyAttribute::mass);
      });
  static BuiltinRigidBodyAttributeProvider<float3, inertia_get_fn, inertia_set_fn> body_inertia(
      BodyAttribute::inertia, BuiltinAttributeProvider::NonDeletable, physics_access, allow_cache);
  static BuiltinRigidBodyAttributeProvider<float4x4, center_of_mass_get_fn, center_of_mass_set_fn>
      body_center_of_mass(BodyAttribute::center_of_mass,
                          BuiltinAttributeProvider::NonDeletable,
                          physics_access,
                          allow_cache);
  static BuiltinRigidBodyAttributeProvider<float3, position_get_fn, position_set_fn> body_position(
      BodyAttribute::position,
      BuiltinAttributeProvider::NonDeletable,
      physics_access,
      allow_cache);
  static BuiltinRigidBodyAttributeProvider<math::Quaternion, rotation_get_fn, rotation_set_fn>
      body_rotation(BodyAttribute::rotation,
                    BuiltinAttributeProvider::NonDeletable,
                    physics_access,
                    allow_cache);
  static BuiltinRigidBodyAttributeProvider<float3, velocity_get_fn, velocity_set_fn> body_velocity(
      BodyAttribute::velocity,
      BuiltinAttributeProvider::NonDeletable,
      physics_access,
      allow_cache);
  static BuiltinRigidBodyAttributeProvider<float3,
                                           angular_velocity_get_fn,
                                           angular_velocity_set_fn>
      body_angular_velocity(BodyAttribute::angular_velocity,
                            BuiltinAttributeProvider::NonDeletable,
                            physics_access,
                            allow_cache);
  static BuiltinRigidBodyAttributeProvider<int, activation_state_get_fn, activation_state_set_fn>
      body_activation_state(BodyAttribute::activation_state,
                            BuiltinAttributeProvider::NonDeletable,
                            physics_access,
                            allow_cache);
  static BuiltinRigidBodyAttributeProvider<float, friction_get_fn, friction_set_fn> body_friction(
      BodyAttribute::friction,
      BuiltinAttributeProvider::NonDeletable,
      physics_access,
      allow_cache);
  static BuiltinRigidBodyAttributeProvider<float, rolling_friction_get_fn, rolling_friction_set_fn>
      body_rolling_friction(BodyAttribute::rolling_friction,
                            BuiltinAttributeProvider::NonDeletable,
                            physics_access,
                            allow_cache);
  static BuiltinRigidBodyAttributeProvider<float,
                                           spinning_friction_get_fn,
                                           spinning_friction_set_fn>
      body_spinning_friction(BodyAttribute::spinning_friction,
                             BuiltinAttributeProvider::NonDeletable,
                             physics_access,
                             allow_cache);
  static BuiltinRigidBodyAttributeProvider<float, restitution_get_fn, restitution_set_fn>
      body_restitution(BodyAttribute::restitution,
                       BuiltinAttributeProvider::NonDeletable,
                       physics_access,
                       allow_cache);
  static BuiltinRigidBodyAttributeProvider<float, linear_damping_get_fn, linear_damping_set_fn>
      body_linear_damping(BodyAttribute::linear_damping,
                          BuiltinAttributeProvider::NonDeletable,
                          physics_access,
                          allow_cache);
  static BuiltinRigidBodyAttributeProvider<float, angular_damping_get_fn, angular_damping_set_fn>
      body_angular_damping(BodyAttribute::angular_damping,
                           BuiltinAttributeProvider::NonDeletable,
                           physics_access,
                           allow_cache);
  static BuiltinRigidBodyAttributeProvider<float,
                                           linear_sleeping_threshold_get_fn,
                                           linear_sleeping_threshold_set_fn>
      body_linear_sleeping_threshold(BodyAttribute::linear_sleeping_threshold,
                                     BuiltinAttributeProvider::NonDeletable,
                                     physics_access,
                                     allow_cache);
  static BuiltinRigidBodyAttributeProvider<float,
                                           angular_sleeping_threshold_get_fn,
                                           angular_sleeping_threshold_set_fn>
      body_angular_sleeping_threshold(BodyAttribute::angular_sleeping_threshold,
                                      BuiltinAttributeProvider::NonDeletable,
                                      physics_access,
                                      allow_cache);
  static BuiltinRigidBodyAttributeProvider<float3, total_force_get_fn, total_force_set_fn>
      body_total_force(BodyAttribute::total_force,
                       BuiltinAttributeProvider::NonDeletable,
                       physics_access,
                       allow_cache);
  static BuiltinRigidBodyAttributeProvider<float3, total_torque_get_fn, total_torque_set_fn>
      body_total_torque(BodyAttribute::total_torque,
                        BuiltinAttributeProvider::NonDeletable,
                        physics_access,
                        allow_cache);
  static BuiltinCustomDataLayerProvider constraint_type(
      physics_attribute_name(ConstraintAttribute::constraint_type),
      AttrDomain::Edge,
      CD_PROP_INT32,
      BuiltinAttributeProvider::NonDeletable,
      constraint_custom_data_access,
      [](void *owner) {
        PhysicsGeometryImpl &impl = *static_cast<PhysicsGeometryImpl *>(owner);
        physics_attribute_finish(impl, PhysicsGeometry::ConstraintAttribute::constraint_type);
      });
  static BuiltinCustomDataLayerProvider constraint_body1(
      physics_attribute_name(ConstraintAttribute::constraint_body1),
      AttrDomain::Edge,
      CD_PROP_INT32,
      BuiltinAttributeProvider::NonDeletable,
      constraint_custom_data_access,
      [](void *owner) {
        PhysicsGeometryImpl &impl = *static_cast<PhysicsGeometryImpl *>(owner);
        physics_attribute_finish(impl, PhysicsGeometry::ConstraintAttribute::constraint_body1);
      });
  static BuiltinCustomDataLayerProvider constraint_body2(
      physics_attribute_name(ConstraintAttribute::constraint_body2),
      AttrDomain::Edge,
      CD_PROP_INT32,
      BuiltinAttributeProvider::NonDeletable,
      constraint_custom_data_access,
      [](void *owner) {
        PhysicsGeometryImpl &impl = *static_cast<PhysicsGeometryImpl *>(owner);
        physics_attribute_finish(impl, PhysicsGeometry::ConstraintAttribute::constraint_body2);
      });
  // static BuiltinConstraintAttributeProvider<int, constraint_type_get_fn, constraint_type_set_fn>
  //     constraint_type(ConstraintAttribute::constraint_type,
  //                     BuiltinAttributeProvider::NonDeletable,
  //                     physics_access,
  //                     allow_cache,
  //                     {});
  // static BuiltinConstraintAttributeProvider<int, constraint_body1_get_fn,
  // constraint_body1_set_fn>
  //     constraint_body1(ConstraintAttribute::constraint_body1,
  //                      BuiltinAttributeProvider::NonDeletable,
  //                      physics_access,
  //                      allow_cache);
  // static BuiltinConstraintAttributeProvider<int, constraint_body2_get_fn,
  // constraint_body2_set_fn>
  //     constraint_body2(ConstraintAttribute::constraint_body2,
  //                      BuiltinAttributeProvider::NonDeletable,
  //                      physics_access,
  //                      allow_cache);
  static BuiltinConstraintAttributeProvider<bool,
                                            constraint_enabled_get_fn,
                                            constraint_enabled_set_fn>
      constraint_enabled(ConstraintAttribute::constraint_enabled,
                         BuiltinAttributeProvider::NonDeletable,
                         physics_access,
                         allow_cache,
                         {});
  static BuiltinConstraintAttributeProvider<float4x4,
                                            constraint_frame1_get_fn,
                                            constraint_frame1_set_fn>
      constraint_frame1(ConstraintAttribute::constraint_frame1,
                        BuiltinAttributeProvider::NonDeletable,
                        physics_access,
                        allow_cache,
                        {});
  static BuiltinConstraintAttributeProvider<float4x4,
                                            constraint_frame2_get_fn,
                                            constraint_frame2_set_fn>
      constraint_frame2(ConstraintAttribute::constraint_frame2,
                        BuiltinAttributeProvider::NonDeletable,
                        physics_access,
                        allow_cache,
                        {});
  static BuiltinConstraintAttributeProvider<float,
                                            constraint_applied_impulse_get_fn,
                                            constraint_applied_impulse_set_fn>
      constraint_applied_impulse(ConstraintAttribute::applied_impulse,
                                 BuiltinAttributeProvider::NonDeletable,
                                 physics_access,
                                 allow_cache,
                                 {});
  static BuiltinConstraintAttributeProvider<float3, applied_force1_get_fn, applied_force1_set_fn>
      applied_force1(ConstraintAttribute::applied_force1,
                     BuiltinAttributeProvider::NonDeletable,
                     physics_access,
                     allow_cache,
                     {});
  static BuiltinConstraintAttributeProvider<float3, applied_force2_get_fn, applied_force2_set_fn>
      applied_force2(ConstraintAttribute::applied_force2,
                     BuiltinAttributeProvider::NonDeletable,
                     physics_access,
                     allow_cache,
                     {});
  static BuiltinConstraintAttributeProvider<float3, applied_torque1_get_fn, applied_torque1_set_fn>
      applied_torque1(ConstraintAttribute::applied_torque1,
                      BuiltinAttributeProvider::NonDeletable,
                      physics_access,
                      allow_cache,
                      {});
  static BuiltinConstraintAttributeProvider<float3, applied_torque2_get_fn, applied_torque2_set_fn>
      applied_torque2(ConstraintAttribute::applied_torque2,
                      BuiltinAttributeProvider::NonDeletable,
                      physics_access,
                      allow_cache,
                      {});
  static BuiltinConstraintAttributeProvider<float,
                                            constraint_breaking_impulse_threshold_get_fn,
                                            constraint_breaking_impulse_threshold_set_fn>
      constraint_breaking_impulse_threshold(ConstraintAttribute::breaking_impulse_threshold,
                                            BuiltinAttributeProvider::NonDeletable,
                                            physics_access,
                                            allow_cache,
                                            {});
  static BuiltinConstraintAttributeProvider<bool,
                                            constraint_disable_collision_get_fn,
                                            constraint_disable_collision_set_fn>
      constraint_disable_collision(ConstraintAttribute::disable_collision,
                                   BuiltinAttributeProvider::NonDeletable,
                                   physics_access,
                                   allow_cache);

  static CustomDataAttributeProvider body_custom_data(AttrDomain::Point, body_custom_data_access);
  static CustomDataAttributeProvider constraint_custom_data(AttrDomain::Edge,
                                                            constraint_custom_data_access);

  return ComponentAttributeProviders({&body_id,
                                      &body_collision_shape,
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

static ComponentAttributeProviders create_attribute_providers_for_physics_custom_data()
{
  using BodyAttribute = PhysicsGeometry::BodyAttribute;
  using ConstraintAttribute = PhysicsGeometry::ConstraintAttribute;

  static CustomDataAccessInfo body_custom_data_access = {
      [](void *owner) -> CustomData * {
        auto &impl = *static_cast<PhysicsGeometryImpl *>(owner);
        return &impl.body_data_;
      },
      [](const void *owner) -> const CustomData * {
        const auto &impl = static_cast<const PhysicsGeometryImpl *>(owner);
        return &impl->body_data_;
      },
      [](const void *owner) -> int {
        const auto &impl = static_cast<const PhysicsGeometryImpl *>(owner);
        return impl->body_num_;
      }};
  static CustomDataAccessInfo constraint_custom_data_access = {
      [](void *owner) -> CustomData * {
        auto &impl = *static_cast<PhysicsGeometryImpl *>(owner);
        return &impl.constraint_data_;
      },
      [](const void *owner) -> const CustomData * {
        const auto &impl = static_cast<const PhysicsGeometryImpl *>(owner);
        return &impl->constraint_data_;
      },
      [](const void *owner) -> int {
        const auto &impl = static_cast<const PhysicsGeometryImpl *>(owner);
        return impl->constraint_num_;
      }};

  const Span<BodyAttribute> body_attributes = all_body_attributes();
  const Span<ConstraintAttribute> constraint_attributes = all_constraint_attributes();
  static Vector<BuiltinCustomDataLayerProvider> builtin_providers_data;
  builtin_providers_data.reserve(body_attributes.size() + constraint_attributes.size());
  for (const int i : body_attributes.index_range()) {
    const BodyAttribute attribute = body_attributes[i];

    BuiltinCustomDataLayerProvider provider(
        physics_attribute_name(attribute),
        AttrDomain::Point,
        cpp_type_to_custom_data_type(physics_attribute_type(attribute)),
        BuiltinAttributeProvider::NonDeletable,
        body_custom_data_access,
        {},
        {});
    builtin_providers_data.append(std::move(provider));
  }
  for (const int i : constraint_attributes.index_range()) {
    const ConstraintAttribute attribute = constraint_attributes[i];

    BuiltinCustomDataLayerProvider provider(
        physics_attribute_name(attribute),
        AttrDomain::Edge,
        cpp_type_to_custom_data_type(physics_attribute_type(attribute)),
        BuiltinAttributeProvider::NonDeletable,
        constraint_custom_data_access,
        {},
        {});
    builtin_providers_data.append(std::move(provider));
  }

  static Array<const BuiltinAttributeProvider *> builtin_providers(builtin_providers_data.size());
  for (const int i : builtin_providers.index_range()) {
    builtin_providers[i] = &builtin_providers_data[i];
  }

  static CustomDataAttributeProvider body_custom_data(AttrDomain::Point, body_custom_data_access);
  static CustomDataAttributeProvider constraint_custom_data(AttrDomain::Point,
                                                            constraint_custom_data_access);

  return ComponentAttributeProviders(builtin_providers,
                                     {&body_custom_data, &constraint_custom_data});
}

static GVArray adapt_physics_attribute_domain(const PhysicsGeometryImpl & /*impl*/,
                                              const GVArray &varray,
                                              const AttrDomain from,
                                              const AttrDomain to)
{
  if (from == to) {
    return varray;
  }
  return {};
}

static AttributeAccessorFunctions get_physics_accessor_functions(const bool enable_custom_data,
                                                                 const bool enable_world_data)
{
  static const ComponentAttributeProviders providers =
      create_attribute_providers_for_physics<true>();
  static const ComponentAttributeProviders custom_data_providers =
      create_attribute_providers_for_physics_custom_data();
  static const ComponentAttributeProviders world_data_providers =
      create_attribute_providers_for_physics<false>();
  BLI_assert(enable_custom_data || enable_world_data);
  AttributeAccessorFunctions fn =
      (enable_world_data ?
           (enable_custom_data ?
                attribute_accessor_functions::accessor_functions_for_providers<providers>() :
                attribute_accessor_functions::accessor_functions_for_providers<
                    world_data_providers>()) :
           attribute_accessor_functions::accessor_functions_for_providers<
               custom_data_providers>());
  fn.domain_size = [](const void *owner, const AttrDomain domain) {
    if (owner == nullptr) {
      return 0;
    }
    const PhysicsGeometryImpl &impl = *static_cast<const PhysicsGeometryImpl *>(owner);
    switch (domain) {
      case AttrDomain::Point:
        return impl.body_num_;
      case AttrDomain::Edge:
        return impl.constraint_num_;
      case blender::bke::AttrDomain::Instance:
        return int(impl.shapes.size());
      default:
        return 0;
    }
  };
  fn.domain_supported = [](const void * /*owner*/, const AttrDomain domain) {
    return ELEM(domain, AttrDomain::Point, AttrDomain::Edge, AttrDomain::Instance);
  };
  fn.adapt_domain = [](const void *owner,
                       const GVArray &varray,
                       const AttrDomain from_domain,
                       const AttrDomain to_domain) -> GVArray {
    if (owner == nullptr) {
      return {};
    }
    const PhysicsGeometryImpl &impl = *static_cast<const PhysicsGeometryImpl *>(owner);
    return adapt_physics_attribute_domain(impl, varray, from_domain, to_domain);
  };
  return fn;
}

const AttributeAccessorFunctions &get_physics_accessor_functions_ref()
{
  static const AttributeAccessorFunctions fn = get_physics_accessor_functions(true, true);
  return fn;
}

const AttributeAccessorFunctions &get_physics_custom_data_accessor_functions_ref()
{
  static const AttributeAccessorFunctions fn = get_physics_accessor_functions(true, false);
  return fn;
}

const AttributeAccessorFunctions &get_physics_world_data_accessor_functions_ref()
{
  static const AttributeAccessorFunctions fn = get_physics_accessor_functions(false, true);
  return fn;
}

/** \} */

#endif

}  // namespace blender::bke
