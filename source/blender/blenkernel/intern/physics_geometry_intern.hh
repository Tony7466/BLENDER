/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#pragma once

#include <optional>

#include "BLI_math_matrix_types.hh"

#include "physics_geometry_world_bullet.hh"
#include "physics_geometry_world_jolt.hh"

#ifdef WITH_BULLET
#  include <btBulletCollisionCommon.h>
#  include <btBulletDynamicsCommon.h>
#endif

namespace blender::bke {

static constexpr bool use_jolt = true;

/* -------------------------------------------------------------------- */
/** \name Utility functions for Bullet types
 * \{ */

#ifdef WITH_BULLET

inline float to_blender(const btScalar v)
{
  return float(v);
}

inline float3 to_blender(const btVector3 &v)
{
  return float3(v.x(), v.y(), v.z());
}

inline math::Quaternion to_blender(const btQuaternion &q)
{
  return math::Quaternion(q.w(), q.x(), q.y(), q.z());
}

inline float3x3 to_blender(const btMatrix3x3 &t)
{
  return math::transpose(
      float3x3(to_blender(t.getRow(0)), to_blender(t.getRow(1)), to_blender(t.getRow(2))));
}

inline float4x4 to_blender(const btTransform &t)
{
  float3x3 rot = to_blender(t.getBasis());
  return float4x4(float4(rot.x_axis(), 0),
                  float4(rot.y_axis(), 0),
                  float4(rot.z_axis(), 0),
                  float4(to_blender(t.getOrigin()), 1));
}

inline btScalar to_bullet(const float v)
{
  return btScalar(v);
}

inline btVector3 to_bullet(const float3 &v)
{
  return btVector3(v.x, v.y, v.z);
}

inline btQuaternion to_bullet(const math::Quaternion &q)
{
  return btQuaternion(q.x, q.y, q.z, q.w);
}

inline btMatrix3x3 to_bullet(const float3x3 &t)
{
  return btMatrix3x3(to_bullet(t.x_axis()), to_bullet(t.y_axis()), to_bullet(t.z_axis()))
      .transpose();
}

inline btTransform to_bullet(const float4x4 &t)
{
  return btTransform(to_bullet(t.view<3, 3>()), to_bullet(t.location()));
}

inline int activation_state_to_bullet(bke::PhysicsWorldData::BodyActivationState state)
{
  using BodyActivationState = PhysicsBodyActivationState;
  switch (state) {
    case BodyActivationState::AlwaysActive:
      return DISABLE_DEACTIVATION;
    case BodyActivationState::Active:
      return ACTIVE_TAG;
    case BodyActivationState::WantsSleep:
      return WANTS_DEACTIVATION;
    case BodyActivationState::Sleeping:
      return ISLAND_SLEEPING;
    case BodyActivationState::AlwaysSleeping:
      return DISABLE_SIMULATION;
  }
  return DISABLE_SIMULATION;
}

inline bke::PhysicsWorldData::BodyActivationState activation_state_to_blender(int bt_state)
{
  using BodyActivationState = PhysicsBodyActivationState;
  switch (bt_state) {
    case ACTIVE_TAG:
      return BodyActivationState::Active;
    case ISLAND_SLEEPING:
      return BodyActivationState::Sleeping;
    case WANTS_DEACTIVATION:
      return BodyActivationState::WantsSleep;
    case DISABLE_DEACTIVATION:
      return BodyActivationState::AlwaysActive;
    case DISABLE_SIMULATION:
      return BodyActivationState::AlwaysSleeping;
    default:
      BLI_assert_unreachable();
      return BodyActivationState::AlwaysSleeping;
  }
}

// inline btTypedConstraintType to_bullet(bke::PhysicsGeometry::ConstraintType type)
// {
//   using ConstraintType = bke::PhysicsGeometry::ConstraintType;
//   switch (type) {
//     case ConstraintType::None:
//       return FIXED_CONSTRAINT_TYPE;
//     case ConstraintType::Fixed:
//       return FIXED_CONSTRAINT_TYPE;
//     case ConstraintType::Point:
//       return POINT2POINT_CONSTRAINT_TYPE;
//     case ConstraintType::Hinge:
//       return HINGE_CONSTRAINT_TYPE;
//     case ConstraintType::Slider:
//       return SLIDER_CONSTRAINT_TYPE;
//     case ConstraintType::ConeTwist:
//       return CONETWIST_CONSTRAINT_TYPE;
//     case ConstraintType::SixDoF:
//       return D6_CONSTRAINT_TYPE;
//     case ConstraintType::SixDoFSpring:
//       return D6_SPRING_CONSTRAINT_TYPE;
//     case ConstraintType::SixDoFSpring2:
//       return D6_SPRING_2_CONSTRAINT_TYPE;
//     case ConstraintType::Contact:
//       return CONTACT_CONSTRAINT_TYPE;
//     case ConstraintType::Gear:
//       return GEAR_CONSTRAINT_TYPE;
//   }
//   return FIXED_CONSTRAINT_TYPE;
// }

// inline bke::PhysicsGeometry::ConstraintType to_blender(btTypedConstraintType type)
//{
//   using ConstraintType = bke::PhysicsGeometry::ConstraintType;
//   switch (type) {
//     case FIXED_CONSTRAINT_TYPE:
//       return ConstraintType::Fixed;
//     case POINT2POINT_CONSTRAINT_TYPE:
//       return ConstraintType::Point;
//     case HINGE_CONSTRAINT_TYPE:
//       return ConstraintType::Hinge;
//     case SLIDER_CONSTRAINT_TYPE:
//       return ConstraintType::Slider;
//     case CONETWIST_CONSTRAINT_TYPE:
//       return ConstraintType::ConeTwist;
//     case D6_CONSTRAINT_TYPE:
//       return ConstraintType::SixDoF;
//     case D6_SPRING_CONSTRAINT_TYPE:
//       return ConstraintType::SixDoFSpring;
//     case D6_SPRING_2_CONSTRAINT_TYPE:
//       return ConstraintType::SixDoFSpring2;
//     case CONTACT_CONSTRAINT_TYPE:
//       return ConstraintType::Contact;
//     case GEAR_CONSTRAINT_TYPE:
//       return ConstraintType::Gear;
//
//     default:
//       return ConstraintType::None;
//   }
// }

inline CollisionShape::ShapeType to_blender(const int /*bt_shape_type*/)
{
  using ShapeType = CollisionShape::ShapeType;

  BLI_assert_unreachable();
  return ShapeType::Box;

  // switch (bt_shape_type) {
  //   case EMPTY_SHAPE_PROXYTYPE:
  //     return ShapeType::Empty;
  //   case BOX_SHAPE_PROXYTYPE:
  //     return ShapeType::Box;
  //   case TRIANGLE_SHAPE_PROXYTYPE:
  //     return ShapeType::Triangle;
  //   case TETRAHEDRAL_SHAPE_PROXYTYPE:
  //     return ShapeType::Tetrahedral;
  //   case CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE:
  //     return ShapeType::TriangleMesh;
  //   case CONVEX_HULL_SHAPE_PROXYTYPE:
  //     return ShapeType::ConvexHull;
  //   case CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE:
  //     return ShapeType::ConvexPointCloud;
  //   // case CUSTOM_POLYHEDRAL_SHAPE_TYPE:
  //   //   return ShapeType::CustomPolyhedral;
  //   case SPHERE_SHAPE_PROXYTYPE:
  //     return ShapeType::Sphere;
  //   case MULTI_SPHERE_SHAPE_PROXYTYPE:
  //     return ShapeType::MultiSphere;
  //   case CAPSULE_SHAPE_PROXYTYPE:
  //     return ShapeType::Capsule;
  //   case CONE_SHAPE_PROXYTYPE:
  //     return ShapeType::Cone;
  //   case CYLINDER_SHAPE_PROXYTYPE:
  //     return ShapeType::Cylinder;
  //   case UNIFORM_SCALING_SHAPE_PROXYTYPE:
  //     return ShapeType::UniformScaling;
  //   case MINKOWSKI_SUM_SHAPE_PROXYTYPE:
  //     return ShapeType::MinkowskiSum;
  //   case MINKOWSKI_DIFFERENCE_SHAPE_PROXYTYPE:
  //     return ShapeType::MinkowskiDifference;
  //   case BOX_2D_SHAPE_PROXYTYPE:
  //     return ShapeType::Box2D;
  //   case CONVEX_2D_SHAPE_PROXYTYPE:
  //     return ShapeType::Convex2D;
  //   // case CUSTOM_CONVEX_SHAPE_TYPE:
  //   //   return ShapeType::CustomConvex;
  //   case TRIANGLE_MESH_SHAPE_PROXYTYPE:
  //     return ShapeType::TriangleMesh;
  //   case SCALED_TRIANGLE_MESH_SHAPE_PROXYTYPE:
  //     return ShapeType::ScaledTriangleMesh;
  //   case STATIC_PLANE_PROXYTYPE:
  //     return ShapeType::StaticPlane;
  //   // case CUSTOM_CONCAVE_SHAPE_TYPE:
  //   //   return ShapeType::CustomConcave;
  //   case COMPOUND_SHAPE_PROXYTYPE:
  //     return ShapeType::Compound;
  //   case INVALID_SHAPE_PROXYTYPE:
  //     return ShapeType::Invalid;

  //  default:
  //    return std::nullopt;
  //}
  // return std::nullopt;
}

inline PhysicsMotionType to_blender(const JPH::EMotionType motion_type)
{
  switch (motion_type) {
    case JPH::EMotionType::Dynamic:
      return PhysicsMotionType::Dynamic;
    case JPH::EMotionType::Static:
      return PhysicsMotionType::Static;
    case JPH::EMotionType::Kinematic:
      return PhysicsMotionType::Kinematic;
  }
  BLI_assert_unreachable();
  return PhysicsMotionType::Dynamic;
}

inline CollisionShape::ShapeType to_blender(const JPH::EShapeSubType subtype)
{
  using ShapeType = CollisionShape::ShapeType;
  switch (subtype) {
    case JPH::EShapeSubType::Sphere:
      return ShapeType::Sphere;
    case JPH::EShapeSubType::Box:
      return ShapeType::Box;
    case JPH::EShapeSubType::Triangle:
      return ShapeType::Triangle;
    case JPH::EShapeSubType::Capsule:
      return ShapeType::Capsule;
    case JPH::EShapeSubType::TaperedCapsule:
      return ShapeType::TaperedCapsule;
    case JPH::EShapeSubType::Cylinder:
      return ShapeType::Cylinder;
    case JPH::EShapeSubType::ConvexHull:
      return ShapeType::ConvexHull;
    case JPH::EShapeSubType::StaticCompound:
      return ShapeType::StaticCompound;
    case JPH::EShapeSubType::MutableCompound:
      return ShapeType::MutableCompound;
    case JPH::EShapeSubType::RotatedTranslated:
      return ShapeType::RotatedTranslated;
    case JPH::EShapeSubType::Scaled:
      return ShapeType::Scaled;
    case JPH::EShapeSubType::OffsetCenterOfMass:
      return ShapeType::OffsetCenterOfMass;
    case JPH::EShapeSubType::Mesh:
      return ShapeType::Mesh;
    case JPH::EShapeSubType::HeightField:
      return ShapeType::HeightField;
    case JPH::EShapeSubType::SoftBody:
      return ShapeType::SoftBody;

    default:
      BLI_assert(subtype >= JPH::EShapeSubType::User1);
      return ShapeType::Sphere;
  }
  BLI_assert_unreachable();
  return ShapeType::Sphere;
}

#endif

#ifdef WITH_JOLT

inline float to_blender(const JPH::Real v)
{
  return float(v);
}

inline float3 to_blender(const JPH::Vector<3> &v)
{
  return float3(v[0], v[1], v[2]);
}

inline float3 to_blender(const JPH::Vec3 &v)
{
  return float3(v[0], v[1], v[2]);
}

inline float3 to_blender(const JPH::DVec3 &v)
{
  return float3(v[0], v[1], v[2]);
}

inline float3 to_blender(const JPH::Float3 &v)
{
  return float3(v[0], v[1], v[2]);
}

inline math::Quaternion to_blender(const JPH::Quat &q)
{
  return math::Quaternion(q.GetW(), q.GetX(), q.GetY(), q.GetZ());
}

inline float3x3 to_blender(const JPH::Matrix<3, 3> &t)
{
  return float3x3(
      to_blender(t.GetColumn(0)), to_blender(t.GetColumn(1)), to_blender(t.GetColumn(2)));
}

inline float4x4 to_blender(const JPH::Mat44 &t)
{
  return float4x4(float4(to_blender(t.GetAxisX()), 0),
                  float4(to_blender(t.GetAxisY()), 0),
                  float4(to_blender(t.GetAxisZ()), 0),
                  float4(to_blender(t.GetTranslation()), 1));
}

inline Bounds<float3> to_blender(const JPH::AABox &box)
{
  return Bounds<float3>(to_blender(box.mMin), to_blender(box.mMax));
}

inline JPH::Real to_jolt(const float v)
{
  return JPH::Real(v);
}

inline JPH::Vec3 to_jolt(const float3 &v)
{
  return JPH::Vec3(v.x, v.y, v.z);
}

inline JPH::Vec4 to_jolt(const float4 &v)
{
  return JPH::Vec4(v.x, v.y, v.z, v.w);
}

inline JPH::Quat to_jolt(const math::Quaternion &q)
{
  return JPH::Quat(q.x, q.y, q.z, q.w);
}

inline JPH::Mat44 to_jolt(const float4x4 &t)
{
  return JPH::Mat44(to_jolt(t.x), to_jolt(t.y), to_jolt(t.z), to_jolt(t.w));
}

inline JPH::EMotionType to_jolt(const PhysicsMotionType motion_type)
{
  switch (motion_type) {
    case PhysicsMotionType::Dynamic:
      return JPH::EMotionType::Dynamic;
    case PhysicsMotionType::Static:
      return JPH::EMotionType::Static;
    case PhysicsMotionType::Kinematic:
      return JPH::EMotionType::Kinematic;
  }
  BLI_assert_unreachable();
  return JPH::EMotionType::Dynamic;
}

inline JPH::AABox to_jolt(const Bounds<float3> &bounds)
{
  return JPH::AABox(to_jolt(bounds.min), to_jolt(bounds.max));
}

#endif

/** \} */

/* -------------------------------------------------------------------- */
/** \name Internal Bullet collision shape access
 * \{ */

struct CollisionShapeImpl {
  ~CollisionShapeImpl() = delete;

  void destroy()
  {
    if (use_jolt) {
#ifdef WITH_JOLT
      this->as_jolt_shape().Release();
#endif
    }
    else {
#ifdef WITH_BULLET
      delete &this->as_bullet_shape();
#endif
    }
  }

  btCollisionShape &as_bullet_shape()
  {
    BLI_assert(!use_jolt);
    return *reinterpret_cast<btCollisionShape *>(this);
  }

  const btCollisionShape &as_bullet_shape() const
  {
    BLI_assert(!use_jolt);
    return *reinterpret_cast<const btCollisionShape *>(this);
  }

  JPH::Shape &as_jolt_shape()
  {
    BLI_assert(use_jolt);
    return *reinterpret_cast<JPH::Shape *>(this);
  }

  const JPH::Shape &as_jolt_shape() const
  {
    BLI_assert(use_jolt);
    return *reinterpret_cast<const JPH::Shape *>(this);
  }

  static CollisionShapeImpl *wrap(btCollisionShape *shape)
  {
    BLI_assert(!use_jolt);
    return reinterpret_cast<CollisionShapeImpl *>(shape);
  }

  static const CollisionShapeImpl *wrap(const btCollisionShape *shape)
  {
    BLI_assert(!use_jolt);
    return reinterpret_cast<const CollisionShapeImpl *>(shape);
  }

  static CollisionShapeImpl *wrap(JPH::Shape *shape)
  {
    BLI_assert(use_jolt);
    shape->AddRef();
    return reinterpret_cast<CollisionShapeImpl *>(shape);
  }

  static const CollisionShapeImpl *wrap(const JPH::Shape *shape)
  {
    BLI_assert(use_jolt);
    shape->AddRef();
    return reinterpret_cast<const CollisionShapeImpl *>(shape);
  }
};

/** \} */

}  // namespace blender::bke
