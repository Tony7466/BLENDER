/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#pragma once

#include <optional>

#include "BLI_math_matrix_types.hh"

#include "physics_geometry_world_jolt.hh"

namespace blender::bke {

/* -------------------------------------------------------------------- */
/** \name Utility functions for Jolt types
 * \{ */

#ifdef WITH_JOLT

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
/** \name Internal collision shape access
 * \{ */

struct CollisionShapeImpl {
  ~CollisionShapeImpl() = delete;

  void destroy()
  {
#ifdef WITH_JOLT
      this->as_jolt_shape().Release();
#endif
  }

  JPH::Shape &as_jolt_shape()
  {
    return *reinterpret_cast<JPH::Shape *>(this);
  }

  const JPH::Shape &as_jolt_shape() const
  {
    return *reinterpret_cast<const JPH::Shape *>(this);
  }

  static CollisionShapeImpl *wrap(JPH::Shape *shape)
  {
    shape->AddRef();
    return reinterpret_cast<CollisionShapeImpl *>(shape);
  }

  static const CollisionShapeImpl *wrap(const JPH::Shape *shape)
  {
    shape->AddRef();
    return reinterpret_cast<const CollisionShapeImpl *>(shape);
  }
};

/** \} */

}  // namespace blender::bke
