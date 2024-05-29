/* SPDX-FileCopyrightText: 2004-2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sim
 */

#include "BKE_collision_shape.hh"

#include "physics_geometry_impl.hh"

#ifdef WITH_BULLET
#  include <LinearMath/btDefaultMotionState.h>
#  include <btBulletCollisionCommon.h>
#endif

namespace blender::bke {

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

CollisionShape::CollisionShape() : impl_(nullptr) {}

CollisionShape::CollisionShape(CollisionShapeImpl *impl) : impl_(impl)
{
  /* Set the internal pointer, this allows finding the CollisionShape from the Bullet shape. */
  impl_->as_bullet_shape().setUserPointer(this);
}

CollisionShape::~CollisionShape()
{
  delete impl_;
}

CollisionShapeImpl &CollisionShape::impl()
{
  return *impl_;
}

const CollisionShapeImpl &CollisionShape::impl() const
{
  return *impl_;
}

CollisionShape::ShapeType CollisionShape::type() const
{
  const auto bt_shape_type = BroadphaseNativeTypes(impl_->as_bullet_shape().getShapeType());
  switch (bt_shape_type) {
    case BOX_SHAPE_PROXYTYPE:
      return ShapeType::Box;
    case SPHERE_SHAPE_PROXYTYPE:
      return ShapeType::Sphere;
    case EMPTY_SHAPE_PROXYTYPE:
    case INVALID_SHAPE_PROXYTYPE:
    default:
      return ShapeType::Unknown;
  }
}

BoxCollisionShape::BoxCollisionShape(const float3 &half_extent)
    : CollisionShape(CollisionShapeImpl::wrap(new btBoxShape(to_bullet(half_extent))))
{
}

float3 BoxCollisionShape::half_extent() const
{
  return to_blender(
      static_cast<const btBoxShape &>(impl_->as_bullet_shape()).getHalfExtentsWithoutMargin());
}

SphereCollisionShape::SphereCollisionShape(const float radius)
    : CollisionShape(CollisionShapeImpl::wrap(new btSphereShape(radius)))
{
}

float SphereCollisionShape::radius() const
{
  return static_cast<const btSphereShape &>(impl_->as_bullet_shape()).getRadius();
}

#else

#endif

}  // namespace blender::bke
