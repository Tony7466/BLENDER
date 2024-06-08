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

void CollisionShapeImpl::destroy()
{
  delete &this->as_bullet_shape();
}

//CollisionShape::CollisionShape() : impl_(nullptr) {}

CollisionShape::CollisionShape(CollisionShapeImpl *impl) : impl_(impl)
{
  /* Set the internal pointer, this allows finding the CollisionShape from the Bullet shape. */
  impl_->as_bullet_shape().setUserPointer(this);
}

CollisionShape::~CollisionShape()
{
  impl_->destroy();
}

void CollisionShape::delete_self()
{
  delete this;
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
    case EMPTY_SHAPE_PROXYTYPE:
      return ShapeType::Empty;
    case BOX_SHAPE_PROXYTYPE:
      return ShapeType::Box;
    case TRIANGLE_SHAPE_PROXYTYPE:
      return ShapeType::Triangle;
    case TETRAHEDRAL_SHAPE_PROXYTYPE:
      return ShapeType::Tetrahedral;
    case CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE:
      return ShapeType::TriangleMesh;
    case CONVEX_HULL_SHAPE_PROXYTYPE:
      return ShapeType::ConvexHull;
    case CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE:
      return ShapeType::ConvexPointCloud;
    // case CUSTOM_POLYHEDRAL_SHAPE_TYPE:
    //   return ShapeType::CustomPolyhedral;
    case SPHERE_SHAPE_PROXYTYPE:
      return ShapeType::Sphere;
    case MULTI_SPHERE_SHAPE_PROXYTYPE:
      return ShapeType::MultiSphere;
    case CAPSULE_SHAPE_PROXYTYPE:
      return ShapeType::Capsule;
    case CONE_SHAPE_PROXYTYPE:
      return ShapeType::Cone;
    case CONVEX_SHAPE_PROXYTYPE:
      return ShapeType::Convex;
    case CYLINDER_SHAPE_PROXYTYPE:
      return ShapeType::Cylinder;
    case UNIFORM_SCALING_SHAPE_PROXYTYPE:
      return ShapeType::UniformScaling;
    case MINKOWSKI_SUM_SHAPE_PROXYTYPE:
      return ShapeType::MinkowskiSum;
    case MINKOWSKI_DIFFERENCE_SHAPE_PROXYTYPE:
      return ShapeType::MinkowskiDifference;
    case BOX_2D_SHAPE_PROXYTYPE:
      return ShapeType::Box2D;
    case CONVEX_2D_SHAPE_PROXYTYPE:
      return ShapeType::Convex2D;
    // case CUSTOM_CONVEX_SHAPE_TYPE:
    //   return ShapeType::CustomConvex;
    case TRIANGLE_MESH_SHAPE_PROXYTYPE:
      return ShapeType::TriangleMesh;
    case SCALED_TRIANGLE_MESH_SHAPE_PROXYTYPE:
      return ShapeType::ScaledTriangleMesh;
    case STATIC_PLANE_PROXYTYPE:
      return ShapeType::StaticPlane;
    // case CUSTOM_CONCAVE_SHAPE_TYPE:
    //   return ShapeType::CustomConcave;
    case COMPOUND_SHAPE_PROXYTYPE:
      return ShapeType::Compound;
    case INVALID_SHAPE_PROXYTYPE:
      return ShapeType::Invalid;

    default:
      return ShapeType::Invalid;
  }
}

EmptyCollisionShape::EmptyCollisionShape()
    : CollisionShape(CollisionShapeImpl::wrap(new btEmptyShape()))
{
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
