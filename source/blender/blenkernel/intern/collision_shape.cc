/* SPDX-FileCopyrightText: 2004-2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sim
 */

#include "BLI_array_utils.hh"
#include "BLI_math_matrix.hh"

#include "BKE_collision_shape.hh"
#include "BKE_mesh.hh"

#include "DNA_meshdata_types.h"

#include "physics_geometry_impl.hh"

#ifdef WITH_BULLET
#  include <LinearMath/btDefaultMotionState.h>
#  include <btBulletCollisionCommon.h>
#  include <BulletCollision/CollisionShapes/btTriangleShape.h>
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

inline float3x3 to_blender(const btMatrix3x3 &t)
{
  return math::transpose(
      float3x3(to_blender(t.getRow(0)), to_blender(t.getRow(1)), to_blender(t.getRow(2))));
}

inline float4x4 to_bullet(const btTransform &t)
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
  return btTransform(to_bullet(t.view<3,3>()), to_bullet(t.location()));
}

static std::optional<CollisionShape::ShapeType> to_blender(const int bt_shape_type)
{
  using ShapeType = CollisionShape::ShapeType;
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
      return std::nullopt;
  }
  return std::nullopt;
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
  const int bt_shape_type = BroadphaseNativeTypes(impl_->as_bullet_shape().getShapeType());
  std::optional<bke::CollisionShape::ShapeType> shape_type = to_blender(bt_shape_type);
  return shape_type ? *shape_type : bke::CollisionShape::ShapeType::Invalid;
}

bool CollisionShape::is_convex() const
{
  return impl_->as_bullet_shape().isConvex();
}

bool CollisionShape::is_concave() const
{
  return impl_->as_bullet_shape().isConcave();
}

float3 CollisionShape::calculate_local_inertia(const float mass) const {
  btVector3 bt_local_inertia;
  impl_->as_bullet_shape().calculateLocalInertia(mass, bt_local_inertia);
  return to_blender(bt_local_inertia);
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

TriangleCollisionShape::TriangleCollisionShape(const float3 &pt0,
                                               const float3 &pt1,
                                               const float3 &pt2)
    : CollisionShape(CollisionShapeImpl::wrap(
          new btTriangleShape(to_bullet(pt0), to_bullet(pt1), to_bullet(pt2))))
{
}

ConvexHullCollisionShape::ConvexHullCollisionShape(const VArray<float3> &points)
    : CollisionShape(CollisionShapeImpl::wrap(new btConvexHullShape(nullptr, points.size())))
{
  /* Constructor only allocates points, data filled in here. */
  btConvexHullShape &impl = static_cast<btConvexHullShape &>(this->impl_->as_bullet_shape());
  MutableSpan<btVector3> bt_points = {impl.getUnscaledPoints(), impl.getNumPoints()};
  for (const int i : bt_points.index_range()) {
    bt_points[i] = to_bullet(points[i]);
  }
  impl.recalcLocalAabb();
}

SphereCollisionShape::SphereCollisionShape(const float radius)
    : CollisionShape(CollisionShapeImpl::wrap(new btSphereShape(radius)))
{
}

float SphereCollisionShape::radius() const
{
  return static_cast<const btSphereShape &>(impl_->as_bullet_shape()).getRadius();
}

CapsuleCollisionShape::CapsuleCollisionShape(const float radius, const float height)
    : CollisionShape(CollisionShapeImpl::wrap(new btCapsuleShapeZ(radius, height)))
{
}

ConeCollisionShape::ConeCollisionShape(const float radius, const float height)
    : CollisionShape(CollisionShapeImpl::wrap(new btConeShapeZ(radius, height)))
{
}

CylinderCollisionShape::CylinderCollisionShape(const float radius, const float height)
    : CollisionShape(CollisionShapeImpl::wrap(new btCylinderShapeZ(btVector3(radius, radius, height*0.5f))))
{
}

static btCollisionShape *make_uniform_scaling_or_empty_shape(const CollisionShape *child_shape,
                                                             const float scale)
{
  BLI_assert(child_shape != nullptr);
  BLI_assert(child_shape->is_convex());
  BLI_assert(dynamic_cast<const btConvexShape *>(&child_shape->impl().as_bullet_shape()));
  if (!child_shape->is_convex()) {
    return new btEmptyShape();
  }
  const btConvexShape *convex_child_shape = static_cast<const btConvexShape *>(
      &child_shape->impl().as_bullet_shape());
  return new btUniformScalingShape(const_cast<btConvexShape *>(convex_child_shape), scale);
}

UniformScalingCollisionShape::UniformScalingCollisionShape(const CollisionShape *child_shape,
                                                           const float scale)
    : CollisionShape(
          CollisionShapeImpl::wrap(make_uniform_scaling_or_empty_shape(child_shape, scale)))
{
}

struct TriangleMeshInterface {
  btTriangleIndexVertexArray bt_mesh_interface;

  TriangleMeshInterface(const Mesh &mesh)
  {
    /* This is assuming the mesh is triangulated. If a face has more than 3 vertices
     * it will create a random (but not invalid) mesh. */
    BLI_assert(mesh.corners_num == 3 * mesh.faces_num);
    const int num_triangles = mesh.faces_num;
    const int num_vertices = mesh.verts_num;
    /* Vertex index for each corner. In a triangle mesh each face has 3 consecutive corners. */
    const Span<int> triangles = mesh.corner_verts();
    int *triangles_ptr = const_cast<int *>(triangles.data());

    const Span<float3> src_positions = mesh.vert_positions();
    Array<btScalar[3]> dst_positions(num_vertices);
    for (const int i : dst_positions.index_range()) {
      dst_positions[i][0] = src_positions[i].x;
      dst_positions[i][1] = src_positions[i].y;
      dst_positions[i][2] = src_positions[i].z;
    }
    btScalar *dst_position_ptr = const_cast<btScalar *>(dst_positions.data()[0]);

    bt_mesh_interface = btTriangleIndexVertexArray(num_triangles,
                                                   triangles_ptr,
                                                   3 * sizeof(int),
                                                   num_vertices,
                                                   dst_position_ptr,
                                                   sizeof(btScalar[3]));
  }
};

static btCollisionShape *make_triangle_mesh_shape(const Mesh &mesh)
{
  constexpr const bool use_quantized_aabb_compression = true;
  constexpr const bool build_bvh = true;
  if (mesh.corners_num != 3 * mesh.faces_num) {
    return new btEmptyShape();
  }

  TriangleMeshInterface mesh_interface(mesh);
  return new btBvhTriangleMeshShape(
      &mesh_interface.bt_mesh_interface, use_quantized_aabb_compression, build_bvh);
}

TriangleMeshCollisionShape::TriangleMeshCollisionShape(const Mesh &mesh)
    : CollisionShape(CollisionShapeImpl::wrap(make_triangle_mesh_shape(mesh)))
{
}

static btScaledBvhTriangleMeshShape *make_scaled_triangle_mesh_shape(
    const TriangleMeshCollisionShape *child_shape, const float3 scale)
{
  BLI_assert(child_shape != nullptr);
  BLI_assert(child_shape->impl().as_bullet_shape().getShapeType() ==
             TRIANGLE_MESH_SHAPE_PROXYTYPE);
  BLI_assert(dynamic_cast<const btBvhTriangleMeshShape *>(&child_shape->impl().as_bullet_shape()));

  const btBvhTriangleMeshShape *triangle_mesh_child_shape =
      static_cast<const btBvhTriangleMeshShape *>(
      &child_shape->impl().as_bullet_shape());
  return new btScaledBvhTriangleMeshShape(
      const_cast<btBvhTriangleMeshShape *>(triangle_mesh_child_shape), to_bullet(scale));
}

ScaledTriangleMeshCollisionShape::ScaledTriangleMeshCollisionShape(
    const TriangleMeshCollisionShape *child_shape, const float3 scale)
    : CollisionShape(CollisionShapeImpl::wrap(make_scaled_triangle_mesh_shape(child_shape, scale)))
{
}

StaticPlaneCollisionShape::StaticPlaneCollisionShape(const float3 &plane_normal,
                                                     const float plane_constant)
    : CollisionShape(CollisionShapeImpl::wrap(
          new btStaticPlaneShape(to_bullet(plane_normal), plane_constant)))
{
}

static btCompoundShape *make_compound_shape(VArray<const CollisionShape *> child_shapes,
                                            VArray<float4x4> child_transforms)
{
  btCompoundShape *shape = new btCompoundShape(true, child_shapes.size());
  for (const int i : child_shapes.index_range()) {
    const btCollisionShape *bt_child_shape = &child_shapes[i]->impl().as_bullet_shape();
    shape->addChildShape(to_bullet(child_transforms[i]),
                         const_cast<btCollisionShape *>(bt_child_shape));
  }
  return shape;
}

CompoundCollisionShape::CompoundCollisionShape(const VArray<const CollisionShape *> &child_shapes,
                                               const VArray<float4x4> &child_transforms)
    : CollisionShape(CollisionShapeImpl::wrap(make_compound_shape(child_shapes, child_transforms)))
{
}

#else

#endif

}  // namespace blender::bke
