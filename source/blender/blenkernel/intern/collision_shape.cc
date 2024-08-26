/* SPDX-FileCopyrightText: 2004-2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sim
 */

#include "BLI_array_utils.hh"
#include "BLI_math_matrix.hh"
#include "BLI_math_vector.hh"

#include "BKE_collision_shape.hh"
#include "BKE_mesh.hh"

#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"

#include "physics_geometry_intern.hh"

#ifdef WITH_BULLET
#  include <BulletCollision/CollisionShapes/btEmptyShape.h>
#  include <BulletCollision/CollisionShapes/btTriangleShape.h>
#  include <LinearMath/btDefaultMotionState.h>
#  include <btBulletCollisionCommon.h>
#endif

namespace blender::bke {

#ifdef WITH_BULLET

void CollisionShapeImpl::destroy()
{
  delete &this->as_bullet_shape();
}

// CollisionShape::CollisionShape() : impl_(nullptr) {}

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

StringRef CollisionShape::type_name(const CollisionShape::ShapeType type)
{
  switch (type) {
    case ShapeType::Invalid:
      return "Invalid";
    case ShapeType::Empty:
      return "Empty";
    case ShapeType::Box:
      return "Box";
    case ShapeType::Triangle:
      return "Triangle";
    case ShapeType::Tetrahedral:
      return "Tetrahedral";
    case ShapeType::ConvexTriangleMesh:
      return "Convex Triangle Mesh";
    case ShapeType::ConvexHull:
      return "Convex Hull";
    case ShapeType::ConvexPointCloud:
      return "Convex Point Cloud";
    // case ShapeType::CustomPolyhedral:
    //   return "CustomPolyhedral";
    case ShapeType::Sphere:
      return "Sphere";
    case ShapeType::MultiSphere:
      return "Multi Sphere";
    case ShapeType::Capsule:
      return "Capsule";
    case ShapeType::Cone:
      return "Cone";
    case ShapeType::Cylinder:
      return "Cylinder";
    case ShapeType::UniformScaling:
      return "Uniform Scaling";
    case ShapeType::MinkowskiSum:
      return "Minkowski Sum";
    case ShapeType::MinkowskiDifference:
      return "Minkowski Difference";
    case ShapeType::Box2D:
      return "Box 2D";
    case ShapeType::Convex2D:
      return "Convex 2D";
    // case ShapeType::CustomConvex:
    //   return "CustomConvex";
    case ShapeType::TriangleMesh:
      return "Triangle Mesh";
    case ShapeType::ScaledTriangleMesh:
      return "Scaled Triangle Mesh";
    case ShapeType::StaticPlane:
      return "Static Plane";
    // case ShapeType::CustomConcave:
    //   return "CustomConcave";
    case ShapeType::Compound:
      return "Compound";
  }
  BLI_assert_unreachable();
  return "";
}

bool CollisionShape::supports_motion() const
{
  return !impl_->as_bullet_shape().isNonMoving();
}

bool CollisionShape::is_convex() const
{
  return impl_->as_bullet_shape().isConvex();
}

bool CollisionShape::is_concave() const
{
  return impl_->as_bullet_shape().isConcave();
}

float3 CollisionShape::calculate_local_inertia(const float mass) const
{
  const btCollisionShape &bt_shape = impl_->as_bullet_shape();
  /* Bullet crashes when calling this for "Empty" shapes. */
  if (dynamic_cast<const btEmptyShape *>(&bt_shape) != nullptr) {
    return float3(0.0f);
  }

  btVector3 bt_local_inertia;
  bt_shape.calculateLocalInertia(mass, bt_local_inertia);
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

static btConvexHullShape *make_convex_hull_shape(const VArray<float3> &points)
{
  if (points.is_empty()) {
    return new btConvexHullShape(nullptr, 0);
  }

  Array<btVector3> bt_points(points.size());
  for (const int i : bt_points.index_range()) {
    bt_points[i] = to_bullet(points[i]);
  }
  btConvexHullShape *shape = new btConvexHullShape(&bt_points.data()->x(), bt_points.size());
  shape->recalcLocalAabb();
  return shape;
}

ConvexHullCollisionShape::ConvexHullCollisionShape(const VArray<float3> &points)
    : CollisionShape(CollisionShapeImpl::wrap(make_convex_hull_shape(points)))
{
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
    : CollisionShape(
          CollisionShapeImpl::wrap(new btCylinderShapeZ(btVector3(radius, radius, height * 0.5f))))
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

UniformScalingCollisionShape::UniformScalingCollisionShape(const CollisionShapePtr &child_shape,
                                                           const float scale)
    : CollisionShape(
          CollisionShapeImpl::wrap(make_uniform_scaling_or_empty_shape(child_shape.get(), scale))),
      child_shape_(child_shape)
{
}

struct TriangleMeshInterface {
  Array<btVector3> positions;
  Array<int> indices;
  btTriangleIndexVertexArray bt_mesh_interface;

  TriangleMeshInterface(const Mesh &mesh)
  {
    /* This is assuming the mesh is triangulated. If a face has more than 3 vertices
     * it will create a random (but not invalid) mesh. */
    BLI_assert(mesh.corners_num == 3 * mesh.faces_num);
    const int num_triangles = mesh.faces_num;
    const int num_vertices = mesh.verts_num;
    /* Vertex index for each corner. In a triangle mesh each face has 3 consecutive corners. */
    this->indices = mesh.corner_verts();

    const Span<float3> src_positions = mesh.vert_positions();
    this->positions.reinitialize(num_vertices);
    for (const int i : this->positions.index_range()) {
      this->positions[i][0] = src_positions[i].x;
      this->positions[i][1] = src_positions[i].y;
      this->positions[i][2] = src_positions[i].z;
      this->positions[i][3] = 0.0f;
    }
    btScalar *bt_position_ptr = const_cast<btScalar *>(&this->positions.data()->x());

    bt_mesh_interface = btTriangleIndexVertexArray(num_triangles,
                                                   const_cast<int *>(this->indices.data()),
                                                   3 * sizeof(int),
                                                   num_vertices,
                                                   bt_position_ptr,
                                                   sizeof(btVector3));
  }
};

static btCollisionShape *make_triangle_mesh_shape(TriangleMeshInterface *mesh_interface)
{
  constexpr const bool use_quantized_aabb_compression = true;
  constexpr const bool build_bvh = true;

  if (mesh_interface == nullptr || mesh_interface->indices.is_empty()) {
    return new btEmptyShape();
  }

  return new btBvhTriangleMeshShape(
      &mesh_interface->bt_mesh_interface, use_quantized_aabb_compression, build_bvh);
}

TriangleMeshCollisionShape::TriangleMeshCollisionShape(TriangleMeshInterface *mesh_interface)
    : CollisionShape(CollisionShapeImpl::wrap(make_triangle_mesh_shape(mesh_interface))),
      mesh_interface(mesh_interface)
{
}

TriangleMeshCollisionShape::~TriangleMeshCollisionShape()
{
  delete this->mesh_interface;
}

TriangleMeshCollisionShape *TriangleMeshCollisionShape::from_mesh(const Mesh &mesh)
{
  TriangleMeshInterface *mesh_interface = (mesh.corners_num == 3 * mesh.faces_num ?
                                               new TriangleMeshInterface(mesh) :
                                               nullptr);
  return new TriangleMeshCollisionShape(mesh_interface);
}

static btScaledBvhTriangleMeshShape *make_scaled_triangle_mesh_shape(
    const TriangleMeshCollisionShape *child_shape, const float3 scale)
{
  BLI_assert(child_shape != nullptr);
  BLI_assert(child_shape->impl().as_bullet_shape().getShapeType() ==
             TRIANGLE_MESH_SHAPE_PROXYTYPE);
  BLI_assert(dynamic_cast<const btBvhTriangleMeshShape *>(&child_shape->impl().as_bullet_shape()));

  const btBvhTriangleMeshShape *triangle_mesh_child_shape =
      static_cast<const btBvhTriangleMeshShape *>(&child_shape->impl().as_bullet_shape());
  return new btScaledBvhTriangleMeshShape(
      const_cast<btBvhTriangleMeshShape *>(triangle_mesh_child_shape), to_bullet(scale));
}

ScaledTriangleMeshCollisionShape::ScaledTriangleMeshCollisionShape(
    const TriangleMeshCollisionShape *child_shape, const float3 scale)
    : CollisionShape(CollisionShapeImpl::wrap(make_scaled_triangle_mesh_shape(child_shape, scale)))
{
}

static btStaticPlaneShape *make_static_plane_shape(const float3 &plane_normal,
                                                   const float plane_constant)
{
  const float3 normalized_normal = math::normalize(plane_normal);
  /* Bullet requires normalized non-zero vectors. */
  const float3 safe_normal = math::is_zero(normalized_normal) ? float3(0, 0, 1) :
                                                                normalized_normal;
  return new btStaticPlaneShape(to_bullet(safe_normal), plane_constant);
}

StaticPlaneCollisionShape::StaticPlaneCollisionShape(const float3 &plane_normal,
                                                     const float plane_constant)
    : CollisionShape(
          CollisionShapeImpl::wrap(make_static_plane_shape(plane_normal, plane_constant)))
{
}

static btCompoundShape *make_compound_shape(VArray<CollisionShapePtr> child_shapes,
                                            VArray<float4x4> child_transforms)
{
  BLI_assert(child_shapes.size() == child_transforms.size());
  btCompoundShape *shape = new btCompoundShape(true, child_shapes.size());
  for (const int i : child_shapes.index_range()) {
    const btCollisionShape *bt_child_shape = &child_shapes[i]->impl().as_bullet_shape();
    shape->addChildShape(to_bullet(child_transforms[i]),
                         const_cast<btCollisionShape *>(bt_child_shape));
  }
  return shape;
}

CompoundCollisionShape::CompoundCollisionShape(const VArray<CollisionShapePtr> &child_shapes,
                                               const VArray<float4x4> &child_transforms)
    : CollisionShape(CollisionShapeImpl::wrap(make_compound_shape(child_shapes, child_transforms)))
{
  child_shapes_.reinitialize(child_shapes.size());
  child_shapes.materialize_to_uninitialized(child_shapes_);
}

#else

#endif

}  // namespace blender::bke
