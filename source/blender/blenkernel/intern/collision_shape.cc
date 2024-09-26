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

#ifdef WITH_JOLT
#  include <Jolt/Jolt.h>

#  include <Jolt/Physics/Collision/Shape/BoxShape.h>
#  include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#  include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#  include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#  include <Jolt/Physics/Collision/Shape/MeshShape.h>
#  include <Jolt/Physics/Collision/Shape/MutableCompoundShape.h>
#  include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#  include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#  include <Jolt/Physics/Collision/Shape/ScaledShape.h>
#  include <Jolt/Physics/Collision/Shape/SphereShape.h>
#  include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#  include <Jolt/Physics/Collision/Shape/TaperedCapsuleShape.h>
#  include <Jolt/Physics/Collision/Shape/TriangleShape.h>
#endif

namespace blender::bke {

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

std::optional<StringRef> CollisionShape::error() const
{
  return error_.has_value() ? std::make_optional(error_.value()) : std::nullopt;
}

StringRef CollisionShape::type_name(const CollisionShape::ShapeType type)
{
  switch (type) {
    case ShapeType::Sphere:
      return "Sphere";
    case ShapeType::Box:
      return "Box";
    case ShapeType::Triangle:
      return "Triangle";
    case ShapeType::Capsule:
      return "Capsule";
    case ShapeType::TaperedCapsule:
      return "TaperedCapsule";
    case ShapeType::Cylinder:
      return "Cylinder";
    case ShapeType::ConvexHull:
      return "ConvexHull";
    case ShapeType::StaticCompound:
      return "StaticCompound";
    case ShapeType::MutableCompound:
      return "MutableCompound";
    case ShapeType::RotatedTranslated:
      return "RotatedTranslated";
    case ShapeType::Scaled:
      return "Scaled";
    case ShapeType::OffsetCenterOfMass:
      return "OffsetCenterOfMass";
    case ShapeType::Mesh:
      return "Mesh";
    case ShapeType::HeightField:
      return "HeightField";
    case ShapeType::SoftBody:
      return "SoftBody";
  }
  BLI_assert_unreachable();
  return "";
}

#if 0
#  ifdef WITH_BULLET

CollisionShape::CollisionShape(CollisionShapeImpl *impl) : impl_(impl)
{
  /* Set the internal pointer, this allows finding the CollisionShape from the Bullet shape. */
  impl_->as_bullet_shape().setUserPointer(this);
}

CollisionShape::ShapeType CollisionShape::type() const
{
  const int bt_shape_type = BroadphaseNativeTypes(impl_->as_bullet_shape().getShapeType());
  bke::CollisionShape::ShapeType shape_type = to_blender(bt_shape_type);
  return shape_type;
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

// static CollisionShapeImpl *make_empty_shape()
// {
//   if (use_jolt) {
// #ifdef WITH_JOLT
//     // TODO How to represent an "empty" shape in Jolt? Just dummy shape and disable collision?
//     return CollisionShapeImpl::wrap(new JPH::SphereShape(1.0f));
// #endif
//   }
// #ifdef WITH_BULLET
//   return CollisionShapeImpl::wrap(new btEmptyShape());
// #endif
// }

// EmptyCollisionShape::EmptyCollisionShape() : CollisionShape(make_empty_shape()) {}

static CollisionShapeImpl *make_box_shape(const float3 &half_extent)
{
  return CollisionShapeImpl::wrap(new btBoxShape(to_bullet(half_extent)));
}

BoxCollisionShape::BoxCollisionShape(const float3 &half_extent)
    : CollisionShape(make_box_shape(half_extent))
{
}

float3 BoxCollisionShape::half_extent() const
{
  return to_blender(
      static_cast<const btBoxShape &>(impl_->as_bullet_shape()).getHalfExtentsWithoutMargin());
}

static CollisionShapeImpl *make_triangle_shape(const float3 &pt0,
                                               const float3 &pt1,
                                               const float3 &pt2)
{
  return CollisionShapeImpl::wrap(
      new btTriangleShape(to_bullet(pt0), to_bullet(pt1), to_bullet(pt2)));
}

TriangleCollisionShape::TriangleCollisionShape(const float3 &pt0,
                                               const float3 &pt1,
                                               const float3 &pt2)
    : CollisionShape(make_triangle_shape(pt0, pt1, pt2))
{
}

static CollisionShapeImpl *make_convex_hull_shape(const VArray<float3> &points)
{
  if (points.is_empty()) {
    return CollisionShapeImpl::wrap(new btConvexHullShape(nullptr, 0));
  }

  Array<btVector3> bt_points(points.size());
  for (const int i : bt_points.index_range()) {
    bt_points[i] = to_bullet(points[i]);
  }
  btConvexHullShape *shape = new btConvexHullShape(&bt_points.data()->x(), bt_points.size());
  shape->recalcLocalAabb();
  return CollisionShapeImpl::wrap(shape);
}

ConvexHullCollisionShape::ConvexHullCollisionShape(const VArray<float3> &points)
    : CollisionShape(make_convex_hull_shape(points))
{
}

static CollisionShapeImpl *make_sphere_shape(const float radius)
{
  return CollisionShapeImpl::wrap(new btSphereShape(to_bullet(radius)));
}

SphereCollisionShape::SphereCollisionShape(const float radius)
    : CollisionShape(make_sphere_shape(radius))
{
}

float SphereCollisionShape::radius() const
{
  return static_cast<const btSphereShape &>(impl_->as_bullet_shape()).getRadius();
}

static CollisionShapeImpl *make_capsule_shape(const float radius, const float height)
{
  return CollisionShapeImpl::wrap(new btCapsuleShapeZ(to_bullet(radius), to_bullet(height)));
}

CapsuleCollisionShape::CapsuleCollisionShape(const float radius, const float height)
    : CollisionShape(make_capsule_shape(radius, height))
{
}

static CollisionShapeImpl *make_tapered_capsule_shape(const float top_radius,
                                                      const float bottom_radius,
                                                      const float height)
{
  BLI_assert_unreachable();
  return nullptr;
}

TaperedCapsuleCollisionShape::TaperedCapsuleCollisionShape(const float top_radius,
                                                           const float bottom_radius,
                                                           const float height)
    : CollisionShape(make_tapered_capsule_shape(top_radius, bottom_radius, height))
{
}

// CollisionShapeImpl *make_cone_shape(const float radius, const float height)
//{
//   if (use_jolt) {
// #ifdef WITH_JOLT
//     // TODO Cone shape does not seem to existing Jolt?
//     BLI_assert_unreachable();
//     return nullptr;
// #endif
//   }
// #ifdef WITH_BULLET
//   return CollisionShapeImpl::wrap(new btConeShapeZ(to_bullet(radius), to_bullet(height)));
// #endif
// }
//
// ConeCollisionShape::ConeCollisionShape(const float radius, const float height)
//     : CollisionShape(make_cone_shape(radius, height))
//{
// }

static CollisionShapeImpl *make_cylinder_shape(const float radius, const float height)
{
  return CollisionShapeImpl::wrap(new btCylinderShapeZ(btVector3(radius, radius, height * 0.5f)));
}

CylinderCollisionShape::CylinderCollisionShape(const float radius, const float height)
    : CollisionShape(make_cylinder_shape(radius, height))
{
}

static CollisionShapeImpl *make_scaled_shape(const CollisionShape *child_shape,
                                             const float3 &scale)
{
  BLI_assert(child_shape != nullptr);
  BLI_assert(child_shape->is_convex());
  BLI_assert(dynamic_cast<const btConvexShape *>(&child_shape->impl().as_bullet_shape()));
  if (!child_shape->is_convex()) {
    return CollisionShapeImpl::wrap(new btEmptyShape());
  }
  const btConvexShape *convex_child_shape = static_cast<const btConvexShape *>(
      &child_shape->impl().as_bullet_shape());
  return CollisionShapeImpl::wrap(new btUniformScalingShape(
      const_cast<btConvexShape *>(convex_child_shape), math::average(scale)));
}

ScaledCollisionShape::ScaledCollisionShape(const CollisionShapePtr &child_shape,
                                           const float3 &scale)
    : CollisionShape(make_scaled_shape(child_shape.get(), scale)), child_shape_(child_shape)
{
}

static CollisionShapeImpl *make_offset_com_shape(const CollisionShape *child_shape,
                                                 const float3 &offset)
{
  BLI_assert_unreachable();
  return nullptr;
}

OffsetCenterOfMassShape::OffsetCenterOfMassShape(const CollisionShapePtr &child_shape,
                                                 const float3 &offset)
    : CollisionShape(make_offset_com_shape(child_shape.get(), offset)), child_shape_(child_shape)
{
}

static CollisionShapeImpl *make_rotated_translated_shape(const CollisionShape *child_shape,
                                                         const math::Quaternion &rotation,
                                                         const float3 &translation)
{
  BLI_assert_unreachable();
  return nullptr;
}

RotatedTranslatedCollisionShape::RotatedTranslatedCollisionShape(
    const CollisionShapePtr &child_shape,
    const math::Quaternion &rotation,
    const float3 &translation)
    : CollisionShape(make_rotated_translated_shape(child_shape.get(), rotation, translation))
{
}

static CollisionShapeImpl *make_mutable_compound_shape(Span<CollisionShapePtr> child_shapes,
                                                       Span<float4x4> child_transforms)
{
  BLI_assert_unreachable();
  return nullptr;
}

static CollisionShapeImpl *make_static_compound_shape(Span<CollisionShapePtr> child_shapes,
                                                      Span<float4x4> child_transforms)
{
  BLI_assert_unreachable();
  return nullptr;
}

MutableCompoundCollisionShape::MutableCompoundCollisionShape(Span<CollisionShapePtr> child_shapes,
                                                             Span<float4x4> child_transforms)
    : CollisionShape(make_mutable_compound_shape(child_shapes, child_transforms))
{
}

StaticCompoundCollisionShape::StaticCompoundCollisionShape(Span<CollisionShapePtr> child_shapes,
                                                           Span<float4x4> child_transforms)
    : CollisionShape(make_static_compound_shape(child_shapes, child_transforms))
{
}

static CollisionShapeImpl *make_mesh_shape(const Mesh &mesh)
{
  BLI_assert_unreachable();
  return nullptr;
}

MeshCollisionShape::MeshCollisionShape(const Mesh &mesh) : CollisionShape(make_mesh_shape(mesh)) {}

// struct TriangleMeshInterface {
//   Array<btVector3> positions;
//   Array<int> indices;
//   btTriangleIndexVertexArray bt_mesh_interface;
//
//   TriangleMeshInterface(const Mesh &mesh)
//   {
//     /* This is assuming the mesh is triangulated. If a face has more than 3 vertices
//      * it will create a random (but not invalid) mesh. */
//     BLI_assert(mesh.corners_num == 3 * mesh.faces_num);
//     const int num_triangles = mesh.faces_num;
//     const int num_vertices = mesh.verts_num;
//     /* Vertex index for each corner. In a triangle mesh each face has 3 consecutive corners. */
//     this->indices = mesh.corner_verts();
//
//     const Span<float3> src_positions = mesh.vert_positions();
//     this->positions.reinitialize(num_vertices);
//     for (const int i : this->positions.index_range()) {
//       this->positions[i][0] = src_positions[i].x;
//       this->positions[i][1] = src_positions[i].y;
//       this->positions[i][2] = src_positions[i].z;
//       this->positions[i][3] = 0.0f;
//     }
//     btScalar *bt_position_ptr = const_cast<btScalar *>(&this->positions.data()->x());
//
//     bt_mesh_interface = btTriangleIndexVertexArray(num_triangles,
//                                                    const_cast<int *>(this->indices.data()),
//                                                    3 * sizeof(int),
//                                                    num_vertices,
//                                                    bt_position_ptr,
//                                                    sizeof(btVector3));
//   }
// };
//
// static CollisionShapeImpl *make_triangle_mesh_shape(TriangleMeshInterface *mesh_interface)
//{
//   if (use_jolt) {
// #ifdef WITH_JOLT
//     // TODO
//     BLI_assert_unreachable();
//     return nullptr;
// #endif
//   }
//
// #ifdef WITH_BULLET
//   constexpr const bool use_quantized_aabb_compression = true;
//   constexpr const bool build_bvh = true;
//
//   if (mesh_interface == nullptr || mesh_interface->indices.is_empty()) {
//     return CollisionShapeImpl::wrap(new btEmptyShape());
//   }
//
//   return CollisionShapeImpl::wrap(new btBvhTriangleMeshShape(
//       &mesh_interface->bt_mesh_interface, use_quantized_aabb_compression, build_bvh));
// #endif
// }
//
// TriangleMeshCollisionShape::TriangleMeshCollisionShape(TriangleMeshInterface *mesh_interface)
//     : CollisionShape(make_triangle_mesh_shape(mesh_interface)), mesh_interface(mesh_interface)
//{
// }
//
// TriangleMeshCollisionShape::~TriangleMeshCollisionShape()
//{
//   delete this->mesh_interface;
// }
//
// TriangleMeshCollisionShape *TriangleMeshCollisionShape::from_mesh(const Mesh &mesh)
//{
//   TriangleMeshInterface *mesh_interface = (mesh.corners_num == 3 * mesh.faces_num ?
//                                                new TriangleMeshInterface(mesh) :
//                                                nullptr);
//   return new TriangleMeshCollisionShape(mesh_interface);
// }

// static CollisionShapeImpl *make_scaled_triangle_mesh_shape(
//     const TriangleMeshCollisionShape *child_shape, const float3 scale)
//{
//   if (use_jolt) {
// #ifdef WITH_JOLT
//     // TODO
//     BLI_assert_unreachable();
//     return nullptr;
// #endif
//   }
//
// #ifdef WITH_BULLET
//   BLI_assert(child_shape != nullptr);
//   BLI_assert(child_shape->impl().as_bullet_shape().getShapeType() ==
//              TRIANGLE_MESH_SHAPE_PROXYTYPE);
//   BLI_assert(dynamic_cast<const btBvhTriangleMeshShape
//   *>(&child_shape->impl().as_bullet_shape()));
//
//   const btBvhTriangleMeshShape *triangle_mesh_child_shape =
//       static_cast<const btBvhTriangleMeshShape *>(&child_shape->impl().as_bullet_shape());
//   return CollisionShapeImpl::wrap(new btScaledBvhTriangleMeshShape(
//       const_cast<btBvhTriangleMeshShape *>(triangle_mesh_child_shape), to_bullet(scale)));
// #endif
// }
//
// ScaledTriangleMeshCollisionShape::ScaledTriangleMeshCollisionShape(
//     const TriangleMeshCollisionShape *child_shape, const float3 scale)
//     : CollisionShape(make_scaled_triangle_mesh_shape(child_shape, scale))
//{
// }

// static CollisionShapeImpl *make_static_plane_shape(const float3 &plane_normal,
//                                                    const float plane_constant)
//{
//   if (use_jolt) {
// #ifdef WITH_JOLT
//     // TODO
//     BLI_assert_unreachable();
//     return nullptr;
// #endif
//   }
//
// #ifdef WITH_BULLET
//   const float3 normalized_normal = math::normalize(plane_normal);
//   /* Bullet requires normalized non-zero vectors. */
//   const float3 safe_normal = math::is_zero(normalized_normal) ? float3(0, 0, 1) :
//                                                                 normalized_normal;
//   return CollisionShapeImpl::wrap(new btStaticPlaneShape(to_bullet(safe_normal),
//   plane_constant));
// #endif
// }
//
// StaticPlaneCollisionShape::StaticPlaneCollisionShape(const float3 &plane_normal,
//                                                      const float plane_constant)
//     : CollisionShape(make_static_plane_shape(plane_normal, plane_constant))
//{
// }

// static CollisionShapeImpl *make_compound_shape(VArray<CollisionShapePtr> child_shapes,
//                                                VArray<float4x4> child_transforms)
//{
//   if (use_jolt) {
// #ifdef WITH_JOLT
//     // TODO
//     BLI_assert_unreachable();
//     return nullptr;
// #endif
//   }
//
// #ifdef WITH_BULLET
//   BLI_assert(child_shapes.size() == child_transforms.size());
//   btCompoundShape *shape = new btCompoundShape(true, child_shapes.size());
//   for (const int i : child_shapes.index_range()) {
//     const btCollisionShape *bt_child_shape = &child_shapes[i]->impl().as_bullet_shape();
//     shape->addChildShape(to_bullet(child_transforms[i]),
//                          const_cast<btCollisionShape *>(bt_child_shape));
//   }
//   return CollisionShapeImpl::wrap(shape);
// #endif
// }
//
// CompoundCollisionShape::CompoundCollisionShape(const VArray<CollisionShapePtr> &child_shapes,
//                                                const VArray<float4x4> &child_transforms)
//     : CollisionShape(make_compound_shape(child_shapes, child_transforms))
//{
//   child_shapes_.reinitialize(child_shapes.size());
//   child_shapes.materialize_to_uninitialized(child_shapes_);
// }

#  endif
#else
#  ifdef WITH_JOLT

/* Limit for shape parameters that may never be zero. */
static constexpr float shape_size_epsilon = 0.0001f;
static constexpr float shape_convex_radius = JPH::cDefaultConvexRadius;
static constexpr JPH::PhysicsMaterial *shape_physics_material = nullptr;

/* Fallback shape that can be used when an input pointer is null but some shape is required. */
static const JPH::Shape *get_shape_or_fallback(const CollisionShape *shape)
{
  if (shape == nullptr) {
    static const JPH::RefConst<JPH::Shape> fallback_shape = JPH::RefConst<JPH::Shape>(
        new JPH::SphereShape(1.0f));
    return fallback_shape.GetPtr();
  }
  return &shape->impl().as_jolt_shape();
}

/* Fallback shape that can be used when an error occurred but some shape is required. */
static CollisionShape::CollisionShapeResult construct_shape(const JPH::ShapeSettings &settings)
{
  const JPH::ShapeSettings::ShapeResult result = settings.Create();
  if (result.HasError()) {
    return {CollisionShapeImpl::wrap(new JPH::SphereShape(1.0f)), std::string(result.GetError())};
  }
  if (result.IsEmpty()) {
    return {CollisionShapeImpl::wrap(new JPH::SphereShape(1.0f)), "Empty shape"};
  }
  return {CollisionShapeImpl::wrap(result.Get())};
}

CollisionShape::CollisionShape(const CollisionShapeResult &result)
    : impl_(result.impl), error_(result.error)
{
  impl_->as_jolt_shape().AddRef();
}

CollisionShape::ShapeType CollisionShape::type() const
{
  const JPH::EShapeSubType subtype = impl_->as_jolt_shape().GetSubType();
  bke::CollisionShape::ShapeType shape_type = to_blender(subtype);
  return shape_type;
}

bool CollisionShape::supports_motion() const
{
  return !impl_->as_jolt_shape().MustBeStatic();
}

bool CollisionShape::is_convex() const
{
  return impl_->as_jolt_shape().GetType() == JPH::EShapeType::Convex;
}

bool CollisionShape::is_concave() const
{
  return impl_->as_jolt_shape().GetType() != JPH::EShapeType::Convex;
}

float3 CollisionShape::center_of_mass() const
{
  return to_blender(impl_->as_jolt_shape().GetCenterOfMass());
}

Bounds<float3> CollisionShape::local_bounds() const
{
  return to_blender(impl_->as_jolt_shape().GetLocalBounds());
}

float3 CollisionShape::calculate_local_inertia(const float mass) const
{
  // XXX This assumes the principal components are aligned with the canonical axes.
  // For complex shapes (e.g. mesh) this is not generally the case, the principal axes are
  // rotated. The DecomposePrincipalMomentsOfInertia function can be used to get the rotation.
  const JPH::MassProperties mass_props = impl().as_jolt_shape().GetMassProperties();
  return to_blender(mass_props.mInertia.GetDiagonal3());
}

static CollisionShape::CollisionShapeResult make_box_shape(const float3 &half_extent)
{
  JPH::BoxShapeSettings settings(
      to_jolt(math::max(half_extent, float3(shape_convex_radius + 1.0e-6f))),
      shape_convex_radius,
      shape_physics_material);
  return construct_shape(settings);
}

BoxCollisionShape::BoxCollisionShape(const float3 &half_extent)
    : CollisionShape(make_box_shape(half_extent))
{
}

float3 BoxCollisionShape::half_extent() const
{
  return to_blender(static_cast<const JPH::BoxShape &>(impl_->as_jolt_shape()).GetHalfExtent());
}

static CollisionShape::CollisionShapeResult make_triangle_shape(const float3 &pt0,
                                                                const float3 &pt1,
                                                                const float3 &pt2)
{
  JPH::TriangleShapeSettings settings(
      to_jolt(pt0), to_jolt(pt1), to_jolt(pt2), shape_convex_radius, shape_physics_material);
  return construct_shape(settings);
}

TriangleCollisionShape::TriangleCollisionShape(const float3 &pt0,
                                               const float3 &pt1,
                                               const float3 &pt2)
    : CollisionShape(make_triangle_shape(pt0, pt1, pt2))
{
}

static CollisionShape::CollisionShapeResult make_convex_hull_shape(const VArray<float3> &points)
{
  JPH::ConvexHullShapeSettings settings;
  settings.mPoints.reserve(points.size());
  for (const int i : points.index_range()) {
    settings.mPoints.push_back(to_jolt(points[i]));
  }
  settings.mMaxConvexRadius = shape_convex_radius;
  settings.mMaterial = shape_physics_material;
  return construct_shape(settings);
}

ConvexHullCollisionShape::ConvexHullCollisionShape(const VArray<float3> &points)
    : CollisionShape(make_convex_hull_shape(points))
{
}

static CollisionShape::CollisionShapeResult make_sphere_shape(const float radius)
{
  JPH::SphereShapeSettings settings(to_jolt(std::max(radius, shape_size_epsilon)),
                                    shape_physics_material);
  return construct_shape(settings);
}

SphereCollisionShape::SphereCollisionShape(const float radius)
    : CollisionShape(make_sphere_shape(radius))
{
}

float SphereCollisionShape::radius() const
{
  return to_blender(static_cast<const JPH::SphereShape &>(impl_->as_jolt_shape()).GetRadius());
}

static CollisionShape::CollisionShapeResult make_capsule_shape(const float radius,
                                                               const float height)
{
  JPH::CapsuleShapeSettings settings(to_jolt(std::max(0.5f * height, shape_size_epsilon)),
                                     to_jolt(std::max(radius, shape_size_epsilon)),
                                     shape_physics_material);
  return construct_shape(settings);
}

CapsuleCollisionShape::CapsuleCollisionShape(const float radius, const float height)
    : CollisionShape(make_capsule_shape(radius, height))
{
}

static CollisionShape::CollisionShapeResult make_tapered_capsule_shape(const float top_radius,
                                                                       const float bottom_radius,
                                                                       const float height)
{
  JPH::TaperedCapsuleShapeSettings settings;
  settings.mTopRadius = to_jolt(std::max(top_radius, shape_size_epsilon));
  settings.mBottomRadius = to_jolt(std::max(bottom_radius, shape_size_epsilon));
  settings.mHalfHeightOfTaperedCylinder = to_jolt(std::max(0.5f * height, shape_size_epsilon));
  settings.mMaterial = shape_physics_material;
  return construct_shape(settings);
}

TaperedCapsuleCollisionShape::TaperedCapsuleCollisionShape(const float top_radius,
                                                           const float bottom_radius,
                                                           const float height)
    : CollisionShape(make_tapered_capsule_shape(top_radius, bottom_radius, height))
{
}

static CollisionShape::CollisionShapeResult make_cylinder_shape(const float radius,
                                                                const float height)
{
  JPH::CylinderShapeSettings settings(to_jolt(std::max(0.5f * height, shape_convex_radius)),
                                      to_jolt(std::max(radius, shape_convex_radius)),
                                      shape_convex_radius,
                                      shape_physics_material);
  return construct_shape(settings);
}

CylinderCollisionShape::CylinderCollisionShape(const float radius, const float height)
    : CollisionShape(make_cylinder_shape(radius, height))
{
}

static CollisionShape::CollisionShapeResult make_scaled_shape(const CollisionShape *child_shape,
                                                              const float3 &scale)
{
  JPH::ScaledShapeSettings settings;
  settings.mInnerShapePtr = get_shape_or_fallback(child_shape);
  settings.mScale = to_jolt(math::is_zero(scale) ? float3(1.0f) : scale);
  return construct_shape(settings);
}

ScaledCollisionShape::ScaledCollisionShape(const CollisionShapePtr &child_shape,
                                           const float3 &scale)
    : CollisionShape(make_scaled_shape(child_shape.get(), scale)), child_shape_(child_shape)
{
}

static CollisionShape::CollisionShapeResult make_offset_com_shape(
    const CollisionShape *child_shape, const float3 &offset)
{
  JPH::OffsetCenterOfMassShapeSettings settings;
  settings.mInnerShapePtr = get_shape_or_fallback(child_shape);
  settings.mOffset = to_jolt(offset);
  return construct_shape(settings);
}

OffsetCenterOfMassShape::OffsetCenterOfMassShape(const CollisionShapePtr &child_shape,
                                                 const float3 &offset)
    : CollisionShape(make_offset_com_shape(child_shape.get(), offset)), child_shape_(child_shape)
{
}

static CollisionShape::CollisionShapeResult make_rotated_translated_shape(
    const CollisionShape *child_shape, const math::Quaternion &rotation, const float3 &translation)
{
  JPH::RotatedTranslatedShapeSettings settings;
  settings.mInnerShapePtr = get_shape_or_fallback(child_shape);
  settings.mRotation = to_jolt(rotation);
  settings.mPosition = to_jolt(translation);
  return construct_shape(settings);
}

RotatedTranslatedCollisionShape::RotatedTranslatedCollisionShape(
    const CollisionShapePtr &child_shape,
    const math::Quaternion &rotation,
    const float3 &translation)
    : CollisionShape(make_rotated_translated_shape(child_shape.get(), rotation, translation))
{
}

static CollisionShape::CollisionShapeResult make_mutable_compound_shape(
    Span<CollisionShapePtr> child_shapes, Span<float4x4> child_transforms)
{
  JPH::MutableCompoundShapeSettings settings;
  settings.mSubShapes.reserve(child_shapes.size());
  for (const int i : child_shapes.index_range()) {
    JPH::CompoundShapeSettings::SubShapeSettings child_settings;
    child_settings.mShapePtr = &child_shapes[i]->impl().as_jolt_shape();
    child_settings.mPosition = to_jolt(child_transforms[i].location());
    child_settings.mRotation = to_jolt(math::to_quaternion(child_transforms[i]));
    settings.mSubShapes.push_back(std::move(child_settings));
  }
  return construct_shape(settings);
}

static CollisionShape::CollisionShapeResult make_static_compound_shape(
    Span<CollisionShapePtr> child_shapes, Span<float4x4> child_transforms)
{
  JPH::StaticCompoundShapeSettings settings;
  settings.mSubShapes.reserve(child_shapes.size());
  for (const int i : child_shapes.index_range()) {
    JPH::CompoundShapeSettings::SubShapeSettings child_settings;
    child_settings.mShapePtr = &child_shapes[i]->impl().as_jolt_shape();
    child_settings.mPosition = to_jolt(child_transforms[i].location());
    child_settings.mRotation = to_jolt(math::to_quaternion(child_transforms[i]));
    settings.mSubShapes.push_back(std::move(child_settings));
  }
  return construct_shape(settings);
}

MutableCompoundCollisionShape::MutableCompoundCollisionShape(Span<CollisionShapePtr> child_shapes,
                                                             Span<float4x4> child_transforms)
    : CollisionShape(make_mutable_compound_shape(child_shapes, child_transforms))
{
}

StaticCompoundCollisionShape::StaticCompoundCollisionShape(Span<CollisionShapePtr> child_shapes,
                                                           Span<float4x4> child_transforms)
    : CollisionShape(make_static_compound_shape(child_shapes, child_transforms))
{
}

static CollisionShape::CollisionShapeResult make_mesh_shape(const Mesh &mesh)
{
  /* This is assuming the mesh is triangulated. If a face has more than 3 vertices
   * it will create a random (but not invalid) mesh. */
  BLI_assert(mesh.corners_num == 3 * mesh.faces_num);

  const Span<float3> src_positions = mesh.vert_positions();
  JPH::VertexList vertices(src_positions.size());
  for (const int i : IndexRange(vertices.size())) {
    vertices[i] = JPH::Float3(src_positions[i].x, src_positions[i].y, src_positions[i].z);
  }

  const Span<int> src_indices = mesh.corner_verts();
  const OffsetIndices corners_by_face = mesh.face_offsets();
  JPH::IndexedTriangleList triangles(corners_by_face.size());
  for (const int i : IndexRange(triangles.size())) {
    const IndexRange corners = corners_by_face[i];
    BLI_assert(corners.size() >= 3);
    triangles[i] = {uint32_t(src_indices[corners[0]]),
                    uint32_t(src_indices[corners[1]]),
                    uint32_t(src_indices[corners[2]])};
  }

  JPH::MeshShapeSettings settings(vertices, triangles);
  return construct_shape(settings);
}

MeshCollisionShape::MeshCollisionShape(const Mesh &mesh) : CollisionShape(make_mesh_shape(mesh)) {}

#  endif
#endif

}  // namespace blender::bke
