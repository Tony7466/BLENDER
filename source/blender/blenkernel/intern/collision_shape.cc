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
#include "BKE_material.h"
#include "BKE_mesh.hh"
#include "BKE_physics_geometry.hh"

#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"

#include "physics_geometry_intern.hh"

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

float3 CollisionShape::calculate_local_inertia(const float /*mass*/) const
{
  // XXX This assumes the principal components are aligned with the canonical axes.
  // For complex shapes (e.g. mesh) this is not generally the case, the principal axes are
  // rotated. The DecomposePrincipalMomentsOfInertia function can be used to get the rotation.
  const JPH::MassProperties mass_props = impl().as_jolt_shape().GetMassProperties();
  return to_blender(mass_props.mInertia.GetDiagonal3());
}

static Mesh *shape_to_mesh(const JPH::Shape &shape, const float4x4 &transform)
{
  const JPH::AABox bounds = to_jolt(Bounds<float3>(float3(-FLT_MAX), float3(FLT_MAX)));
  float3 loc;
  math::Quaternion rot;
  float3 scale;
  math::to_loc_rot_scale(transform, loc, rot, scale);

  const int max_triangles = shape.GetStats().mNumTriangles;
  /* Simple triangle soup mesh. */
  Mesh *mesh = BKE_mesh_new_nomain(
      max_triangles * 3, max_triangles * 3, max_triangles, max_triangles * 3);
  bke::mesh_smooth_set(*mesh, false);

  JPH::Shape::GetTrianglesContext context;
  shape.GetTrianglesStart(context, bounds, to_jolt(loc), to_jolt(rot), to_jolt(scale));

  MutableSpan<JPH::Float3> positions = mesh->vert_positions_for_write().cast<JPH::Float3>();
  while (!positions.is_empty()) {
    const int num_triangles = shape.GetTrianglesNext(context, positions.size(), positions.data());
    if (num_triangles == 0) {
      BLI_assert_unreachable();
      break;
    }
    positions = positions.drop_front(num_triangles * 3);
  }

  MutableSpan<int2> edges = mesh->edges_for_write();
  MutableSpan<int> face_offsets = mesh->face_offsets_for_write();
  MutableSpan<int> corner_verts = mesh->corner_verts_for_write();
  MutableSpan<int> corner_edges = mesh->corner_edges_for_write();
  for (const int i : IndexRange(max_triangles)) {
    const int v0 = i * 3;
    const int v1 = v0 + 1;
    const int v2 = v0 + 2;
    face_offsets[i] = v0;
    edges[v0] = {v0, v1};
    edges[v1] = {v1, v2};
    edges[v2] = {v2, v0};
    corner_verts[v0] = v0;
    corner_verts[v1] = v1;
    corner_verts[v2] = v2;
    corner_edges[v0] = v0;
    corner_edges[v1] = v1;
    corner_edges[v2] = v2;
  }
  face_offsets.last() = mesh->faces_num * 3;
  mesh->tag_topology_changed();
  mesh->tag_positions_changed();

  mesh->tag_loose_verts_none();
  mesh->tag_loose_edges_none();
  mesh->tag_overlapping_none();
  mesh->bounds_set_eager(to_blender(shape.GetLocalBounds()));

  BKE_id_material_eval_ensure_default_slot(&mesh->id);

  return mesh;
}

GeometrySet CollisionShape::create_geometry() const
{
  const JPH::Shape &shape = impl_->as_jolt_shape();
  switch (shape.GetSubType()) {
    case JPH::EShapeSubType::ConvexHull:
    case JPH::EShapeSubType::Mesh:
    case JPH::EShapeSubType::Scaled:
    case JPH::EShapeSubType::OffsetCenterOfMass:
    case JPH::EShapeSubType::RotatedTranslated:
    case JPH::EShapeSubType::StaticCompound:
    case JPH::EShapeSubType::MutableCompound:
      return GeometrySet::from_mesh(shape_to_mesh(shape, float4x4::identity()));
    default:
      return {};
  }
  BLI_assert_unreachable();
  return {};
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

namespace physics_attributes {

template<typename T> using ShapeGetFn = T (*)(const JPH::Shape &shape);

template<typename ElemT, ShapeGetFn<ElemT> GetFn>
class VArrayImpl_For_PhysicsShapes final : public VArrayImpl<ElemT> {
 private:
  Span<bke::CollisionShapePtr> shapes_;

 public:
  VArrayImpl_For_PhysicsShapes(const Span<bke::CollisionShapePtr> shapes)
      : VArrayImpl<ElemT>(shapes.size()), shapes_(shapes)
  {
  }

 private:
  ElemT get(const int64_t index) const override
  {
    return GetFn(shapes_[index]->impl().as_jolt_shape());
  }

  void materialize(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>(
        [&](const int64_t i) { dst[i] = GetFn(shapes_[i]->impl().as_jolt_shape()); });
  }

  void materialize_to_uninitialized(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>(
        [&](const int64_t i) { new (dst + i) ElemT(GetFn(shapes_[i]->impl().as_jolt_shape())); });
  }

  void materialize_compressed(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>([&](const int64_t i, const int64_t pos) {
      dst[pos] = GetFn(shapes_[i]->impl().as_jolt_shape());
    });
  }

  void materialize_compressed_to_uninitialized(const IndexMask &mask, ElemT *dst) const override
  {
    mask.foreach_index_optimized<int64_t>([&](const int64_t i, const int64_t pos) {
      new (dst + pos) ElemT(GetFn(shapes_[i]->impl().as_jolt_shape()));
    });
  }
};

static float3 shape_translation_get_fn(const JPH::Shape &shape)
{
  switch (shape.GetSubType()) {
    case JPH::EShapeSubType::RotatedTranslated:
      return to_blender(static_cast<const JPH::RotatedTranslatedShape &>(shape).GetPosition());
    case JPH::EShapeSubType::OffsetCenterOfMass:
      return to_blender(static_cast<const JPH::OffsetCenterOfMassShape &>(shape).GetOffset());
    default:
      return float3(0.0f);
  }
  BLI_assert_unreachable();
  return float3(0.0f);
}

static math::Quaternion shape_rotation_get_fn(const JPH::Shape &shape)
{
  switch (shape.GetSubType()) {
    case JPH::EShapeSubType::RotatedTranslated:
      return to_blender(static_cast<const JPH::RotatedTranslatedShape &>(shape).GetRotation());
    default:
      return math::Quaternion();
  }
  BLI_assert_unreachable();
  return math::Quaternion();
}

static float3 shape_scale_get_fn(const JPH::Shape &shape)
{
  switch (shape.GetSubType()) {
    case JPH::EShapeSubType::Scaled:
      return to_blender(static_cast<const JPH::ScaledShape &>(shape).GetScale());
    default:
      return float3(1.0f);
  }
  BLI_assert_unreachable();
  return float3(0.0f);
}

static float3 shape_size_get_fn(const JPH::Shape &shape)
{
  switch (shape.GetSubType()) {
    case JPH::EShapeSubType::Box:
      return to_blender(static_cast<const JPH::BoxShape &>(shape).GetHalfExtent()) * 2.0f;
    default:
      return float3(1.0f);
  }
  BLI_assert_unreachable();
  return float3(0.0f);
}

static float shape_radius_get_fn(const JPH::Shape &shape)
{
  switch (shape.GetSubType()) {
    case JPH::EShapeSubType::Sphere:
      return to_blender(static_cast<const JPH::SphereShape &>(shape).GetRadius());
    case JPH::EShapeSubType::Capsule:
      return to_blender(static_cast<const JPH::CapsuleShape &>(shape).GetRadius());
    case JPH::EShapeSubType::TaperedCapsule: {
      /* Reconstruct parameters from local bounds.
       * TODO It may be beneficial to store "uncooked"
       * JPH::ShapeSettings along with the "cooked" JPH::Shape.
       * For now this should work ok.
       */
      const Bounds<float3> local_bounds = to_blender(
          static_cast<const JPH::TaperedCapsuleShape &>(shape).GetLocalBounds());
      if (-local_bounds.min.y < local_bounds.max.y) {
        const float top_radius = local_bounds.max.x;
        const float half_height = local_bounds.max.y - top_radius;
        const float bottom_radius = -local_bounds.min.y - half_height;
        return bottom_radius;
      }
      else {
        const float bottom_radius = local_bounds.max.x;
        // const float half_height = -local_bounds.min.y - bottom_radius;
        // const float top_radius = local_bounds.max.y - half_height;
        return bottom_radius;
      }
    }
    case JPH::EShapeSubType::Cylinder:
      return static_cast<const JPH::CylinderShape &>(shape).GetInnerRadius();
    default:
      return 1.0f;
  }
  BLI_assert_unreachable();
  return 0.0f;
}

static float shape_radius2_get_fn(const JPH::Shape &shape)
{
  switch (shape.GetSubType()) {
    case JPH::EShapeSubType::TaperedCapsule: {
      /* Reconstruct parameters from local bounds.
       * TODO It may be beneficial to store "uncooked"
       * JPH::ShapeSettings along with the "cooked" JPH::Shape.
       * For now this should work ok.
       */
      const Bounds<float3> local_bounds = to_blender(
          static_cast<const JPH::TaperedCapsuleShape &>(shape).GetLocalBounds());
      if (-local_bounds.min.y < local_bounds.max.y) {
        const float top_radius = local_bounds.max.x;
        // const float half_height = local_bounds.max.y - top_radius;
        // const float bottom_radius = -local_bounds.min.y - half_height;
        return top_radius;
      }
      else {
        const float bottom_radius = local_bounds.max.x;
        const float half_height = -local_bounds.min.y - bottom_radius;
        const float top_radius = local_bounds.max.y - half_height;
        return top_radius;
      }
    }
    default:
      return 1.0f;
  }
  BLI_assert_unreachable();
  return 0.0f;
}

static float shape_height_get_fn(const JPH::Shape &shape)
{
  switch (shape.GetSubType()) {
    case JPH::EShapeSubType::Capsule:
      return 2.0f *
             to_blender(static_cast<const JPH::CapsuleShape &>(shape).GetHalfHeightOfCylinder());
    case JPH::EShapeSubType::TaperedCapsule: {
      /* Reconstruct parameters from local bounds.
       * TODO It may be beneficial to store "uncooked"
       * JPH::ShapeSettings along with the "cooked" JPH::Shape.
       * For now this should work ok.
       */
      const Bounds<float3> local_bounds = to_blender(
          static_cast<const JPH::TaperedCapsuleShape &>(shape).GetLocalBounds());
      if (-local_bounds.min.y < local_bounds.max.y) {
        const float top_radius = local_bounds.max.x;
        const float half_height = local_bounds.max.y - top_radius;
        return 2.0f * half_height;
      }
      else {
        const float bottom_radius = local_bounds.max.x;
        const float half_height = -local_bounds.min.y - bottom_radius;
        return 2.0f * half_height;
      }
    }
    case JPH::EShapeSubType::Cylinder:
      return 2.0f * static_cast<const JPH::CylinderShape &>(shape).GetHalfHeight();
    default:
      return 1.0f;
  }
  BLI_assert_unreachable();
  return 0.0f;
}

static float3 get_triangle_shape_point(const JPH::Shape &shape, const int point)
{
  BLI_assert(shape.GetSubType() == JPH::EShapeSubType::Triangle);
  BLI_assert(IndexRange(3).contains(point));
  const auto &triangle_shape = static_cast<const JPH::TriangleShape &>(shape);

  JPH::TriangleShape::GetTrianglesContext context;
  triangle_shape.GetTrianglesStart(
      context, {}, to_jolt(float3(0)), to_jolt(math::Quaternion::identity()), to_jolt(float3(1)));
  JPH::Float3 triangle[3];
  triangle_shape.GetTrianglesNext(context, 1, triangle);
  return to_blender(triangle[point]);
}

static float3 shape_point0_get_fn(const JPH::Shape &shape)
{
  switch (shape.GetSubType()) {
    case JPH::EShapeSubType::Triangle:
      return get_triangle_shape_point(shape, 0);
    default:
      return float3(0.0f);
  }
  BLI_assert_unreachable();
  return float3(0.0f);
}

static float3 shape_point1_get_fn(const JPH::Shape &shape)
{
  switch (shape.GetSubType()) {
    case JPH::EShapeSubType::Triangle:
      return get_triangle_shape_point(shape, 1);
    default:
      return float3(0.0f);
  }
  BLI_assert_unreachable();
  return float3(0.0f);
}

static float3 shape_point2_get_fn(const JPH::Shape &shape)
{
  switch (shape.GetSubType()) {
    case JPH::EShapeSubType::Triangle:
      return get_triangle_shape_point(shape, 2);
    default:
      return float3(0.0f);
  }
  BLI_assert_unreachable();
  return float3(0.0f);
}

StringRef physics_shape_attribute_label(const bke::CollisionShapeType shape_type,
                                        bke::PhysicsShapeAttribute attribute)
{
  using ShapeType = bke::CollisionShape::ShapeType;
  using ShapeAttribute = bke::PhysicsShapeAttribute;

  const StringRef default_label = bke::physics_attributes::physics_attribute_name(attribute);

  switch (shape_type) {
    case ShapeType::Sphere:
      switch (attribute) {
        case ShapeAttribute::radius:
          return "Radius";
        default:
          return default_label;
      }
    case ShapeType::Box:
      switch (attribute) {
        case ShapeAttribute::size:
          return "Size";
        default:
          return default_label;
      }
    case ShapeType::Triangle:
      switch (attribute) {
        case ShapeAttribute::point0:
          return "Point 0";
        case ShapeAttribute::point1:
          return "Point 1";
        case ShapeAttribute::point2:
          return "Point 2";
        default:
          return default_label;
      }
    case ShapeType::Capsule:
      switch (attribute) {
        case ShapeAttribute::radius:
          return "Radius";
        case ShapeAttribute::height:
          return "Height";
        default:
          return default_label;
      }
    case ShapeType::TaperedCapsule:
      switch (attribute) {
        case ShapeAttribute::radius:
          return "Bottom Radius";
        case ShapeAttribute::radius2:
          return "Top Radius";
        case ShapeAttribute::height:
          return "Height";
        default:
          return default_label;
      }
    case ShapeType::Cylinder:
      switch (attribute) {
        case ShapeAttribute::radius:
          return "Radius";
        case ShapeAttribute::height:
          return "Height";
        default:
          return default_label;
      }
    case ShapeType::RotatedTranslated:
      switch (attribute) {
        case ShapeAttribute::rotation:
          return "Rotation";
        case ShapeAttribute::translation:
          return "Translation";
        default:
          return default_label;
      }
    case ShapeType::Scaled:
      switch (attribute) {
        case ShapeAttribute::scale:
          return "Scale";
        default:
          return default_label;
      }
    case ShapeType::OffsetCenterOfMass:
      switch (attribute) {
        case ShapeAttribute::translation:
          return "Offset";
        default:
          return default_label;
      }

    case ShapeType::ConvexHull:
    case ShapeType::StaticCompound:
    case ShapeType::MutableCompound:
    case ShapeType::Mesh:
    case ShapeType::HeightField:
    case ShapeType::SoftBody:
      return default_label;
  }
  BLI_assert_unreachable();
  return "";
}

GVArray physics_attribute_varray(PhysicsShapeAttribute attribute,
                                 Span<bke::CollisionShapePtr> shapes)
{
  using ShapeAttribute = PhysicsShapeAttribute;

  switch (attribute) {
    case ShapeAttribute::translation:
      return VArray<float3>::For<VArrayImpl_For_PhysicsShapes<float3, shape_translation_get_fn>>(
          shapes);
    case ShapeAttribute::rotation:
      return VArray<math::Quaternion>::For<
          VArrayImpl_For_PhysicsShapes<math::Quaternion, shape_rotation_get_fn>>(shapes);
    case ShapeAttribute::scale:
      return VArray<float3>::For<VArrayImpl_For_PhysicsShapes<float3, shape_scale_get_fn>>(shapes);
    case ShapeAttribute::size:
      return VArray<float3>::For<VArrayImpl_For_PhysicsShapes<float3, shape_size_get_fn>>(shapes);
    case ShapeAttribute::radius:
      return VArray<float>::For<VArrayImpl_For_PhysicsShapes<float, shape_radius_get_fn>>(shapes);
    case ShapeAttribute::radius2:
      return VArray<float>::For<VArrayImpl_For_PhysicsShapes<float, shape_radius2_get_fn>>(shapes);
    case ShapeAttribute::height:
      return VArray<float>::For<VArrayImpl_For_PhysicsShapes<float, shape_height_get_fn>>(shapes);
    case ShapeAttribute::point0:
      return VArray<float3>::For<VArrayImpl_For_PhysicsShapes<float3, shape_point0_get_fn>>(
          shapes);
    case ShapeAttribute::point1:
      return VArray<float3>::For<VArrayImpl_For_PhysicsShapes<float3, shape_point1_get_fn>>(
          shapes);
    case ShapeAttribute::point2:
      return VArray<float3>::For<VArrayImpl_For_PhysicsShapes<float3, shape_point2_get_fn>>(
          shapes);
  }
  BLI_assert_unreachable();
  return {};
}

bool physics_shape_attribute_valid(const CollisionShapeType shape_type,
                                   PhysicsShapeAttribute attribute)
{
  using ShapeType = CollisionShape::ShapeType;
  using ShapeAttribute = PhysicsShapeAttribute;

  switch (shape_type) {
    case ShapeType::Sphere:
      return ELEM(attribute, ShapeAttribute::radius);
    case ShapeType::Box:
      return ELEM(attribute, ShapeAttribute::size);
    case ShapeType::Triangle:
      return ELEM(
          attribute, ShapeAttribute::point0, ShapeAttribute::point1, ShapeAttribute::point2);
    case ShapeType::Capsule:
      return ELEM(attribute, ShapeAttribute::radius, ShapeAttribute::height);
    case ShapeType::TaperedCapsule:
      return ELEM(
          attribute, ShapeAttribute::radius, ShapeAttribute::radius2, ShapeAttribute::height);
    case ShapeType::Cylinder:
      return ELEM(attribute, ShapeAttribute::radius, ShapeAttribute::height);
    case ShapeType::RotatedTranslated:
      return ELEM(attribute, ShapeAttribute::translation, ShapeAttribute::rotation);
    case ShapeType::Scaled:
      return ELEM(attribute, ShapeAttribute::scale);
    case ShapeType::OffsetCenterOfMass:
      return ELEM(attribute, ShapeAttribute::translation);

    case ShapeType::ConvexHull:
    case ShapeType::StaticCompound:
    case ShapeType::MutableCompound:
    case ShapeType::Mesh:
    case ShapeType::HeightField:
    case ShapeType::SoftBody:
      return false;
  }
  BLI_assert_unreachable();
  return false;
}

bool physics_shape_geometry_valid(const CollisionShapeType shape_type)
{
  using ShapeType = CollisionShape::ShapeType;

  return ELEM(shape_type,
              ShapeType::ConvexHull,
              ShapeType::Mesh,
              ShapeType::Scaled,
              ShapeType::OffsetCenterOfMass,
              ShapeType::RotatedTranslated,
              ShapeType::StaticCompound,
              ShapeType::MutableCompound);
}

}  // namespace physics_attributes

#endif

}  // namespace blender::bke
