/* SPDX-FileCopyrightText: Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sim
 */

#pragma once

#include "BLI_array.hh"
#include "BLI_bounds_types.hh"
#include "BLI_implicit_sharing_ptr.hh"
#include "BLI_math_matrix_types.hh"
#include "BLI_math_quaternion_types.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_span.hh"
#include "BLI_string_ref.hh"
#include "BLI_utility_mixins.hh"
#include "BLI_virtual_array_fwd.hh"

#include <functional>
#include <optional>

struct Mesh;

namespace blender::bke {

class CollisionShape;
class BoxCollisionShape;
class SphereCollisionShape;

struct CollisionShapeImpl;
struct TriangleMeshInterface;

class CollisionShape : public ImplicitSharingMixin {
 public:
  using Ptr = ImplicitSharingPtr<CollisionShape>;

  /* Not all shape types may be supported. */
  enum class ShapeType {
    Sphere,
    Box,
    Triangle,
    Capsule,
    TaperedCapsule,
    Cylinder,
    ConvexHull,
    StaticCompound,
    MutableCompound,
    RotatedTranslated,
    Scaled,
    OffsetCenterOfMass,
    Mesh,
    HeightField,
    SoftBody,
  };

  struct CollisionShapeResult {
    CollisionShapeImpl *impl;
    std::optional<std::string> error;
  };

 protected:
  CollisionShapeImpl *impl_;
  std::optional<std::string> error_;

 public:
  CollisionShape() = delete;
  virtual ~CollisionShape();

  void delete_self() override;

  CollisionShapeImpl &impl();
  const CollisionShapeImpl &impl() const;

  ShapeType type() const;

  static StringRef type_name(const ShapeType type);

  /** Shape type is supported for dynamic bodies.
   *  Concave shapes and some other types can only be used for static bodies.
   */
  bool supports_motion() const;

  template<typename T> bool is_a()
  {
    if constexpr (std::is_same_v<T, BoxCollisionShape>) {
      return this->type() == ShapeType::Box;
    }
    if constexpr (std::is_same_v<T, SphereCollisionShape>) {
      return this->type() == ShapeType::Sphere;
    }
    return false;
  }

  template<typename T> T &get_as()
  {
    BLI_assert(this->is_a<T>());
    return *static_cast<T *>(this);
  }

  bool is_convex() const;
  bool is_concave() const;

  float3 center_of_mass() const;
  Bounds<float3> local_bounds() const;

  float3 calculate_local_inertia(float mass) const;

 protected:
  CollisionShape(const CollisionShapeResult &result);

  friend class PhysicsGeometry;
};

using CollisionShapePtr = CollisionShape::Ptr;

// class EmptyCollisionShape : public CollisionShape {
//  public:
//   EmptyCollisionShape();
// };

class BoxCollisionShape : public CollisionShape {
 public:
  BoxCollisionShape(const float3 &half_extent);

  float3 half_extent() const;
};

class TriangleCollisionShape : public CollisionShape {
 public:
  TriangleCollisionShape(const float3 &pt0, const float3 &pt1, const float3 &pt2);
};

class ConvexHullCollisionShape : public CollisionShape {
 public:
  ConvexHullCollisionShape(const VArray<float3> &points);
};

class SphereCollisionShape : public CollisionShape {
 public:
  SphereCollisionShape(float radius);

  float radius() const;
};

class CapsuleCollisionShape : public CollisionShape {
 public:
  CapsuleCollisionShape(float radius, float height);
};

class TaperedCapsuleCollisionShape : public CollisionShape {
 public:
  TaperedCapsuleCollisionShape(float top_radius, float bottom_radius, float height);
};

class CylinderCollisionShape : public CollisionShape {
 public:
  CylinderCollisionShape(float radius, float height);
};

class ScaledCollisionShape : public CollisionShape {
 public:
  CollisionShapePtr child_shape_;

  ScaledCollisionShape(const CollisionShapePtr &child_shape, const float3 &scale);
};

class OffsetCenterOfMassShape : public CollisionShape {
 public:
  CollisionShapePtr child_shape_;

  OffsetCenterOfMassShape(const CollisionShapePtr &child_shape, const float3 &offset);
};

class RotatedTranslatedCollisionShape : public CollisionShape {
 public:
  CollisionShapePtr child_shape_;

  RotatedTranslatedCollisionShape(const CollisionShapePtr &child_shape,
                                  const math::Quaternion &rotation,
                                  const float3 &translation);
};

class MutableCompoundCollisionShape : public CollisionShape {
 public:
  MutableCompoundCollisionShape(Span<CollisionShapePtr> child_shapes,
                                Span<float4x4> child_transforms);
};

class StaticCompoundCollisionShape : public CollisionShape {
 public:
  StaticCompoundCollisionShape(Span<CollisionShapePtr> child_shapes,
                               Span<float4x4> child_transforms);
};

class MeshCollisionShape : public CollisionShape {
 public:
  MeshCollisionShape(const Mesh &mesh);
};

// class TriangleMeshCollisionShape : public CollisionShape {
//  public:
//   struct TriangleMeshInterface *mesh_interface = nullptr;
//
//   TriangleMeshCollisionShape(TriangleMeshInterface *mesh_interface);
//
//   static TriangleMeshCollisionShape *from_mesh(const Mesh &mesh);
//
//   virtual ~TriangleMeshCollisionShape();
// };

// class ScaledTriangleMeshCollisionShape : public CollisionShape {
//  public:
//   ScaledTriangleMeshCollisionShape(const TriangleMeshCollisionShape *child_shape, float3 scale);
// };
//
// class StaticPlaneCollisionShape : public CollisionShape {
//  public:
//   StaticPlaneCollisionShape(const float3 &plane_normal, float plane_constant);
// };
//
// class CompoundCollisionShape : public CollisionShape {
//  public:
//   Array<CollisionShape::Ptr> child_shapes_;
//
//   CompoundCollisionShape(const VArray<CollisionShape::Ptr> &child_shapes,
//                          const VArray<float4x4> &child_transforms);
// };

}  // namespace blender::bke
