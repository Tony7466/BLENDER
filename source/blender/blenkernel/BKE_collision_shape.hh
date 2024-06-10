/* SPDX-FileCopyrightText: Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sim
 */

#pragma once

#include "BKE_physics_geometry.hh"

#include "BLI_math_quaternion_types.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_utility_mixins.hh"

#include <functional>

namespace blender::bke {

class CollisionShape;
class BoxCollisionShape;
class SphereCollisionShape;

struct CollisionShapeImpl;

class CollisionShape : public ImplicitSharingMixin {
 public:
  using Ptr = ImplicitSharingPtr<CollisionShape>;

  /* Not all shape types may be supported. */
  enum class ShapeType {
    Invalid,
    Empty,

    /* Polyhedral convex shapes. */
    Box,
    Triangle,
    Tetrahedral,
    ConvexTriangleMesh,
    ConvexHull,
    ConvexPointCloud,
    //CustomPolyhedral,

    /* Implicit convex shapes. */
    Sphere,
    MultiSphere,
    Capsule,
    Cone,
    Cylinder,
    UniformScaling,
    MinkowskiSum,
    MinkowskiDifference,
    Box2D,
    Convex2D,
    //CustomConvex,

    /* Concave shapes. */
    TriangleMesh,
    ScaledTriangleMesh,
    StaticPlane,
    //CustomConcave,

    /* Others */
    Compound,
  };

 protected:
  CollisionShapeImpl *impl_;

 public:
  CollisionShape() = delete;
  ~CollisionShape();

  void delete_self() override;

  CollisionShapeImpl &impl();
  const CollisionShapeImpl &impl() const;

  ShapeType type() const;

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

  float3 calculate_local_inertia(float mass) const;

 protected:
  CollisionShape(CollisionShapeImpl *impl);

  friend class PhysicsGeometry;
};

class EmptyCollisionShape : public CollisionShape {
 public:
  EmptyCollisionShape();
};

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

class ConeCollisionShape : public CollisionShape {
 public:
  ConeCollisionShape(float radius, float height);
};

class CylinderCollisionShape : public CollisionShape {
 public:
  CylinderCollisionShape(float radius, float height);
};

class UniformScalingCollisionShape : public CollisionShape {
 public:
  UniformScalingCollisionShape(const CollisionShape *child_shape, float scale);
};

class TriangleMeshCollisionShape : public CollisionShape {
 public:
  TriangleMeshCollisionShape(const Mesh &mesh);
};

class ScaledTriangleMeshCollisionShape : public CollisionShape {
 public:
  ScaledTriangleMeshCollisionShape(const TriangleMeshCollisionShape *child_shape, float3 scale);
};

class StaticPlaneCollisionShape : public CollisionShape {
 public:
  StaticPlaneCollisionShape(const float3 &plane_normal, float plane_constant);
};

class CompoundCollisionShape : public CollisionShape {
 public:
  CompoundCollisionShape(const VArray<const CollisionShape *> &child_shapes,
                         const VArray<float4x4> &child_transforms);
};

}  // namespace blender::bke
