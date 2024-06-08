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
    Convex,
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

class SphereCollisionShape : public CollisionShape {
 public:
  SphereCollisionShape(float radius);

  float radius() const;
};

}  // namespace blender::bke
