/* SPDX-FileCopyrightText: Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sim
 */

#pragma once

#include "BLI_array.hh"
#include "BLI_cache_mutex.hh"
#include "BLI_math_matrix.hh"

#include "BKE_collision_shape.hh"
#include "BKE_physics_geometry.hh"

#include <shared_mutex>

#ifdef WITH_BULLET
#  include <btBulletDynamicsCommon.h>
#endif

class btDiscreteDynamicsWorld;
class btCollisionConfiguration;
class btCollisionDispatcher;
class btBroadphaseInterface;
class btConstraintSolver;
struct btOverlapFilterCallback;
class btRigidBody;
class btMotionState;
class btCollisionShape;
class btTypedConstraint;

namespace blender::bke {

/**
 * Physics data can be in one of three states:
 * - Mutable: Only one user and has physics data.
 * - Read-Only: Has physics data, but more than one user.
 * - Cached: No physics data (user count irrelevant, always read-only).
 *
 * State can change between Mutable and Read-Only when adding or removing users.
 * Once physics data has been moved the component becomes Cached, at which point going back to
 * mutable or read-only state is impossible (data cannot be added back).
 */

struct PhysicsGeometryImpl : public ImplicitSharingMixin {
  /* If true then the data is read-only and the attribute cache is used instead of direct access to
   * physics data. No Bullet instances are held by this component when cached. */
  std::atomic<bool> is_empty = true;

  /* Protects shared read/write access to physics data. */
  mutable std::shared_mutex data_mutex;

  /* Cache to ensure mapping from body and constraint pointers to indices. */
  mutable CacheMutex shape_index_cache;
  mutable CacheMutex body_index_cache;
  mutable CacheMutex body_collision_shape_cache;
  mutable Array<int> body_collision_shapes;
  mutable CacheMutex constraint_index_cache;
  /* Cache for disable_collisions flags of constraints. These are stored indirectly by Bullet: a
   * constraint disables collisions by adding "constraint refs" to bodies, when adding a constraint
   * to the world. To determine if a constraint disables collisions after the fact requires looping
   * over all constraint refs of the affected bodies. The cache avoids doing that multiple times
   * for each body. */
  mutable CacheMutex constraint_disable_collision_cache;
  mutable Array<bool> constraint_disable_collision;

  int body_num_;
  int constraint_num_;
  CustomData body_data_;
  CustomData constraint_data_;

  btDiscreteDynamicsWorld *world = nullptr;
  btCollisionConfiguration *config = nullptr;
  btCollisionDispatcher *dispatcher = nullptr;
  btBroadphaseInterface *broadphase = nullptr;
  btConstraintSolver *constraint_solver = nullptr;
  btOverlapFilterCallback *overlap_filter = nullptr;

  Array<CollisionShapePtr> shapes;
  Array<btRigidBody *> rigid_bodies;
  Array<btMotionState *> motion_states;
  Array<btTypedConstraint *> constraints;
  Array<btJointFeedback> constraint_feedback;

  PhysicsGeometryImpl();
  PhysicsGeometryImpl(int bodies_num, int constraints_num, int shapes_num);
  PhysicsGeometryImpl(const PhysicsGeometryImpl &other);
  ~PhysicsGeometryImpl();

  PhysicsGeometryImpl &operator=(const PhysicsGeometryImpl &other);

  void delete_self() override;

  void resize(int body_num, int constraint_num);
  void realloc();

  void tag_body_topology_changed();
  void tag_body_collision_shape_changed();
  void tag_constraint_disable_collision_changed();

  void ensure_body_indices() const;
  void ensure_body_collision_shapes() const;
  void ensure_constraint_indices() const;
  void ensure_constraint_disable_collision() const;

  void create_world();
  void destroy_world();
  void move_world(PhysicsGeometryImpl &src);

  bool try_copy_to_world_data(const PhysicsGeometryImpl &src,
                              const IndexMask &src_body_mask,
                              const IndexMask &src_constraint_mask,
                              const Set<std::string> &ignored_attributes);
  bool try_copy_to_custom_data(const PhysicsGeometryImpl &src,
                               const IndexMask &src_body_mask,
                               const IndexMask &src_constraint_mask,
                               const Set<std::string> &ignored_attributes);
  void remove_attributes_from_customdata();
  bool try_move_data(const PhysicsGeometryImpl &src,
                     bool move_world,
                     const IndexMask &src_body_mask,
                     const IndexMask &src_constraint_mask,
                     const IndexMask &src_shape_mask,
                     int dst_body_offset,
                     int dst_constraint_offset,
                     int dst_shape_offset);

  bke::AttributeAccessor attributes(bool force_cache = false) const;
  bke::MutableAttributeAccessor attributes_for_write(bool force_cache = false);
};

struct CollisionShapeImpl {
  ~CollisionShapeImpl() = delete;

  void destroy();

  btCollisionShape &as_bullet_shape()
  {
    return *reinterpret_cast<btCollisionShape *>(this);
  }

  const btCollisionShape &as_bullet_shape() const
  {
    return *reinterpret_cast<const btCollisionShape *>(this);
  }

  operator btCollisionShape *()
  {
    return reinterpret_cast<btCollisionShape *>(this);
  }

  operator const btCollisionShape *() const
  {
    return reinterpret_cast<const btCollisionShape *>(this);
  }

  static CollisionShapeImpl *wrap(btCollisionShape *shape)
  {
    return reinterpret_cast<CollisionShapeImpl *>(shape);
  }

  static const CollisionShapeImpl *wrap(const btCollisionShape *shape)
  {
    return reinterpret_cast<const CollisionShapeImpl *>(shape);
  }
};

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

inline float4x4 to_blender(const btTransform &t)
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
  return btTransform(to_bullet(t.view<3, 3>()), to_bullet(t.location()));
}

inline int activation_state_to_bullet(bke::PhysicsGeometry::BodyActivationState state)
{
  using BodyActivationState = bke::PhysicsGeometry::BodyActivationState;
  switch (state) {
    case BodyActivationState::AlwaysActive:
      return DISABLE_DEACTIVATION;
    case BodyActivationState::Active:
      return ACTIVE_TAG;
    case BodyActivationState::WantsSleep:
      return WANTS_DEACTIVATION;
    case BodyActivationState::Sleeping:
      return ISLAND_SLEEPING;
    case BodyActivationState::AlwaysSleeping:
      return DISABLE_SIMULATION;
  }
  return DISABLE_SIMULATION;
}

inline bke::PhysicsGeometry::BodyActivationState activation_state_to_blender(int bt_state)
{
  using BodyActivationState = bke::PhysicsGeometry::BodyActivationState;
  switch (bt_state) {
    case ACTIVE_TAG:
      return BodyActivationState::Active;
    case ISLAND_SLEEPING:
      return BodyActivationState::Sleeping;
    case WANTS_DEACTIVATION:
      return BodyActivationState::WantsSleep;
    case DISABLE_DEACTIVATION:
      return BodyActivationState::AlwaysActive;
    case DISABLE_SIMULATION:
      return BodyActivationState::AlwaysSleeping;
    default:
      BLI_assert_unreachable();
      return BodyActivationState::AlwaysSleeping;
  }
}

inline btTypedConstraintType to_bullet(bke::PhysicsGeometry::ConstraintType type)
{
  using ConstraintType = bke::PhysicsGeometry::ConstraintType;
  switch (type) {
    case ConstraintType::None:
      return FIXED_CONSTRAINT_TYPE;
    case ConstraintType::Fixed:
      return FIXED_CONSTRAINT_TYPE;
    case ConstraintType::Point:
      return POINT2POINT_CONSTRAINT_TYPE;
    case ConstraintType::Hinge:
      return HINGE_CONSTRAINT_TYPE;
    case ConstraintType::Slider:
      return SLIDER_CONSTRAINT_TYPE;
    case ConstraintType::ConeTwist:
      return CONETWIST_CONSTRAINT_TYPE;
    case ConstraintType::SixDoF:
      return D6_CONSTRAINT_TYPE;
    case ConstraintType::SixDoFSpring:
      return D6_SPRING_CONSTRAINT_TYPE;
    case ConstraintType::SixDoFSpring2:
      return D6_SPRING_2_CONSTRAINT_TYPE;
    case ConstraintType::Contact:
      return CONTACT_CONSTRAINT_TYPE;
    case ConstraintType::Gear:
      return GEAR_CONSTRAINT_TYPE;
  }
  return FIXED_CONSTRAINT_TYPE;
}

inline bke::PhysicsGeometry::ConstraintType to_blender(btTypedConstraintType type)
{
  using ConstraintType = bke::PhysicsGeometry::ConstraintType;
  switch (type) {
    case FIXED_CONSTRAINT_TYPE:
      return ConstraintType::Fixed;
    case POINT2POINT_CONSTRAINT_TYPE:
      return ConstraintType::Point;
    case HINGE_CONSTRAINT_TYPE:
      return ConstraintType::Hinge;
    case SLIDER_CONSTRAINT_TYPE:
      return ConstraintType::Slider;
    case CONETWIST_CONSTRAINT_TYPE:
      return ConstraintType::ConeTwist;
    case D6_CONSTRAINT_TYPE:
      return ConstraintType::SixDoF;
    case D6_SPRING_CONSTRAINT_TYPE:
      return ConstraintType::SixDoFSpring;
    case D6_SPRING_2_CONSTRAINT_TYPE:
      return ConstraintType::SixDoFSpring2;
    case CONTACT_CONSTRAINT_TYPE:
      return ConstraintType::Contact;
    case GEAR_CONSTRAINT_TYPE:
      return ConstraintType::Gear;

    default:
      return ConstraintType::None;
  }
}

inline std::optional<CollisionShape::ShapeType> to_blender(const int bt_shape_type)
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

}  // namespace blender::bke
