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

#ifdef WITH_BULLET
#  include <BulletDynamics/Dynamics/btDynamicsWorld.h>
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

class PhysicsWorldData;

// /* Utility with an atomic flag indicating when a cache needs to be updated. */
// struct CacheGuard {
//  private:
//   std::atomic<bool> flag_ = false;

//  public:
//   bool is_valid() const;
//   bool is_dirty() const;
//   void tag_dirty();
//   void ensure(std::mutex &mutex, FunctionRef<void()> compute_cache);
// };

struct PhysicsGeometryImpl : public ImplicitSharingMixin {
  using BodyAttribute = PhysicsGeometry::BodyAttribute;
  using ConstraintAttribute = PhysicsGeometry::ConstraintAttribute;

  using OverlapFilterFn = std::function<bool(const int a, const int b)>;

  using CacheFlag = std::atomic<bool>;

  /* Cache for readers storing copies of physics data in custom data. */
  mutable CacheFlag custom_data_read_cache_valid = false;
  /* Valid when body collision shape pointers match pointers from the
   * shapes list, as stored in the body shapes index attribute. */
  mutable CacheFlag body_collision_shapes_valid = false;
  /* Valid when is_static flags match the world data motion type for each body. */
  mutable CacheFlag body_is_static_valid = false;
  /* Valid when mass matches the world data motion type for each body. */
  mutable CacheFlag body_mass_valid = false;
  /* Valid when internal constraints have been updated to specified types and bodies. */
  mutable CacheFlag constraints_valid = false;
  /* Cache for disable_collisions flags of constraints. These are stored indirectly by Bullet: a
   * constraint disables collisions by adding "constraint refs" to bodies, when adding a constraint
   * to the world. To determine if a constraint disables collisions after the fact requires looping
   * over all constraint refs of the affected bodies. The cache avoids doing that multiple times
   * for each body. */
  mutable CacheFlag constraint_disable_collision_valid = false;

  int body_num_;
  int constraint_num_;
  CustomData body_data_;
  CustomData constraint_data_;

  Array<CollisionShapePtr> shapes;

  mutable PhysicsWorldData *world_data = nullptr;
  /* Protects shared read/write access to world data. */
  mutable std::mutex data_mutex;

  PhysicsGeometryImpl();
  PhysicsGeometryImpl(int body_num, int constraint_num, int shape_num);
  PhysicsGeometryImpl(const PhysicsGeometryImpl &other);
  ~PhysicsGeometryImpl();

  PhysicsGeometryImpl &operator=(const PhysicsGeometryImpl &other);

  void delete_self() override;

  void tag_read_cache_changed();
  void tag_body_topology_changed();
  void tag_constraint_disable_collision_changed();
  void tag_body_collision_shape_changed();
  void tag_body_is_static_changed();
  void tag_body_mass_changed();
  void tag_constraints_changed();

  bool has_builtin_attribute_custom_data_layer(BodyAttribute attribute) const;
  bool has_builtin_attribute_custom_data_layer(ConstraintAttribute attribute) const;

  /* Make sure all world data has been copied to the custom data read cache. */
  void ensure_read_cache() const;
  /* Make sure attributes with write caches have been transferred to world data. */
  void ensure_motion_type();
  void ensure_constraints();
  void ensure_custom_data_attribute(BodyAttribute attribute) const;
  void ensure_custom_data_attribute(ConstraintAttribute attribute) const;

  void create_world();
  void destroy_world();

  void set_overlap_filter(OverlapFilterFn fn);
  void clear_overlap_filter();

  float3 gravity() const;
  void set_gravity(const float3 &gravity);
  void set_solver_iterations(int num_solver_iterations);
  void set_split_impulse(bool split_impulse);

  void step_simulation(float delta_time);

  void remove_attributes_from_customdata();

  void move_or_copy_selection(const PhysicsGeometryImpl &src,
                              const IndexMask &src_body_mask,
                              const IndexMask &src_constraint_mask,
                              const bke::AnonymousAttributePropagationInfo &propagation_info);
  bool try_move_data(const PhysicsGeometryImpl &src,
                     int body_num,
                     int constraint_num,
                     const IndexMask &src_body_mask,
                     const IndexMask &src_constraint_mask,
                     int dst_body_offset,
                     int dst_constraint_offset);

  void compute_local_inertia(const IndexMask &selection);

  void apply_force(const IndexMask &selection,
                   const VArray<float3> &forces,
                   const VArray<float3> &relative_positions = {});
  void apply_torque(const IndexMask &selection, const VArray<float3> &torques);

  void apply_impulse(const IndexMask &selection,
                     const VArray<float3> &impulses,
                     const VArray<float3> &relative_positions = {});
  void apply_angular_impulse(const IndexMask &selection, const VArray<float3> &angular_impulses);
  void clear_forces(const IndexMask &selection);

  // void create_constraints(const IndexMask &selection,
  //                         const VArray<int> &types,
  //                         const VArray<int> &bodies1,
  //                         const VArray<int> &bodies2);

  /* Validate internal relations, for debugging and testing purposes.
   * Should only be used in tests or debug mode. */
  bool validate_world_data();

  bke::AttributeAccessor attributes() const;
  bke::MutableAttributeAccessor attributes_for_write();

 private:
  void ensure_motion_type_no_lock();
  void ensure_body_collision_shapes_no_lock();
  void ensure_body_is_static_no_lock();
  void ensure_body_masses_no_lock();
  void ensure_constraints_no_lock();
  void ensure_custom_data_attribute_no_lock(BodyAttribute attribute);
  void ensure_custom_data_attribute_no_lock(ConstraintAttribute attribute);

  bke::AttributeAccessor custom_data_attributes() const;
  bke::MutableAttributeAccessor custom_data_attributes_for_write();
  bke::AttributeAccessor world_data_attributes() const;
  bke::MutableAttributeAccessor world_data_attributes_for_write();
};

class PhysicsWorldData : NonCopyable, NonMovable {
 public:
  using OverlapFilterFn = std::function<bool(const int a, const int b)>;

 private:
  btDiscreteDynamicsWorld *world_ = nullptr;
  btCollisionConfiguration *config_ = nullptr;
  btCollisionDispatcher *dispatcher_ = nullptr;
  btBroadphaseInterface *broadphase_ = nullptr;
  btConstraintSolver *constraint_solver_ = nullptr;
  btOverlapFilterCallback *overlap_filter_ = nullptr;

  Array<btRigidBody *> rigid_bodies_;
  Array<btMotionState *> motion_states_;
  Array<btTypedConstraint *> constraints_;
  Array<btJointFeedback> constraint_feedback_;

  mutable CacheMutex body_index_cache_;
  mutable CacheMutex constraint_index_cache_;
  mutable CacheMutex bodies_in_world_cache_;

 public:
  PhysicsWorldData();
  PhysicsWorldData(int body_num, int constraint_num);
  ~PhysicsWorldData();

  void resize(int body_num, int constraint_num);
  void resize(int body_num,
              int constraint_num,
              const IndexMask &src_body_mask,
              const IndexMask &src_constraint_mask,
              int dst_body_offset,
              int dst_constraint_offset);
  void move(const IndexMask &src_body_mask,
            const IndexMask &src_constraint_mask,
            int dst_body_offset,
            int dst_constraint_offset);

  void tag_bodies_in_world() const;

  void ensure_body_indices() const;
  void ensure_constraint_indices() const;
  void ensure_bodies_in_world();

  /* Compute the indices of body collision shape pointers in the span. */
  void compute_body_shape_indices(Span<CollisionShapePtr> shapes,
                                  MutableSpan<int> r_indices) const;
  /* Compute which constraints disable collisions, this is not stored directly in Bullet
   * constraints. */
  void compute_disable_collision_flags(MutableSpan<bool> r_flags);

  void create_constraints(const IndexMask &selection,
                          const VArray<int> &types,
                          const VArray<int> &bodies1,
                          const VArray<int> &bodies2);

  void set_overlap_filter(OverlapFilterFn fn);
  void clear_overlap_filter();

  float3 gravity() const;
  void set_gravity(const float3 &gravity);
  void set_solver_iterations(int num_solver_iterations);
  void set_split_impulse(bool split_impulse);

  void step_simulation(float delta_time);

  void set_body_shapes(const IndexMask &selection,
                       const Span<CollisionShapePtr> shapes,
                       const Span<int> shape_handles);
  void set_body_static(const IndexMask &selection, const Span<bool> is_static);
  void set_body_mass(const IndexMask &selection, const Span<float> masses);

  void apply_force(const IndexMask &selection,
                   const VArray<float3> &forces,
                   const VArray<float3> &relative_positions = {});
  void apply_torque(const IndexMask &selection, const VArray<float3> &torques);

  void apply_impulse(const IndexMask &selection,
                     const VArray<float3> &impulses,
                     const VArray<float3> &relative_positions = {});
  void apply_angular_impulse(const IndexMask &selection, const VArray<float3> &angular_impulses);
  void clear_forces(const IndexMask &selection);

  /* For VArray access. */
  Span<btRigidBody *> bodies() const;
  Span<btTypedConstraint *> constraints() const;
  const btDynamicsWorld &world() const;

 private:
  void create_world();
  void destroy_world();
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

// inline bke::PhysicsGeometry::ConstraintType to_blender(btTypedConstraintType type)
//{
//   using ConstraintType = bke::PhysicsGeometry::ConstraintType;
//   switch (type) {
//     case FIXED_CONSTRAINT_TYPE:
//       return ConstraintType::Fixed;
//     case POINT2POINT_CONSTRAINT_TYPE:
//       return ConstraintType::Point;
//     case HINGE_CONSTRAINT_TYPE:
//       return ConstraintType::Hinge;
//     case SLIDER_CONSTRAINT_TYPE:
//       return ConstraintType::Slider;
//     case CONETWIST_CONSTRAINT_TYPE:
//       return ConstraintType::ConeTwist;
//     case D6_CONSTRAINT_TYPE:
//       return ConstraintType::SixDoF;
//     case D6_SPRING_CONSTRAINT_TYPE:
//       return ConstraintType::SixDoFSpring;
//     case D6_SPRING_2_CONSTRAINT_TYPE:
//       return ConstraintType::SixDoFSpring2;
//     case CONTACT_CONSTRAINT_TYPE:
//       return ConstraintType::Contact;
//     case GEAR_CONSTRAINT_TYPE:
//       return ConstraintType::Gear;
//
//     default:
//       return ConstraintType::None;
//   }
// }

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
