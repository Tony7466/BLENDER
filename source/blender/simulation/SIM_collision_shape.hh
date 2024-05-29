/* SPDX-FileCopyrightText: Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sim
 */

#pragma once

#include "BLI_math_quaternion_types.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_utility_mixins.hh"
#include "SIM_physics_geometry.hh"

#include <functional>

namespace blender::simulation {

// using RigidBodyID = int;

class CollisionShape;
class BoxCollisionShape;
class SphereCollisionShape;
// class RigidBody;
// class RigidBodyWorld;

// struct RigidBodyWorldImpl;
// struct RigidBodyImpl;
struct CollisionShapeImpl;

// class RigidBodyWorld {
//  public:
//   using OverlapFilterFn = std::function<bool(const RigidBodyID a, const RigidBodyID b)>;
//
//  private:
//   RigidBodyWorldImpl *impl_;
//
//  public:
//   RigidBodyWorld();
//   RigidBodyWorld(const RigidBodyWorld &other);
//   ~RigidBodyWorld();
//
//   int bodies_num() const;
//   int constraints_num() const;
//
//   void set_overlap_filter(OverlapFilterFn fn);
//   void clear_overlap_filter();
//
//   float3 gravity() const;
//   void set_gravity(const float3 &gravity);
//   void set_solver_iterations(int num_solver_iterations);
//   void set_split_impulse(bool split_impulse);
//
//   void add_rigid_body(RigidBody *body);
//   void remove_rigid_body(RigidBody *body);
//
//   float body_mass(RigidBody *body) const;
//   float3 body_inertia(RigidBody *body) const;
//   void set_body_mass(RigidBody *body, float mass, const float3 &inertia = float3(0.0f));

// float body_friction(RigidBodyHandle handle) const;
// void set_body_friction(RigidBodyHandle handle, float value);
// float body_restitution(RigidBodyHandle handle) const;
// void body_set_restitution(RigidBodyHandle handle, float value);

// float body_linear_damping(RigidBodyHandle handle) const;
// void body_set_linear_damping(RigidBodyHandle handle, float value);
// float body_angular_damping(RigidBodyHandle handle) const;
// void body_set_angular_damping(RigidBodyHandle handle, float value);

// float body_linear_sleep_thresh(RigidBodyHandle handle) const;
// void body_set_linear_sleep_thresh(RigidBodyHandle handle, float value);
// float body_angular_sleep_thresh(RigidBodyHandle handle) const;
// void body_set_angular_sleep_thresh(RigidBodyHandle handle, float value);
// void body_set_sleep_thresh(RigidBodyHandle handle, float linear, float angular);

// void body_linear_velocity(RigidBodyHandle handle, float v_out[3]) const;
// void body_set_linear_velocity(RigidBodyHandle handle, const float v_in[3]);
// void body_angular_velocity(RigidBodyHandle handle, float v_out[3]) const;
// void body_set_angular_velocity(RigidBodyHandle handle, const float v_in[3]);
///* Linear/Angular factor, used to lock translation/rotation axes. */
// void body_set_linear_factor(RigidBodyHandle handle, float x, float y, float z);
// void body_set_angular_factor(RigidBodyHandle handle, float x, float y, float z);

// void body_set_kinematic_state(RigidBodyHandle handle, int kinematic);

// int body_activation_state(RigidBodyHandle handle) const;
// void body_set_activation_state(RigidBodyHandle handle, bool use_deactivation);
// void body_activate(RigidBodyHandle handle);
// void body_deactivate(RigidBodyHandle handle);
//};

// class RigidBody {
//  private:
//   RigidBodyImpl *impl_;
//
//  public:
//   RigidBody(const CollisionShape *shape, float mass, const float3 &local_inertia);
//   ~RigidBody();
//
//   friend class RigidBodyWorld;
// };

class CollisionShape : public NonCopyable {
 public:
  enum class ShapeType {
    Unknown,
    Empty,
    Box,
    Sphere,
  };

 protected:
  CollisionShapeImpl *impl_;

 public:
  CollisionShape();
  ~CollisionShape();

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

}  // namespace blender::simulation
