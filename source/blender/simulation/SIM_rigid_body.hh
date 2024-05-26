/* SPDX-FileCopyrightText: Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sim
 */

#pragma once

#include "BLI_math_vector_types.hh"
#include "BLI_math_quaternion_types.hh"
#include "BLI_set.hh"
#include "BLI_virtual_array_fwd.hh"

#include <functional>

namespace blender::simulation {

using RigidBodyHandle = uintptr_t;

using RigidBodyID = int;

class RigidBodyWorld {
 public:
  using OverlapFilterFn = std::function<bool(const RigidBodyID a, const RigidBodyID b)>;

 private:
  struct RigidBodyWorldImpl *impl_;

 public:
  RigidBodyWorld();
  RigidBodyWorld(const RigidBodyWorld &other);
  ~RigidBodyWorld();

  int bodies_num() const;
  int constraints_num() const;
  int shapes_num() const;

  void set_overlap_filter(OverlapFilterFn fn);
  void clear_overlap_filter();

  float3 gravity() const;
  void set_gravity(const float3 &gravity);
  void set_solver_iterations(int num_solver_iterations);
  void set_split_impulse(bool split_impulse);

  VArray<RigidBodyID> body_ids() const;
  IndexRange add_rigid_bodies(const VArray<float> &masses,
                              const VArray<float3> &inertiae = {});
  void remove_rigid_bodies(const IndexMask &mask);

  //RigidBodyHandle add_rigid_body(float mass, const float3 &inertia = float3(0.0f));
  //void remove_rigid_body(RigidBodyHandle handle);

  //float body_mass(RigidBodyHandle handle) const;
  //float3 body_inertia(RigidBodyHandle handle) const;
  //void set_body_mass(RigidBodyHandle handle, float mass, const float3 &inertia = float3(0.0f));

  //float body_friction(RigidBodyHandle handle) const;
  //void set_body_friction(RigidBodyHandle handle, float value);
  //float body_restitution(RigidBodyHandle handle) const;
  //void body_set_restitution(RigidBodyHandle handle, float value);

  //float body_linear_damping(RigidBodyHandle handle) const;
  //void body_set_linear_damping(RigidBodyHandle handle, float value);
  //float body_angular_damping(RigidBodyHandle handle) const;
  //void body_set_angular_damping(RigidBodyHandle handle, float value);

  //float body_linear_sleep_thresh(RigidBodyHandle handle) const;
  //void body_set_linear_sleep_thresh(RigidBodyHandle handle, float value);
  //float body_angular_sleep_thresh(RigidBodyHandle handle) const;
  //void body_set_angular_sleep_thresh(RigidBodyHandle handle, float value);
  //void body_set_sleep_thresh(RigidBodyHandle handle, float linear, float angular);

  //void body_linear_velocity(RigidBodyHandle handle, float v_out[3]) const;
  //void body_set_linear_velocity(RigidBodyHandle handle, const float v_in[3]);
  //void body_angular_velocity(RigidBodyHandle handle, float v_out[3]) const;
  //void body_set_angular_velocity(RigidBodyHandle handle, const float v_in[3]);
  ///* Linear/Angular factor, used to lock translation/rotation axes. */
  //void body_set_linear_factor(RigidBodyHandle handle, float x, float y, float z);
  //void body_set_angular_factor(RigidBodyHandle handle, float x, float y, float z);

  //void body_set_kinematic_state(RigidBodyHandle handle, int kinematic);

  //int body_activation_state(RigidBodyHandle handle) const;
  //void body_set_activation_state(RigidBodyHandle handle, bool use_deactivation);
  //void body_activate(RigidBodyHandle handle);
  //void body_deactivate(RigidBodyHandle handle);
};

}  // namespace blender::simulation
