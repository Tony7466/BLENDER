/* SPDX-FileCopyrightText: Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sim
 */

#pragma once

#include "BLI_math_vector_types.hh"
#include "BLI_math_quaternion_types.hh"

#include <functional>

namespace blender::simulation {

struct RigidBodyWorldImpl;

using RigidBodyID = int64_t;

class RigidBodyWorld {
 public:
  using OverlapFilterFn = std::function<bool(const RigidBodyID a, const RigidBodyID b)>;

 private:
  RigidBodyWorldImpl *impl_;

 public:
  RigidBodyWorld();
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
};

}  // namespace blender::simulation
