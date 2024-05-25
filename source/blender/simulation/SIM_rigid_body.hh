/* SPDX-FileCopyrightText: Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sim
 */

#pragma once

namespace blender::simulation {

class RigidBodyWorld {
 public:
  RigidBodyWorld();
  ~RigidBodyWorld();

  int bodies_num() const;
  int constraints_num() const;
  int shapes_num() const;
};

}  // namespace blender::simulation
