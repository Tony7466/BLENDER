/* SPDX-FileCopyrightText: 2004-2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sim
 */

#include "SIM_rigid_body.hh"

namespace blender::simulation {

RigidBodyWorld::RigidBodyWorld() {}

RigidBodyWorld::~RigidBodyWorld() {}

int RigidBodyWorld::bodies_num() const
{
  return 0;
}

int RigidBodyWorld::constraints_num() const
{
  return 0;
}

int RigidBodyWorld::shapes_num() const
{
  return 0;
}

}  // namespace blender::simulation
