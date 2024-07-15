/* SPDX-FileCopyrightText: 2020 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_physics_geometry.hh"

#include "testing/testing.h"

namespace blender::bke::tests {

TEST(PhysicsGeometryTest, construct)
{
  bke::PhysicsGeometry geo1 = bke::PhysicsGeometry();
  EXPECT_EQ(geo1.bodies_num(), 0);
  EXPECT_EQ(geo1.constraints_num(), 0);
  EXPECT_EQ(geo1.shapes_num(), 0);

  bke::PhysicsGeometry geo2 = bke::PhysicsGeometry(1, 0, 0);
  EXPECT_EQ(geo2.bodies_num(), 1);
  EXPECT_EQ(geo2.constraints_num(), 0);
  EXPECT_EQ(geo2.shapes_num(), 0);

  bke::PhysicsGeometry geo3 = bke::PhysicsGeometry(0, 1, 0);
  EXPECT_EQ(geo3.bodies_num(), 0);
  EXPECT_EQ(geo3.constraints_num(), 1);
  EXPECT_EQ(geo3.shapes_num(), 0);

  bke::PhysicsGeometry geo4 = bke::PhysicsGeometry(0, 0, 1);
  EXPECT_EQ(geo4.bodies_num(), 0);
  EXPECT_EQ(geo4.constraints_num(), 0);
  EXPECT_EQ(geo4.shapes_num(), 1);

  bke::PhysicsGeometry geo5 = bke::PhysicsGeometry(1, 1, 1);
  EXPECT_EQ(geo5.bodies_num(), 1);
  EXPECT_EQ(geo5.constraints_num(), 1);
  EXPECT_EQ(geo5.shapes_num(), 1);

  bke::PhysicsGeometry geo6 = bke::PhysicsGeometry(geo5);
  EXPECT_EQ(geo6.bodies_num(), 1);
  EXPECT_EQ(geo6.constraints_num(), 1);
  EXPECT_EQ(geo6.shapes_num(), 1);
}

}  // namespace blender::bke::tests
