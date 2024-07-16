/* SPDX-FileCopyrightText: 2020 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_physics_geometry.hh"

#include "testing/testing.h"

namespace blender::bke::tests {

class PhysicsGeometryTest : public testing::Test {
 protected:
  PhysicsGeometryTest() {}

  void test_data(const bke::PhysicsGeometry &physics,
                 const bool has_world,
                 const int bodies_num,
                 const int constraints_num,
                 const int shapes_num)
  {
    EXPECT_EQ(physics.has_world(), has_world);
    EXPECT_EQ(physics.bodies_num(), bodies_num);
    EXPECT_EQ(physics.constraints_num(), constraints_num);
    EXPECT_EQ(physics.shapes_num(), shapes_num);
  }
};

TEST_F(PhysicsGeometryTest, construct)
{
  bke::PhysicsGeometry geo1 = bke::PhysicsGeometry();
  test_data(geo1, false, 0, 0, 0);

  bke::PhysicsGeometry geo2 = bke::PhysicsGeometry(1, 0, 0);
  test_data(geo2, false, 1, 0, 0);

  bke::PhysicsGeometry geo3 = bke::PhysicsGeometry(0, 1, 0);
  test_data(geo3, false, 0, 1, 0);

  bke::PhysicsGeometry geo4 = bke::PhysicsGeometry(0, 0, 1);
  test_data(geo4, false, 0, 0, 1);

  bke::PhysicsGeometry geo5 = bke::PhysicsGeometry(1, 1, 1);
  test_data(geo5, false, 1, 1, 1);
}

TEST_F(PhysicsGeometryTest, create_and_remove_world)
{
  bke::PhysicsGeometry geo = bke::PhysicsGeometry(5, 2, 3);
  test_data(geo, false, 5, 2, 3);

  geo.create_world();
  test_data(geo, true, 5, 2, 3);

  geo.destroy_world();
  test_data(geo, false, 5, 2, 3);
}

}  // namespace blender::bke::tests
