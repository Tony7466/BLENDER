/* SPDX-FileCopyrightText: 2020 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_geometry_set.hh"
#include "BKE_physics_geometry.hh"

#include "GEO_join_geometries.hh"

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

TEST_F(PhysicsGeometryTest, join_geometry)
{
  bke::PhysicsGeometry geo1 = bke::PhysicsGeometry(5, 2, 3);
  test_data(geo1, false, 5, 2, 3);

  bke::PhysicsGeometry geo2 = bke::PhysicsGeometry(0, 0, 0);
  geo2.create_world();
  test_data(geo2, true, 0, 0, 0);

  bke::PhysicsGeometry geo3 = bke::PhysicsGeometry(2, 1, 1);
  geo3.create_world();
  test_data(geo3, true, 2, 1, 1);

  Array<bke::GeometrySet> geometry_sets = {bke::GeometrySet::from_physics(&geo1),
                                           bke::GeometrySet::from_physics(&geo2),
                                           bke::GeometrySet::from_physics(&geo3)};
  GeometrySet result = geometry::join_geometries(geometry_sets, {});
  EXPECT_TRUE(result.has_physics());
  const bke::PhysicsGeometry &geo_result = *result.get_physics();
  test_data(geo_result, true, 7, 3, 4);
}

}  // namespace blender::bke::tests
