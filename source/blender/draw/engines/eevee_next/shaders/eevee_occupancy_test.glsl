/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Directive for resetting the line numbering so the failing tests lines can be printed.
 * This conflict with the shader compiler error logging scheme.
 * Comment out for correct compilation error line. */
#line 5

#pragma BLENDER_REQUIRE(eevee_occupancy_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_test_lib.glsl)

#define TEST(a, b) if (true)

uvec4 occupancy_to_uint4(OccupancyBits occupancy)
{
  return uvec4(occupancy.bits[0], occupancy.bits[1], occupancy.bits[2], occupancy.bits[3]);
}

void main()
{
  TEST(eevee_occupancy, Occupancy)
  {
    OccupancyBits occup;

    occup = occupancy_from_depth(0.1, 1);
    EXPECT_EQ(occupancy_to_uint4(occup), uvec4(0xFFFFFFFFu, ~0u, ~0u, ~0u));

    occup = occupancy_from_depth(0.6, 1);
    EXPECT_EQ(occupancy_to_uint4(occup), uvec4(0xFFFFFFFEu, ~0u, ~0u, ~0u));

    occup = occupancy_from_depth(0.5, 32);
    EXPECT_EQ(occupancy_to_uint4(occup), uvec4(0xFFFF0000u, ~0u, ~0u, ~0u));

    occup = occupancy_from_depth(0.5, 64);
    EXPECT_EQ(occupancy_to_uint4(occup), uvec4(0u, ~0u, ~0u, ~0u));

    occup = occupancy_from_depth(0.5, 128);
    EXPECT_EQ(occupancy_to_uint4(occup), uvec4(0u, 0u, ~0u, ~0u));

    occup = occupancy_from_depth(33.0 / 64.0, 64);
    EXPECT_EQ(occupancy_to_uint4(occup), uvec4(0u, 0xFFFFFFFEu, ~0u, ~0u));
  }
}
