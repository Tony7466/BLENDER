/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Directive for resetting the line numbering so the failing tests lines can be printed.
 * This conflict with the shader compiler error logging scheme.
 * Comment out for correct compilation error line. */
#line 9

#pragma BLENDER_REQUIRE(gpu_shader_utildefines_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_math_vector_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_occupancy_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_test_lib.glsl)

#define TEST(a, b) if (true)

uint horizon_scan_angles_bitmask(float theta_min, float theta_max)
{
  vec2 theta = vec2(theta_min, theta_max);
  const int bitmask_len = 32;
  /* Algorithm 1, line 18. Re-ordered to make sure to clamp to the hemisphere range. */
  vec2 ratio = saturate(theta * M_1_PI + 0.5);
  uint a = uint(floor(float(bitmask_len) * ratio.x));
  /* The paper is wrong here. The additional half Pi is not needed . */
  uint b = uint(ceil(float(bitmask_len) * (ratio.y - ratio.x)));
  /* Algorithm 1, line 19. */
  return (((b < 32u) ? 1u << b : 0u) - 1u) << a;
}

void main()
{
  TEST(eevee_fff, Slices)
  {
    EXPECT_EQ(horizon_scan_angles_bitmask(M_PI_2 * -1.0, M_PI_2 * -1.0), 0x00000000u);
    EXPECT_EQ(horizon_scan_angles_bitmask(M_PI_2 * -1.0, M_PI_2 * -0.97), 0x00000001u);
    EXPECT_EQ(horizon_scan_angles_bitmask(M_PI_2 * -1.0, M_PI_2 * -0.5), 0x000000FFu);
    EXPECT_EQ(horizon_scan_angles_bitmask(M_PI_2 * -1.0, M_PI_2 * 0.0), 0x0000FFFFu);
    EXPECT_EQ(horizon_scan_angles_bitmask(M_PI_2 * -0.5, M_PI_2 * 0.0), 0x0000FF00u);
    EXPECT_EQ(horizon_scan_angles_bitmask(M_PI_2 * -0.5, M_PI_2 * 0.5), 0x00FFFF00u);
    EXPECT_EQ(horizon_scan_angles_bitmask(M_PI_2 * 0.0, M_PI_2 * 0.5), 0x00FF0000u);
    EXPECT_EQ(horizon_scan_angles_bitmask(M_PI_2 * 0.0, M_PI_2 * 1.0), 0xFFFF0000u);
    EXPECT_EQ(horizon_scan_angles_bitmask(M_PI_2 * 0.5, M_PI_2 * 1.0), 0xFF000000u);
    EXPECT_EQ(horizon_scan_angles_bitmask(M_PI_2 * -1.0, M_PI_2 * 1.0), 0xFFFFFFFFu);
    EXPECT_EQ(horizon_scan_angles_bitmask(M_PI_2 * -2.0, M_PI_2 * 2.0), 0xFFFFFFFFu);
    EXPECT_EQ(horizon_scan_angles_bitmask(M_PI_2 * 2.0 / 16.0, M_PI_2 * 2.0 / 16.0), 0x00000000u);

    // float theta = M_PI_2 * 0.5;
    // uint slice_visbits_a = 1u << uint(saturate((theta + M_PI_2) / M_PI) * 32.0);
    // float unpack = float(findLSB(slice_visbits_a >> 16)) * M_PI_2 / 16.0;
    // EXPECT_NEAR(unpack, M_PI_2 * 0.5, 1e-2f);

    // theta = -M_PI_2 * 0.5;
    // uint slice_visbits_b = 1u << uint(saturate((theta + M_PI_2) / M_PI) * 32.0);
    // unpack = ;
    // EXPECT_NEAR(unpack, -M_PI_2 * 0.5, 1e-2f);
  }
}
