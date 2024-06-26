/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "testing/testing.h"

#include "util/math.h"
#include "util/types.h"

#include "kernel/device/cpu/compat.h"
#include "kernel/device/cpu/globals.h"

#include "kernel/camera/camera.h"
#include "kernel/camera/projection.h"
#include "kernel/types.h"

CCL_NAMESPACE_BEGIN

/**
 * @brief Test #fisheye_lens_polynomial_to_direction and its inverse
 * #direction_to_fisheye_lens_polynomial by checking if sensor position equals
 * direction_to_fisheye_lens_polynomial(fisheye_lens_polynomial_to_direction/sensor position))
 * for a couple of sensor positions and a couple of different sets of parameters.
 */
TEST(KernelCamera, FisheyeLensPolynomialRoundtrip)
{
  const float fov = 150.0f / 180.0f * M_PI_F;
  const float width = 36;
  const float height = 41.142857142857144;

  /* Trivial case: The coefficients create a perfect equidistant fisheye */
  float4 k_equidistant = make_float4(-5.79e-02, 0.0, 0.0, 0.0);

  /* The coefficients mimic a stereographic fisheye model */
  float4 k_stereographic = make_float4(-5.79e-02, 0.0, 9.48e-05, -7.67e-06);

  /* The coefficients mimic a rectilinear camera (badly, but the point is to have a wide range of
   * tests). */
  float4 k_rectilinear = make_float4(-6.50e-02, 0.0, 8.32e-05, -1.80e-06);

  float4 parameters[]{k_equidistant, k_stereographic, k_rectilinear};

  const std::pair<float, float> points[]{
      {0.1, 0.4},
      {0.1, 0.5},
      {0.1, 0.7},
      {0.5, 0.5},
      {0.5, 0.9},
      {0.6, 0.9},
  };

  /* In the test cases k0 = k2 = 0, because for non-zero values the model is not smooth at the
   * center, but real lenses are really smooth near the center. In order to test the method
   * thoroughly, nonzero values are tested for both parameters. */
  for (const float k0 : {0.0, -1e-2}) {
    for (const float k2 : {0.0, -1e-4}) {
      for (float4 &k : parameters) {
        k.y = k2;
        for (std::pair<float, float> const &pt : points) {
          const float x = pt.first;
          const float y = pt.second;
          const float3 direction = fisheye_lens_polynomial_to_direction(
              pt.first, pt.second, k0, k, fov, width, height);

          ASSERT_NEAR(len(direction), 1, 1e-6) << "x: " << x << std::endl
                                               << "y: " << y << std::endl
                                               << "k2: " << k2;

          const float2 reprojection = direction_to_fisheye_lens_polynomial(
              direction, k0, k, width, height);

          ASSERT_NEAR(reprojection.x, x, 1e-6);
          ASSERT_NEAR(reprojection.y, y, 1e-6);
        }
      }
    }
  }
}

CCL_NAMESPACE_END
