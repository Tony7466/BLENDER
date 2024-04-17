/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <cstdint>
#include <memory>

#include "BLI_map.hh"
#include "BLI_math_vector_types.hh"

#include "COM_cached_resource.hh"

namespace blender::realtime_compositor {

class Context;

/* ------------------------------------------------------------------------------------------------
 * VanVliet Gaussian Coefficients Key.
 */
class VanVlietGaussianCoefficientsKey {
 public:
  float sigma;

  VanVlietGaussianCoefficientsKey(float sigma);

  uint64_t hash() const;
};

bool operator==(const VanVlietGaussianCoefficientsKey &a,
                const VanVlietGaussianCoefficientsKey &b);

/* -------------------------------------------------------------------------------------------------
 * VanVliet Gaussian Coefficients.
 *
 * A caches resource that computes and caches the coefficients of the fourth order IIR filter
 * approximating a Gaussian filter computed using Van Vliet's design method. */
class VanVlietGaussianCoefficients : public CachedResource {
 private:
  double4 feedback_coefficients_;
  double feedforward_coefficient_;

 public:
  VanVlietGaussianCoefficients(Context &context, float sigma);

  const double4 &feedback_coefficients() const;
  const double feedforward_coefficient() const;
};

/* ------------------------------------------------------------------------------------------------
 * VanVliet Gaussian Coefficients Container.
 */
class VanVlietGaussianCoefficientsContainer : CachedResourceContainer {
 private:
  Map<VanVlietGaussianCoefficientsKey, std::unique_ptr<VanVlietGaussianCoefficients>> map_;

 public:
  void reset() override;

  /* Check if there is an available VanVlietGaussianCoefficients cached resource with the given
   * parameters in the container, if one exists, return it, otherwise, return a newly created one
   * and add it to the container. In both cases, tag the cached resource as needed to keep it
   * cached for the next evaluation. */
  VanVlietGaussianCoefficients &get(Context &context, float sigma);
};

}  // namespace blender::realtime_compositor
