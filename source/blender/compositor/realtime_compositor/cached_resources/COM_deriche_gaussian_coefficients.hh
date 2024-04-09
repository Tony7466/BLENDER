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
 * Deriche Gaussian Coefficients Key.
 */
class DericheGaussianCoefficientsKey {
 public:
  float sigma;

  DericheGaussianCoefficientsKey(float sigma);

  uint64_t hash() const;
};

bool operator==(const DericheGaussianCoefficientsKey &a, const DericheGaussianCoefficientsKey &b);

/* -------------------------------------------------------------------------------------------------
 * Deriche Gaussian Coefficients.
 *
 * A caches resource that computes and caches the coefficients of the fourth order IIR filter
 * approximating a Gaussian filter computed using Deriche's design method. */
class DericheGaussianCoefficients : public CachedResource {
 private:
  float4 feedback_coefficients_;
  float4 causal_feedforward_coefficients_;
  float4 non_causal_feedforward_coefficients_;

 public:
  DericheGaussianCoefficients(Context &context, float sigma);

  const float4 &feedback_coefficients() const;
  const float4 &causal_feedforward_coefficients() const;
  const float4 &non_causal_feedforward_coefficients() const;
};

/* ------------------------------------------------------------------------------------------------
 * Deriche Gaussian Coefficients Container.
 */
class DericheGaussianCoefficientsContainer : CachedResourceContainer {
 private:
  Map<DericheGaussianCoefficientsKey, std::unique_ptr<DericheGaussianCoefficients>> map_;

 public:
  void reset() override;

  /* Check if there is an available DericheGaussianCoefficients cached resource with the given
   * parameters in the container, if one exists, return it, otherwise, return a newly created one
   * and add it to the container. In both cases, tag the cached resource as needed to keep it
   * cached for the next evaluation. */
  DericheGaussianCoefficients &get(Context &context, float sigma);
};

}  // namespace blender::realtime_compositor
