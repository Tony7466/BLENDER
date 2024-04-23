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
 * Van Vliet Gaussian Coefficients Key.
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
 * Van Vliet Gaussian Coefficients.
 *
 * A caches resource that computes and caches the coefficients of the fourth order IIR filter
 * approximating a Gaussian filter computed using Van Vliet's design method. */
class VanVlietGaussianCoefficients : public CachedResource {
 private:
  double2 first_causal_feedforward_coefficients_;
  double2 first_non_causal_feedforward_coefficients_;
  double2 first_feedback_coefficients_;

  double2 second_causal_feedforward_coefficients_;
  double2 second_non_causal_feedforward_coefficients_;
  double2 second_feedback_coefficients_;

  double first_causal_boundary_coefficient_;
  double first_non_causal_boundary_coefficient_;
  double second_causal_boundary_coefficient_;
  double second_non_causal_boundary_coefficient_;

 public:
  VanVlietGaussianCoefficients(Context &context, float sigma);

  const double2 &first_causal_feedforward_coefficients() const;
  const double2 &first_non_causal_feedforward_coefficients() const;
  const double2 &first_feedback_coefficients() const;

  const double2 &second_causal_feedforward_coefficients() const;
  const double2 &second_non_causal_feedforward_coefficients() const;
  const double2 &second_feedback_coefficients() const;

  double first_causal_boundary_coefficient() const;
  double first_non_causal_boundary_coefficient() const;
  double second_causal_boundary_coefficient() const;
  double second_non_causal_boundary_coefficient() const;
};

/* ------------------------------------------------------------------------------------------------
 * Van Vliet Gaussian Coefficients Container.
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
