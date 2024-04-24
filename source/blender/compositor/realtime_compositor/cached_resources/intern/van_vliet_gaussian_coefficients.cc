/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* -------------------------------------------------------------------------------------------------
 * Van Vliet Gaussian Coefficients.
 *
 * Computes the coefficients of the fourth order IIR filter approximating a Gaussian filter
 * computed using Van Vliet's design method. This is based on the following paper:
 *
 *   Van Vliet, Lucas J., Ian T. Young, and Piet W. Verbeek. "Recursive Gaussian derivative
 *   filters." Proceedings. Fourteenth International Conference on Pattern Recognition (Cat. No.
 *   98EX170). Vol. 1. IEEE, 1998.
 *
 * The filter is computed as the cascade of a causal and a non causal sequences of second order
 * difference equations as can be seen in Equation (11) in Van Vliet's paper. The coefficients are
 * the same for both the causal and non causal sequences. */

#include <array>
#include <complex>
#include <cstdint>
#include <memory>

#include "BLI_assert.h"
#include "BLI_hash.hh"
#include "BLI_math_base.hh"
#include "BLI_math_vector.hh"

#include "COM_context.hh"
#include "COM_van_vliet_gaussian_coefficients.hh"

namespace blender::realtime_compositor {

/* --------------------------------------------------------------------
 * Van Vliet Gaussian Coefficients Key.
 */

VanVlietGaussianCoefficientsKey::VanVlietGaussianCoefficientsKey(float sigma) : sigma(sigma) {}

uint64_t VanVlietGaussianCoefficientsKey::hash() const
{
  return get_default_hash(sigma);
}

bool operator==(const VanVlietGaussianCoefficientsKey &a, const VanVlietGaussianCoefficientsKey &b)
{
  return a.sigma == b.sigma;
}

/* -------------------------------------------------------------------------------------------------
 * Van Vliet Gaussian Coefficients.
 */

/* Computes the variance of the Gaussian filter represented by the given poles scaled by the given
 * scale factor. This is based on Equation (20) in Van Vliet's paper. */
static double compute_scaled_poles_variance(const std::array<std::complex<double>, 4> &poles,
                                            double scale_factor)
{
  std::complex<double> variance = std::complex<double>(0.0, 0.0);
  for (const std::complex<double> &pole : poles) {
    const double magnitude = std::pow(std::abs(pole), 1.0 / scale_factor);
    const double phase = std::arg(pole) / scale_factor;
    const std::complex<double> multiplier1 = std::polar(magnitude, phase);
    const std::complex<double> multiplier2 = std::pow(magnitude - std::polar(1.0, phase), -2.0);
    variance += 2.0 * multiplier1 * multiplier2;
  }

  /* The variance is actually real valued as guaranteed by Equations (10) and (2) since the poles
   * are complex conjugate pairs. See Section 3.3 of the paper. */
  return variance.real();
}

/* Computes the partial derivative with respect to the scale factor at the given scale factor of
 * the variance of the Gaussian filter represented by the given poles scaled by the given scale
 * factor. This is based on the partial derivative with respect to the scale factor of Equation
 * (20) in Van Vliet's paper.
 *
 * The derivative is not listed in the paper, but was computed manually as the sum of the following
 * for each of the poles:
 *
 *   \dfrac{
 *     2a^\frac{1}{x}\mathrm{e}^\frac{\mathrm{i}b}{x}
 *     \cdot
 *     \left(\mathrm{e}^\frac{\mathrm{i}b}{x}+a^\frac{1}{x}\right)
 *     \cdot
 *     \left(\ln\left(a\right)-\mathrm{i}b\right)
 *   }
 *   {
 *     x^2
 *     \cdot
 *     \left(a^\frac{1}{x}-\mathrm{e}^\frac{\mathrm{i}b}{x}\right)^3
 *   }
 *
 * Where "x" is the scale factor, "a" is the magnitude of the pole, and "b" is its phase. */
static double compute_scaled_poles_variance_derivative(
    const std::array<std::complex<double>, 4> &poles, double scale_factor)
{
  std::complex<double> variance_derivative = std::complex<double>(0.0, 0.0);
  for (const std::complex<double> &pole : poles) {
    const double magnitude = std::pow(std::abs(pole), 1.0 / scale_factor);
    const double phase = std::arg(pole) / scale_factor;

    const std::complex<double> multiplier1 = std::polar(magnitude, phase);
    const std::complex<double> multiplier2 = magnitude + std::polar(1.0, phase);
    const std::complex<double> multiplier3 = std::log(std::abs(pole)) -
                                             std::complex<double>(0.0, std::arg(pole));

    const std::complex<double> divisor1 = std::pow(magnitude - std::polar(1.0, phase), 3.0);
    const std::complex<double> divisor2 = math::square(scale_factor);

    variance_derivative += 2.0 * multiplier1 * multiplier2 * multiplier3 / (divisor1 * divisor2);
  }

  /* The variance derivative is actually real valued as guaranteed by Equations (10) and (2) since
   * the poles are complex conjugate pairs. See Section 3.3 of the paper. */
  return variance_derivative.real();
}

/* The poles were computed for a Gaussian filter with a sigma value of 2, in order to generalize
 * that for any sigma value, we need to scale the poles by a certain scaling factor as described in
 * Section 4.2 of Van Vliet's paper. To find the scaling factor, we start from an initial guess of
 * half sigma, then iteratively improve the guess using Newton's method by computing the variance
 * and its derivative based on Equation (20). */
static double find_scale_factor(const std::array<std::complex<double>, 4> &poles,
                                float reference_sigma)
{
  const double reference_variance = math::square(reference_sigma);

  /* Note that the poles were computed for a Gaussian filter with a sigma value of 2, so it it
   * as if we have a base scale of 2, and we start with half sigma as an initial guess. See
   * Section 4.2 for more information */
  double scale_factor = reference_sigma / 2.0;

  const int maximum_interations = 100;
  for (int i = 0; i < maximum_interations; i++) {
    const double variance = compute_scaled_poles_variance(poles, scale_factor);

    /* Close enough, we have found our scale factor. */
    if (math::abs(reference_variance - variance) < 1.0e-9) {
      return scale_factor;
    }

    /* Improve guess using Newton's method. Notice that Newton's method is a root finding method,
     * so we supply the difference to the reference variance as our function, since the zero point
     * will be when the variance is equal to the reference one. The derivative is not affected
     * since the reference variance is a constant. */
    const double derivative = compute_scaled_poles_variance_derivative(poles, scale_factor);
    scale_factor -= (variance - reference_variance) / derivative;
  }

  /* The paper mentions that only a few iterations are needed, so if we didn't converge after
   * maximum_interations, something is probably wrong. */
  BLI_assert_unreachable();
  return scale_factor;
}

/* The poles were computed for a Gaussian filter with a sigma value of 2, so scale them using
 * Equation (19) in Van Vliet's paper to have the given sigma value. This involves finding the
 * appropriate scale factor based on Equation (20), see Section 4.2 and the find_scale_factor
 * method for more information. */
static std::array<std::complex<double>, 4> scale_poles(
    const std::array<std::complex<double>, 4> &poles, float sigma)
{
  const double scale_factor = find_scale_factor(poles, sigma);

  std::array<std::complex<double>, 4> scaled_poles;
  for (int i = 0; i < poles.size(); i++) {
    const std::complex<double> &pole = poles[i];
    const double magnitude = std::pow(std::abs(pole), 1.0 / scale_factor);
    const double phase = std::arg(pole) / scale_factor;
    scaled_poles[i] = 1.0 / std::polar(magnitude, phase);
  }

  return scaled_poles;
}

/* Computes the feedback coefficients from the given poles based on the equations in Equation (13)
 * in Van Vliet's paper. See Section 3.2 for more information. */
static double4 compute_feedback_coefficients(const std::array<std::complex<double>, 4> &poles)
{
  /* Compute the gain of the poles, which is the "b" at the end of Equation (13). */
  std::complex<double> gain = std::complex<double>(1.0, 0.0);
  for (const std::complex<double> &pole : poles) {
    gain /= pole;
  }

  /* Compute the coefficients b4, b3, b2, and b1 based on the expressions b_N, b_N-1, b_N-2, and
   * b_N-3 respectively in Equation (13). b4 and b3 are trivial, while b2 and b1 can be computed by
   * drawing the following summation trees, where each path from the root to the leaf is multiplied
   * and added:
   *
   *                  b2
   *             ____/|\____
   *            /     |     \
   *   i -->   2      3      4
   *           |     / \    /|\
   *   j -->   1    1   2  1 2 3
   *
   *                 b1
   *             ___/ \___
   *            /         \
   *   i -->   3           4
   *           |          / \
   *   j -->   2         2   3
   *           |         |  / \
   *   k -->   1         1 1   2
   *
   * Notice that the values of i, j, and k are 1-index, so we need to subtract one when accessing
   * the poles. */
  const std::complex<double> b4 = gain;
  const std::complex<double> b3 = -gain * (poles[0] + poles[1] + poles[2] + poles[3]);
  const std::complex<double> b2 = gain * (poles[1] * poles[0] + poles[2] * poles[0] +
                                          poles[2] * poles[1] + poles[3] * poles[0] +
                                          poles[3] * poles[1] + poles[3] * poles[2]);
  const std::complex<double> b1 = -gain * (poles[2] * poles[1] * poles[0] +
                                           poles[3] * poles[1] * poles[0] +
                                           poles[3] * poles[2] * poles[0] +
                                           poles[3] * poles[2] * poles[1]);

  /* The coefficients are actually real valued as guaranteed by Equations (10) and (2) since
   * the poles are complex conjugate pairs. See Section 3.3 of the paper. */
  const double4 coefficients = double4(b1.real(), b2.real(), b3.real(), b4.real());

  return coefficients;
}

/* Computes the feedforward coefficient from the feedback coefficients based on Equation (12) of
 * Van Vliet's paper. See Section 3.2 for more information. */
static double compute_feedforward_coefficient(const double4 &feedback_coefficients)
{
  return 1.0 + math::reduce_add(feedback_coefficients);
}

static double compute_gain(const std::array<std::complex<double>, 4> &poles)
{
  std::complex<double> gain = std::complex<double>(1.0, 0.0);
  for (const std::complex<double> &pole : poles) {
    gain *= 1.0 - pole;
  }

  return gain.real();
}

static std::complex<double> compute_partial_fraction_residue(
    const std::array<std::complex<double>, 4> &poles, const std::complex<double> &target_pole)
{
  std::complex<double> target_pole_inverse = 1.0 / target_pole;
  std::complex<double> residue = std::complex<double>(1.0, 0.0);
  for (const std::complex<double> &pole : poles) {
    if (pole != target_pole && pole != std::conj(target_pole)) {
      residue *= 1.0 - pole * target_pole_inverse;
    }

    residue *= 1.0 - pole * target_pole;
  }

  const double gain = math::square(compute_gain(poles));

  return (1.0 / residue) * gain;
}

static void compute_second_order_section(const std::array<std::complex<double>, 4> &poles,
                                         const std::complex<double> &pole,
                                         double2 &r_feedback_coefficients,
                                         double2 &r_causal_feedforward_coefficients,
                                         double2 &r_non_causal_feedforward_coefficients)
{
  const std::complex<double> pole_inverse = 1.0 / pole;
  const std::complex<double> residue = compute_partial_fraction_residue(poles, pole);

  r_feedback_coefficients = double2(-2.0 * pole.real(), std::norm(pole));

  const double causal_feedforward_1 = residue.imag() / pole_inverse.imag();
  const double causal_feedforward_0 = residue.real() - causal_feedforward_1 * pole_inverse.real();
  r_causal_feedforward_coefficients = double2(causal_feedforward_0, causal_feedforward_1);

  const double non_causal_feedforward_1 = causal_feedforward_1 -
                                          causal_feedforward_0 * r_feedback_coefficients[0];
  const double non_causal_feedforward_2 = -causal_feedforward_0 * r_feedback_coefficients[1];
  r_non_causal_feedforward_coefficients = double2(non_causal_feedforward_1,
                                                  non_causal_feedforward_2);
}

/* Computes the coefficient that needs to be multiplied to the boundary value in order to simulate
 * an infinite previous stream of that value. The boundary value is used to initialize the previous
 * outputs for the causal and non-causal filters. This works for both Dirichlet and Neumann
 * boundaries. The equation for that coefficient can be derived by rearranging the difference
 * equation to compute the current input from the output and previous outputs, substituting the
 * boundary value for previous outputs. */
static double compute_boundary_coefficient(const double2 &feedback_coefficients,
                                           const double2 &feedforward_coefficients)
{
  return math::reduce_add(feedforward_coefficients) /
         (1.0 + math::reduce_add(feedback_coefficients));
}

/* Computes the feedback and feedforward coefficients for the 4th order Van Vliet Gaussian filter
 * given a target Gaussian sigma value. We first scale the poles of the filter to match the sigma
 * value based on the method described in Section 4.2 of Van Vliet's paper, then we compute the
 * coefficients from the scaled poles based on Equations (12) and (13). */
VanVlietGaussianCoefficients::VanVlietGaussianCoefficients(Context & /*context*/, float sigma)
{

  /* The 4th order (N=4) poles for the Gaussian filter of a sigma of 2 computed by minimizing the
   * maximum error (L-infinity) to true Gaussian as provided in Van Vliet's paper Table (1) fourth
   * column. Notice that the second and fourth poles are the complex conjugates of the first and
   * third poles respectively as noted in the table description. */
  const std::array<std::complex<double>, 4> poles = {
      std::complex<double>(1.12075, 1.27788),
      std::complex<double>(1.12075, -1.27788),
      std::complex<double>(1.76952, 0.46611),
      std::complex<double>(1.76952, -0.46611),
  };

  const std::array<std::complex<double>, 4> scaled_poles = scale_poles(poles, sigma);

  const double4 feedback_coefficients = compute_feedback_coefficients(scaled_poles);

  const double feedforward_coefficient = compute_feedforward_coefficient(feedback_coefficients);

  compute_second_order_section(scaled_poles,
                               scaled_poles[0],
                               first_feedback_coefficients_,
                               first_causal_feedforward_coefficients_,
                               first_non_causal_feedforward_coefficients_);

  compute_second_order_section(scaled_poles,
                               scaled_poles[2],
                               second_feedback_coefficients_,
                               second_causal_feedforward_coefficients_,
                               second_non_causal_feedforward_coefficients_);

  first_causal_boundary_coefficient_ = compute_boundary_coefficient(
      first_feedback_coefficients_, first_causal_feedforward_coefficients_);
  first_non_causal_boundary_coefficient_ = compute_boundary_coefficient(
      first_feedback_coefficients_, first_non_causal_feedforward_coefficients_);
  second_causal_boundary_coefficient_ = compute_boundary_coefficient(
      second_feedback_coefficients_, second_causal_feedforward_coefficients_);
  second_non_causal_boundary_coefficient_ = compute_boundary_coefficient(
      second_feedback_coefficients_, second_non_causal_feedforward_coefficients_);
}

const double2 &VanVlietGaussianCoefficients::first_causal_feedforward_coefficients() const
{
  return first_causal_feedforward_coefficients_;
}

const double2 &VanVlietGaussianCoefficients::first_non_causal_feedforward_coefficients() const
{
  return first_non_causal_feedforward_coefficients_;
}

const double2 &VanVlietGaussianCoefficients::first_feedback_coefficients() const
{
  return first_feedback_coefficients_;
}

const double2 &VanVlietGaussianCoefficients::second_causal_feedforward_coefficients() const
{
  return second_causal_feedforward_coefficients_;
}

const double2 &VanVlietGaussianCoefficients::second_non_causal_feedforward_coefficients() const
{
  return second_non_causal_feedforward_coefficients_;
}

const double2 &VanVlietGaussianCoefficients::second_feedback_coefficients() const
{
  return second_feedback_coefficients_;
}

double VanVlietGaussianCoefficients::first_causal_boundary_coefficient() const
{
  return first_causal_boundary_coefficient_;
}

double VanVlietGaussianCoefficients::first_non_causal_boundary_coefficient() const
{
  return first_non_causal_boundary_coefficient_;
}

double VanVlietGaussianCoefficients::second_causal_boundary_coefficient() const
{
  return second_causal_boundary_coefficient_;
}

double VanVlietGaussianCoefficients::second_non_causal_boundary_coefficient() const
{
  return second_non_causal_boundary_coefficient_;
}

/* --------------------------------------------------------------------
 * Van Vliet Gaussian Coefficients Container.
 */

void VanVlietGaussianCoefficientsContainer::reset()
{
  /* First, delete all resources that are no longer needed. */
  map_.remove_if([](auto item) { return !item.value->needed; });

  /* Second, reset the needed status of the remaining resources to false to ready them to track
   * their needed status for the next evaluation. */
  for (auto &value : map_.values()) {
    value->needed = false;
  }
}

VanVlietGaussianCoefficients &VanVlietGaussianCoefficientsContainer::get(Context &context,
                                                                         float sigma)
{
  const VanVlietGaussianCoefficientsKey key(sigma);

  auto &deriche_gaussian_coefficients = *map_.lookup_or_add_cb(
      key, [&]() { return std::make_unique<VanVlietGaussianCoefficients>(context, sigma); });

  deriche_gaussian_coefficients.needed = true;
  return deriche_gaussian_coefficients;
}

}  // namespace blender::realtime_compositor
