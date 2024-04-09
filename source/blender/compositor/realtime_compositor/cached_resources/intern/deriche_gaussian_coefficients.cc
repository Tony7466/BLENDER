/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* -------------------------------------------------------------------------------------------------
 * Deriche Gaussian Coefficients.
 *
 * Computes the coefficients of the fourth order IIR filter approximating a Gaussian filter
 * computed using Deriche's design method. This is based on the following paper:
 *
 *   Deriche, Rachid. Recursively implementating the Gaussian and its derivatives. Diss. INRIA,
 *   1993.
 *
 * But with corrections in the normalization scale from the following paper, as will be seen in the
 * implementation:
 *
 *   Farneback, Gunnar, and Carl-Fredrik Westin. Improving Deriche-style recursive Gaussian
 *   filters. Journal of Mathematical Imaging and Vision 26.3 (2006): 293-299.
 *
 * The filter is computed as the sum of a causal and a non causal sequences of second order
 * difference equations as can be seen in Equation (30) in Deriche's paper. */

#include <cstdint>
#include <memory>

#include "BLI_hash.hh"
#include "BLI_math_base.hh"
#include "BLI_math_vector.hh"

#include "COM_context.hh"
#include "COM_deriche_gaussian_coefficients.hh"

namespace blender::realtime_compositor {

/* --------------------------------------------------------------------
 * Deriche Gaussian Coefficients Key.
 */

DericheGaussianCoefficientsKey::DericheGaussianCoefficientsKey(float sigma) : sigma(sigma) {}

uint64_t DericheGaussianCoefficientsKey::hash() const
{
  return get_default_hash(sigma);
}

bool operator==(const DericheGaussianCoefficientsKey &a, const DericheGaussianCoefficientsKey &b)
{
  return a.sigma == b.sigma;
}

/* -------------------------------------------------------------------------------------------------
 * Deriche Gaussian Coefficients.
 */

/* The base constant coefficients computed using Deriche's method with 10 digits of precision.
 * Those are available in Deriche's paper by comparing Equations (19) and (38). */
inline constexpr static float a0 = 1.6797292232361107f;
inline constexpr static float a1 = 3.7348298269103580f;
inline constexpr static float b0 = 1.7831906544515104f;
inline constexpr static float b1 = 1.7228297663338028f;
inline constexpr static float c0 = -0.6802783501806897f;
inline constexpr static float c1 = -0.2598300478959625f;
inline constexpr static float w0 = 0.6318113174569493f;
inline constexpr static float w1 = 1.9969276832487770f;

/* Computes n00 in Equation (21) in Deriche's paper. */
static float compute_numerator_0()
{
  return a0 + c0;
}

/* Computes n11 in Equation (21) in Deriche's paper. */
static float compute_numerator_1(float sigma)
{
  const float multiplier1 = math::exp(-b1 / sigma);
  const float term1 = c1 * math::sin(w1 / sigma) - (c0 + 2.0f * a0) * math::cos(w1 / sigma);

  const float multiplier2 = math::exp(-b0 / sigma);
  const float term2 = a1 * math::sin(w0 / sigma) - (2.0f * c0 + a0) * math::cos(w0 / sigma);

  return multiplier1 * term1 + multiplier2 * term2;
}

/* Computes n22 in Equation (21) in Deriche's paper. */
static float compute_numerator_2(float sigma)
{
  const float multiplier1 = 2.0f * math::exp(-(b0 / sigma) - (b1 / sigma));
  const float term11 = (a0 + c0) * math::cos(w1 / sigma) * math::cos(w0 / sigma);
  const float term12 = math::cos(w1 / sigma) * a1 * math::sin(w0 / sigma);
  const float term13 = math::cos(w0 / sigma) * c1 * math::sin(w1 / sigma);
  const float term1 = term11 - term12 - term13;

  const float term2 = c0 * math::exp(-2.0f * (b0 / sigma));
  const float term3 = a0 * math::exp(-2.0f * (b1 / sigma));

  return multiplier1 * term1 + term2 + term3;
}

/* Computes n33 in Equation (21) in Deriche's paper. */
static float compute_numerator_3(float sigma)
{
  const float multiplier1 = math::exp(-(b1 / sigma) - 2.0f * (b0 / sigma));
  const float term1 = c1 * math::sin(w1 / sigma) - math::cos(w1 / sigma) * c0;

  const float multiplier2 = math::exp(-(b0 / sigma) - 2.0f * (b1 / sigma));
  const float term2 = a1 * math::sin(w0 / sigma) - math::cos(w0 / sigma) * a0;

  return multiplier1 * term1 + multiplier2 * term2;
}

/* Computes and packs the numerators in Equation (21) in Deriche's paper. */
static float4 compute_numerator(float sigma)
{
  const float n0 = compute_numerator_0();
  const float n1 = compute_numerator_1(sigma);
  const float n2 = compute_numerator_2(sigma);
  const float n3 = compute_numerator_3(sigma);

  return float4(n0, n1, n2, n3);
}

/* Computes d11 in Equation (22) in Deriche's paper. */
static float compute_denominator_1(float sigma)
{
  const float term1 = -2.0f * math::exp(-(b0 / sigma)) * math::cos(w0 / sigma);
  const float term2 = 2.0f * math::exp(-(b1 / sigma)) * math::cos(w1 / sigma);

  return term1 - term2;
}

/* Computes d22 in Equation (22) in Deriche's paper. */
static float compute_denominator_2(float sigma)
{
  const float term1 = 4.0f * math::cos(w1 / sigma) * math::cos(w0 / sigma);
  const float multiplier1 = math::exp(-(b0 / sigma) - (b1 / sigma));

  const float term2 = math::exp(-2.0f * (b1 / sigma));
  const float term3 = math::exp(-2.0f * (b0 / sigma));

  return term1 * multiplier1 + term2 + term3;
}

/* Computes d33 in Equation (22) in Deriche's paper. */
static float compute_denominator_3(float sigma)
{
  const float term1 = -2.0f * math::cos(w0 / sigma);
  const float multiplier1 = math::exp(-(b0 / sigma) - 2.0f * (b1 / sigma));

  const float term2 = 2.0f * math::cos(w1 / sigma);
  const float multiplier2 = math::exp(-(b1 / sigma) - 2.0f * (b0 / sigma));

  return term1 * multiplier1 - term2 * multiplier2;
}

/* Computes d44 in Equation (22) in Deriche's paper. */
static float compute_denominator_4(float sigma)
{
  return math::exp(-2.0f * (b0 / sigma) - 2.0f * (b1 / sigma));
}

/* Computes and packs the denominators in Equation (22) in Deriche's paper. */
static float4 compute_denominator(float sigma)
{
  const float d1 = compute_denominator_1(sigma);
  const float d2 = compute_denominator_2(sigma);
  const float d3 = compute_denominator_3(sigma);
  const float d4 = compute_denominator_4(sigma);

  return float4(d1, d2, d3, d4);
}

/* Computes the normalization scale that the feedforward coefficients should be divided by to
 * match the unit integral of the Gaussian. The scaling factor proposed by Deriche's paper in
 * Equation (50) is wrong due to missing terms. A correct scaling factor is presented in
 * Farneback's paper in Equation (25), which is implemented in this method. */
static float compute_normalization_scale(const DericheGaussianCoefficients &coefficients)
{
  const float4 &causal_feedforward = coefficients.causal_feedforward_coefficients();
  const float4 &feedback = coefficients.feedback_coefficients();
  const float causal_feedforwad_sum = math::reduce_add(causal_feedforward);
  const float feedback_sum = 1.0f + math::reduce_add(feedback);
  return 2.0f * (causal_feedforwad_sum / feedback_sum) - causal_feedforward[0];
}

/* Computes the non causal feedforward coefficients from the feedback and causal feedforward
 * coefficients based on Equation (31) in Deriche's paper. Notice that the equation is linear, so
 * the coefficients can be computed after the normalization of the causal feedforward
 * coefficients. */
static float4 compute_non_causal_feedforward_coefficients(
    const DericheGaussianCoefficients &coefficients)
{
  const float4 &causal_feedforward = coefficients.causal_feedforward_coefficients();
  const float4 &feedback = coefficients.feedback_coefficients();

  const float n1 = causal_feedforward[1] - feedback[0] * causal_feedforward[0];
  const float n2 = causal_feedforward[2] - feedback[1] * causal_feedforward[0];
  const float n3 = causal_feedforward[3] - feedback[2] * causal_feedforward[0];
  const float n4 = -feedback[3] * causal_feedforward[0];

  return float4(n1, n2, n3, n4);
}

/* Computes the feedback, causal feedforward, and non causal feedforward coefficients given a
 * target Gaussian sigma value as used in Equations (28) and (29) in Deriche's paper. */
DericheGaussianCoefficients::DericheGaussianCoefficients(Context & /*context*/, float sigma)
{
  /* The numerator coefficients are the causal feedforward coefficients and the denominator
   * coefficients are the feedback coefficients as can be seen in Equation (28). */
  causal_feedforward_coefficients_ = compute_numerator(sigma);
  feedback_coefficients_ = compute_denominator(sigma);

  /* Normalize the feedforward coefficients as discussed in Section "5.4 Normalization" in
   * Deriche's paper. Feedback coefficients do not need normalization. */
  causal_feedforward_coefficients_ /= compute_normalization_scale(*this);

  /* Compute the non causal feedforward coefficients from the feedback and normalized causal
   * feedforward coefficients based on Equation (31) from Deriche's paper. Since the causal
   * coefficients are already normalized, this doesn't need normalization. */
  non_causal_feedforward_coefficients_ = compute_non_causal_feedforward_coefficients(*this);
}

const float4 &DericheGaussianCoefficients::feedback_coefficients() const
{
  return feedback_coefficients_;
}

const float4 &DericheGaussianCoefficients::causal_feedforward_coefficients() const
{
  return causal_feedforward_coefficients_;
}

const float4 &DericheGaussianCoefficients::non_causal_feedforward_coefficients() const
{
  return non_causal_feedforward_coefficients_;
}

/* --------------------------------------------------------------------
 * Deriche Gaussian Coefficients Container.
 */

void DericheGaussianCoefficientsContainer::reset()
{
  /* First, delete all resources that are no longer needed. */
  map_.remove_if([](auto item) { return !item.value->needed; });

  /* Second, reset the needed status of the remaining resources to false to ready them to track
   * their needed status for the next evaluation. */
  for (auto &value : map_.values()) {
    value->needed = false;
  }
}

DericheGaussianCoefficients &DericheGaussianCoefficientsContainer::get(Context &context,
                                                                       float sigma)
{
  const DericheGaussianCoefficientsKey key(sigma);

  auto &deriche_gaussian_coefficients = *map_.lookup_or_add_cb(
      key, [&]() { return std::make_unique<DericheGaussianCoefficients>(context, sigma); });

  deriche_gaussian_coefficients.needed = true;
  return deriche_gaussian_coefficients;
}

}  // namespace blender::realtime_compositor
