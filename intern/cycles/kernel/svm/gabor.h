/* SPDX-FileCopyrightText: 2024 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

/* Implements Gabor noise based on the paper:
 *
 *   Lagae, Ares, et al. "Procedural noise using sparse Gabor convolution." ACM Transactions on
 *   Graphics (TOG) 28.3 (2009): 1-10.
 *
 * But with the improvements from the paper:
 *
 *   Tavernier, Vincent, et al. "Making gabor noise fast and normalized." Eurographics 2019-40th
 *   Annual Conference of the European Association for Computer Graphics. 2019.
 *
 */

#pragma once

CCL_NAMESPACE_BEGIN

/* Computes a 2D Gabor kernel based on Equation (6) in the original Gabor noise paper. Where the
 * frequency argument is the F_0 parameter and the orientation argument is the w_0 parameter. We
 * assume the Gaussian envelope has a unit magnitude, that is, K = 1. That is because we will
 * eventually normalize the final noise value to the unit range, so the multiplication by the
 * magnitude will be canceled by the normalization. Further, we also assume a unit Gaussian width,
 * that is, a = 1. That is because it does not provide much artistic control. It follows that the
 * Gaussian will be truncated at pi.
 *
 * We also replace the cosine function of the harmonic with a sine function, as suggested by
 * Tavernier's paper in "Section 3.3. Instance stationarity and normalization", to ensure a zero
 * mean, which should help with normalization.
 *
 * Finally, the Gaussian is windowed using a Hann window, that is because contrary to the claim in
 * the original Gabor paper, truncating the Gaussian produces significant artifacts especially when
 * differentiated for bump mapping. The Hann window is C1 continuous and has limited effect on the
 * shape of the Gaussian, so it felt like an appropriate choice. */
ccl_device float compute_2d_gabor_kernel(float2 position, float frequency, float orientation)
{
  /* The kernel is windowed beyond the unit distance, so early exist with a zero for points that
   * are further than a unit radius. */
  float distance_squared = dot(position, position);
  if (distance_squared >= 1.0f) {
    return 0.0f;
  }

  float hann_window = 0.5f + 0.5f * cosf(M_PI_F * distance_squared);
  float gaussian_envelop = expf(-M_PI_F * distance_squared);
  float windowed_gaussian_envelope = gaussian_envelop * hann_window;

  float2 frequency_vector = frequency * make_float2(cosf(orientation), sinf(orientation));
  float sinusoidal_wave = sinf(2.0f * M_PI_F * dot(position, frequency_vector));

  return windowed_gaussian_envelope * sinusoidal_wave;
}

/* The original Gabor noise paper specifies that the impulses count for each cell should be
 * computed by sampling a Poisson distribution whose mean is the impulse density. However,
 * Tavernier's paper showed that stratified Poisson point sampling is better assuming the weights
 * are sampled using a Bernoulli distribution, as shown in Figure (3). By stratified sampling, they
 * mean a constant number of impulses per cell, so the stratification is the grid itself in a
 * sense. However, to allow fractional impulses count, an additional impulse is added following a
 * Bernoulli distribution whose probability is the fractional part of the impulses count. */
ccl_device int compute_impulses_count_for_2d_cell(float2 cell, float impulses_count)
{
  return int(impulses_count) + (hash_float2_to_float(cell) < fractf(impulses_count) ? 1 : 0);
}

/* Computes the Gabor noise value at the given position for the given cell. This is essentially the
 * sum in Equation (8) in the original Gabor noise paper, where we sum Gabor kernels sampled at a
 * random position with a random weight. The orientation of the kernel is constant for anisotropic
 * noise while it is random for isotropic noise. The original Gabor noise paper mentions that the
 * weights should be uniformly distributed in the [-1, 1] range, however, Tavernier's paper showed
 * that using a Bernoulli distribution yields better results, so that is what we do. */
ccl_device float compute_2d_gabor_noise_cell(float2 cell,
                                             float2 position,
                                             float impulses_count,
                                             float frequency,
                                             float isotropy,
                                             float base_orientation)

{
  float noise = 0.0f;
  int impulses_count_for_cell = compute_impulses_count_for_2d_cell(cell, impulses_count);
  for (int i = 0; i < impulses_count_for_cell; ++i) {
    /* Compute unique seeds for each of the needed random variables. */
    float3 seed_for_orientation = make_float3(cell.x, cell.y, i * 3);
    float3 seed_for_kernel_center = make_float3(cell.x, cell.y, i * 3 + 1);
    float3 seed_for_weight = make_float3(cell.x, cell.y, i * 3 + 2);

    /* For isotropic noise, add a random orientation amount, while for anisotropic noise, use the
     * base orientation. Linearly interpolate between the two cases using the isotropy factor. */
    float random_orientation = hash_float3_to_float(seed_for_orientation) * 2.0f * M_PI_F;
    float orientation = base_orientation + random_orientation * isotropy;

    float2 kernel_center = hash_float3_to_float2(seed_for_kernel_center);
    float2 position_in_kernel_space = position - kernel_center;

    /* We either add or subtract the Gabor kernel based on a Bernoulli distribution of equal
     * probability. */
    float weight = hash_float3_to_float(seed_for_weight) < 0.5f ? -1.0f : 1.0f;

    noise += weight * compute_2d_gabor_kernel(position_in_kernel_space, frequency, orientation);
  }
  return noise;
}

/* Computes the Gabor noise value by dividing the space into a grid and evaluating the Gabor noise
 * in the space of each cell of the 3x3 cell neighbourhood. */
ccl_device float compute_2d_gabor_noise(float2 coordinates,
                                        float impulses_count,
                                        float frequency,
                                        float isotropy,
                                        float base_orientation)
{
  float2 cell_position = floor(coordinates);
  float2 local_position = coordinates - cell_position;

  float sum = 0.0f;
  for (int j = -1; j <= 1; j++) {
    for (int i = -1; i <= 1; i++) {
      float2 cell_offset = make_float2(i, j);
      float2 current_cell_position = cell_position + cell_offset;
      float2 position_in_cell_space = local_position - cell_offset;
      sum += compute_2d_gabor_noise_cell(current_cell_position,
                                         position_in_cell_space,
                                         impulses_count,
                                         frequency,
                                         isotropy,
                                         base_orientation);
    }
  }

  return sum;
}

/* Identical to compute_2d_gabor_kernel, except it is evaluated in 3D space. Notice that Equation
 * (6) in the original Gabor noise paper computes the frequency vector using (cos(w_0), sin(w_0)),
 * which we also do in the 2D variant, however, for 3D, the orientation is already a unit frequency
 * vector, so we just need to scale it by the frequency value. */
ccl_device float compute_3d_gabor_kernel(float3 position, float frequency, float3 orientation)
{
  /* The kernel is windowed beyond the unit distance, so early exist with a zero for points that
   * are further than a unit radius. */
  float distance_squared = dot(position, position);
  if (distance_squared >= 1.0f) {
    return 0.0f;
  }

  float hann_window = 0.5f + 0.5f * cosf(M_PI_F * distance_squared);
  float gaussian_envelop = expf(-M_PI_F * distance_squared);
  float windowed_gaussian_envelope = gaussian_envelop * hann_window;

  float3 frequency_vector = frequency * orientation;
  float sinusoidal_wave = sinf(2.0f * M_PI_F * dot(position, frequency_vector));

  return windowed_gaussian_envelope * sinusoidal_wave;
}

/* Identical to compute_impulses_count_for_2d_cell but works on 3D cells. */
ccl_device int compute_impulses_count_for_3d_cell(float3 cell, float impulses_count)
{
  return int(impulses_count) + (hash_float3_to_float(cell) < fractf(impulses_count) ? 1 : 0);
}

/* Computes the orientation of the Gabor kernel such that it is is constant for anisotropic
 * noise while it is random for isotropic noise. We randomize in spherical coordinates for a
 * uniform distribution. */
ccl_device float3 compute_3d_orientation(float3 orientation, float isotropy, float4 seed)
{
  /* Return the base orientation in case we are completely anisotropic. */
  if (isotropy == 0.0f) {
    return orientation;
  }

  /* Compute the orientation in spherical coordinates. */
  float inclination = acos(orientation.z);
  float azimuth = (orientation.y < 0.0f ? -1.0f : 1.0f) *
                  acos(orientation.x / len(make_float2(orientation.x, orientation.y)));

  /* For isotropic noise, add a random orientation amount, while for anisotropic noise, use the
   * base orientation. Linearly interpolate between the two cases using the isotropy factor. */
  float2 random_angles = hash_float4_to_float2(seed) * 2.0f * M_PI_F;
  inclination += random_angles.x * isotropy;
  azimuth += random_angles.y * isotropy;

  /* Convert back to Cartesian coordinates, */
  return make_float3(
      sinf(inclination) * cosf(azimuth), sinf(inclination) * sinf(azimuth), cosf(inclination));
}

ccl_device float compute_3d_gabor_noise_cell(float3 cell,
                                             float3 position,
                                             float impulses_count,
                                             float frequency,
                                             float isotropy,
                                             float3 base_orientation)

{
  float noise = 0.0f;
  int impulses_count_for_cell = compute_impulses_count_for_3d_cell(cell, impulses_count);
  for (int i = 0; i < impulses_count_for_cell; ++i) {
    /* Compute unique seeds for each of the needed random variables. */
    float4 seed_for_orientation = make_float4(cell.x, cell.y, cell.z, i * 3);
    float4 seed_for_kernel_center = make_float4(cell.x, cell.y, cell.z, i * 3 + 1);
    float4 seed_for_weight = make_float4(cell.x, cell.y, cell.z, i * 3 + 2);

    float3 orientation = compute_3d_orientation(base_orientation, isotropy, seed_for_orientation);

    float3 kernel_center = hash_float4_to_float3(seed_for_kernel_center);
    float3 position_in_kernel_space = position - kernel_center;

    /* We either add or subtract the Gabor kernel based on a Bernoulli distribution of equal
     * probability. */
    float weight = hash_float4_to_float(seed_for_weight) < 0.5f ? -1.0f : 1.0f;

    noise += weight * compute_3d_gabor_kernel(position_in_kernel_space, frequency, orientation);
  }
  return noise;
}

/* Identical to compute_2d_gabor_noise but works in the 3D neighbourhood of the noise. */
ccl_device float compute_3d_gabor_noise(float3 coordinates,
                                        float impulses_count,
                                        float frequency,
                                        float isotropy,
                                        float3 base_orientation)
{
  float3 cell_position = floor(coordinates);
  float3 local_position = coordinates - cell_position;

  float sum = 0.0f;
  for (int k = -1; k <= 1; k++) {
    for (int j = -1; j <= 1; j++) {
      for (int i = -1; i <= 1; i++) {
        float3 cell_offset = make_float3(i, j, k);
        float3 current_cell_position = cell_position + cell_offset;
        float3 position_in_cell_space = local_position - cell_offset;
        sum += compute_3d_gabor_noise_cell(current_cell_position,
                                           position_in_cell_space,
                                           impulses_count,
                                           frequency,
                                           isotropy,
                                           base_orientation);
      }
    }
  }

  return sum;
}

ccl_device_noinline int svm_node_tex_gabor(KernelGlobals kg,
                                           ccl_private ShaderData *sd,
                                           ccl_private float *stack,
                                           uint type,
                                           uint stack_offsets_1,
                                           uint stack_offsets_2,
                                           int offset)
{
  uint coordinates_stack_offset;
  uint scale_stack_offset;
  uint impulses_stack_offset;
  uint frequency_stack_offset;
  uint anisotropy_stack_offset;
  uint orientation_2d_stack_offset;
  uint orientation_3d_stack_offset;
  uint output_stack_offset;

  svm_unpack_node_uchar4(stack_offsets_1,
                         &coordinates_stack_offset,
                         &scale_stack_offset,
                         &impulses_stack_offset,
                         &frequency_stack_offset);
  svm_unpack_node_uchar4(stack_offsets_2,
                         &anisotropy_stack_offset,
                         &orientation_2d_stack_offset,
                         &orientation_3d_stack_offset,
                         &output_stack_offset);

  float3 coordinates = stack_load_float3(stack, coordinates_stack_offset);

  uint4 defaults_1 = read_node(kg, &offset);
  float scale = stack_load_float_default(stack, scale_stack_offset, defaults_1.x);
  float impulses_count = stack_load_float_default(stack, impulses_stack_offset, defaults_1.y);
  float frequency = stack_load_float_default(stack, frequency_stack_offset, defaults_1.z);
  float anisotropy = stack_load_float_default(stack, anisotropy_stack_offset, defaults_1.w);

  uint4 defaults_2 = read_node(kg, &offset);
  float orientation_2d = stack_load_float_default(
      stack, orientation_2d_stack_offset, defaults_2.x);
  float3 orientation_3d = stack_load_float3(stack, orientation_3d_stack_offset);

  float3 scaled_coordinates = coordinates * scale;
  float isotropy = 1.0f - clamp(anisotropy, 0.0f, 1.0f);

  float output_value = 0.0f;
  switch ((NodeGaborType)type) {
    case NODE_GABOR_TYPE_2D: {
      output_value = compute_2d_gabor_noise(
          make_float2(scaled_coordinates.x, scaled_coordinates.y),
          impulses_count,
          frequency,
          isotropy,
          orientation_2d);
      break;
    }
    case NODE_GABOR_TYPE_3D: {
      float3 orientation = normalize(orientation_3d);
      output_value = compute_3d_gabor_noise(
          scaled_coordinates, impulses_count, frequency, isotropy, orientation);
      break;
    }
  }

  if (stack_valid(output_stack_offset)) {
    stack_store_float(stack, output_stack_offset, output_value);
  }

  return offset;
}

CCL_NAMESPACE_END
