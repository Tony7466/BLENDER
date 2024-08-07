/* SPDX-FileCopyrightText: 2024 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#pragma once

CCL_NAMESPACE_BEGIN

/* Gabor Noise
 *
 * Based on: Blender patch D287
 *
 * Adapted from Open Shading Language
 * Copyright (c) 2009-2010 Sony Pictures Imageworks Inc., et al.
 * All Rights Reserved.
 *
 * Gabor noise originally based on:
 * Lagae, A. and Drettakis, G. 2011. Filtering Solid Gabor Noise.
 */

/* See GLSL implementation for code comments. */

#define GABOR_SEED 1259

typedef struct GaborParams {
  float frequency;
  float radius;
  float impulses;
  float phase;
  float phase_variance;
  float rotation;
  float init_rotation;
  float rot_variance;
  float tilt_randomness;
  float cell_randomness;
  float anisotropy;
  int mode;
  float3 direction;
} GaborParams;

typedef struct FractalParams {
  float octaves;
  float roughness;
  float scl_lacunarity;
  float fre_lacunarity;
  float rot_lacunarity;
} FractalParams;

ccl_device int impulses_per_cell(float3 cell, float impulses, int seed)
{
  int n = int(impulses);
  float rmd = impulses - floorf(impulses);
  if (rmd > 0.0f) {
    float t = hash_float4_to_float(make_float4(cell.x, cell.y, cell.z, float(seed - GABOR_SEED)));
    return (t <= rmd) ? n + 1 : n;
  }
  return n;
}

ccl_device float3 gabor_kernel(GaborParams gp, float3 omega, float phi, float3 position, float dv)
{
  float g = cosf(M_PI_F * sqrtf(dv)) * 0.5f + 0.5f;
  float3 r = zero_float3();
  float h;

  if (gp.mode == SHD_GABOR_MODE_GABOR) {
    h = gp.frequency * dot(omega, position) + phi;
    r = make_float3(cosf(h), 0.0f, 0.0f);
  }
  else if (gp.mode == SHD_GABOR_MODE_PHASOR) {
    h = gp.frequency * dot(omega, position) + phi;
    r = make_float3(cosf(h), sin(h), 0.0f);
  }
  else if (gp.mode == SHD_GABOR_MODE_CROSS) {
    h = gp.frequency * len(omega * position) + phi;
    r = make_float3(cosf(h), 0.0f, 0.0f);
  }
  else if (gp.mode == SHD_GABOR_MODE_PHASOR_CROSS) {
    h = gp.frequency * len(omega * position) + phi;
    r = make_float3(cosf(h), sinf(h), 0.0f);
  }
  else if (gp.mode == SHD_GABOR_MODE_RING) {
    h = cosf(gp.frequency * dot(omega, position) + phi) + cosf(gp.frequency * len(position) + phi);
    r = make_float3(h, 0.0f, 0.0f) * 0.5f;
  }
  else if (gp.mode == SHD_GABOR_MODE_PHASOR_RING) {
    h = cosf(gp.frequency * dot(omega, position) + phi) + cosf(gp.frequency * len(position) + phi);
    const float h2 = sinf(gp.frequency * dot(omega, position) + phi) +
                     sinf(gp.frequency * len(position) + phi);
    r = make_float3(h, h2, 0.0f) * 0.5f;
  }
  else if (gp.mode == SHD_GABOR_MODE_SQUARE) {
    const float3 positionyxz = make_float3(position.y, position.x, position.z);
    h = cosf(gp.frequency * dot(omega, position) + phi) +
        cosf(gp.frequency * dot(omega, positionyxz) + phi);
    r = make_float3(h, 0.0f, 0.0f) * 0.5f;
  }
  else if (gp.mode == SHD_GABOR_MODE_PHASOR_SQUARE) {
    const float3 positionyxz = make_float3(position.y, position.x, position.z);
    h = cosf(gp.frequency * dot(omega, position) + phi) +
        cosf(gp.frequency * dot(omega, positionyxz) + phi);
    const float h2 = sinf(gp.frequency * dot(omega, position) + phi) +
                     sinf(gp.frequency * dot(omega, positionyxz) + phi);
    r = make_float3(h, h2, 0.0f) * 0.5f;
  }

  return r * g;
}

ccl_device float3 gabor_sample(GaborParams gp, float3 cell, int seed, ccl_private float *phi)
{
  float3 rand_values = hash_float4_to_float3(make_float4(cell.x, cell.y, cell.z, float(seed))) *
                           2.0f -
                       1.0f;
  float pvar = mix(0.0f, rand_values.z, gp.phase_variance);
  *phi = M_2PI_F * pvar + gp.phase;

  float omega_t = M_PI_F * (rand_values.x) * gp.rot_variance - gp.rotation - gp.init_rotation;
  float cos_omega_p = clamp(rand_values.y * gp.tilt_randomness, -1.0f, 1.0f);
  float sin_omega_p = sqrtf(1.0f - cos_omega_p * cos_omega_p);
  float sin_omega_t = sinf(omega_t);
  float cos_omega_t = cosf(omega_t);

  Transform rotationTransform = euler_to_transform(make_float3(0.0f, 0.0f, -gp.rotation));
  return mix(
      normalize(make_float3(cos_omega_t * sin_omega_p, sin_omega_t * sin_omega_p, cos_omega_p)),
      normalize(transform_direction(&rotationTransform, gp.direction)),
      gp.anisotropy);
}

ccl_device float3 gabor_cell_3d(GaborParams gp, float3 cell, float3 cell_position, int seed)
{
  int num_impulses = impulses_per_cell(cell, gp.impulses, seed);
  float3 sum = zero_float3();
  for (int i = 0; i < num_impulses; ++i) {
    float3 rand_position = mix(
        zero_float3(),
        hash_float4_to_float3(make_float4(cell.x, cell.y, cell.z, float(seed + i * GABOR_SEED))),
        gp.cell_randomness);

    float3 kernel_position = (cell_position - rand_position);

    float dv = dot(kernel_position, kernel_position) / gp.radius;

    if (dv <= 1.0f) {
      float phi;
      float3 omega = gabor_sample(gp, cell, seed + (num_impulses + i) * GABOR_SEED, &phi);
      sum += gabor_kernel(gp, omega, phi, kernel_position, dv);
    }
  }
  return sum;
}

ccl_device float3 gabor_cell_2d(GaborParams gp, float3 cell, float3 cell_position, int seed)
{
  int num_impulses = impulses_per_cell(cell, gp.impulses, seed);
  float3 sum = zero_float3();
  for (int i = 0; i < num_impulses; ++i) {
    float3 rand_position = mix(
        zero_float3(),
        hash_float4_to_float3(make_float4(cell.x, cell.y, cell.z, float(seed + i * GABOR_SEED))),
        gp.cell_randomness);
    rand_position.z = 0.0f;

    float3 kernel_position = (cell_position - rand_position);

    float dv = dot(kernel_position, kernel_position) / gp.radius;

    if (dv <= 1.0f) {
      float phi;
      float3 omega = gabor_sample(gp, cell, seed + (num_impulses + i) * GABOR_SEED, &phi);
      sum += gabor_kernel(gp, omega, phi, kernel_position, dv);
    }
  }
  return sum;
}

ccl_device float gabor_coord_wrap(float a, float b)
{
  return (b != 0.0f) ? a - b * floorf(a / b) : 0.0f;
}

ccl_device float3 gabor_grid_3d(GaborParams gp, float3 p, float scale, int periodic, int seed)
{
  float3 coords = p * scale;
  float3 position = floor(coords);
  float3 local_position = coords - position;

  float3 sum = zero_float3();
  for (int k = -1; k <= 1; k++) {
    for (int j = -1; j <= 1; j++) {
      for (int i = -1; i <= 1; i++) {
        float3 cell_offset = make_float3(i, j, k);
        float3 cell = position + cell_offset;
        float3 cell_position = local_position - cell_offset;

        /* Skip this cell if it's too far away to contribute - Bruemmer.osl */
        float3 Pr = (make_float3(i > 0, j > 0, k > 0) - local_position) * cell_offset;
        if (dot(Pr, Pr) >= 1.0f) {
          continue;
        }

        if (periodic) {
          cell.x = gabor_coord_wrap(cell.x, scale);
          cell.y = gabor_coord_wrap(cell.y, scale);
          cell.z = gabor_coord_wrap(cell.z, scale);
        }

        sum += gabor_cell_3d(gp, cell, cell_position, seed);
      }
    }
  }
  return sum;
}

ccl_device float3 gabor_grid_2d(GaborParams gp, float3 p, float scale, int periodic, int seed)
{
  float3 coords = make_float3(p.x, p.y, 0.0f) * scale;
  float3 position = floor(coords);
  float3 local_position = coords - position;

  float3 sum = zero_float3();
  for (int j = -1; j <= 1; j++) {
    for (int i = -1; i <= 1; i++) {
      float3 cell_offset = make_float3(i, j, 0.0f);
      float3 cell = position + cell_offset;
      float3 cell_position = local_position - cell_offset;

      /* Skip this cell if it's too far away to contribute - Bruemmer.osl */
      float3 Pr = (make_float3(i > 0, j > 0, 0) - local_position) * cell_offset;
      if (dot(Pr, Pr) >= 1.0f) {
        continue;
      }

      if (periodic) {
        cell.x = gabor_coord_wrap(cell.x, scale);
        cell.y = gabor_coord_wrap(cell.y, scale);
      }

      sum += gabor_cell_2d(gp, cell, cell_position, seed);
    }
  }
  return sum;
}

ccl_device float gabor_fractal_noise(FractalParams fp,
                                     GaborParams gp,
                                     float3 p,
                                     float scale,
                                     int dimensions,
                                     int periodic,
                                     int use_origin_offset)
{
  float fscale = 1.0f;
  float amp = 1.0f;
  float maxamp = 0.0f;
  float3 sum = zero_float3();
  float octaves = clamp(fp.octaves, 0.0f, 15.0f);
  if (fp.roughness == 0.0f) {
    octaves = 0.0f;
  }
  int n = int(octaves);
  for (int i = 0; i <= n; i++) {
    int seed = use_origin_offset * i * GABOR_SEED;
    float3 t = (dimensions == 3) ? gabor_grid_3d(gp, fscale * p, scale, periodic, seed) :
                                   gabor_grid_2d(gp, fscale * p, scale, periodic, seed);
    gp.frequency *= fp.fre_lacunarity;
    gp.rotation -= fp.rot_lacunarity;
    sum += t * amp;
    maxamp += amp;
    amp *= fp.roughness;
    fscale *= fp.scl_lacunarity;
  }
  float rmd = octaves - floorf(octaves);
  if (rmd != 0.0f) {
    int seed = use_origin_offset * (n + 1) * GABOR_SEED;
    float3 t = (dimensions == 3) ? gabor_grid_3d(gp, fscale * p, scale, periodic, seed) :
                                   gabor_grid_2d(gp, fscale * p, scale, periodic, seed);
    float3 sum2 = sum + t * amp;
    sum = mix(sum, sum2, rmd);
  }
  sum /= maxamp;

  if (gp.mode == SHD_GABOR_MODE_PHASOR || gp.mode == SHD_GABOR_MODE_PHASOR_RING ||
      gp.mode == SHD_GABOR_MODE_PHASOR_CROSS || gp.mode == SHD_GABOR_MODE_PHASOR_SQUARE)
  {
    float pn = atan2f(sum.y, sum.x) / M_PI_F;
    return pn;
  }
  else {
    return sum.x;
  }
}

ccl_device GaborParams gabor_parameters(float3 direction,
                                        float frequency,
                                        float radius,
                                        float impulses,
                                        float phase,
                                        float phase_variance,
                                        float rotation,
                                        float rot_variance,
                                        float tilt_randomness,
                                        float cell_randomness,
                                        float anisotropy,
                                        int mode)
{
  GaborParams gp;
  gp.impulses = clamp(impulses, 0.0001f, 32.0f);
  gp.rot_variance = rot_variance;
  gp.anisotropy = anisotropy;
  gp.mode = mode;
  gp.direction = direction;
  gp.phase = phase;
  gp.rotation = 0.0f;
  gp.init_rotation = rotation;
  gp.phase_variance = phase_variance;
  gp.tilt_randomness = tilt_randomness;
  gp.cell_randomness = cell_randomness;
  gp.radius = radius;
  gp.frequency = frequency * M_PI_F;
  return gp;
}

/* Gabor shader */

ccl_device float gabor_noise(float3 p,
                             float3 direction,
                             float scale,
                             float frequency,
                             float detail,
                             float roughness,
                             float scl_lacunarity,
                             float fre_lacunarity,
                             float rot_lacunarity,
                             float gain,
                             float radius,
                             float impulses,
                             float phase,
                             float phase_variance,
                             float rotation,
                             float rot_variance,
                             float tilt_randomness,
                             float cell_randomness,
                             float anisotropy,
                             int dimensions,
                             int mode,
                             int normalize,
                             int periodic,
                             int use_origin_offset)
{
  if (impulses == 0.0f || gain == 0.0f || radius <= 0.0f || scale == 0.0f) {
    return (normalize == 1) ? 0.5f : 0.0f;
  }

  FractalParams fp;
  fp.roughness = roughness;
  fp.octaves = detail;
  fp.scl_lacunarity = scl_lacunarity;
  fp.fre_lacunarity = fre_lacunarity;
  fp.rot_lacunarity = rot_lacunarity;

  GaborParams gp = gabor_parameters(direction,
                                    frequency,
                                    radius,
                                    impulses,
                                    phase,
                                    phase_variance,
                                    rotation,
                                    rot_variance,
                                    tilt_randomness,
                                    cell_randomness,
                                    anisotropy,
                                    mode);

  float g = gabor_fractal_noise(fp, gp, p, scale, dimensions, periodic, use_origin_offset) * gain;

  if (gp.mode == SHD_GABOR_MODE_GABOR || gp.mode == SHD_GABOR_MODE_RING ||
      gp.mode == SHD_GABOR_MODE_CROSS || gp.mode == SHD_GABOR_MODE_SQUARE)
  {
    float impulse_scale = impulses > 1.0f ? 1.2613446229f * sqrt(gp.impulses) : 1.2613446229f;
    g = g / impulse_scale;
  }

  if (normalize == 1) {
    return clamp(0.5f * g + 0.5f, 0.0f, 1.0f);
  }
  return g;
}

ccl_device_noinline int svm_node_tex_gabor(
    KernelGlobals kg, ccl_private ShaderData *sd, ccl_private float *stack, uint4 node, int offset)
{
  uint4 node2 = read_node(kg, &offset);
  uint4 defaults_node3 = read_node(kg, &offset);
  uint4 defaults_node4 = read_node(kg, &offset);
  uint4 defaults_node5 = read_node(kg, &offset);
  uint4 defaults_node6 = read_node(kg, &offset);
  uint4 defaults_node7 = read_node(kg, &offset);

  /* Input and Output Sockets */
  uint vector_in_offset, scale_offset, detail_offset, phase_offset, impulse_offset;
  uint direction_offset, value_out_offset, fre_lacunarity_offset;
  uint mode_offset, aniso_offset, periodic_offset, roughness_offset;
  uint rot_variance_offset, phase_variance_offset, rotation_offset, gain_offset,
      tilt_randomness_offset, cell_randomness_offset;
  uint scl_lacunarity_offset, use_normalize_offset, dimension_offset, rot_lacunarity_offset;
  uint frequency_offset, radius_offset;

  svm_unpack_node_uchar4(
      node.y, &vector_in_offset, &scale_offset, &frequency_offset, &detail_offset);
  svm_unpack_node_uchar4(node.z,
                         &roughness_offset,
                         &scl_lacunarity_offset,
                         &fre_lacunarity_offset,
                         &rot_lacunarity_offset);
  svm_unpack_node_uchar4(node.w, &gain_offset, &radius_offset, &impulse_offset, &phase_offset);

  svm_unpack_node_uchar4(node2.x,
                         &phase_variance_offset,
                         &cell_randomness_offset,
                         &rotation_offset,
                         &tilt_randomness_offset);
  svm_unpack_node_uchar4(
      node2.y, &rot_variance_offset, &direction_offset, &value_out_offset, &dimension_offset);
  svm_unpack_node_uchar4(
      node2.z, &mode_offset, &aniso_offset, &periodic_offset, &use_normalize_offset);

  float3 vector_in = stack_load_float3(stack, vector_in_offset);

  float scale = stack_load_float_default(stack, scale_offset, defaults_node3.x);
  float frequency = stack_load_float_default(stack, frequency_offset, defaults_node3.y);
  float detail = stack_load_float_default(stack, detail_offset, defaults_node3.z);
  float roughness = stack_load_float_default(stack, roughness_offset, defaults_node3.w);

  float scl_lacunarity = stack_load_float_default(stack, scl_lacunarity_offset, defaults_node4.x);
  float fre_lacunarity = stack_load_float_default(stack, fre_lacunarity_offset, defaults_node4.y);
  float rot_lacunarity = stack_load_float_default(stack, rot_lacunarity_offset, defaults_node4.z);
  float gain = stack_load_float_default(stack, gain_offset, defaults_node4.w);

  float radius = stack_load_float_default(stack, radius_offset, defaults_node5.x);
  float impulses = stack_load_float_default(stack, impulse_offset, defaults_node5.y);
  float phase = stack_load_float_default(stack, phase_offset, defaults_node5.z);
  float phase_variance = stack_load_float_default(stack, phase_variance_offset, defaults_node5.w);

  float cell_randomness = stack_load_float_default(
      stack, cell_randomness_offset, defaults_node6.x);
  float rotation = stack_load_float_default(stack, rotation_offset, defaults_node6.y);
  float rot_variance = stack_load_float_default(stack, rot_variance_offset, defaults_node6.z);
  float tilt_randomness = stack_load_float_default(
      stack, tilt_randomness_offset, defaults_node6.w);
  float anisotropy = stack_load_float_default(stack, aniso_offset, defaults_node7.x);

  float3 direction = stack_load_float3(stack, direction_offset);

  if (stack_valid(value_out_offset)) {
    float value = gabor_noise(vector_in,
                              direction,
                              scale,
                              frequency,
                              detail,
                              roughness,
                              scl_lacunarity,
                              fre_lacunarity,
                              rot_lacunarity,
                              gain,
                              radius,
                              impulses,
                              phase,
                              phase_variance,
                              rotation,
                              rot_variance,
                              tilt_randomness,
                              cell_randomness,
                              anisotropy,
                              dimension_offset,
                              mode_offset,
                              use_normalize_offset,
                              periodic_offset,
                              int(node2.w));
    stack_store_float(stack, value_out_offset, value);
  }

  return offset;
}

CCL_NAMESPACE_END
