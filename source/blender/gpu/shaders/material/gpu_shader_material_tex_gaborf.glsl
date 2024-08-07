/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma BLENDER_REQUIRE(gpu_shader_common_hash.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_math_base_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_math_vector_lib.glsl)

/* Gabor Noise
 *
 * Based on: Blender patch D287 & D3495
 * With additional controls for kernel variance.
 *
 * This implementation is adapted from the original built-in OSL implementation based on the 2009
 * paper "Procedural noise using sparse Gabor convolution". Some parts are also adapted from the
 * 2011 paper "Filtering Solid Gabor Noise" but this does not include the filtering and slicing.
 * References to the papers are for copyright and reference.
 *
 * Notes and changes from the original OSL implementation and reference papers:
 * - For 2D noise, as with Voronoi the calculation uses a 3x3x1 grid rather than slicing 3D noise.
 * This is more performant when only 2D texture is required.
 * - For artistic control, calculations for Bandwidth have been simplified and replaced with
 * separate controls for Frequency, Gain and Radius. This provides finer control where
 * before frequency and bandwidth were bound to the same parameter. Radius values over 1 may result
 * in artefacts and discontinuities. This is not clamped as pushing the radius can create some
 * potentially useful noise.
 * - Phasor noise has been added. Since this is based on Gabor and sums the changes in phase it is
 * trivial to add.
 * - Additional sincos based kernels have been added which provide different texture control in a
 * similar way to distance metrics in Voronoi.
 * - Added Roughness, Scale Lacunarity, Frequency Lacunarity and Rotation Lacunarity to control
 * additive fractal noise.
 * - Uses built-in Blender hashes instead of adding a separate gabor rng.
 *
 * Adapted from Open Shading Language implementation.
 * Copyright (c) 2009-2010 Sony Pictures Imageworks Inc., et al.
 * All Rights Reserved.
 *
 * OSL Gabor noise references:
 * Lagae, Lefebvre, Drettakis and Dutré, 2009. Procedural noise using sparse Gabor convolution
 * Lagae, A. and Drettakis, G. 2011. Filtering Solid Gabor Noise
 * Tavernier et al. 2018, Gabor Noise Revisited
 *
 * Phasor noise reference:
 * Tricard et al. 2019. Procedural Phasor Noise
 */

#define SHD_GABOR_MODE_GABOR 0
#define SHD_GABOR_MODE_RING 1
#define SHD_GABOR_MODE_CROSS 2
#define SHD_GABOR_MODE_SQUARE 3
#define SHD_GABOR_MODE_PHASOR 4
#define SHD_GABOR_MODE_PHASOR_RING 5
#define SHD_GABOR_MODE_PHASOR_CROSS 6
#define SHD_GABOR_MODE_PHASOR_SQUARE 7

/* Large prime number for grid offset and impulse seed. */
#define GABOR_SEED 1259

/* Struct to hold Gabor parameters. */
struct GaborParams {
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
  vec3 direction;
};

/* Struct to hold Fractal parameters. */
struct FractalParams {
  float octaves;
  float roughness;
  float scl_lacunarity;
  float fre_lacunarity;
  float rot_lacunarity;
};

/* Calculate impulses per cell. Performance is optimised when impulses are set to whole numbers.
 * It's useful to allow impulses less than 1 to create spotted textures which is why int(impulses)
 * is not directly used. Above 1 this provides a linear increase in impulses as the input value
 * increases. */
int impulses_per_cell(vec3 cell, float impulses, int seed)
{
  int n = int(impulses);
  float rmd = impulses - floor(impulses);
  if (rmd > 0.0) {
    float t = hash_vec4_to_float(vec4(cell, float(seed - GABOR_SEED)));
    return (t <= rmd) ? n + 1 : n;
  }
  return n;
}

/* Calculates the kernel shape that is multiplied by the gaussian envelope. For Phasor a sum of
 * two values is required. For Gabor a sum of one value is required. These are passed as a
 * vector in all cases. */
vec3 gabor_kernel(GaborParams gp, vec3 omega, float phi, vec3 position, float dv)
{
  float g = cos(M_PI * sqrt(dv)) * 0.5 + 0.5;
  vec3 r = vec3(0.0);
  float h;

  if (gp.mode == SHD_GABOR_MODE_GABOR) {
    h = gp.frequency * dot(omega, position) + phi;
    r = vec3(cos(h), 0.0, 0.0);
  }
  else if (gp.mode == SHD_GABOR_MODE_PHASOR) {
    h = gp.frequency * dot(omega, position) + phi;
    r = vec3(cos(h), sin(h), 0.0);
  }
  else if (gp.mode == SHD_GABOR_MODE_CROSS) {
    h = gp.frequency * length(omega * position) + phi;
    r = vec3(cos(h), 0.0, 0.0);
  }
  else if (gp.mode == SHD_GABOR_MODE_PHASOR_CROSS) {
    h = gp.frequency * length(omega * position) + phi;
    r = vec3(cos(h), sin(h), 0.0);
  }
  else if (gp.mode == SHD_GABOR_MODE_RING) {
    h = cos(gp.frequency * dot(omega, position) + phi) +
        cos(gp.frequency * length(position) + phi);
    r = vec3(h, 0.0, 0.0) * 0.5;
  }
  else if (gp.mode == SHD_GABOR_MODE_PHASOR_RING) {
    h = cos(gp.frequency * dot(omega, position) + phi) +
        cos(gp.frequency * length(position) + phi);
    float h2 = sin(gp.frequency * dot(omega, position) + phi) +
               sin(gp.frequency * length(position) + phi);
    r = vec3(h, h2, 0.0) * 0.5;
  }
  else if (gp.mode == SHD_GABOR_MODE_SQUARE) {
    h = cos(gp.frequency * dot(omega, position) + phi) +
        cos(gp.frequency * dot(omega, position.yxz) + phi);
    r = vec3(h, 0.0, 0.0) * 0.5;
  }
  else if (gp.mode == SHD_GABOR_MODE_PHASOR_SQUARE) {
    h = cos(gp.frequency * dot(omega, position) + phi) +
        cos(gp.frequency * dot(omega, position.yxz) + phi);
    float h2 = sin(gp.frequency * dot(omega, position) + phi) +
               sin(gp.frequency * dot(omega, position.yxz) + phi);
    r = vec3(h, h2, 0.0) * 0.5;
  }

  return r * g;
}

/* Set omega (angular frequency/direction) and phi (phase) for the impulse based on the anisotropy
 * parameters. Isotropic and Anisotropic modes have been combined using a factor to mix between
 * these modes. */
vec3 gabor_sample(GaborParams gp, vec3 cell, int seed, out float phi)
{
  vec3 rand_values = hash_vec4_to_vec3(vec4(cell, float(seed))) * 2.0 - 1.0;

  /* Phase. */
  float pvar = mix(0.0, rand_values.z, gp.phase_variance);
  phi = 2.0 * M_PI * pvar + gp.phase;

  /* Isotropic direction. */
  float omega_t = M_PI * (rand_values.x) * gp.rot_variance - gp.rotation - gp.init_rotation;
  float cos_omega_p = clamp(rand_values.y * gp.tilt_randomness, -1.0, 1.0);
  float sin_omega_p = sqrt(1.0 - cos_omega_p * cos_omega_p);
  float sin_omega_t = sin(omega_t);
  float cos_omega_t = cos(omega_t);

  /* Mix between Isotropic and Anisotropic direction. */
  return mix(normalize(vec3(cos_omega_t * sin_omega_p, sin_omega_t * sin_omega_p, cos_omega_p)),
             normalize(euler_to_mat3(vec3(0.0, 0.0, -gp.rotation)) * gp.direction),
             gp.anisotropy);
}

/* Generate noise based on the cell position and number of impulses. */
vec3 gabor_cell_3d(GaborParams gp, vec3 cell, vec3 cell_position, int seed)
{
  int num_impulses = impulses_per_cell(cell, gp.impulses, seed);
  vec3 sum = vec3(0.0);
  for (int i = 0; i < num_impulses; ++i) {
    vec3 rand_position = mix(vec3(0.0),
                             hash_vec4_to_vec3(vec4(cell, float(seed + i * GABOR_SEED))),
                             gp.cell_randomness);

    vec3 kernel_position = (cell_position - rand_position);

    float dv = dot(kernel_position, kernel_position) / gp.radius;

    if (dv <= 1.0) {
      float phi;
      vec3 omega = gabor_sample(gp, cell, seed + (num_impulses + i) * GABOR_SEED, phi);
      sum += gabor_kernel(gp, omega, phi, kernel_position, dv);
    }
  }
  return sum;
}

/* Generate noise based on the cell position and number of impulses. Z position is zeroed for 2D
 * cell as the kernel is still processed in 3D. */
vec3 gabor_cell_2d(GaborParams gp, vec3 cell, vec3 cell_position, int seed)
{
  int num_impulses = impulses_per_cell(cell, gp.impulses, seed);
  vec3 sum = vec3(0.0);
  for (int i = 0; i < num_impulses; ++i) {
    vec3 rand_position = mix(vec3(0.0),
                             hash_vec4_to_vec3(vec4(cell, float(seed + i * GABOR_SEED))),
                             gp.cell_randomness);
    rand_position.z = 0.0;

    vec3 kernel_position = (cell_position - rand_position);

    float dv = dot(kernel_position, kernel_position) / gp.radius;

    if (dv <= 1.0) {
      float phi;
      vec3 omega = gabor_sample(gp, cell, seed + (num_impulses + i) * GABOR_SEED, phi);
      sum += gabor_kernel(gp, omega, phi, kernel_position, dv);
    }
  }
  return sum;
}

/* Utility function to wrap coords for periodic mode. */
float gabor_coord_wrap(float s, float p)
{
  return (p != 0.0) ? s - p * floor(s / p) : 0.0;
}

/* Calculate 3D noise using 3x3x3 cell grid. */
vec3 gabor_grid_3d(GaborParams gp, vec3 p, float scale, int periodic, int seed)
{
  vec3 coords = p * scale;
  vec3 position = floor(coords);
  vec3 local_position = coords - position;

  vec3 sum = vec3(0.0);
  for (int k = -1; k <= 1; k++) {
    for (int j = -1; j <= 1; j++) {
      for (int i = -1; i <= 1; i++) {
        vec3 cell_offset = vec3(i, j, k);
        vec3 cell = position + cell_offset;
        vec3 cell_position = local_position - cell_offset;

        /* Skip this cell if it's too far away to contribute - Lee Bruemmer.osl */
        vec3 Pr = (vec3(i > 0, j > 0, k > 0) - local_position) * cell_offset;
        if (dot(Pr, Pr) >= 1.0) {
          continue;
        }

        /* Wrap cells for periodic noise. */
        if (periodic == 1) {
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

/* Calculate 2D noise using 3x3x1 cell grid. Less computational than 3x3x3 grid. */
vec3 gabor_grid_2d(GaborParams gp, vec3 p, float scale, int periodic, int seed)
{
  vec3 coords = vec3(p.xy, 0.0) * scale;
  vec3 position = floor(coords);
  vec3 local_position = coords - position;

  vec3 sum = vec3(0.0);
  for (int j = -1; j <= 1; j++) {
    for (int i = -1; i <= 1; i++) {
      vec3 cell_offset = vec3(i, j, 0.0);
      vec3 cell = position + cell_offset;
      vec3 cell_position = local_position - cell_offset;

      /* Skip this cell if it's too far away to contribute - Lee Bruemmer.osl */
      vec3 Pr = (vec3(i > 0, j > 0, 0) - local_position) * cell_offset;
      if (dot(Pr, Pr) >= 1.0) {
        continue;
      }

      /* Wrap cells for periodic noise. */
      if (periodic == 1) {
        cell.x = gabor_coord_wrap(cell.x, scale);
        cell.y = gabor_coord_wrap(cell.y, scale);
      }

      sum += gabor_cell_2d(gp, cell, cell_position, seed);
    }
  }
  return sum;
}

/* Layered fractal noise. Optionally, for each layer, the offset is changed. This ensures that
 * the impulses occur in different places if scale lacunarity is set to 1. Gabor has more variables
 * to consider when layering the noise compared to Perlin noise. Kernel frequency and rotation have
 * lacunarity parameters in addition to scale lacunarity and roughness that is found in Noise and
 * Voronoi textures.  */
float gabor_fractal_noise(FractalParams fp,
                          GaborParams gp,
                          vec3 p,
                          float scale,
                          int dimensions,
                          int periodic,
                          int use_origin_offset)
{
  float fscale = 1.0;
  float amp = 1.0;
  float maxamp = 0.0;
  float3 sum = vec3(0.0);
  float octaves = clamp(fp.octaves, 0.0, 15.0);
  if (fp.roughness == 0.0) {
    octaves = 0.0;
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
  float rmd = octaves - floor(octaves);
  if (rmd != 0.0) {
    int seed = use_origin_offset * (n + 1) * GABOR_SEED;
    float3 t = (dimensions == 3) ? gabor_grid_3d(gp, fscale * p, scale, periodic, seed) :
                                   gabor_grid_2d(gp, fscale * p, scale, periodic, seed);
    float3 sum2 = sum + t * amp;
    sum = mix(sum, sum2, rmd);
  }
  sum /= maxamp;

  /* Extract summed values for Phasor mode. */
  if (gp.mode == SHD_GABOR_MODE_PHASOR || gp.mode == SHD_GABOR_MODE_PHASOR_RING ||
      gp.mode == SHD_GABOR_MODE_PHASOR_CROSS || gp.mode == SHD_GABOR_MODE_PHASOR_SQUARE)
  {
    float pn = atan(sum.y, sum.x) / M_PI;
    return pn;
  }
  else {
    return sum.x;
  }
}

/* Gabor parameters. Impulses are clamped as these directly impact performance. */
GaborParams gabor_parameters(vec3 direction,
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
  gp.impulses = clamp(impulses, 0.0001, 32.0);
  gp.rot_variance = rot_variance;
  gp.anisotropy = anisotropy;
  gp.mode = mode;
  gp.direction = direction;
  gp.phase = phase;
  gp.rotation = 0.0;
  gp.init_rotation = rotation;
  gp.phase_variance = phase_variance;
  gp.tilt_randomness = tilt_randomness;
  gp.cell_randomness = cell_randomness;
  gp.radius = radius;
  gp.frequency = frequency * M_PI;
  return gp;
}

/* Gabor texture node. */

void node_tex_gaborf(vec3 co,
                     float scale,
                     float gain,
                     float detail,
                     float roughness,
                     float scl_lacunarity,
                     float fre_lacunarity,
                     float rot_lacunarity,
                     float frequency,
                     float radius,
                     float impulses,
                     float phase,
                     float phase_variance,
                     float cell_randomness,
                     float rotation,
                     float rot_variance,
                     float tilt_randomness,
                     float anisotropy,
                     vec3 direction,
                     float dimensions,
                     float mode,
                     float use_normalize,
                     float periodic,
                     float use_origin_offset,
                     out float value)
{
  /* Return early with mid level value. */
  if (impulses == 0.0 || gain == 0.0 || radius <= 0.0 || scale == 0.0) {
    value = (int(use_normalize) == 1) ? 0.5 : 0.0;
    return;
  }

  /* Set Fractal params. */
  FractalParams fp;
  fp.roughness = roughness;
  fp.octaves = detail;
  fp.scl_lacunarity = scl_lacunarity;
  fp.fre_lacunarity = fre_lacunarity;
  fp.rot_lacunarity = rot_lacunarity;

  /* Set Gabor params. */
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
                                    int(mode));

  float g = gabor_fractal_noise(
                fp, gp, co, scale, int(dimensions), int(periodic), int(use_origin_offset)) *
            gain;

  /* For Gabor modes, scale height of noise by the number of impulses. This is empircal as
   * there is no easy way to analytically determine the scaling factor for the sum of impulses.
   * The following calculation is taken from Lee Bruemmer's OSL script implementation.
   *
   * Scale the noise so the range [-1,1] covers 6 standard deviations, or 3*sqrt(variance)
   * Since the radius is set to 1/a the scale simplifies to 3/4*sqrt(3*n/(pi*sqrt(2))) float
   * scale = 0.6162961511 * sqrt( Impulses ); but with a fixed number of impulses also divide
   * by sqrt(0.75/pi) (0.4886025119) which give 1.2613446229 * sqrt(gp.impulses).
   *
   * In tests I've found that sqrt(2) 1.41 clips less but the previous value is better overall.
   * Artists can overcome this limitation by using the gain control to tweak the texture as
   * required.
   *
   * Phasor does not require this because it always returns [-1,1].
   */
  if (gp.mode == SHD_GABOR_MODE_GABOR || gp.mode == SHD_GABOR_MODE_RING ||
      gp.mode == SHD_GABOR_MODE_CROSS || gp.mode == SHD_GABOR_MODE_SQUARE)
  {
    float impulse_scale = impulses > 1.0 ? 1.2613446229 * sqrt(gp.impulses) : 1.2613446229;
    g = g / impulse_scale;
  }

  /* Normalise and clamp noise to [0.1] range. */
  if (int(use_normalize) == 1) {
    g = clamp(0.5 * g + 0.5, 0.0, 1.0);
  }
  value = g;
}
