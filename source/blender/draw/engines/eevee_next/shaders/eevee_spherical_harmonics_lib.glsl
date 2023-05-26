
#pragma BLENDER_REQUIRE(gpu_shader_math_base_lib.glsl)

/* -------------------------------------------------------------------- */
/** \name Spherical Harmonics Functions
 *
 * `L` denote the row and `M` the column in the spherical harmonics table (1).
 * `p` denote positive column and `n` negative ones.
 *
 * Use precomputed constants to avoid constant folding differences across compilers.
 * Note that (2) doesn't use Condon-Shortley phase whereas our implementation does.
 *
 * Reference:
 * (1) https://en.wikipedia.org/wiki/Spherical_harmonics#/media/File:Sphericalfunctions.svg
 * (2) https://en.wikipedia.org/wiki/Table_of_spherical_harmonics#Real_spherical_harmonics
 * (3) https://seblagarde.wordpress.com/2012/01/08/pi-or-not-to-pi-in-game-lighting-equation/
 *
 * \{ */

/* L0 Band. */
float spherical_harmonics_L0_M0(vec3 v)
{
  return 0.282094792;
}

/* L1 Band. */
float spherical_harmonics_L1_Mn1(vec3 v)
{
  return -0.488602512 * v.y;
}
float spherical_harmonics_L1_M0(vec3 v)
{
  return 0.488602512 * v.z;
}
float spherical_harmonics_L1_Mp1(vec3 v)
{
  return -0.488602512 * v.x;
}

/* L2 Band. */
float spherical_harmonics_L2_Mn2(vec3 v)
{
  return 1.092548431 * (v.x * v.y);
}
float spherical_harmonics_L2_Mn1(vec3 v)
{
  return -1.092548431 * (v.y * v.z);
}
float spherical_harmonics_L2_M0(vec3 v)
{
  return 0.315391565 * (3.0 * v.z * v.z - 1.0);
}
float spherical_harmonics_L2_Mp1(vec3 v)
{
  return -1.092548431 * (v.x * v.z);
}
float spherical_harmonics_L2_Mp2(vec3 v)
{
  return 0.546274215 * (v.x * v.x - v.y * v.y);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Structure
 * \{ */

struct SphericalHarmonicBandL0 {
  vec4 M0;
};

struct SphericalHarmonicBandL1 {
  vec4 Mn1;
  vec4 M0;
  vec4 Mp1;
};

struct SphericalHarmonicBandL2 {
  vec4 Mn2;
  vec4 Mn1;
  vec4 M0;
  vec4 Mp1;
  vec4 Mp2;
};

struct SphericalHarmonicL0 {
  SphericalHarmonicBandL0 L0;
};

struct SphericalHarmonicL1 {
  SphericalHarmonicBandL0 L0;
  SphericalHarmonicBandL1 L1;
};

struct SphericalHarmonicL2 {
  SphericalHarmonicBandL0 L0;
  SphericalHarmonicBandL1 L1;
  SphericalHarmonicBandL2 L2;
};

/** \} */

/* -------------------------------------------------------------------- */
/** \name Encode
 *
 * Decompose an input signal into spherical harmonic coefficients.
 * Note that `amplitude` need to be scaled by solid angle.
 * \{ */

void spherical_harmonics_L0_encode_signal_sample(vec3 direction,
                                                 vec4 amplitude,
                                                 inout SphericalHarmonicBandL0 r_L0)
{
  r_L0.M0 += spherical_harmonics_L0_M0(direction) * amplitude;
}

void spherical_harmonics_L1_encode_signal_sample(vec3 direction,
                                                 vec4 amplitude,
                                                 inout SphericalHarmonicBandL1 r_L1)
{
  r_L1.Mn1 += spherical_harmonics_L1_Mn1(direction) * amplitude;
  r_L1.M0 += spherical_harmonics_L1_M0(direction) * amplitude;
  r_L1.Mp1 += spherical_harmonics_L1_Mp1(direction) * amplitude;
}

void spherical_harmonics_L2_encode_signal_sample(vec3 direction,
                                                 vec4 amplitude,
                                                 inout SphericalHarmonicBandL2 r_L2)
{
  r_L2.Mn2 += spherical_harmonics_L2_Mn2(direction) * amplitude;
  r_L2.Mn1 += spherical_harmonics_L2_Mn1(direction) * amplitude;
  r_L2.M0 += spherical_harmonics_L2_M0(direction) * amplitude;
  r_L2.Mp1 += spherical_harmonics_L2_Mp1(direction) * amplitude;
  r_L2.Mp2 += spherical_harmonics_L2_Mp2(direction) * amplitude;
}

void spherical_harmonics_encode_signal_sample(vec3 direction,
                                              vec4 amplitude,
                                              inout SphericalHarmonicL0 sh)
{
  spherical_harmonics_L0_encode_signal_sample(direction, amplitude, sh.L0);
}

void spherical_harmonics_encode_signal_sample(vec3 direction,
                                              vec4 amplitude,
                                              inout SphericalHarmonicL1 sh)
{
  spherical_harmonics_L0_encode_signal_sample(direction, amplitude, sh.L0);
  spherical_harmonics_L1_encode_signal_sample(direction, amplitude, sh.L1);
}

void spherical_harmonics_encode_signal_sample(vec3 direction,
                                              vec4 amplitude,
                                              inout SphericalHarmonicL2 sh)
{
  spherical_harmonics_L0_encode_signal_sample(direction, amplitude, sh.L0);
  spherical_harmonics_L1_encode_signal_sample(direction, amplitude, sh.L1);
  spherical_harmonics_L2_encode_signal_sample(direction, amplitude, sh.L2);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Decode
 *
 * Evaluate an encoded signal in a given unit vector direction.
 * \{ */

vec4 spherical_harmonics_L0_evaluate(vec3 direction, SphericalHarmonicBandL0 L0)
{
  return spherical_harmonics_L0_M0(direction) * L0.M0;
}

vec4 spherical_harmonics_L1_evaluate(vec3 direction, SphericalHarmonicBandL1 L1)
{
  return spherical_harmonics_L1_Mn1(direction) * L1.Mn1 +
         spherical_harmonics_L1_M0(direction) * L1.M0 +
         spherical_harmonics_L1_Mp1(direction) * L1.Mp1;
}

vec4 spherical_harmonics_L2_evaluate(vec3 direction, SphericalHarmonicBandL2 L2)
{
  return spherical_harmonics_L2_Mn2(direction) * L2.Mn2 +
         spherical_harmonics_L2_Mn1(direction) * L2.Mn1 +
         spherical_harmonics_L2_M0(direction) * L2.M0 +
         spherical_harmonics_L2_Mp1(direction) * L2.Mp1 +
         spherical_harmonics_L2_Mp2(direction) * L2.Mp2;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Rotation
 * \{ */

void spherical_harmonics_L0_rotate(mat3x3 rotation, inout SphericalHarmonicBandL0 L0)
{
  /* L0 band being a constant function (i.e: there is no directionallity) there is nothing to
   * rotate. This is a no-op. */
}

void spherical_harmonics_L1_rotate(mat3x3 rotation, inout SphericalHarmonicBandL1 L1)
{
  /* Convert L1 coefficients to per channel column.
   * Note the component shuffle to match blender coordinate system. */
  mat4x3 per_channel = transpose(mat3x4(L1.Mp1, L1.Mn1, -L1.M0));
  /* Rotate each channel. */
  per_channel[0] = rotation * per_channel[0];
  per_channel[1] = rotation * per_channel[1];
  per_channel[2] = rotation * per_channel[2];
  /* Convert back to L1 coefficients to per channel column.
   * Note the component shuffle to match blender coordinate system. */
  mat3x4 per_coef = transpose(per_channel);
  L1.Mn1 = per_coef[1];
  L1.M0 = -per_coef[2];
  L1.Mp1 = per_coef[0];
}

void spherical_harmonics_L2_rotate(mat3x3 rotation, inout SphericalHarmonicBandL2 L2)
{
  /* TODO */
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Evaluation
 * \{ */

/**
 * Convolve a spherical harmonic encoded irradiance signal as a lambertian reflection.
 * Returns the lambertian radiance (cosine lobe divided by PI) so the coefficients simplify to 1,
 * 2/3 and 1/4. See this reference for more explanation:
 * https://seblagarde.wordpress.com/2012/01/08/pi-or-not-to-pi-in-game-lighting-equation/
 */
vec3 spherical_harmonics_evaluate_lambert(vec3 N, SphericalHarmonicL0 sh)
{
  vec3 radiance = spherical_harmonics_L0_evaluate(N, sh.L0).rgb;
  return radiance;
}
vec3 spherical_harmonics_evaluate_lambert(vec3 N, SphericalHarmonicL1 sh)
{
  vec3 radiance = spherical_harmonics_L0_evaluate(N, sh.L0).rgb +
                  spherical_harmonics_L1_evaluate(N, sh.L1).rgb * (2.0 / 3.0);
  return radiance;
}
vec3 spherical_harmonics_evaluate_lambert(vec3 N, SphericalHarmonicL2 sh)
{
  vec3 radiance = spherical_harmonics_L0_evaluate(N, sh.L0).rgb +
                  spherical_harmonics_L1_evaluate(N, sh.L1).rgb * (2.0 / 3.0) +
                  spherical_harmonics_L2_evaluate(N, sh.L2).rgb * (1.0 / 4.0);
  return radiance;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Load/Store
 *
 * This section define the compression scheme of spherical harmonic data.
 * \{ */

SphericalHarmonicL1 spherical_harmonics_unpack(vec4 L0_L1_a,
                                               vec4 L0_L1_b,
                                               vec4 L0_L1_c,
                                               vec4 L0_L1_vis)
{
  SphericalHarmonicL1 sh;
  sh.L0.M0.xyz = L0_L1_a.xyz;
  sh.L1.Mn1.xyz = L0_L1_b.xyz;
  sh.L1.M0.xyz = L0_L1_c.xyz;
  sh.L1.Mp1.xyz = vec3(L0_L1_a.w, L0_L1_b.w, L0_L1_c.w);
  sh.L0.M0.w = L0_L1_vis.x;
  sh.L1.Mn1.w = L0_L1_vis.y;
  sh.L1.M0.w = L0_L1_vis.z;
  sh.L1.Mp1.w = L0_L1_vis.w;
  return sh;
}

void spherical_harmonics_pack(SphericalHarmonicL1 sh,
                              out vec4 L0_L1_a,
                              out vec4 L0_L1_b,
                              out vec4 L0_L1_c,
                              out vec4 L0_L1_vis)
{
  L0_L1_a.xyz = sh.L0.M0.xyz;
  L0_L1_b.xyz = sh.L1.Mn1.xyz;
  L0_L1_c.xyz = sh.L1.M0.xyz;
  L0_L1_a.w = sh.L1.Mp1.x;
  L0_L1_b.w = sh.L1.Mp1.y;
  L0_L1_c.w = sh.L1.Mp1.z;
  L0_L1_vis = vec4(sh.L0.M0.w, sh.L1.Mn1.w, sh.L1.M0.w, sh.L1.Mp1.w);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Deringing
 *
 * Adapted from Google's Filament under Apache 2.0 licence.
 * https://github.com/google/filament
 *
 * SH from environment with high dynamic range (or high frequencies -- high dynamic range creates
 * high frequencies) exhibit "ringing" and negative values when reconstructed.
 * To mitigate this, we need to low-pass the input image -- or equivalently window the SH by
 * coefficient that tapper towards zero with the band.
 *
 * "Stupid Spherical Harmonics (SH)"
 * "Deringing Spherical Harmonics"
 * by Peter-Pike Sloan
 * https://www.ppsloan.org/publications/shdering.pdf
 *
 * \{ */

float spherical_harmonics_sinc_window(const int l, float w)
{
  if (l == 0) {
    return 1.0;
  }
  else if (float(l) >= w) {
    return 0.0;
  }

  /* We use a sinc window scaled to the desired window size in bands units a sinc window only has
   * zonal harmonics. */
  float x = (M_PI * float(l)) / w;
  x = sin(x) / x;

  /* The convolution of a SH function f and a ZH function h is just the product of both
   * scaled by 1 / K(0,l) -- the window coefficients include this scale factor. */

  /* Taking the window to power N is equivalent to applying the filter N times. */
  return pow4f(x);
}

void spherical_harmonics_apply_windowing(inout SphericalHarmonicL2 sh, float cutoff)
{
  float w_l0 = spherical_harmonics_sinc_window(0, cutoff);
  sh.L0.M0 *= w_l0;

  float w_l1 = spherical_harmonics_sinc_window(1, cutoff);
  sh.L1.Mn1 *= w_l1;
  sh.L1.M0 *= w_l1;
  sh.L1.Mp1 *= w_l1;

  float w_l2 = spherical_harmonics_sinc_window(2, cutoff);
  sh.L2.Mn2 *= w_l2;
  sh.L2.Mn1 *= w_l2;
  sh.L2.M0 *= w_l2;
  sh.L2.Mp1 *= w_l2;
  sh.L2.Mp2 *= w_l2;
}

/**
 * Find the minimum of the Spherical harmonic function.
 * Section 2.3 from the paper.
 * Only work on a single channel at a time.
 */
float spherical_harmonics_find_minimum(SphericalHarmonicL2 sh, int channel)
{
  const int comp = channel;
  const float coef_L0_M0 = 0.282094792;
  const float coef_L1_Mn1 = -0.488602512;
  const float coef_L1_M0 = 0.488602512;
  const float coef_L1_Mp1 = -0.488602512;
  const float coef_L2_Mn2 = 1.092548431;
  const float coef_L2_Mn1 = -1.092548431;
  const float coef_L2_M0 = 0.315391565;
  const float coef_L2_Mp1 = -1.092548431;
  const float coef_L2_Mp2 = 0.546274215;

  /* Align the SH Z axis with the optimal linear direction. */
  vec3 z_axis = normalize(vec3(sh.L1.Mp1[comp], sh.L1.Mn1[comp], -sh.L1.M0[comp]));
  vec3 x_axis = normalize(cross(z_axis, vec3(0.0, 1.0, 0.0)));
  vec3 y_axis = cross(x_axis, z_axis);
  mat3x3 rotation = transpose(mat3x3(x_axis, y_axis, -z_axis));

  spherical_harmonics_L1_rotate(rotation, sh.L1);

  /* 2.3.2: Find the min for |m| = 2:
   *
   * Peter-Pike Sloan shows that the minimum can be expressed as a function
   * of z such as:  m2min = -m2_max * (1 - z^2) =  m2_max * z^2 - m2_max
   *      with m2_max = A[8] * sqrt(f[8] * f[8] + f[4] * f[4]);
   * We can therefore include this in the ZH min computation (which is function of z^2 as well).
   */
  float m2_max = coef_L2_Mp2 * sqrt(square_f(sh.L2.Mp2[comp]) + square_f(sh.L2.Mn2[comp]));

  /* 2.3.1: Find the min of the zonal harmonics:
   *
   * This comes from minimizing the function:
   *      ZH(z) = (coef_L0_M0 * sh.L0.M0)
   *            + (coef_L1_M0 * sh.L1.M0) * z
   *            + (coef_L2_M0 * sh.L2.M0) * (3 * sqr(s.z) - 1)
   *
   * We do that by finding where it's derivative d/dz is zero:
   *      dZH(z)/dz = a * z^2 + b * z + c
   *      which is zero for z = -b / 2 * a
   *
   * We also needs to check that -1 < z < 1, otherwise the min is either in z = -1 or 1
   */
  float a = 3.0 * coef_L2_M0 * sh.L2.M0[comp] + m2_max;
  float b = coef_L1_M0 * sh.L1.M0[comp];
  float c = coef_L0_M0 * sh.L0.M0[comp] - coef_L2_M0 * sh.L2.M0[comp] - m2_max;

  float z_min = -b / (2.0 * a);
  float m0_min_z = a * pow2f(z_min) + b * z_min + c;
  float m0_min_b = min(a + b + c, a - b + c);

  float m0_min = (a > 0.0 && z_min >= -1.0 && z_min <= 1.0) ? m0_min_z : m0_min_b;

  /* 2.3.3: Find the min for l = 2, |m| = 1:
   *
   * Note l = 1, |m| = 1 is guaranteed to be 0 because of the rotation step
   *
   * The function considered is:
   *        Y(x, y, z) = A[5] * f[5] * s.y * s.z
   *                   + A[7] * f[7] * s.z * s.x
   */
  float d = coef_L2_Mn2 * sqrt(square_f(sh.L2.Mp1[comp]) + square_f(sh.L2.Mn1[comp]));

  /* The |m|=1 function is minimal in -0.5 -- use that to skip the Newton's loop when possible. */
  float minimum = m0_min - 0.5 * d;
  if (minimum < 0) {
    /* We could be negative, to find the minimum we will use Newton's method
     * See https://en.wikipedia.org/wiki/Newton%27s_method_in_optimization */

    /* This is the function we're trying to minimize:
     * y = a * sqr(x) + b * x + c) + (d * x * sqrt(1.0 - sqr(x))
     * First term accounts for ZH + |m| = 2, second terms for |m| = 1. */

    /* We start guessing at the min of |m|=1 function. */
    float z = -M_SQRT1_2;
    float dz;
    do {
      /* Evaluate our function. */
      minimum = (a * pow2f(z) + b * z + c) + (d * z * sqrt(1.0 - pow2f(z)));
      /* This is `func' / func''`. This was computed with Mathematica. */
      dz = (pow2f(z) - 1.0) * (d - 2 * d * pow2f(z) + (b + 2.0 * a * z) * sqrt(1 - pow2f(z))) /
           (3 * d * z - 2 * d * pow3f(z) - 2.0 * a * pow(1 - pow2f(z), 1.5f));
      /* Refine our guess by this amount. */
      z = z - dz;
      /* Exit if z goes out of range, or if we have reached enough precision. */
    } while (abs(z) <= 1.0 && abs(dz) > 1.0e-5);

    if (abs(z) > 1.0) {
      /* z was out of range. Compute `min(function(1), function(-1))`. */
      float func_pos = (a + b + c);
      float func_neg = (a - b + c);
      minimum = min(func_pos, func_neg);
    }
  }
  return minimum;
}

void spherical_harmonics_dering(inout SphericalHarmonicL2 sh)
{
  float cutoff = 0.0;
  if (true) {
    /* Auto windowing.
     * Find cutoff threshold automatically. Can be replaced by manual cutoff value. */
    SphericalHarmonicL2 tmp_sh = sh;

    const int num_bands = 3;
    /* Start at a large band. */
    float cutoff = float(num_bands * 4 + 1);
    /* We need to process each channel separately. */
    for (int channel = 0; channel < 4; channel++) {
      /* Find a cut-off band that works. */
      float l = num_bands;
      float r = cutoff;
      for (int i = 0; i < 16 && (l + 0.1) < r; i++) {
        float m = 0.5 * (l + r);
        spherical_harmonics_apply_windowing(tmp_sh, m);

        float sh_min = spherical_harmonics_find_minimum(tmp_sh, channel);
        if (sh_min < 0.0) {
          r = m;
        }
        else {
          l = m;
        }
      }
      cutoff = min(cutoff, l);
    }
  }

  spherical_harmonics_apply_windowing(sh, cutoff);
}

/** \} */
