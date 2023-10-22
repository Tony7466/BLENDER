/* SPDX-FileCopyrightText: 2017-2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma BLENDER_REQUIRE(draw_view_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_utildefines_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_sampling_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_ray_types_lib.glsl)

float bsdf_eval(vec3 N, vec3 L)
{
  return dot(N, L);
}

float horizon_scan_get_random(ivec2 texel)
{
  return interlieved_gradient_noise(vec2(texel), 0, sampling_rng_1D_get(SAMPLING_AO_U));
}

vec2 horizon_scan_get_dir(float rand)
{
  /* Only a quarter of a turn because we integrate using 2 slices.
   * We use this instead of using utiltex circle noise to improve cache hits
   * since all tracing direction will be in the same quadrant. */
  rand *= M_PI_2;
  return vec2(cos(rand), sin(rand));
}

struct HorizonScanResult {
  float visibility;
  vec3 indirect_lighting;
  vec3 bent_normal;
};

void horizon_scan_bitmask(vec3 vV,
                          vec3 vP,
                          vec3 vN,
                          float angle_vN,
                          float noise,
                          ScreenSpaceRay ssray,
                          sampler2D depth_tx,
                          float search_distance,
                          float thickness,
                          inout uint slice_visbits,
                          inout vec3 accum_light,
                          const bool inverted,
                          const int sample_count)
{
  ssray.max_time -= 1.0;

  if (ssray.max_time <= 2.0) {
    return;
  }

  const float bitmask_len = 32.0;
  for (int j = 0; j < sample_count; j++) {
    /* Gives us good precision at center and ensure we cross at least one pixel per iteration. */
    float time = 1.0 + square((float(j) + noise) / float(sample_count)) * ssray.max_time;
    float lod = float(j >> 2) / (1.0 + uniform_buf.ao.quality);

    vec2 sample_uv = ssray.origin.xy + ssray.direction.xy * time;
    float sample_depth = textureLod(depth_tx, sample_uv * uniform_buf.hiz.uv_scale, floor(lod)).r;

    if (sample_depth == 1.0 && inverted) {
      /* Skip background. Avoids making shadow on the geometry near the far plane. */
      continue;
    }

    vec3 front_vP = drw_point_screen_to_view(vec3(sample_uv, sample_depth));
    vec3 back_vP = front_vP - thickness * vV;
    float theta_front = acos_fast(dot(normalize(front_vP - vP), vV)) - angle_vN;
    float theta_back = acos_fast(dot(normalize(back_vP - vP), vV)) - angle_vN;
    float theta_min = min(theta_front, theta_back);
    float theta_max = max(theta_front, theta_back);
    int a = int(floor(bitmask_len * ((theta_min + M_PI_2) / M_PI)));
    int b = int(ceil(bitmask_len * ((theta_max - theta_min + M_PI_2) / M_PI)));
    /* Create bitmask from b-1 bit to a. */
    uint sample_visbits = (1u << b) - (1u << a);
    slice_visbits |= sample_visbits;
#if 0
    vec3 sample_radiance = horizon_scan_sample_radiance(sample_uv);
    vec3 sample_normal = horizon_scan_sample_normal(sample_uv);
    vec3 vL = normalize(front_vP - vP);
    float sample_visibility = float(bitCount(sample_visbits & ~slice_visbits)) / bitmask_len;
    sample_visibility *= dot(sample_normal, -vL);
    accum_light += sample_radiance * (bsdf_eval(vN, vL) * sample_visibility);
#endif
  }
}

vec2 horizon_scan_search_horizon(vec3 vV,
                                 vec3 vP,
                                 vec3 vN,
                                 float angle_vN,
                                 float noise,
                                 ScreenSpaceRay ssray,
                                 sampler2D depth_tx,
                                 float search_distance,
                                 float thickness,
                                 const bool inverted,
                                 const int sample_count)
{
  const float max_angle = M_PI_2 - 0.05;
  vec2 h = vec2(max_angle, -max_angle);

  ssray.max_time -= 1.0;

  if (ssray.max_time <= 2.0) {
    /* Produces self shadowing under this threshold. */
    return h;
  }

  const float bitmask_len = 32.0;
  float prev_time, time = 0.0;
  uint slice_visbits = 0u;
  for (float iter = 0.0; time < ssray.max_time && iter < sample_count; iter++) {
    prev_time = time;
    /* Gives us good precision at center and ensure we cross at least one pixel per iteration. */
    time = 1.0 + iter + square((iter + noise) / sample_count) * ssray.max_time;
    float stride = time - prev_time;
    float lod = (log2(stride) - noise) / (1.0 + uniform_buf.ao.quality);

    for (int i = 0; i < 2; i++) {
      vec2 sample_uv = ssray.origin.xy + ssray.direction.xy * ((i == 0) ? time : -time);
      float sample_depth =
          textureLod(depth_tx, sample_uv * uniform_buf.hiz.uv_scale, floor(lod)).r;

      if (sample_depth == 1.0 && inverted) {
        /* Skip background. Avoids making shadow on the geometry near the far plane. */
        continue;
      }

      /* Bias depth a bit to avoid self shadowing issues. */
      const float bias = 2.0 * 2.4e-7;
      sample_depth += bias;

      vec3 front_vP = drw_point_screen_to_view(vec3(sample_uv, sample_depth));
      vec3 back_vP = front_vP - thickness * vV;

      if (length(front_vP - vP) > search_distance) {
        // continue;
      }

      vec2 theta;
      theta.x = acos_fast(dot(normalize(front_vP - vP), vV));
      theta.y = acos_fast(dot(normalize(back_vP - vP), vV));
      theta = (i == 0) ? theta : -theta;
      theta -= angle_vN;
      float theta_min = reduce_min(theta);
      float theta_max = reduce_max(theta);
      int a = int(floor(bitmask_len * ((theta_min + M_PI_2) / M_PI)));
      int b = int(ceil(bitmask_len * ((theta_max - theta_min + M_PI_2) / M_PI)));

      /* Sample's horizon angle cosine. */
      if (i == 0) {
        h[i] = min(h[i], theta.x);
      }
      else {
        h[i] = max(h[i], theta.x);
      }
    }
  }
  return h;
}

HorizonScanResult horizon_scan(vec3 vP,
                               vec3 vN,
                               sampler2D depth_tx,
                               ivec2 texel,
                               vec2 pixel_size,
                               float search_distance,
                               float thickness,
                               const bool inverted,
                               const int sample_count)
{
  vec3 vV = drw_view_incident_vector(vP);

  float noise = horizon_scan_get_random(texel);
  vec2 v_dir = horizon_scan_get_dir(noise);

  float accum_visibility = 0.0;
  vec3 accum_light = vec3(0);
  vec3 accum_bent_normal = vec3(0);

  const int slice_count = 2;
  for (int i = 0; i < slice_count; i++) {

    /* Setup integration domain around V. */
    vec3 vB = normalize(cross(vV, vec3(v_dir, 0.0)));
    vec3 vT = cross(vB, vV);
    /* Projected view normal onto the integration plane. */
    float proj_vN_len;
    vec3 proj_vN = normalize_and_get_length(vN - vB * dot(vN, vB), proj_vN_len);

    float N_sin = dot(proj_vN, vT);
    float N_cos = saturate(dot(proj_vN, vV));
    /* Gamma, angle between normalized projected normal and view vector. */
    float angle_N = sign(N_sin) * acos_fast(N_cos);

    Ray ray;
    ray.origin = vP;
    ray.direction = vec3(v_dir, 0.0);
    ray.max_time = search_distance;

#if 0
    for (int j = 0; j < 2; j++) {
      const float bitmask_len = 32.0;
      ScreenSpaceRay ssray = raytrace_screenspace_ray_create(ray, pixel_size);
      uint slice_visbits = 0u;
      horizon_scan_bitmask(vV,
                           vP,
                           vN,
                           angle_N,
                           noise,
                           ssray,
                           depth_tx,
                           search_distance,
                           thickness,
                           slice_visbits,
                           accum_light,
                           inverted,
                           sample_count);
      accum_visibility += 1.0 - float(bitCount(slice_visbits)) / bitmask_len;
      ray.direction = -ray.direction;
    }
#else
    ScreenSpaceRay ssray = raytrace_screenspace_ray_create(ray, pixel_size);
    vec2 h = horizon_scan_search_horizon(vV,
                                         vP,
                                         vN,
                                         angle_N,
                                         noise,
                                         ssray,
                                         depth_tx,
                                         search_distance,
                                         thickness,
                                         inverted,
                                         sample_count);

    float bent_angle = (h.x + h.y + angle_N) * 0.5;
    /* NOTE: here we multiply z by 0.5 as it shows less difference with the geometric normal.
     * Also modulate by projected normal length to reduce issues with slanted surfaces.
     * All of this is ad-hoc and not really grounded. */
    accum_bent_normal += proj_vN_len * (vT * sin(bent_angle) + vV * 0.5 * cos(bent_angle));
    /* Inner integral (Eq. 7). */
    float a = dot(-cos(2.0 * h) + N_cos + 2.0 * (h + angle_N) * N_sin, vec2(0.25));
    /* Correct normal not on plane (Eq. 8). */
    accum_visibility += proj_vN_len * a;
#endif

    /* Rotate 90 degrees. */
    v_dir = orthogonal(v_dir);
  }

  HorizonScanResult result;
  result.visibility = accum_visibility / float(slice_count);
  result.indirect_lighting = accum_light / float(slice_count);
  result.bent_normal = accum_bent_normal / float(slice_count);
  return result;
}