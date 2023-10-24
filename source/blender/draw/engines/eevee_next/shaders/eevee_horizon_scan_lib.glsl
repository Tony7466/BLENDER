/* SPDX-FileCopyrightText: 2017-2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma BLENDER_REQUIRE(draw_view_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_utildefines_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_math_vector_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_sampling_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_ray_types_lib.glsl)

/**
 * Returns the bitmask for a given ordered pair of angle in [-pi/2..pi/2] range.
 * Clamps the inputs to the valid range.
 */
uint horizon_scan_bitmask(vec2 theta)
{
  const int bitmask_len = 32;
  /* Algorithm 1, line 18. Re-ordered to make sure to clamp to the hemisphere range. */
  vec2 ratio = saturate(theta * M_1_PI + 0.5);
  uint a = uint(floor(float(bitmask_len) * ratio.x));
  /* The paper is wrong here. The additional half Pi is not needed . */
  uint b = uint(ceil(float(bitmask_len) * (ratio.y - ratio.x)));
  /* Algorithm 1, line 19. */
  return ((1u << b) - 1u) << a;
}

float bsdf_eval(vec3 N, vec3 L)
{
  return dot(N, L);
}

uint horizon_scan_bitmask_scan(vec3 vV,
                               vec3 vP,
                               vec3 vN,
                               float angle_vN,
                               float noise,
                               ScreenSpaceRay ssray,
                               sampler2D depth_tx,
                               float search_distance,
                               float thickness,
                               inout vec3 accum_light,
                               const bool inverted,
                               const int sample_count)
{
  ssray.max_time -= 1.0;

  if (ssray.max_time <= 2.0) {
    return 0u;
  }

  const float bitmask_len = 32.0;
  uint slice_bitmask = 0u;
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < sample_count; j++) {
      /* Gives us good precision at center and ensure we cross at least one pixel per iteration. */
      float time = 1.0 + square((float(j) + noise) / float(sample_count)) * ssray.max_time;
      float lod = float(j >> 2) / (1.0 + uniform_buf.ao.quality);

      vec2 sample_uv = ssray.origin.xy + ssray.direction.xy * ((i == 0) ? time : -time);
      float sample_depth =
          textureLod(depth_tx, sample_uv * uniform_buf.hiz.uv_scale, floor(lod)).r;

      if (sample_depth == 1.0) {
        /* Skip background. Avoids making shadow on the geometry near the far plane. */
        continue;
      }

      /* Bias depth a bit to avoid self shadowing issues. */
      const float bias = 2.0 * 2.4e-7;
      sample_depth += bias;

      vec3 vP_front = drw_point_screen_to_view(vec3(sample_uv, sample_depth));
      vec3 vP_back = vP_front - thickness * vV;

      if (distance(vP_front, vP) > search_distance) {
        continue;
      }

      float dist_front, dist_back;
      vec3 vL_front = normalize_and_get_length(vP_front - vP, dist_front);
      vec3 vL_back = normalize_and_get_length(vP_back - vP, dist_back);

      /* Ordered pair of angle. Mininum in X, Maximum in Y.
       * Front will always have the smallest angle here since it is the closest to the view. */
      vec2 theta = acos_fast(vec2(dot(vL_front, vV), dot(vL_back, vV)));
      /* If we are tracing backward, the angles are negative. Swizzle to keep correct order. */
      theta = (i == 0) ? theta.xy : -theta.yx;
      theta -= angle_vN;

      uint sample_bitmask = horizon_scan_bitmask(theta);
#ifdef USE_RADIANCE_ACCUMULATION
      float sample_visibility = float(bitCount(sample_bitmask & ~slice_bitmask)) / bitmask_len;
      if (sample_visibility > 0.0) {
        vec3 sample_radiance = horizon_scan_sample_radiance(sample_uv);
#  ifdef USE_NORMAL_MASKING
        vec3 sample_normal = horizon_scan_sample_normal(sample_uv);
        sample_visibility *= dot(sample_normal, -vL_front);
#  endif
        accum_light += sample_radiance * (bsdf_eval(vN, vL_front) * sample_visibility);
      }
#endif
      slice_bitmask |= horizon_scan_bitmask(theta);
    }
  }
  return slice_bitmask;
}

vec3 horizon_scan(vec3 vP,
                  vec3 vN,
                  sampler2D depth_tx,
                  vec2 noise,
                  vec2 pixel_size,
                  float search_distance,
                  float thickness,
                  const bool inverted,
                  const int sample_count)
{
  vec3 vV = drw_view_incident_vector(vP);

  /* NOTE(@fclem): This could help in some cases. But don't know how much useful it is right now.
   * Also doesn't work at every distances. */
#if 0
  /* Bias to avoid self shadowing. */
  vP = vP + vN * 0.02;
#endif

  /* Only a quarter of a turn because we integrate using 2 slices.
   * We use this instead of using full circle noise to improve cache hits
   * since all tracing direction will be in the same quadrant. */
  vec2 v_dir = sample_circle(noise.x * 0.25);

  vec3 accum_light = vec3(0.0);
  float accum_weight = 0.0;

  for (int i = 0; i < 2; i++) {
    /* Setup integration domain around V. */
    vec3 vB = normalize(cross(vV, vec3(v_dir, 0.0)));
    vec3 vT = cross(vB, vV);
    /* Projected view normal onto the integration plane. */
    float proj_vN_len;
    vec3 proj_vN = normalize_and_get_length(vN - vB * dot(vN, vB), proj_vN_len);

    float N_sin = dot(proj_vN, vT);
    float N_cos = saturate(dot(proj_vN, vV));
    /* Angle between normalized projected normal and view vector. */
    float N_angle = sign(N_sin) * acos_fast(N_cos);

    Ray ray;
    ray.origin = vP;
    ray.direction = vec3(v_dir, 0.0);
    ray.max_time = search_distance;

    ScreenSpaceRay ssray = raytrace_screenspace_ray_create(ray, pixel_size);

    vec3 light_slice = vec3(0.0);
    uint slice_bitmask = horizon_scan_bitmask_scan(vV,
                                                   vP,
                                                   vN,
                                                   N_angle,
                                                   noise.y,
                                                   ssray,
                                                   depth_tx,
                                                   search_distance,
                                                   thickness,
                                                   light_slice,
                                                   inverted,
                                                   sample_count);

    const int bitmask_len = 32;
    /* Add distant lighting. */
#if 0
    /* Uniformly Weighted Occlusion. */
    light_slice = vec3(1.0 - float(bitCount(slice_bitmask)) / float(bitmask_len));
#else
    /* Cosine Weighted Occlusion. */
    for (int bit = 0; bit < bitmask_len; bit++, slice_bitmask >>= 1u) {
      if ((slice_bitmask & 1u) == 0u) {
        float angle = (((float(bit) + 0.5) / float(bitmask_len)) - 0.5) * M_PI;
        /* Integrating over the hemisphere. */
        light_slice += cos(angle) * M_PI_2 / float(bitmask_len);
      }
    }
#endif
    /* Correct normal not on plane (Eq. 8 of GTAO paper). */
    accum_light += light_slice * proj_vN_len;
    accum_weight += proj_vN_len;

    /* Rotate 90 degrees. */
    v_dir = orthogonal(v_dir);
  }
  return accum_light * safe_rcp(accum_weight);
}