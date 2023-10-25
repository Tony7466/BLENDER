/* SPDX-FileCopyrightText: 2017-2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Implementation of Horizon Based Global Illumination and Ambient Occlusion.
 *
 * This mostly follows the paper:
 * "Screen Space Indirect Lighting with Visibility Bitmask"
 * by Olivier Therrien, Yannick Levesque, Guillaume Gilet
 */

#pragma BLENDER_REQUIRE(draw_view_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_sampling_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_ray_types_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_horizon_scan_lib.glsl)

uint horizon_scan_slice(vec3 vV,
                        vec3 vP,
                        vec3 vN,
                        float vN_angle,
                        float noise,
                        ScreenSpaceRay ssray,
                        sampler2D depth_tx,
                        float search_distance,
                        float global_thickness,
                        inout vec3 accum_light,
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

      bool front_facing = vN.z > 0.0;

      /* Bias depth a bit to avoid self shadowing issues. */
      const float bias = 2.0 * 2.4e-7;
      sample_depth += front_facing ? bias : -bias;

      vec3 vP_front = drw_point_screen_to_view(vec3(sample_uv, sample_depth));
      vec3 vP_back = vP_front - global_thickness * vV;

      float dist_front, dist_back;
      vec3 vL_front = normalize_and_get_length(vP_front - vP, dist_front);
      vec3 vL_back = normalize_and_get_length(vP_back - vP, dist_back);

      /* TODO(fclem): This will not fade object correctly.
       * The correct way is to clip front and back to the sphere intersection. */
      if (vP_front.z > vP.z && vP.z > vP_back.z) {
        /* The surface is interesecting the search_distance sphere. */
      }
      else if (vP_front.z > vP.z) {
        /* The surface is in-front of the search_distance sphere center. */
        if (dist_back > search_distance) {
          /* No intersection with the search_distance sphere. */
          continue;
        }
      }
      else if (vP_front.z > vP.z) {
        /* The surface is behind of the search_distance sphere center. */
        if (dist_front > search_distance) {
          /* No intersection with the search_distance sphere. */
          continue;
        }
      }

      /* Ordered pair of angle. Mininum in X, Maximum in Y.
       * Front will always have the smallest angle here since it is the closest to the view. */
      vec2 theta = acos_fast(vec2(dot(vL_front, vV), dot(vL_back, vV)));
      /* If we are tracing backward, the angles are negative. Swizzle to keep correct order. */
      theta = (i == 0) ? theta.xy : -theta.yx;
      theta -= vN_angle;
      /* Angular bias. */
      theta *= 1.05;

      uint sample_bitmask = horizon_scan_angles_to_bitmask(theta);
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
      slice_bitmask |= sample_bitmask;
    }
  }
  return slice_bitmask;
}

vec3 horizon_scan_eval(vec3 vP,
                       vec3 vN,
                       sampler2D depth_tx,
                       vec2 noise,
                       vec2 pixel_size,
                       float search_distance,
                       float global_thickness,
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
    float vN_proj_len;
    vec3 vN_proj = normalize_and_get_length(vN - vB * dot(vN, vB), vN_proj_len);

    float vN_sin = dot(vN_proj, vT);
    float vN_cos = saturate(dot(vN_proj, vV));
    /* Angle between normalized projected normal and view vector. */
    float vN_angle = sign(vN_sin) * acos_fast(vN_cos);

    Ray ray;
    ray.origin = vP;
    ray.direction = vec3(v_dir, 0.0);
    ray.max_time = search_distance;

    ScreenSpaceRay ssray = raytrace_screenspace_ray_create(ray, pixel_size);

    vec3 light_slice = vec3(0.0);
    uint slice_bitmask = horizon_scan_slice(vV,
                                            vP,
                                            vN_proj,
                                            vN_angle,
                                            noise.y,
                                            ssray,
                                            depth_tx,
                                            search_distance,
                                            global_thickness,
                                            light_slice,
                                            sample_count);

    /* Add distant lighting. */
    light_slice = vec3(horizon_scan_bitmask_to_occlusion_cosine(slice_bitmask));
    /* Correct normal not on plane (Eq. 8 of GTAO paper). */
    accum_light += light_slice * vN_proj_len;
    accum_weight += vN_proj_len;

    /* Rotate 90 degrees. */
    v_dir = orthogonal(v_dir);
  }
  return accum_light * safe_rcp(accum_weight);
}