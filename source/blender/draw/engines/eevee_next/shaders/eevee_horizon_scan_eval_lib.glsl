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

#pragma BLENDER_REQUIRE(common_shape_lib.glsl)
#pragma BLENDER_REQUIRE(draw_view_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_sampling_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_horizon_scan_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_ray_types_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_codegen_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_bxdf_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_spherical_harmonics_lib.glsl)

#ifdef RAYTRACE_DIFFUSE
#  define HORIZON_DIFFUSE
#endif
#ifdef RAYTRACE_REFLECT
#  define HORIZON_REFLECT
#endif
#ifdef RAYTRACE_REFRACT
#  define HORIZON_REFRACT
#endif

vec3 horizon_scan_sample_radiance(vec2 uv)
{
#ifndef HORIZON_OCCLUSION
  return texture(screen_radiance_tx, uv).rgb;
#else
  return vec3(0.0);
#endif
}

vec3 horizon_scan_sample_normal(vec2 uv)
{
  return vec3(0.0);
}

/* Note: Expects all normals to be in view-space. */
struct HorizonScanContextCommon {
  float N_angle;
  float N_length;
  uint bitmask;
  vec3 light_slice;
  vec4 light_accum;
};

struct HorizonScanContext {
#ifdef HORIZON_OCCLUSION
  ClosureOcclusion occlusion;
  HorizonScanContextCommon occlusion_common;
  vec3 occlusion_result;
#endif
#ifdef HORIZON_DIFFUSE
  ClosureDiffuse diffuse;
  HorizonScanContextCommon diffuse_common;
  vec3 diffuse_result;
#endif
#ifdef HORIZON_REFLECT
  ClosureReflection reflection;
  HorizonScanContextCommon reflection_common;
  vec3 reflection_result;
#endif
#ifdef HORIZON_REFRACT
  ClosureRefraction refraction;
  HorizonScanContextCommon refraction_common;
  vec3 refraction_result;
#endif
  SphericalHarmonicL1 irradiance_distant;
};

void horizon_scan_context_accumulation_reset(inout HorizonScanContext context)
{
#ifdef HORIZON_OCCLUSION
  context.occlusion_common.light_accum = vec4(0.0);
#endif
#ifdef HORIZON_DIFFUSE
  context.diffuse_common.light_accum = vec4(0.0);
#endif
#ifdef HORIZON_REFLECT
  context.reflection_common.light_accum = vec4(0.0);
#endif
#ifdef HORIZON_REFRACT
  context.refraction_common.light_accum = vec4(0.0);
#endif
}

void horizon_scan_context_slice_start(
    inout HorizonScanContextCommon context, vec3 vN, vec3 vV, vec3 vT, vec3 vB)
{
  context.bitmask = 0u;
  context.light_slice = vec3(0.0);
  horizon_scan_projected_normal_to_plane_angle_and_length(
      vN, vV, vT, vB, context.N_length, context.N_angle);
}

void horizon_scan_context_slice_start(inout HorizonScanContext context, vec3 vV, vec3 vT, vec3 vB)
{
#ifdef HORIZON_OCCLUSION
  horizon_scan_context_slice_start(context.occlusion_common, context.occlusion.N, vV, vT, vB);
#endif
#ifdef HORIZON_DIFFUSE
  horizon_scan_context_slice_start(context.diffuse_common, context.diffuse.N, vV, vT, vB);
#endif
#ifdef HORIZON_REFLECT
  horizon_scan_context_slice_start(context.reflection_common, context.reflection.N, vV, vT, vB);
#endif
#ifdef HORIZON_REFRACT
  horizon_scan_context_slice_start(context.refraction_common, context.refraction.N, vV, vT, vB);
#endif
}

void horizon_scan_context_sample_finish(inout HorizonScanContextCommon context,
                                        vec3 sample_radiance,
                                        vec2 sample_theta,
                                        float angle_bias)
{
  /* Angular bias shrinks the visibility bitmask around the projected normal. */
  sample_theta = (sample_theta - context.N_angle) * angle_bias;
  uint sample_bitmask = horizon_scan_angles_to_bitmask(sample_theta);
  float sample_visibility = horizon_scan_bitmask_to_visibility_uniform(sample_bitmask &
                                                                       ~context.bitmask);
  context.light_slice += sample_radiance * sample_visibility;
  context.bitmask |= sample_bitmask;
}

float bxdf_eval(ClosureDiffuse closure, vec3 L, vec3 V)
{
  return bsdf_lambert(closure.N, L);
}

float bxdf_eval(ClosureRefraction closure, vec3 L, vec3 V)
{
  return btdf_ggx(closure.N, L, V, closure.roughness, closure.ior);
}

float bxdf_eval(ClosureReflection closure, vec3 L, vec3 V)
{
  return bsdf_ggx(closure.N, L, V, closure.roughness);
}

void horizon_scan_context_sample_finish(
    inout HorizonScanContext context, vec3 L, vec3 V, vec2 sample_uv, vec2 theta, float bias)
{
  vec3 sample_radiance = horizon_scan_sample_radiance(sample_uv);
#if 0
  /* TODO: Optionally take emitter surface normal into consideration. */
  vec3 sample_normal = horizon_scan_sample_normal(sample_uv);
  sample_radiance *= dot(sample_normal, -L);
#endif
#ifdef HORIZON_OCCLUSION
  horizon_scan_context_sample_finish(context.occlusion_common, vec3(0.0), theta, bias);
#endif
#ifdef HORIZON_DIFFUSE
  sample_radiance *= bxdf_eval(context.diffuse, L, V);
  horizon_scan_context_sample_finish(context.diffuse_common, sample_radiance, theta, bias);
#endif
#ifdef HORIZON_REFLECT
  /* TODO(fclem): Loosing energy here. */
  sample_radiance *= bxdf_eval(context.reflection, L, V);
  horizon_scan_context_sample_finish(context.reflection_common, sample_radiance, theta, bias);
#endif
#ifdef HORIZON_REFRACT
  /* TODO(fclem): Broken: Black. */
  sample_radiance *= bxdf_eval(context.refraction, L, V);
  horizon_scan_context_sample_finish(context.refraction_common, sample_radiance, theta, bias);
#endif
}

void horizon_scan_context_slice_finish_occlusion(inout HorizonScanContextCommon context)
{
  float slice_occlusion = horizon_scan_bitmask_to_occlusion_cosine(context.bitmask);
  /* Correct normal not on plane (Eq. 8 of GTAO paper). */
  context.light_accum += vec4(vec3(slice_occlusion), 1.0) * context.N_length;
}

void horizon_scan_context_slice_finish_distant_light(HorizonScanContext context,
                                                     inout HorizonScanContextCommon common_ctx,
                                                     vec3 vN,
                                                     vec3 vV,
                                                     vec3 vT)
{
  vec3 N = drw_normal_view_to_world(vN);
  vec3 light_distant = spherical_harmonics_evaluate_lambert(N, context.irradiance_distant);
  float visibility = horizon_scan_bitmask_to_visibility_uniform(~common_ctx.bitmask);

  /* Add distant lighting. */
  vec3 slice = common_ctx.light_slice + light_distant * visibility;
  /* Correct normal not on plane (Eq. 8 of GTAO paper). */
  common_ctx.light_accum += vec4(slice, 1.0) * common_ctx.N_length;
}

void horizon_scan_context_slice_finish(inout HorizonScanContext context, vec3 vV, vec3 vT)
{
#ifdef HORIZON_OCCLUSION
  horizon_scan_context_slice_finish_occlusion(context.occlusion_common);
#endif
#ifdef HORIZON_DIFFUSE
  horizon_scan_context_slice_finish_distant_light(
      context, context.diffuse_common, context.diffuse.N, vV, vT);
#endif
#ifdef HORIZON_REFLECT
  horizon_scan_context_slice_finish_distant_light(
      context, context.reflection_common, context.reflection.N, vV, vT);
#endif
#ifdef HORIZON_REFRACT
  horizon_scan_context_slice_finish_distant_light(
      context, context.refraction_common, context.refraction.N, vV, vT);
#endif
}

void horizon_scan_context_accumulation_finish(HorizonScanContextCommon context, out vec3 result)
{
  result = context.light_accum.xyz * safe_rcp(context.light_accum.w);
}

void horizon_scan_context_accumulation_finish(inout HorizonScanContext context)
{
#ifdef HORIZON_OCCLUSION
  horizon_scan_context_accumulation_finish(context.occlusion_common, context.occlusion_result);
#endif
#ifdef HORIZON_DIFFUSE
  horizon_scan_context_accumulation_finish(context.diffuse_common, context.diffuse_result);
#endif
#ifdef HORIZON_REFLECT
  horizon_scan_context_accumulation_finish(context.reflection_common, context.reflection_result);
#endif
#ifdef HORIZON_REFRACT
  horizon_scan_context_accumulation_finish(context.refraction_common, context.refraction_result);
#endif
}

/**
 * Returns the start and end point of a ray clipped to its intersection
 * with a sphere.
 */
void horizon_scan_occluder_intersection_ray_sphere_clip(Ray ray,
                                                        Sphere sphere,
                                                        out vec3 P_entry,
                                                        out vec3 P_exit)
{
  vec3 f = ray.origin - sphere.center;
  float a = length_squared(ray.direction);
  float b = 2.0 * dot(ray.direction, f);
  float c = length_squared(f) - square(sphere.radius);
  float determinant = b * b - 4.0 * a * c;
  if (determinant <= 0.0) {
    /* No intersection. Return null segment. */
    P_entry = P_exit = ray.origin;
    return;
  }
  /* Using fast sqrt_fast doesn't seem to cause artifact here. */
  float t_min = (-b - sqrt_fast(determinant)) / (2.0 * a);
  float t_max = (-b + sqrt_fast(determinant)) / (2.0 * a);
  /* Clip segment to the intersection range. */
  float t_entry = clamp(0.0, t_min, t_max);
  float t_exit = clamp(ray.max_time, t_min, t_max);

  P_entry = ray.origin + ray.direction * t_entry;
  P_exit = ray.origin + ray.direction * t_exit;
}

/**
 * Scans the horizon in many directions and returns the indirect lighting radiance.
 * Returned lighting is stored inside the context in `_accum` members already normalized.
 */
void horizon_scan_eval(vec3 vP,
                       inout HorizonScanContext context,
                       sampler2D depth_tx,
                       vec2 noise,
                       vec2 pixel_size,
                       float search_distance,
                       float global_thickness,
                       float angle_bias,
                       const int sample_count)
{
  vec3 vV = drw_view_incident_vector(vP);

#ifdef HORIZON_OCCLUSION
  const int slice_len = 2;
#else
  /* Cost of having another iteration is too high. Prefer denoising. */
  const int slice_len = 2;
#endif

  vec2 v_dir = sample_circle(noise.x * (0.5 / float(slice_len)));

  horizon_scan_context_accumulation_reset(context);

  for (int slice = 0; slice < slice_len; slice++) {
    /* Setup integration domain around V. */
    vec3 vB = normalize(cross(vV, vec3(v_dir, 0.0)));
    vec3 vT = cross(vB, vV);

    horizon_scan_context_slice_start(context, vV, vT, vB);

    /* For both sides of the view vector. */
    for (int side = 0; side < 2; side++) {
      Ray ray;
      ray.origin = vP;
      ray.direction = vec3((side == 0) ? v_dir : -v_dir, 0.0);
      ray.max_time = search_distance;

      /* TODO(fclem): Could save some computation here by computing entry and exit point on the
       * screen at once and just scan through. */
      ScreenSpaceRay ssray = raytrace_screenspace_ray_create(ray, pixel_size);

      for (int j = 0; j < sample_count; j++) {
        /* Always cross at least one pixel. */
        float time = 1.0 + square((float(j) + noise.y) / float(sample_count)) * ssray.max_time;

        float lod = float(j >> 2) / (1.0 + uniform_buf.ao.quality);

        vec2 sample_uv = ssray.origin.xy + ssray.direction.xy * time;
        float sample_depth =
            textureLod(depth_tx, sample_uv * uniform_buf.hiz.uv_scale, floor(lod)).r;

        if (sample_depth == 1.0) {
          /* Skip background. Avoids making shadow on the geometry near the far plane. */
          continue;
        }

        /* TODO(fclem): Re-introduce bias. But this is difficult to do per closure. */
        bool front_facing = true;  // vN.z > 0.0;

        /* Bias depth a bit to avoid self shadowing issues. */
        const float bias = 2.0 * 2.4e-7;
        sample_depth += front_facing ? bias : -bias;

        vec3 vP_sample = drw_point_screen_to_view(vec3(sample_uv, sample_depth));

        Ray ray;
        ray.origin = vP_sample;
        ray.direction = -vV;
        ray.max_time = global_thickness;

        Sphere sphere = shape_sphere(vP, search_distance);

        vec3 vP_front, vP_back;
        horizon_scan_occluder_intersection_ray_sphere_clip(ray, sphere, vP_front, vP_back);

        vec3 vL_front = normalize(vP_front - vP);
        vec3 vL_back = normalize(vP_back - vP);

        /* Ordered pair of angle. Mininum in X, Maximum in Y.
         * Front will always have the smallest angle here since it is the closest to the view. */
        vec2 theta = acos_fast(vec2(dot(vL_front, vV), dot(vL_back, vV)));
        /* If we are tracing backward, the angles are negative. Swizzle to keep correct order. */
        theta = (side == 0) ? theta.xy : -theta.yx;

        horizon_scan_context_sample_finish(context, vL_front, vV, sample_uv, theta, angle_bias);
      }
    }

    horizon_scan_context_slice_finish(context, vV, vT);

    /* Rotate 90 degrees. */
    v_dir = orthogonal(v_dir);
  }

  horizon_scan_context_accumulation_finish(context);
}