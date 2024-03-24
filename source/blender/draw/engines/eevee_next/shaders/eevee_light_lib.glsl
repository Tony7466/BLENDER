/* SPDX-FileCopyrightText: 2022-2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma BLENDER_REQUIRE(draw_math_geom_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_math_base_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_ltc_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_light_iter_lib.glsl)

/* Attenuation cutoff needs to be the same in the shadow loop and the light eval loop. */
#define LIGHT_ATTENUATION_THRESHOLD 1e-6

/* ---------------------------------------------------------------------- */
/** \name Light Functions
 * \{ */

struct LightVector {
  /* World space light vector. From the shading point to the light center. Normalized. */
  vec3 L;
  /* Distance from the shading point to the light center. */
  float dist;
};

LightVector light_vector_get(LightData light, const bool is_directional, vec3 P)
{
  LightVector lv;
  if (is_directional) {
    lv.L = light._back;
    lv.dist = 1.0;
  }
  else {
    lv.L = light._position - P;
    float inv_distance = inversesqrt(length_squared(lv.L));
    lv.L *= inv_distance;
    lv.dist = 1.0 / inv_distance;
  }
  return lv;
}

/* Light vector to the closest point in the light shape. */
LightVector light_shape_vector_get(LightData light, const bool is_directional, vec3 P)
{
  if (!is_directional && is_area_light(light.type)) {
    vec3 L = P - light._position;
    vec2 closest_point = vec2(dot(light._right, L), dot(light._up, L));
    vec2 max_pos = vec2(light._area_size_x, light._area_size_y);
    closest_point /= max_pos;

    if (light.type == LIGHT_ELLIPSE) {
      closest_point /= max(1.0, length(closest_point));
    }
    else {
      closest_point = clamp(closest_point, -1.0, 1.0);
    }
    closest_point *= max_pos;

    vec3 L_prime = light._right * closest_point.x + light._up * closest_point.y;

    L = L_prime - L;
    float inv_distance = inversesqrt(length_squared(L));
    LightVector lv;
    lv.L = L * inv_distance;
    lv.dist = 1.0 / inv_distance;
    return lv;
  }
  /* TODO(@fclem): other light shape? */
  return light_vector_get(light, is_directional, P);
}

/* Rotate vector to light's local space. Does not translate. */
vec3 light_world_to_local(LightData light, vec3 L)
{
  /* Avoid relying on compiler to optimize this.
   * vec3 lL = transpose(mat3(light.object_mat)) * L; */
  vec3 lL;
  lL.x = dot(light.object_mat[0].xyz, L);
  lL.y = dot(light.object_mat[1].xyz, L);
  lL.z = dot(light.object_mat[2].xyz, L);
  return lL;
}

/* Transform position from light's local space to world space. Does translation. */
vec3 light_local_position_to_world(LightData light, vec3 lP)
{
  return mat3(light.object_mat) * lP + light._position;
}

/* From Frostbite PBR Course
 * Distance based attenuation
 * http://www.frostbite.com/wp-content/uploads/2014/11/course_notes_moving_frostbite_to_pbr.pdf */
float light_influence_attenuation(float dist, float inv_sqr_influence)
{
  float factor = square(dist) * inv_sqr_influence;
  float fac = saturate(1.0 - square(factor));
  return square(fac);
}

float light_spot_attenuation(LightData light, vec3 L)
{
  vec3 lL = light_world_to_local(light, L);
  float ellipse = inversesqrt(1.0 + length_squared(lL.xy * light.spot_size_inv / lL.z));
  float spotmask = smoothstep(0.0, 1.0, ellipse * light._spot_mul + light._spot_bias);
  return spotmask * step(0.0, -dot(L, -light._back));
}

/**
 * Approximate the ratio of the area of intersection of two spherical caps divided by the area of
 * the smallest cap.
 */
float light_cone_cone_ratio(float cone_angle_1, float cone_angle_2, float angle_between_axes)
{
  /* From "Ambient Aperture Lighting" by Chris Oat
   * Slide 15.
   * Simplified since we divide by the solid angle of the smallest cone. */
  return smoothstep(
      cone_angle_1 + cone_angle_2, abs(cone_angle_1 - cone_angle_2), abs(angle_between_axes));
}

float linearstep(float edge0, float edge1, float x)
{
  return saturate((x - edge0) / (edge1 - edge0));
}

float ratio_cone_cone(float cone_angle_1, float cone_angle_2, float angle_between_axes)
{
  return light_cone_cone_ratio(cone_angle_1, cone_angle_2, angle_between_axes);
}

float area_cone(float cone_angle)
{
  return M_TAU * (1.0 - cos(cone_angle));
}

float area_lune(float lune_angle)
{
  return linearstep(0.0, M_TAU, abs(lune_angle));
}

float area_cone_cone(float cone_angle_1, float cone_angle_2, float angle_between_axes)
{
  return ratio_cone_cone(cone_angle_1, cone_angle_2, angle_between_axes) *
         area_cone(min(cone_angle_1, cone_angle_2));
}

float area_cone_hemisphere(float cone_angle, float angle_between_axes)
{
  float r = abs(cone_angle);
  float d = abs(angle_between_axes);
  d = clamp(d, M_PI_2 - r + 0.0001, M_PI_2 + r - 0.0001);
  return -2.0 * acos(cos(d) / sin(r)) - 2.0 * acos(-cos(d) * cos(r) / (sin(d) * sin(r))) * cos(r) +
         M_TAU;
}

float light_cone_lune_area(float spread_half_angle, float edge_angle_1, float edge_angle_2)
{
  float angle_hemi1 = edge_angle_1 + M_PI_2;
  float angle_hemi2 = edge_angle_2 - M_PI_2;
  float ratio_hemi1_cone_isect = area_cone_hemisphere(spread_half_angle, angle_hemi1) /
                                 min(area_cone_hemisphere(M_PI_2, angle_hemi1),
                                     area_cone(spread_half_angle));
  float ratio_hemi2_cone_isect = area_cone_hemisphere(spread_half_angle, angle_hemi2) /
                                 min(area_cone_hemisphere(M_PI_2, angle_hemi2),
                                     area_cone(spread_half_angle));
  return min(ratio_hemi1_cone_isect, ratio_hemi2_cone_isect);
}

float light_lune_cone_ratio(float spread_half_angle, float edge_angle_1, float edge_angle_2)
{
  return light_cone_lune_area(spread_half_angle, edge_angle_1, edge_angle_2);
}

/* https://www.shadertoy.com/view/4dVcR1 */

vec2 msign(vec2 x)
{
  return vec2((x.x < 0.0) ? -1.0 : 1.0, (x.y < 0.0) ? -1.0 : 1.0);
}

vec2 sdEllipseNearestPoint(vec2 p, vec2 ab)
{
  vec2 p_sign = msign(p);
  // symmetry
  p = abs(p);

  // find root with Newton solver
  vec2 q = ab * (p - ab);

  float w = (q.x < q.y) ? 1.570796327 : 0.0;
  for (int i = 0; i < 4; i++) {
    vec2 cs = vec2(cos(w), sin(w));
    vec2 u = ab * vec2(cs.x, cs.y);
    vec2 v = ab * vec2(-cs.y, cs.x);
    w = w + dot(p - u, v) / (dot(p - u, u) + dot(v, v));
  }

  return ab * vec2(cos(w), sin(w)) * p_sign;
}

vec2 sdEllipseFarthestPoint(vec2 p, vec2 ab)
{
  vec2 p_sign = msign(p);
  // symmetry
  p = abs(p);

  // find root with Newton solver
  vec2 q = ab * (p + ab);

  float w = 3.1415 + ((q.x < q.y) ? 1.570796327 : 0.0);
  for (int i = 0; i < 4; i++) {
    vec2 cs = vec2(cos(w), sin(w));
    vec2 u = ab * vec2(cs.x, cs.y);
    vec2 v = ab * vec2(-cs.y, cs.x);
    w = w + dot(p - u, v) / (dot(p - u, u) + dot(v, v));
  }

  return ab * vec2(cos(w), sin(w)) * p_sign;
}

/* from Real-Time Area Lighting: a Journey from Research to Production
 * Stephen Hill and Eric Heitz */
vec3 light_edge_integral_vec(vec3 v1, vec3 v2)
{
  float x = dot(v1, v2);
  float y = abs(x);

  float a = 0.8543985 + (0.4965155 + 0.0145206 * y) * y;
  float b = 3.4175940 + (4.1616724 + y) * y;
  float v = a / b;

  float theta_sintheta = (x > 0.0) ? v : 0.5 * inversesqrt(max(1.0 - x * x, 1e-7)) - v;

  return cross(v1, v2) * theta_sintheta;
}

float light_spread_angle_attenuation(LightData light, vec3 L, float dist)
{
  vec3 lL = light_world_to_local(light, L * dist);
  float distance_to_plane = abs(lL.z);
  vec2 area_size = vec2(light._area_size_x, light._area_size_y);

  if (light.type == LIGHT_RECT) {
    vec2 r1 = lL.xy - area_size;
    vec2 r2 = lL.xy + area_size;

    vec3 corners[4];
    corners[0] = vec3(area_size.x, -area_size.y, 0.0);
    corners[1] = vec3(area_size.x, area_size.y, 0.0);
    corners[2] = -corners[0];
    corners[3] = -corners[1];

    corners[0] = normalize(corners[0] + lL);
    corners[1] = normalize(corners[1] + lL);
    corners[2] = normalize(corners[2] + lL);
    corners[3] = normalize(corners[3] + lL);

    vec3 avg_dir;
    avg_dir = light_edge_integral_vec(corners[0], corners[1]);
    avg_dir += light_edge_integral_vec(corners[1], corners[2]);
    avg_dir += light_edge_integral_vec(corners[2], corners[3]);
    avg_dir += light_edge_integral_vec(corners[3], corners[0]);

    float form_factor = length(avg_dir);
    float avg_dir_z = (avg_dir / form_factor).z;

    float half_light_angle = acos(1.0 - form_factor);

    return light_cone_cone_ratio(light.spread_half_angle, half_light_angle, acos(avg_dir_z));

    /* angle_1 is min angle of intersection with first edge of the lune.
     * angle_2 is min angle of intersection with second edge of the lune. */
    /* TODO(fclem): Port fast_atanf from cycles. */
    vec2 angle_1 = atan(r1, vec2(distance_to_plane));
    vec2 angle_2 = atan(r2, vec2(distance_to_plane));

    float x = light_lune_cone_ratio(light.spread_half_angle, angle_1.x, angle_2.x);
    float y = light_lune_cone_ratio(light.spread_half_angle, angle_1.y, angle_2.y);
    return x * y;
  }

  float r1 = distance(sdEllipseNearestPoint(lL.xy, area_size), lL.xy);
  float r2 = distance(sdEllipseFarthestPoint(lL.xy, area_size), lL.xy);

  bool inside_ellipse = length_squared(lL.xy / area_size) < 1.0;
  if (inside_ellipse) {
    /* Signed distance. */
    r1 = -r1;
  }

  /* TODO(fclem): Port fast_atanf from cycles. */
  float angle_1 = atan(r1, distance_to_plane);
  float angle_2 = atan(r2, distance_to_plane);

  float half_light_angle = abs(angle_1 - angle_2) / 2.0;
  float elevation_angle = (angle_1 + angle_2) / 2.0;

  return light_cone_cone_ratio(light.spread_half_angle, half_light_angle, elevation_angle);
}

float light_attenuation_common(LightData light, const bool is_directional, vec3 L, float dist)
{
  if (is_directional) {
    return 1.0;
  }
  if (is_spot_light(light.type)) {
    return light_spot_attenuation(light, L);
  }
  if (is_area_light(light.type)) {
    return step(0.0, -dot(L, -light._back)) * light_spread_angle_attenuation(light, L, dist);
  }
  return 1.0;
}

/**
 * Fade light influence when surface is not facing the light.
 * This is needed because LTC leaks light at roughness not 0 or 1
 * when the light is below the horizon.
 * L is normalized vector to light shape center.
 * Ng is ideally the geometric normal.
 */
vec2 light_attenuation_facing(LightData light, vec3 L, float distance_to_light, vec3 Ng)
{
  float radius;
  if (is_area_light(light.type)) {
    radius = length(vec2(light._area_size_x, light._area_size_y));
  }
  else {
    radius = light._radius;
  }
  /* Sine of angle between light center and light edge. */
  float sin_solid_angle = radius / distance_to_light;
  /* Sine of angle between light center and shading plane. */
  float sin_light_angle = dot(L, Ng);
  /* Do attenuation after the horizon line to avoid harsh cut
   * or biasing of surfaces without light bleeding. */
  /* Compute for both front facing and back-facing. */
  return saturate((vec2(sin_light_angle, -sin_light_angle) + sin_solid_angle + 0.1) * 10.0);
}

vec2 light_attenuation_surface(LightData light, const bool is_directional, vec3 Ng, LightVector lv)
{
  return light_attenuation_common(light, is_directional, lv.L, lv.dist) *
         light_attenuation_facing(light, lv.L, lv.dist, Ng) *
         light_influence_attenuation(lv.dist, light.influence_radius_invsqr_surface);
}

float light_attenuation_volume(LightData light, const bool is_directional, LightVector lv)
{
  return light_attenuation_common(light, is_directional, lv.L, lv.dist) *
         light_influence_attenuation(lv.dist, light.influence_radius_invsqr_volume);
}

/* Cheaper alternative than evaluating the LTC.
 * The result needs to be multiplied by BSDF or Phase Function. */
float light_point_light(LightData light, const bool is_directional, LightVector lv)
{
  if (is_directional) {
    return 1.0;
  }
  /* Using "Point Light Attenuation Without Singularity" from Cem Yuksel
   * http://www.cemyuksel.com/research/pointlightattenuation/pointlightattenuation.pdf
   * http://www.cemyuksel.com/research/pointlightattenuation/
   */
  float d_sqr = square(lv.dist);
  float r_sqr = light.radius_squared;
  /* Using reformulation that has better numerical precision. */
  float power = 2.0 / (d_sqr + r_sqr + lv.dist * sqrt(d_sqr + r_sqr));

  if (is_area_light(light.type)) {
    /* Modulate by light plane orientation / solid angle. */
    power *= saturate(dot(light._back, lv.L));
  }
  return power;
}

/**
 * Return the radius of the disk at the sphere origin spanning the same solid angle as the sphere
 * from a given distance.
 * Assumes `distance_to_sphere > sphere_radius`.
 */
float light_sphere_disk_radius(float sphere_radius, float distance_to_sphere)
{
  /* The sine of the half-angle spanned by a sphere light is equal to the tangent of the
   * half-angle spanned by a disk light with the same radius. */
  return sphere_radius * inversesqrt(1.0 - square(sphere_radius / distance_to_sphere));
}

float light_ltc(
    sampler2DArray utility_tx, LightData light, vec3 N, vec3 V, LightVector lv, vec4 ltc_mat)
{
  if (is_sphere_light(light.type) && lv.dist < light._radius) {
    /* Inside the sphere light, integrate over the hemisphere. */
    return 1.0;
  }

  if (light.type == LIGHT_RECT) {
    vec3 corners[4];
    corners[0] = light._right * light._area_size_x + light._up * -light._area_size_y;
    corners[1] = light._right * light._area_size_x + light._up * light._area_size_y;
    corners[2] = -corners[0];
    corners[3] = -corners[1];

    vec3 L = mix(light._back, lv.L * lv.dist, light.spread_mix_fac);
    corners[0] += L;
    corners[1] += L;
    corners[2] += L;
    corners[3] += L;

    ltc_transform_quad(N, V, ltc_matrix(ltc_mat), corners);

    return ltc_evaluate_quad(utility_tx, corners, vec3(0.0, 0.0, 1.0));
  }
  else {
    vec3 Px = light._right;
    vec3 Py = light._up;

    if (!is_area_light(light.type)) {
      make_orthonormal_basis(lv.L, Px, Py);
    }

    vec2 size;
    if (is_sphere_light(light.type)) {
      /* Spherical omni or spot light. */
      size = vec2(light_sphere_disk_radius(light._radius, lv.dist));
    }
    else if (light.type == LIGHT_OMNI_DISK || light.type == LIGHT_SPOT_DISK) {
      /* View direction-aligned disk. */
      size = vec2(light._radius);
    }
    else {
      /* Sun light and elliptical area light. */
      size = vec2(light._area_size_x, light._area_size_y);
    }

    vec3 points[3];
    points[0] = Px * -size.x + Py * -size.y;
    points[1] = Px * size.x + Py * -size.y;
    points[2] = -points[0];

    vec3 L = mix(light._back, lv.L * lv.dist, light.spread_mix_fac);
    points[0] += L;
    points[1] += L;
    points[2] += L;

    return ltc_evaluate_disk(utility_tx, N, V, ltc_matrix(ltc_mat), points);
  }
}

/** \} */
