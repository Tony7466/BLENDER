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

float light_circular_segment_area(float segment_height, float disk_radius)
{
  /* Using notation from https://mathworld.wolfram.com/CircularSegment.html . */
  float R = disk_radius;
  float r = segment_height;
#if 0
  return (R * R) * acos(r / R) - r * sqrt(R * R - r * r);
#else
  /* Approximation. This gives smoother result but a bit darker on the edges and a bit brighter on
   * the inner parts. */
  return smoothstep(-R, R, -r) * (M_PI * R * R);
#endif
}

/* Optimized version already divided by the disk area. */
float light_circular_segment_area_opti(float segment_height, float disk_radius)
{
  /* Using notation from https://mathworld.wolfram.com/CircularSegment.html . */
  float R = disk_radius;
  float r = segment_height;
  return smoothstep(-R, R, -r);
}

float light_disk_area(float disk_radius)
{
  return square(disk_radius) * M_PI;
}

float light_spread_angle_rect_1D(float disk_center_distance, float disk_radius, float rect_extent)
{
  return light_circular_segment_area_opti(disk_center_distance - rect_extent, disk_radius) -
         light_circular_segment_area_opti(disk_center_distance + rect_extent, disk_radius);
}

/**
 * Approximate the area of intersection of two spherical caps
 * radius1 : First cap’s radius (arc length in radians)
 * radius2 : Second caps’ radius (in radians)
 * dist : Distance between caps (radians between centers of caps)
 * NOTE: Result is divided by pi to save one multiply.
 */
float light_spread_spherical_cap_intersection(float radius1, float radius2, float dist)
{
  /* From "Ambient Aperture Lighting" by Chris Oat
   * Slide 15. */
  float max_radius = max(radius1, radius2);
  float min_radius = min(radius1, radius2);
  float sum_radius = radius1 + radius2;
  float area;
  if (dist <= max_radius - min_radius) {
    /* One cap in completely inside the other */
    area = 1.0;
  }
  else if (dist >= sum_radius) {
    /* No intersection exists */
    area = 0;
  }
  else {
    float diff = max_radius - min_radius;
    area = smoothstep(0.0, 1.0, 1.0 - saturate((dist - diff) / (sum_radius - diff)));
    area *= 1.0;
  }
  return area;
}

float light_cone_cone_ratio(float cone_half_angle,
                            float light_half_angle,
                            float angle_cone_hemisphere)
{
  float cone_hemisphere_ratio = light_spread_spherical_cap_intersection(
      cone_half_angle, light_half_angle, angle_cone_hemisphere);
  /* But we don't want the clipped ratio wrt the shading hemisphere (otherwise, we duplicate what
   * the LTC computes and do not converge towards 1 at full aperture), so divide by the shading
   * hemisphere ratio. */
  cone_hemisphere_ratio /= light_spread_spherical_cap_intersection(
      M_PI / 2.0, light_half_angle, angle_cone_hemisphere);
  return cone_hemisphere_ratio;
}

float light_hemisphere_cone_ratio(float cone_half_angle, float angle_cone_hemisphere)
{
  float cone_hemisphere_ratio = light_spread_spherical_cap_intersection(
      cone_half_angle, M_PI / 2.0, angle_cone_hemisphere);
  /* But we don't want the clipped ratio wrt the shading hemisphere (otherwise, we duplicate what
   * the LTC computes and do not converge towards 1 at full aperture), so divide by the shading
   * hemisphere ratio. */
  cone_hemisphere_ratio /= light_spread_spherical_cap_intersection(
      M_PI / 2.0, M_PI / 2.0, angle_cone_hemisphere);
  return cone_hemisphere_ratio;
}

float light_spread_angle_rect_1D_spherical(
    float spread_half_angle, vec3 corners0, vec3 corners1, vec3 corners2, vec3 corners3)
{
  float hemisphere_angle_1 = acos(normalize(cross(corners0, corners1)).z);
  float hemisphere_angle_2 = acos(normalize(cross(corners2, corners3)).z);

  float max_ratio = light_hemisphere_cone_ratio(spread_half_angle,
                                                min(hemisphere_angle_1, hemisphere_angle_2));
  float min_ratio = light_hemisphere_cone_ratio(spread_half_angle,
                                                max(hemisphere_angle_1, hemisphere_angle_2));

  /* Should maybe be this. But need signed angles */
  // return (max_ratio - min_ratio);
  return min(max_ratio, min_ratio);
}

/* https://www.shadertoy.com/view/4dVcR1 */

float sdEllipse(vec2 p, vec2 e)
{
  vec2 pAbs = abs(p);
  vec2 ei = 1.0 / e;
  vec2 e2 = e * e;
  vec2 ve = ei * vec2(e2.x - e2.y, e2.y - e2.x);

  vec2 t = vec2(0.70710678118654752, 0.70710678118654752);
  for (int i = 0; i < 3; i++) {
    vec2 v = ve * t * t * t;
    vec2 u = normalize(pAbs - v) * length(t * e - v);
    vec2 w = ei * (v + u);
    t = normalize(clamp(w, 0.0, 1.0));
  }

  vec2 nearestAbs = t * e;
  float dist = length(pAbs - nearestAbs);
  return dot(pAbs, pAbs) < dot(nearestAbs, nearestAbs) ? -dist : dist;
}

float sdBox(vec2 p, vec2 b)
{
  vec2 q = abs(p) - b;
  return length(max(q, 0.0)) + min(max(q.x, q.y), 0.0);
}

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

float light_spread_angle_attenuation(LightData light, vec3 L, float dist)
{
  vec3 lL = light_world_to_local(light, L * dist);
  vec3 lL_norm = light_world_to_local(light, L);
  float spread_half_angle_tan = tan(light.spread_half_angle);
  float distance_to_plane = abs(lL.z);
  /* Using notation from https://mathworld.wolfram.com/Circle-CircleIntersection.html . */
  float r = spread_half_angle_tan;
  if (light.type == LIGHT_RECT) {
    vec3 corners[4];
    corners[0] = vec3(light._area_size_x, -light._area_size_y, 0.0);
    corners[1] = vec3(light._area_size_x, light._area_size_y, 0.0);
    corners[2] = -corners[0];
    corners[3] = -corners[1];

    corners[0] = normalize(corners[0] + lL);
    corners[1] = normalize(corners[1] + lL);
    corners[2] = normalize(corners[2] + lL);
    corners[3] = normalize(corners[3] + lL);

    float min_angle = atan(sdBox(lL.xy, vec2(light._area_size_x, light._area_size_y)),
                           distance_to_plane);
    float max_angle = acos(min(min(corners[0].z, corners[1].z), min(corners[2].z, corners[3].z)));

    float half_light_angle = abs(max_angle - min_angle) / 2.0;
    float elevation_angle = (max_angle + min_angle) / 2.0;

    // return light_cone_cone_ratio(light.spread_half_angle, half_light_angle, elevation_angle);

    /* Intuition. The coverage of each sector of */
    float x = light_spread_angle_rect_1D_spherical(
        light.spread_half_angle, corners[0], corners[1], corners[2], corners[3]);
    float y = light_spread_angle_rect_1D_spherical(
        light.spread_half_angle, corners[1], corners[2], corners[3], corners[0]);
    return x * y;
  }
  /* Ellipse approximation. Stretch the configuration and resize the projected disk. This has the
   * benefit to be very simple (simplify a few instructions since the light radius is 1 afterwards)
   * and give the proper result for disk lights. */
  vec2 area_size = vec2(light._area_size_x, light._area_size_y);

  float min_axis = reduce_min(area_size);
  float maj_axis = reduce_max(area_size);
  vec2 segment_size = vec2(maj_axis - min_axis, 0.0);
  if (area_size.x < area_size.y) {
    segment_size = segment_size.yx;
  }

  vec2 nearest_point_on_segment = clamp(lL.xy, -segment_size, segment_size);
  vec2 nearest_point_on_pill = normalize(lL.xy - nearest_point_on_segment) * min_axis +
                               nearest_point_on_segment;
  vec2 nearest_point_on_shape = normalize(lL.xy / area_size) * area_size;

  // nearest_point_on_shape = nearest_point_on_segment;

  vec2 normalized_dir = normalize(lL.xy);
  vec2 line_perp = orthogonal(normalized_dir);

  /* Align with on the Cone-Light Line */
  // nearest_point_on_shape -= dot(line_perp, nearest_point_on_shape) * line_perp;
  /* test 1 Shit. */
  // nearest_point_on_shape = dot(line_perp, segment_size) * line_perp;
  /* test 2 Shit. */
  // nearest_point_on_shape = length(area_size * normalized_dir) * normalized_dir;

  // nearest_point_on_shape = nearest_point_distance_ellipse_exact(lL.xy, area_size);

  /* Creates inflation because major axis is not considered properly. */
  vec2 farsest_point_on_shape = -nearest_point_on_shape;

  farsest_point_on_shape = -normalize(lL.xy / area_size) * area_size;

  float r1 = distance(sdEllipseNearestPoint(lL.xy, area_size), lL.xy);
  float r2 = distance(sdEllipseFarthestPoint(lL.xy, area_size), lL.xy);

  bool inside_ellipse = length_squared(lL.xy / area_size) < 1.0;
  if (inside_ellipse) {
    /* Signed distance. */
    r1 = -r1;
  }

  /* Circle test */
  // r1 = length(lL.xy) - area_size.x;
  // r2 = length(lL.xy) + area_size.x;

  float angle_1 = atan(r1, distance_to_plane);
  float angle_2 = atan(r2, distance_to_plane);

  /* This is assuming both points lie on the same line in the direction of the spread cone. */
  /* TODO, clamp to hemisphere */
  float half_light_angle = abs(angle_1 - angle_2) / 2.0;
  float elevation_angle = (angle_1 + angle_2) / 2.0;

  return light_cone_cone_ratio(light.spread_half_angle, half_light_angle, elevation_angle);
  /* Ellipse approximation. Stretch the configuration and resize the projected disk. This has the
   * benefit to be very simple (simplify a few instructions since the light radius is 1 afterwards)
   * and give the proper result for disk lights. */
  lL.x /= light._area_size_x;
  lL.y /= light._area_size_y;
  /* This is the root of the approximation. After stretching, the projected disk should become an
   * ellipse. But that would just shift the problem. So we approximate the new ellipse by a disk of
   * same radius. This as the effect of making the fade thinner on the thinner axis and larger on
   * the larger axis compared to what they should. */
  r /= length(vec2(light._area_size_x, light._area_size_y)) * M_SQRT1_2;

  float d = length(lL.xy);
  /* Should be light radius, but we normalized the configuration, so simplifies to 1. */
  const float R = 1.0;
  /* Special cases where the bottom would fails. */
  if (d >= r + R) {
    /* No intersection. */
    return 0.0;
  }
  if (d + r <= R) {
    /* Total intersection. */
    return 1.0;
  }
  if (d + R <= r) {
    /* Light area is completly inside the cone footprint. Compute ratio. */
    return 1.0;
  }
  /* Eq. 14. */
  float d1 = (d * d - r * r + R * R) / (2.0 * d);
  float d2 = (d * d + r * r - R * R) / (2.0 * d);
#if 0 /* Reference method. */
  float area_section = (r * r) * acos(d2 / r) + (R * R) * acos(d1 / R) -
                       0.5 * sqrt((-d + r + R) * (d + r - R) * (d - r + R) * (d + r + R));
  float area_projection = square(r) * M_PI;
  return area_section / area_projection;
#else
  return light_circular_segment_area_opti(d1, R) * square(R / r) +
         light_circular_segment_area_opti(d2, r);
#endif
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
