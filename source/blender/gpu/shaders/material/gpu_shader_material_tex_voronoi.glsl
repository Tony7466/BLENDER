#pragma BLENDER_REQUIRE(gpu_shader_common_hash.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_common_math_utils.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_material_voronoi.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_material_fractal_voronoi.glsl)

/*
 * Original code is under the MIT License, Copyright (c) 2013 Inigo Quilez.
 *
 * Smooth Voronoi:
 *
 * - https://wiki.blender.org/wiki/User:OmarSquircleArt/GSoC2019/Documentation/Smooth_Voronoi
 *
 * Distance To Edge based on:
 *
 * - https://www.iquilezles.org/www/articles/voronoilines/voronoilines.htm
 * - https://www.shadertoy.com/view/ldl3W8
 *
 * With optimization to change -2..2 scan window to -1..1 for better performance,
 * as explained in https://www.shadertoy.com/view/llG3zy.
 */

/* **** 1D Voronoi **** */

void node_tex_voronoi_f1_1d(vec3 coord,
                            float w,
                            float scale,
                            float detail,
                            float roughness,
                            float lacunarity,
                            float smoothness,
                            float exponent,
                            float randomness,
                            float metric,
                            float bool_normalize,
                            out float outDistance,
                            out vec4 outColor,
                            out vec3 outPosition,
                            out float outW,
                            out float outRadius)
{
  detail = clamp(detail, 0.0, 15.0);
  roughness = clamp(roughness, 0.0, 1.0);
  randomness = clamp(randomness, 0.0, 1.0);
  float max_amplitude = 0.0;

  float scaledCoord = w * scale;

  fractal_voronoi_f1(scaledCoord,
                     detail,
                     roughness,
                     lacunarity,
                     randomness,
                     1.0,
                     max_amplitude,
                     outDistance,
                     outColor,
                     outW);

  outW = safe_divide(outW, scale);
  if (bool_normalize != 0.0) {
    /* Optimized lerp(max_amplitude*0.5, max_amplitude, randomness)  */
    outDistance /= (0.5f + 0.5f * randomness) * max_amplitude;
  }
}

void node_tex_voronoi_smooth_f1_1d(vec3 coord,
                                   float w,
                                   float scale,
                                   float detail,
                                   float roughness,
                                   float lacunarity,
                                   float smoothness,
                                   float exponent,
                                   float randomness,
                                   float metric,
                                   float bool_normalize,
                                   out float outDistance,
                                   out vec4 outColor,
                                   out vec3 outPosition,
                                   out float outW,
                                   out float outRadius)
{
  detail = clamp(detail, 0.0, 15.0);
  roughness = clamp(roughness, 0.0, 1.0);
  randomness = clamp(randomness, 0.0, 1.0);
  smoothness = clamp(smoothness / 2.0, 0.0, 0.5);
  float max_amplitude = 0.0;

  float scaledCoord = w * scale;

  fractal_voronoi_smooth_f1(scaledCoord,
                            detail,
                            roughness,
                            lacunarity,
                            smoothness,
                            randomness,
                            1.0,
                            max_amplitude,
                            outDistance,
                            outColor,
                            outW);

  outW = safe_divide(outW, scale);
  if (bool_normalize != 0.0) {
    /* Optimized lerp(max_amplitude*0.5, max_amplitude, randomness)  */
    outDistance /= (0.5f + 0.5f * randomness) * max_amplitude;
  }
}

void node_tex_voronoi_f2_1d(vec3 coord,
                            float w,
                            float scale,
                            float detail,
                            float roughness,
                            float lacunarity,
                            float smoothness,
                            float exponent,
                            float randomness,
                            float metric,
                            float bool_normalize,
                            out float outDistance,
                            out vec4 outColor,
                            out vec3 outPosition,
                            out float outW,
                            out float outRadius)
{
  detail = clamp(detail, 0.0, 15.0);
  roughness = clamp(roughness, 0.0, 1.0);
  randomness = clamp(randomness, 0.0, 1.0);
  float max_amplitude = 0.0;

  float scaledCoord = w * scale;

  fractal_voronoi_f2(scaledCoord,
                     detail,
                     roughness,
                     lacunarity,
                     randomness,
                     1.0,
                     max_amplitude,
                     outDistance,
                     outColor,
                     outW);

  outW = safe_divide(outW, scale);
  if (bool_normalize != 0.0) {
    if (detail == 0.0f || roughness == 0.0f || lacunarity == 0.0f) {
      outDistance /= (1.0 - randomness) + randomness * max_amplitude;
    }
    else {
      outDistance /= (1.0 - randomness) * ceil(detail + 1.0) + randomness * max_amplitude;
    }
  }
}

void node_tex_voronoi_distance_to_edge_1d(vec3 coord,
                                          float w,
                                          float scale,
                                          float detail,
                                          float roughness,
                                          float lacunarity,
                                          float smoothness,
                                          float exponent,
                                          float randomness,
                                          float metric,
                                          float bool_normalize,
                                          out float outDistance,
                                          out vec4 outColor,
                                          out vec3 outPosition,
                                          out float outW,
                                          out float outRadius)
{
  detail = clamp(detail, 0.0, 15.0);
  randomness = clamp(randomness, 0.0, 1.0);

  float scaledCoord = w * scale;

  fractal_voronoi_distance_to_edge(
      scaledCoord, detail, lacunarity, randomness, bool_normalize, outDistance);
}

void node_tex_voronoi_n_sphere_radius_1d(vec3 coord,
                                         float w,
                                         float scale,
                                         float detail,
                                         float roughness,
                                         float lacunarity,
                                         float smoothness,
                                         float exponent,
                                         float randomness,
                                         float metric,
                                         float bool_normalize,
                                         out float outDistance,
                                         out vec4 outColor,
                                         out vec3 outPosition,
                                         out float outW,
                                         out float outRadius)
{
  randomness = clamp(randomness, 0.0, 1.0);

  float scaledCoord = w * scale;

  voronoi_n_sphere_radius(scaledCoord, randomness, outRadius);
}

/* **** 2D Voronoi **** */

void node_tex_voronoi_f1_2d(vec3 coord,
                            float w,
                            float scale,
                            float detail,
                            float roughness,
                            float lacunarity,
                            float smoothness,
                            float exponent,
                            float randomness,
                            float metric,
                            float bool_normalize,
                            out float outDistance,
                            out vec4 outColor,
                            out vec3 outPosition,
                            out float outW,
                            out float outRadius)
{
  detail = clamp(detail, 0.0, 15.0);
  roughness = clamp(roughness, 0.0, 1.0);
  randomness = clamp(randomness, 0.0, 1.0);
  const float max_distance = voronoi_distance(vec2(1.0, 1.0), vec2(0.0, 0.0), metric, exponent);
  float max_amplitude = max_distance;

  vec2 scaledCoord = coord.xy * scale;
  vec2 outPosition_2d;

  fractal_voronoi_f1(scaledCoord,
                     detail,
                     roughness,
                     lacunarity,
                     exponent,
                     randomness,
                     metric,
                     max_distance,
                     max_amplitude,
                     outDistance,
                     outColor,
                     outPosition_2d);

  outPosition = vec3(safe_divide(outPosition_2d, scale), 0.0);
  if (bool_normalize != 0.0) {
    /* Optimized lerp(max_amplitude*0.5, max_amplitude, randomness)  */
    outDistance /= (0.5f + 0.5f * randomness) * max_amplitude;
  }
}

void node_tex_voronoi_smooth_f1_2d(vec3 coord,
                                   float w,
                                   float scale,
                                   float detail,
                                   float roughness,
                                   float lacunarity,
                                   float smoothness,
                                   float exponent,
                                   float randomness,
                                   float metric,
                                   float bool_normalize,
                                   out float outDistance,
                                   out vec4 outColor,
                                   out vec3 outPosition,
                                   out float outW,
                                   out float outRadius)
{
  detail = clamp(detail, 0.0, 15.0);
  roughness = clamp(roughness, 0.0, 1.0);
  randomness = clamp(randomness, 0.0, 1.0);
  smoothness = clamp(smoothness / 2.0, 0.0, 0.5);
  const float max_distance = voronoi_distance(vec2(1.0, 1.0), vec2(0.0, 0.0), metric, exponent);
  float max_amplitude = max_distance;

  vec2 scaledCoord = coord.xy * scale;
  vec2 outPosition_2d;

  fractal_voronoi_smooth_f1(scaledCoord,
                            detail,
                            roughness,
                            lacunarity,
                            smoothness,
                            exponent,
                            randomness,
                            metric,
                            max_distance,
                            max_amplitude,
                            outDistance,
                            outColor,
                            outPosition_2d);

  outPosition = vec3(safe_divide(outPosition_2d, scale), 0.0);
  if (bool_normalize != 0.0) {
    /* Optimized lerp(max_amplitude*0.5, max_amplitude, randomness)  */
    outDistance /= (0.5f + 0.5f * randomness) * max_amplitude;
  }
}

void node_tex_voronoi_f2_2d(vec3 coord,
                            float w,
                            float scale,
                            float detail,
                            float roughness,
                            float lacunarity,
                            float smoothness,
                            float exponent,
                            float randomness,
                            float metric,
                            float bool_normalize,
                            out float outDistance,
                            out vec4 outColor,
                            out vec3 outPosition,
                            out float outW,
                            out float outRadius)
{
  detail = clamp(detail, 0.0, 15.0);
  roughness = clamp(roughness, 0.0, 1.0);
  randomness = clamp(randomness, 0.0, 1.0);
  const float max_distance = voronoi_distance(vec2(1.0, 1.0), vec2(0.0, 0.0), metric, exponent);
  float max_amplitude = max_distance;

  vec2 scaledCoord = coord.xy * scale;
  vec2 outPosition_2d;

  fractal_voronoi_f2(scaledCoord,
                     detail,
                     roughness,
                     lacunarity,
                     exponent,
                     randomness,
                     metric,
                     max_distance,
                     max_amplitude,
                     outDistance,
                     outColor,
                     outPosition_2d);

  outPosition = vec3(safe_divide(outPosition_2d, scale), 0.0);
  if (bool_normalize != 0.0) {
    if (detail == 0.0f || roughness == 0.0f || lacunarity == 0.0f) {
      outDistance /= (1.0 - randomness) + randomness * max_amplitude;
    }
    else {
      outDistance /= (1.0 - randomness) * ceil(detail + 1.0) + randomness * max_amplitude;
    }
  }
}

void node_tex_voronoi_distance_to_edge_2d(vec3 coord,
                                          float w,
                                          float scale,
                                          float detail,
                                          float roughness,
                                          float lacunarity,
                                          float smoothness,
                                          float exponent,
                                          float randomness,
                                          float metric,
                                          float bool_normalize,
                                          out float outDistance,
                                          out vec4 outColor,
                                          out vec3 outPosition,
                                          out float outW,
                                          out float outRadius)
{
  detail = clamp(detail, 0.0, 15.0);
  randomness = clamp(randomness, 0.0, 1.0);

  vec2 scaledCoord = coord.xy * scale;

  fractal_voronoi_distance_to_edge(
      scaledCoord, detail, lacunarity, randomness, bool_normalize, outDistance);
}

void node_tex_voronoi_n_sphere_radius_2d(vec3 coord,
                                         float w,
                                         float scale,
                                         float detail,
                                         float roughness,
                                         float lacunarity,
                                         float smoothness,
                                         float exponent,
                                         float randomness,
                                         float metric,
                                         float bool_normalize,
                                         out float outDistance,
                                         out vec4 outColor,
                                         out vec3 outPosition,
                                         out float outW,
                                         out float outRadius)
{
  randomness = clamp(randomness, 0.0, 1.0);

  vec2 scaledCoord = coord.xy * scale;

  voronoi_n_sphere_radius(scaledCoord, randomness, outRadius);
}

/* **** 3D Voronoi **** */

void node_tex_voronoi_f1_3d(vec3 coord,
                            float w,
                            float scale,
                            float detail,
                            float roughness,
                            float lacunarity,
                            float smoothness,
                            float exponent,
                            float randomness,
                            float metric,
                            float bool_normalize,
                            out float outDistance,
                            out vec4 outColor,
                            out vec3 outPosition,
                            out float outW,
                            out float outRadius)
{
  detail = clamp(detail, 0.0, 15.0);
  roughness = clamp(roughness, 0.0, 1.0);
  randomness = clamp(randomness, 0.0, 1.0);
  const float max_distance = voronoi_distance(
      vec3(1.0, 1.0, 1.0), vec3(0.0, 0.0, 0.0), metric, exponent);
  float max_amplitude = max_distance;

  vec3 scaledCoord = coord * scale;

  fractal_voronoi_f1(scaledCoord,
                     detail,
                     roughness,
                     lacunarity,
                     exponent,
                     randomness,
                     metric,
                     max_distance,
                     max_amplitude,
                     outDistance,
                     outColor,
                     outPosition);

  outPosition = safe_divide(outPosition, scale);
  if (bool_normalize != 0.0) {
    /* Optimized lerp(max_amplitude*0.5, max_amplitude, randomness)  */
    outDistance /= (0.5f + 0.5f * randomness) * max_amplitude;
  }
}

void node_tex_voronoi_smooth_f1_3d(vec3 coord,
                                   float w,
                                   float scale,
                                   float detail,
                                   float roughness,
                                   float lacunarity,
                                   float smoothness,
                                   float exponent,
                                   float randomness,
                                   float metric,
                                   float bool_normalize,
                                   out float outDistance,
                                   out vec4 outColor,
                                   out vec3 outPosition,
                                   out float outW,
                                   out float outRadius)
{
  detail = clamp(detail, 0.0, 15.0);
  roughness = clamp(roughness, 0.0, 1.0);
  randomness = clamp(randomness, 0.0, 1.0);
  smoothness = clamp(smoothness / 2.0, 0.0, 0.5);
  const float max_distance = voronoi_distance(
      vec3(1.0, 1.0, 1.0), vec3(0.0, 0.0, 0.0), metric, exponent);
  float max_amplitude = max_distance;

  vec3 scaledCoord = coord * scale;

  fractal_voronoi_smooth_f1(scaledCoord,
                            detail,
                            roughness,
                            lacunarity,
                            smoothness,
                            exponent,
                            randomness,
                            metric,
                            max_distance,
                            max_amplitude,
                            outDistance,
                            outColor,
                            outPosition);

  outPosition = safe_divide(outPosition, scale);
  if (bool_normalize != 0.0) {
    /* Optimized lerp(max_amplitude*0.5, max_amplitude, randomness)  */
    outDistance /= (0.5f + 0.5f * randomness) * max_amplitude;
  }
}

void node_tex_voronoi_f2_3d(vec3 coord,
                            float w,
                            float scale,
                            float detail,
                            float roughness,
                            float lacunarity,
                            float smoothness,
                            float exponent,
                            float randomness,
                            float metric,
                            float bool_normalize,
                            out float outDistance,
                            out vec4 outColor,
                            out vec3 outPosition,
                            out float outW,
                            out float outRadius)
{
  detail = clamp(detail, 0.0, 15.0);
  roughness = clamp(roughness, 0.0, 1.0);
  randomness = clamp(randomness, 0.0, 1.0);
  const float max_distance = voronoi_distance(
      vec3(1.0, 1.0, 1.0), vec3(0.0, 0.0, 0.0), metric, exponent);
  float max_amplitude = max_distance;

  vec3 scaledCoord = coord * scale;

  fractal_voronoi_f2(scaledCoord,
                     detail,
                     roughness,
                     lacunarity,
                     exponent,
                     randomness,
                     metric,
                     max_distance,
                     max_amplitude,
                     outDistance,
                     outColor,
                     outPosition);

  outPosition = safe_divide(outPosition, scale);
  if (bool_normalize != 0.0) {
    if (detail == 0.0f || roughness == 0.0f || lacunarity == 0.0f) {
      outDistance /= (1.0 - randomness) + randomness * max_amplitude;
    }
    else {
      outDistance /= (1.0 - randomness) * ceil(detail + 1.0) + randomness * max_amplitude;
    }
  }
}

void node_tex_voronoi_distance_to_edge_3d(vec3 coord,
                                          float w,
                                          float scale,
                                          float detail,
                                          float roughness,
                                          float lacunarity,
                                          float smoothness,
                                          float exponent,
                                          float randomness,
                                          float metric,
                                          float bool_normalize,
                                          out float outDistance,
                                          out vec4 outColor,
                                          out vec3 outPosition,
                                          out float outW,
                                          out float outRadius)
{
  detail = clamp(detail, 0.0, 15.0);
  randomness = clamp(randomness, 0.0, 1.0);

  vec3 scaledCoord = coord * scale;

  fractal_voronoi_distance_to_edge(
      scaledCoord, detail, lacunarity, randomness, bool_normalize, outDistance);
}

void node_tex_voronoi_n_sphere_radius_3d(vec3 coord,
                                         float w,
                                         float scale,
                                         float detail,
                                         float roughness,
                                         float lacunarity,
                                         float smoothness,
                                         float exponent,
                                         float randomness,
                                         float metric,
                                         float bool_normalize,
                                         out float outDistance,
                                         out vec4 outColor,
                                         out vec3 outPosition,
                                         out float outW,
                                         out float outRadius)
{
  randomness = clamp(randomness, 0.0, 1.0);

  vec3 scaledCoord = coord * scale;

  voronoi_n_sphere_radius(scaledCoord, randomness, outRadius);
}

/* **** 4D Voronoi **** */

void node_tex_voronoi_f1_4d(vec3 coord,
                            float w,
                            float scale,
                            float detail,
                            float roughness,
                            float lacunarity,
                            float smoothness,
                            float exponent,
                            float randomness,
                            float metric,
                            float bool_normalize,
                            out float outDistance,
                            out vec4 outColor,
                            out vec3 outPosition,
                            out float outW,
                            out float outRadius)
{
  detail = clamp(detail, 0.0, 15.0);
  roughness = clamp(roughness, 0.0, 1.0);
  randomness = clamp(randomness, 0.0, 1.0);
  const float max_distance = voronoi_distance(
      vec4(1.0, 1.0, 1.0, 1.0), vec4(0.0, 0.0, 0.0, 0.0), metric, exponent);
  float max_amplitude = max_distance;

  vec4 scaledCoord = vec4(coord, w) * scale;
  vec4 outPosition_4d;

  fractal_voronoi_f1(scaledCoord,
                     detail,
                     roughness,
                     lacunarity,
                     exponent,
                     randomness,
                     metric,
                     max_distance,
                     max_amplitude,
                     outDistance,
                     outColor,
                     outPosition_4d);

  outPosition_4d = safe_divide(outPosition_4d, scale);
  outPosition = outPosition_4d.xyz;
  outW = outPosition_4d.w;
  if (bool_normalize != 0.0) {
    /* Optimized lerp(max_amplitude*0.5, max_amplitude, randomness)  */
    outDistance /= (0.5f + 0.5f * randomness) * max_amplitude;
  }
}

void node_tex_voronoi_smooth_f1_4d(vec3 coord,
                                   float w,
                                   float scale,
                                   float detail,
                                   float roughness,
                                   float lacunarity,
                                   float smoothness,
                                   float exponent,
                                   float randomness,
                                   float metric,
                                   float bool_normalize,
                                   out float outDistance,
                                   out vec4 outColor,
                                   out vec3 outPosition,
                                   out float outW,
                                   out float outRadius)
{
  detail = clamp(detail, 0.0, 15.0);
  roughness = clamp(roughness, 0.0, 1.0);
  randomness = clamp(randomness, 0.0, 1.0);
  smoothness = clamp(smoothness / 2.0, 0.0, 0.5);
  const float max_distance = voronoi_distance(
      vec4(1.0, 1.0, 1.0, 1.0), vec4(0.0, 0.0, 0.0, 0.0), metric, exponent);
  float max_amplitude = max_distance;

  vec4 scaledCoord = vec4(coord, w) * scale;
  vec4 outPosition_4d;

  fractal_voronoi_smooth_f1(scaledCoord,
                            detail,
                            roughness,
                            lacunarity,
                            smoothness,
                            exponent,
                            randomness,
                            metric,
                            max_distance,
                            max_amplitude,
                            outDistance,
                            outColor,
                            outPosition_4d);

  outPosition_4d = safe_divide(outPosition_4d, scale);
  outPosition = outPosition_4d.xyz;
  outW = outPosition_4d.w;
  if (bool_normalize != 0.0) {
    /* Optimized lerp(max_amplitude*0.5, max_amplitude, randomness)  */
    outDistance /= (0.5f + 0.5f * randomness) * max_amplitude;
  }
}

void node_tex_voronoi_f2_4d(vec3 coord,
                            float w,
                            float scale,
                            float detail,
                            float roughness,
                            float lacunarity,
                            float smoothness,
                            float exponent,
                            float randomness,
                            float metric,
                            float bool_normalize,
                            out float outDistance,
                            out vec4 outColor,
                            out vec3 outPosition,
                            out float outW,
                            out float outRadius)
{
  detail = clamp(detail, 0.0, 15.0);
  roughness = clamp(roughness, 0.0, 1.0);
  randomness = clamp(randomness, 0.0, 1.0);
  const float max_distance = voronoi_distance(
      vec4(1.0, 1.0, 1.0, 1.0), vec4(0.0, 0.0, 0.0, 0.0), metric, exponent);
  float max_amplitude = max_distance;

  vec4 scaledCoord = vec4(coord, w) * scale;
  vec4 outPosition_4d;

  fractal_voronoi_f2(scaledCoord,
                     detail,
                     roughness,
                     lacunarity,
                     exponent,
                     randomness,
                     metric,
                     max_distance,
                     max_amplitude,
                     outDistance,
                     outColor,
                     outPosition_4d);

  outPosition_4d = safe_divide(outPosition_4d, scale);
  outPosition = outPosition_4d.xyz;
  outW = outPosition_4d.w;
  if (bool_normalize != 0.0) {
    if (detail == 0.0f || roughness == 0.0f || lacunarity == 0.0f) {
      outDistance /= (1.0 - randomness) + randomness * max_amplitude;
    }
    else {
      outDistance /= (1.0 - randomness) * ceil(detail + 1.0) + randomness * max_amplitude;
    }
  }
}

void node_tex_voronoi_distance_to_edge_4d(vec3 coord,
                                          float w,
                                          float scale,
                                          float detail,
                                          float roughness,
                                          float lacunarity,
                                          float smoothness,
                                          float exponent,
                                          float randomness,
                                          float metric,
                                          float bool_normalize,
                                          out float outDistance,
                                          out vec4 outColor,
                                          out vec3 outPosition,
                                          out float outW,
                                          out float outRadius)
{
  detail = clamp(detail, 0.0, 15.0);
  randomness = clamp(randomness, 0.0, 1.0);

  vec4 scaledCoord = vec4(coord, w) * scale;

  fractal_voronoi_distance_to_edge(
      scaledCoord, detail, lacunarity, randomness, bool_normalize, outDistance);
}

void node_tex_voronoi_n_sphere_radius_4d(vec3 coord,
                                         float w,
                                         float scale,
                                         float detail,
                                         float roughness,
                                         float lacunarity,
                                         float smoothness,
                                         float exponent,
                                         float randomness,
                                         float metric,
                                         float bool_normalize,
                                         out float outDistance,
                                         out vec4 outColor,
                                         out vec3 outPosition,
                                         out float outW,
                                         out float outRadius)
{
  randomness = clamp(randomness, 0.0, 1.0);

  vec4 scaledCoord = vec4(coord, w) * scale;

  voronoi_n_sphere_radius(scaledCoord, randomness, outRadius);
}
