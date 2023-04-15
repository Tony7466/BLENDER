/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

CCL_NAMESPACE_BEGIN

/*
 * SPDX-License-Identifier: MIT
 * Original code is copyright (c) 2013 Inigo Quilez.
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

struct VoronoiParamsBase {
  float scale;
  float detail;
  float roughness;
  float lacunarity;
  float smoothness;
  float exponent;
  float randomness;
  float max_amplitude;
  float max_distance;
  NodeVoronoiDistanceMetric metric;
  float octave_scale;
  float octave_amplitude;
  float octave_distance;
  float3 octave_color;

  VoronoiParamsBase() = default;

  VoronoiParamsBase(const VoronoiParamsBase &vpb)
      : scale(vpb.scale),
        detail(vpb.detail),
        roughness(vpb.roughness),
        lacunarity(vpb.lacunarity),
        smoothness(vpb.smoothness),
        exponent(vpb.exponent),
        randomness(vpb.randomness),
        max_amplitude(vpb.max_amplitude),
        max_distance(vpb.max_distance),
        metric(vpb.metric),
        octave_scale(vpb.octave_scale),
        octave_amplitude(vpb.octave_amplitude),
        octave_distance(vpb.octave_distance),
        octave_color(vpb.octave_color)
  {
  }
};

template<typename T> struct VoronoiParams : public VoronoiParamsBase {
  T octave_coord;
  T octave_postion;

  VoronoiParams(const VoronoiParamsBase &vpb) : VoronoiParamsBase(vpb) {}
};

struct VoronoiOutputBase {
  float distance_out;
  float radius_out;
  float3 color_out;

  VoronoiOutputBase() = default;

  VoronoiOutputBase(const VoronoiOutputBase &vob)
      : distance_out(vob.distance_out), radius_out(vob.radius_out), color_out(vob.color_out)
  {
  }
};

template<typename T> struct VoronoiOutput : public VoronoiOutputBase {
  T position_out;

  VoronoiOutput(const VoronoiOutputBase &vob) : VoronoiOutputBase(vob) {}
};

/* **** 1D Voronoi **** */

/* The "float exponent" and "NodeVoronoiDistanceMetric metric" function parameters are unused in
 * the 1D Voronoi calculations but needed for overload resolution to work */
ccl_device float voronoi_distance_1d(float a,
                                     float b,
                                     NodeVoronoiDistanceMetric metric,
                                     float exponent)
{
  /* Supress compiler warnings */
  (void)exponent;
  (void)metric;

  return fabsf(b - a);
}

/* The "float exponent" and "NodeVoronoiDistanceMetric metric" function parameters are unused in
 * the 1D Voronoi calculations but needed for overload resolution to work */
ccl_device void voronoi_f1(float w,
                           float exponent,
                           float randomness,
                           NodeVoronoiDistanceMetric metric,
                           ccl_private float *distance_out,
                           ccl_private float3 *color_out,
                           ccl_private float *outW)
{
  /* Supress compiler warnings */
  (void)exponent;
  (void)metric;

  float cellPosition = floorf(w);
  float localPosition = w - cellPosition;

  float minDistance = 8.0f;
  float targetOffset = 0.0f;
  float targetPosition = 0.0f;
  for (int i = -1; i <= 1; i++) {
    float cellOffset = i;
    float pointPosition = cellOffset + hash_float_to_float(cellPosition + cellOffset) * randomness;
    float distanceToPoint = voronoi_distance_1d(pointPosition, localPosition, metric, exponent);
    if (distanceToPoint < minDistance) {
      targetOffset = cellOffset;
      minDistance = distanceToPoint;
      targetPosition = pointPosition;
    }
  }
  *distance_out = minDistance;
  *color_out = hash_float_to_float3(cellPosition + targetOffset);
  *outW = targetPosition + cellPosition;
}

/* The "float exponent" and "NodeVoronoiDistanceMetric metric" function parameters are unused in
 * the 1D Voronoi calculations but needed for overload resolution to work */
ccl_device void voronoi_smooth_f1(float w,
                                  float smoothness,
                                  float exponent,
                                  float randomness,
                                  NodeVoronoiDistanceMetric metric,
                                  ccl_private float *distance_out,
                                  ccl_private float3 *color_out,
                                  ccl_private float *outW)
{
  /* Supress compiler warnings */
  (void)exponent;
  (void)metric;

  float cellPosition = floorf(w);
  float localPosition = w - cellPosition;

  float smoothDistance = 8.0f;
  float smoothPosition = 0.0f;
  float3 smoothColor = make_float3(0.0f, 0.0f, 0.0f);
  for (int i = -2; i <= 2; i++) {
    float cellOffset = i;
    float pointPosition = cellOffset + hash_float_to_float(cellPosition + cellOffset) * randomness;
    float distanceToPoint = voronoi_distance_1d(pointPosition, localPosition, metric, exponent);
    float h = smoothstep(
        0.0f, 1.0f, 0.5f + 0.5f * (smoothDistance - distanceToPoint) / smoothness);
    float correctionFactor = smoothness * h * (1.0f - h);
    smoothDistance = mix(smoothDistance, distanceToPoint, h) - correctionFactor;
    correctionFactor /= 1.0f + 3.0f * smoothness;
    float3 cellColor = hash_float_to_float3(cellPosition + cellOffset);
    smoothColor = mix(smoothColor, cellColor, h) - correctionFactor;
    smoothPosition = mix(smoothPosition, pointPosition, h) - correctionFactor;
  }
  *distance_out = smoothDistance;
  *color_out = smoothColor;
  *outW = cellPosition + smoothPosition;
}

/* The "float exponent" and "NodeVoronoiDistanceMetric metric" function parameters are unused in
 * the 1D Voronoi calculations but needed for overload resolution to work */
ccl_device void voronoi_f2(float w,
                           float exponent,
                           float randomness,
                           NodeVoronoiDistanceMetric metric,
                           ccl_private float *distance_out,
                           ccl_private float3 *color_out,
                           ccl_private float *outW)
{
  /* Supress compiler warnings */
  (void)exponent;
  (void)metric;

  float cellPosition = floorf(w);
  float localPosition = w - cellPosition;

  float distanceF1 = 8.0f;
  float distanceF2 = 8.0f;
  float offsetF1 = 0.0f;
  float positionF1 = 0.0f;
  float offsetF2 = 0.0f;
  float positionF2 = 0.0f;
  for (int i = -1; i <= 1; i++) {
    float cellOffset = i;
    float pointPosition = cellOffset + hash_float_to_float(cellPosition + cellOffset) * randomness;
    float distanceToPoint = voronoi_distance_1d(pointPosition, localPosition, metric, exponent);
    if (distanceToPoint < distanceF1) {
      distanceF2 = distanceF1;
      distanceF1 = distanceToPoint;
      offsetF2 = offsetF1;
      offsetF1 = cellOffset;
      positionF2 = positionF1;
      positionF1 = pointPosition;
    }
    else if (distanceToPoint < distanceF2) {
      distanceF2 = distanceToPoint;
      offsetF2 = cellOffset;
      positionF2 = pointPosition;
    }
  }
  *distance_out = distanceF2;
  *color_out = hash_float_to_float3(cellPosition + offsetF2);
  *outW = positionF2 + cellPosition;
}

ccl_device void voronoi_distance_to_edge(float w, float randomness, ccl_private float *distance_out)
{
  float cellPosition = floorf(w);
  float localPosition = w - cellPosition;

  float midPointPosition = hash_float_to_float(cellPosition) * randomness;
  float leftPointPosition = -1.0f + hash_float_to_float(cellPosition - 1.0f) * randomness;
  float rightPointPosition = 1.0f + hash_float_to_float(cellPosition + 1.0f) * randomness;
  float distanceToMidLeft = fabsf((midPointPosition + leftPointPosition) / 2.0f - localPosition);
  float distanceToMidRight = fabsf((midPointPosition + rightPointPosition) / 2.0f - localPosition);

  *distance_out = min(distanceToMidLeft, distanceToMidRight);
}

ccl_device void voronoi_n_sphere_radius_1d(float w, float randomness, ccl_private float *radius_out)
{
  float cellPosition = floorf(w);
  float localPosition = w - cellPosition;

  float closestPoint = 0.0f;
  float closestPointOffset = 0.0f;
  float minDistance = 8.0f;
  for (int i = -1; i <= 1; i++) {
    float cellOffset = i;
    float pointPosition = cellOffset + hash_float_to_float(cellPosition + cellOffset) * randomness;
    float distanceToPoint = fabsf(pointPosition - localPosition);
    if (distanceToPoint < minDistance) {
      minDistance = distanceToPoint;
      closestPoint = pointPosition;
      closestPointOffset = cellOffset;
    }
  }

  minDistance = 8.0f;
  float closestPointToClosestPoint = 0.0f;
  for (int i = -1; i <= 1; i++) {
    if (i == 0) {
      continue;
    }
    float cellOffset = i + closestPointOffset;
    float pointPosition = cellOffset + hash_float_to_float(cellPosition + cellOffset) * randomness;
    float distanceToPoint = fabsf(closestPoint - pointPosition);
    if (distanceToPoint < minDistance) {
      minDistance = distanceToPoint;
      closestPointToClosestPoint = pointPosition;
    }
  }
  *radius_out = fabsf(closestPointToClosestPoint - closestPoint) / 2.0f;
}

/* **** 2D Voronoi **** */

ccl_device float voronoi_distance_2d(float2 a,
                                     float2 b,
                                     NodeVoronoiDistanceMetric metric,
                                     float exponent)
{
  if (metric == NODE_VORONOI_EUCLIDEAN) {
    return distance(a, b);
  }
  else if (metric == NODE_VORONOI_MANHATTAN) {
    return fabsf(a.x - b.x) + fabsf(a.y - b.y);
  }
  else if (metric == NODE_VORONOI_CHEBYCHEV) {
    return max(fabsf(a.x - b.x), fabsf(a.y - b.y));
  }
  else if (metric == NODE_VORONOI_MINKOWSKI) {
    return powf(powf(fabsf(a.x - b.x), exponent) + powf(fabsf(a.y - b.y), exponent),
                1.0f / exponent);
  }
  else {
    return 0.0f;
  }
}

ccl_device void voronoi_f1(float2 coord,
                           float exponent,
                           float randomness,
                           NodeVoronoiDistanceMetric metric,
                           ccl_private float *distance_out,
                           ccl_private float3 *color_out,
                           ccl_private float2 *position_out)
{
  float2 cellPosition = floor(coord);
  float2 localPosition = coord - cellPosition;

  float minDistance = 8.0f;
  float2 targetOffset = make_float2(0.0f, 0.0f);
  float2 targetPosition = make_float2(0.0f, 0.0f);
  for (int j = -1; j <= 1; j++) {
    for (int i = -1; i <= 1; i++) {
      float2 cellOffset = make_float2(i, j);
      float2 pointPosition = cellOffset +
                             hash_float2_to_float2(cellPosition + cellOffset) * randomness;
      float distanceToPoint = voronoi_distance_2d(pointPosition, localPosition, metric, exponent);
      if (distanceToPoint < minDistance) {
        targetOffset = cellOffset;
        minDistance = distanceToPoint;
        targetPosition = pointPosition;
      }
    }
  }
  *distance_out = minDistance;
  *color_out = hash_float2_to_float3(cellPosition + targetOffset);
  *position_out = targetPosition + cellPosition;
}

ccl_device void voronoi_smooth_f1(float2 coord,
                                  float smoothness,
                                  float exponent,
                                  float randomness,
                                  NodeVoronoiDistanceMetric metric,
                                  ccl_private float *distance_out,
                                  ccl_private float3 *color_out,
                                  ccl_private float2 *position_out)
{
  float2 cellPosition = floor(coord);
  float2 localPosition = coord - cellPosition;

  float smoothDistance = 8.0f;
  float3 smoothColor = make_float3(0.0f, 0.0f, 0.0f);
  float2 smoothPosition = make_float2(0.0f, 0.0f);
  for (int j = -2; j <= 2; j++) {
    for (int i = -2; i <= 2; i++) {
      float2 cellOffset = make_float2(i, j);
      float2 pointPosition = cellOffset +
                             hash_float2_to_float2(cellPosition + cellOffset) * randomness;
      float distanceToPoint = voronoi_distance_2d(pointPosition, localPosition, metric, exponent);
      float h = smoothstep(
          0.0f, 1.0f, 0.5f + 0.5f * (smoothDistance - distanceToPoint) / smoothness);
      float correctionFactor = smoothness * h * (1.0f - h);
      smoothDistance = mix(smoothDistance, distanceToPoint, h) - correctionFactor;
      correctionFactor /= 1.0f + 3.0f * smoothness;
      float3 cellColor = hash_float2_to_float3(cellPosition + cellOffset);
      smoothColor = mix(smoothColor, cellColor, h) - correctionFactor;
      smoothPosition = mix(smoothPosition, pointPosition, h) - correctionFactor;
    }
  }
  *distance_out = smoothDistance;
  *color_out = smoothColor;
  *position_out = cellPosition + smoothPosition;
}

ccl_device void voronoi_f2(float2 coord,
                           float exponent,
                           float randomness,
                           NodeVoronoiDistanceMetric metric,
                           ccl_private float *distance_out,
                           ccl_private float3 *color_out,
                           ccl_private float2 *position_out)
{
  float2 cellPosition = floor(coord);
  float2 localPosition = coord - cellPosition;

  float distanceF1 = 8.0f;
  float distanceF2 = 8.0f;
  float2 offsetF1 = make_float2(0.0f, 0.0f);
  float2 positionF1 = make_float2(0.0f, 0.0f);
  float2 offsetF2 = make_float2(0.0f, 0.0f);
  float2 positionF2 = make_float2(0.0f, 0.0f);
  for (int j = -1; j <= 1; j++) {
    for (int i = -1; i <= 1; i++) {
      float2 cellOffset = make_float2(i, j);
      float2 pointPosition = cellOffset +
                             hash_float2_to_float2(cellPosition + cellOffset) * randomness;
      float distanceToPoint = voronoi_distance_2d(pointPosition, localPosition, metric, exponent);
      if (distanceToPoint < distanceF1) {
        distanceF2 = distanceF1;
        distanceF1 = distanceToPoint;
        offsetF2 = offsetF1;
        offsetF1 = cellOffset;
        positionF2 = positionF1;
        positionF1 = pointPosition;
      }
      else if (distanceToPoint < distanceF2) {
        distanceF2 = distanceToPoint;
        offsetF2 = cellOffset;
        positionF2 = pointPosition;
      }
    }
  }
  *distance_out = distanceF2;
  *color_out = hash_float2_to_float3(cellPosition + offsetF2);
  *position_out = positionF2 + cellPosition;
}

ccl_device void voronoi_distance_to_edge(float2 coord,
                                         float randomness,
                                         ccl_private float *distance_out)
{
  float2 cellPosition = floor(coord);
  float2 localPosition = coord - cellPosition;

  float2 vectorToClosest = make_float2(0.0f, 0.0f);
  float minDistance = 8.0f;
  for (int j = -1; j <= 1; j++) {
    for (int i = -1; i <= 1; i++) {
      float2 cellOffset = make_float2(i, j);
      float2 vectorToPoint = cellOffset +
                             hash_float2_to_float2(cellPosition + cellOffset) * randomness -
                             localPosition;
      float distanceToPoint = dot(vectorToPoint, vectorToPoint);
      if (distanceToPoint < minDistance) {
        minDistance = distanceToPoint;
        vectorToClosest = vectorToPoint;
      }
    }
  }

  minDistance = 8.0f;
  for (int j = -1; j <= 1; j++) {
    for (int i = -1; i <= 1; i++) {
      float2 cellOffset = make_float2(i, j);
      float2 vectorToPoint = cellOffset +
                             hash_float2_to_float2(cellPosition + cellOffset) * randomness -
                             localPosition;
      float2 perpendicularToEdge = vectorToPoint - vectorToClosest;
      if (dot(perpendicularToEdge, perpendicularToEdge) > 0.0001f) {
        float distanceToEdge = dot((vectorToClosest + vectorToPoint) / 2.0f,
                                   normalize(perpendicularToEdge));
        minDistance = min(minDistance, distanceToEdge);
      }
    }
  }
  *distance_out = minDistance;
}

ccl_device void voronoi_n_sphere_radius_2d(float2 coord,
                                           float randomness,
                                           ccl_private float *radius_out)
{
  float2 cellPosition = floor(coord);
  float2 localPosition = coord - cellPosition;

  float2 closestPoint = make_float2(0.0f, 0.0f);
  float2 closestPointOffset = make_float2(0.0f, 0.0f);
  float minDistance = 8.0f;
  for (int j = -1; j <= 1; j++) {
    for (int i = -1; i <= 1; i++) {
      float2 cellOffset = make_float2(i, j);
      float2 pointPosition = cellOffset +
                             hash_float2_to_float2(cellPosition + cellOffset) * randomness;
      float distanceToPoint = distance(pointPosition, localPosition);
      if (distanceToPoint < minDistance) {
        minDistance = distanceToPoint;
        closestPoint = pointPosition;
        closestPointOffset = cellOffset;
      }
    }
  }

  minDistance = 8.0f;
  float2 closestPointToClosestPoint = make_float2(0.0f, 0.0f);
  for (int j = -1; j <= 1; j++) {
    for (int i = -1; i <= 1; i++) {
      if (i == 0 && j == 0) {
        continue;
      }
      float2 cellOffset = make_float2(i, j) + closestPointOffset;
      float2 pointPosition = cellOffset +
                             hash_float2_to_float2(cellPosition + cellOffset) * randomness;
      float distanceToPoint = distance(closestPoint, pointPosition);
      if (distanceToPoint < minDistance) {
        minDistance = distanceToPoint;
        closestPointToClosestPoint = pointPosition;
      }
    }
  }
  *radius_out = distance(closestPointToClosestPoint, closestPoint) / 2.0f;
}

/* **** 3D Voronoi **** */

ccl_device float voronoi_distance_3d(float3 a,
                                     float3 b,
                                     NodeVoronoiDistanceMetric metric,
                                     float exponent)
{
  if (metric == NODE_VORONOI_EUCLIDEAN) {
    return distance(a, b);
  }
  else if (metric == NODE_VORONOI_MANHATTAN) {
    return fabsf(a.x - b.x) + fabsf(a.y - b.y) + fabsf(a.z - b.z);
  }
  else if (metric == NODE_VORONOI_CHEBYCHEV) {
    return max(fabsf(a.x - b.x), max(fabsf(a.y - b.y), fabsf(a.z - b.z)));
  }
  else if (metric == NODE_VORONOI_MINKOWSKI) {
    return powf(powf(fabsf(a.x - b.x), exponent) + powf(fabsf(a.y - b.y), exponent) +
                    powf(fabsf(a.z - b.z), exponent),
                1.0f / exponent);
  }
  else {
    return 0.0f;
  }
}

ccl_device void voronoi_f1_3d(VoronoiParams<float3> &vp)
{
  float3 cellPosition = floor(vp.octave_coord);
  float3 localPosition = vp.octave_coord - cellPosition;

  float minDistance = 8.0f;
  float3 targetOffset = make_float3(0.0f, 0.0f, 0.0f);
  float3 targetPosition = make_float3(0.0f, 0.0f, 0.0f);
  for (int k = -1; k <= 1; k++) {
    for (int j = -1; j <= 1; j++) {
      for (int i = -1; i <= 1; i++) {
        float3 cellOffset = make_float3(i, j, k);
        float3 pointPosition = cellOffset +
                               hash_float3_to_float3(cellPosition + cellOffset) * vp.randomness;
        float distanceToPoint = voronoi_distance_3d(
            pointPosition, localPosition, vp.metric, vp.exponent);
        if (distanceToPoint < minDistance) {
          targetOffset = cellOffset;
          minDistance = distanceToPoint;
          targetPosition = pointPosition;
        }
      }
    }
  }
  vp.octave_distance = minDistance;
  vp.octave_color = hash_float3_to_float3(cellPosition + targetOffset);
  vp.octave_postion = targetPosition + cellPosition;
}

ccl_device void voronoi_smooth_f1(float3 coord,
                                  float smoothness,
                                  float exponent,
                                  float randomness,
                                  NodeVoronoiDistanceMetric metric,
                                  ccl_private float *distance_out,
                                  ccl_private float3 *color_out,
                                  ccl_private float3 *position_out)
{
  float3 cellPosition = floor(coord);
  float3 localPosition = coord - cellPosition;

  float smoothDistance = 8.0f;
  float3 smoothColor = make_float3(0.0f, 0.0f, 0.0f);
  float3 smoothPosition = make_float3(0.0f, 0.0f, 0.0f);
  for (int k = -2; k <= 2; k++) {
    for (int j = -2; j <= 2; j++) {
      for (int i = -2; i <= 2; i++) {
        float3 cellOffset = make_float3(i, j, k);
        float3 pointPosition = cellOffset +
                               hash_float3_to_float3(cellPosition + cellOffset) * randomness;
        float distanceToPoint = voronoi_distance_3d(
            pointPosition, localPosition, metric, exponent);
        float h = smoothstep(
            0.0f, 1.0f, 0.5f + 0.5f * (smoothDistance - distanceToPoint) / smoothness);
        float correctionFactor = smoothness * h * (1.0f - h);
        smoothDistance = mix(smoothDistance, distanceToPoint, h) - correctionFactor;
        correctionFactor /= 1.0f + 3.0f * smoothness;
        float3 cellColor = hash_float3_to_float3(cellPosition + cellOffset);
        smoothColor = mix(smoothColor, cellColor, h) - correctionFactor;
        smoothPosition = mix(smoothPosition, pointPosition, h) - correctionFactor;
      }
    }
  }
  *distance_out = smoothDistance;
  *color_out = smoothColor;
  *position_out = cellPosition + smoothPosition;
}

ccl_device void voronoi_f2(float3 coord,
                           float exponent,
                           float randomness,
                           NodeVoronoiDistanceMetric metric,
                           ccl_private float *distance_out,
                           ccl_private float3 *color_out,
                           ccl_private float3 *position_out)
{
  float3 cellPosition = floor(coord);
  float3 localPosition = coord - cellPosition;

  float distanceF1 = 8.0f;
  float distanceF2 = 8.0f;
  float3 offsetF1 = make_float3(0.0f, 0.0f, 0.0f);
  float3 positionF1 = make_float3(0.0f, 0.0f, 0.0f);
  float3 offsetF2 = make_float3(0.0f, 0.0f, 0.0f);
  float3 positionF2 = make_float3(0.0f, 0.0f, 0.0f);
  for (int k = -1; k <= 1; k++) {
    for (int j = -1; j <= 1; j++) {
      for (int i = -1; i <= 1; i++) {
        float3 cellOffset = make_float3(i, j, k);
        float3 pointPosition = cellOffset +
                               hash_float3_to_float3(cellPosition + cellOffset) * randomness;
        float distanceToPoint = voronoi_distance_3d(
            pointPosition, localPosition, metric, exponent);
        if (distanceToPoint < distanceF1) {
          distanceF2 = distanceF1;
          distanceF1 = distanceToPoint;
          offsetF2 = offsetF1;
          offsetF1 = cellOffset;
          positionF2 = positionF1;
          positionF1 = pointPosition;
        }
        else if (distanceToPoint < distanceF2) {
          distanceF2 = distanceToPoint;
          offsetF2 = cellOffset;
          positionF2 = pointPosition;
        }
      }
    }
  }
  *distance_out = distanceF2;
  *color_out = hash_float3_to_float3(cellPosition + offsetF2);
  *position_out = positionF2 + cellPosition;
}

ccl_device void voronoi_distance_to_edge(float3 coord,
                                         float randomness,
                                         ccl_private float *distance_out)
{
  float3 cellPosition = floor(coord);
  float3 localPosition = coord - cellPosition;

  float3 vectorToClosest = make_float3(0.0f, 0.0f, 0.0f);
  float minDistance = 8.0f;
  for (int k = -1; k <= 1; k++) {
    for (int j = -1; j <= 1; j++) {
      for (int i = -1; i <= 1; i++) {
        float3 cellOffset = make_float3(i, j, k);
        float3 vectorToPoint = cellOffset +
                               hash_float3_to_float3(cellPosition + cellOffset) * randomness -
                               localPosition;
        float distanceToPoint = dot(vectorToPoint, vectorToPoint);
        if (distanceToPoint < minDistance) {
          minDistance = distanceToPoint;
          vectorToClosest = vectorToPoint;
        }
      }
    }
  }

  minDistance = 8.0f;
  for (int k = -1; k <= 1; k++) {
    for (int j = -1; j <= 1; j++) {
      for (int i = -1; i <= 1; i++) {
        float3 cellOffset = make_float3(i, j, k);
        float3 vectorToPoint = cellOffset +
                               hash_float3_to_float3(cellPosition + cellOffset) * randomness -
                               localPosition;
        float3 perpendicularToEdge = vectorToPoint - vectorToClosest;
        if (dot(perpendicularToEdge, perpendicularToEdge) > 0.0001f) {
          float distanceToEdge = dot((vectorToClosest + vectorToPoint) / 2.0f,
                                     normalize(perpendicularToEdge));
          minDistance = min(minDistance, distanceToEdge);
        }
      }
    }
  }
  *distance_out = minDistance;
}

ccl_device void voronoi_n_sphere_radius_3d(float3 coord,
                                           float randomness,
                                           ccl_private float *radius_out)
{
  float3 cellPosition = floor(coord);
  float3 localPosition = coord - cellPosition;

  float3 closestPoint = make_float3(0.0f, 0.0f, 0.0f);
  float3 closestPointOffset = make_float3(0.0f, 0.0f, 0.0f);
  float minDistance = 8.0f;
  for (int k = -1; k <= 1; k++) {
    for (int j = -1; j <= 1; j++) {
      for (int i = -1; i <= 1; i++) {
        float3 cellOffset = make_float3(i, j, k);
        float3 pointPosition = cellOffset +
                               hash_float3_to_float3(cellPosition + cellOffset) * randomness;
        float distanceToPoint = distance(pointPosition, localPosition);
        if (distanceToPoint < minDistance) {
          minDistance = distanceToPoint;
          closestPoint = pointPosition;
          closestPointOffset = cellOffset;
        }
      }
    }
  }

  minDistance = 8.0f;
  float3 closestPointToClosestPoint = make_float3(0.0f, 0.0f, 0.0f);
  for (int k = -1; k <= 1; k++) {
    for (int j = -1; j <= 1; j++) {
      for (int i = -1; i <= 1; i++) {
        if (i == 0 && j == 0 && k == 0) {
          continue;
        }
        float3 cellOffset = make_float3(i, j, k) + closestPointOffset;
        float3 pointPosition = cellOffset +
                               hash_float3_to_float3(cellPosition + cellOffset) * randomness;
        float distanceToPoint = distance(closestPoint, pointPosition);
        if (distanceToPoint < minDistance) {
          minDistance = distanceToPoint;
          closestPointToClosestPoint = pointPosition;
        }
      }
    }
  }
  *radius_out = distance(closestPointToClosestPoint, closestPoint) / 2.0f;
}

/* **** 4D Voronoi **** */

ccl_device float voronoi_distance_4d(float4 a,
                                     float4 b,
                                     NodeVoronoiDistanceMetric metric,
                                     float exponent)
{
  if (metric == NODE_VORONOI_EUCLIDEAN) {
    return distance(a, b);
  }
  else if (metric == NODE_VORONOI_MANHATTAN) {
    return fabsf(a.x - b.x) + fabsf(a.y - b.y) + fabsf(a.z - b.z) + fabsf(a.w - b.w);
  }
  else if (metric == NODE_VORONOI_CHEBYCHEV) {
    return max(fabsf(a.x - b.x), max(fabsf(a.y - b.y), max(fabsf(a.z - b.z), fabsf(a.w - b.w))));
  }
  else if (metric == NODE_VORONOI_MINKOWSKI) {
    return powf(powf(fabsf(a.x - b.x), exponent) + powf(fabsf(a.y - b.y), exponent) +
                    powf(fabsf(a.z - b.z), exponent) + powf(fabsf(a.w - b.w), exponent),
                1.0f / exponent);
  }
  else {
    return 0.0f;
  }
}

ccl_device void voronoi_f1(float4 coord,
                           float exponent,
                           float randomness,
                           NodeVoronoiDistanceMetric metric,
                           ccl_private float *distance_out,
                           ccl_private float3 *color_out,
                           ccl_private float4 *position_out)
{
  float4 cellPosition = floor(coord);
  float4 localPosition = coord - cellPosition;

  float minDistance = 8.0f;
  float4 targetOffset = zero_float4();
  float4 targetPosition = zero_float4();
  for (int u = -1; u <= 1; u++) {
    for (int k = -1; k <= 1; k++) {
      ccl_loop_no_unroll for (int j = -1; j <= 1; j++)
      {
        for (int i = -1; i <= 1; i++) {
          float4 cellOffset = make_float4(i, j, k, u);
          float4 pointPosition = cellOffset +
                                 hash_float4_to_float4(cellPosition + cellOffset) * randomness;
          float distanceToPoint = voronoi_distance_4d(
              pointPosition, localPosition, metric, exponent);
          if (distanceToPoint < minDistance) {
            targetOffset = cellOffset;
            minDistance = distanceToPoint;
            targetPosition = pointPosition;
          }
        }
      }
    }
  }
  *distance_out = minDistance;
  *color_out = hash_float4_to_float3(cellPosition + targetOffset);
  *position_out = targetPosition + cellPosition;
}

ccl_device void voronoi_smooth_f1(float4 coord,
                                  float smoothness,
                                  float exponent,
                                  float randomness,
                                  NodeVoronoiDistanceMetric metric,
                                  ccl_private float *distance_out,
                                  ccl_private float3 *color_out,
                                  ccl_private float4 *position_out)
{
  float4 cellPosition = floor(coord);
  float4 localPosition = coord - cellPosition;

  float smoothDistance = 8.0f;
  float3 smoothColor = make_float3(0.0f, 0.0f, 0.0f);
  float4 smoothPosition = zero_float4();
  for (int u = -2; u <= 2; u++) {
    for (int k = -2; k <= 2; k++) {
      ccl_loop_no_unroll for (int j = -2; j <= 2; j++)
      {
        for (int i = -2; i <= 2; i++) {
          float4 cellOffset = make_float4(i, j, k, u);
          float4 pointPosition = cellOffset +
                                 hash_float4_to_float4(cellPosition + cellOffset) * randomness;
          float distanceToPoint = voronoi_distance_4d(
              pointPosition, localPosition, metric, exponent);
          float h = smoothstep(
              0.0f, 1.0f, 0.5f + 0.5f * (smoothDistance - distanceToPoint) / smoothness);
          float correctionFactor = smoothness * h * (1.0f - h);
          smoothDistance = mix(smoothDistance, distanceToPoint, h) - correctionFactor;
          correctionFactor /= 1.0f + 3.0f * smoothness;
          float3 cellColor = hash_float4_to_float3(cellPosition + cellOffset);
          smoothColor = mix(smoothColor, cellColor, h) - correctionFactor;
          smoothPosition = mix(smoothPosition, pointPosition, h) - correctionFactor;
        }
      }
    }
  }
  *distance_out = smoothDistance;
  *color_out = smoothColor;
  *position_out = cellPosition + smoothPosition;
}

ccl_device void voronoi_f2(float4 coord,
                           float exponent,
                           float randomness,
                           NodeVoronoiDistanceMetric metric,
                           ccl_private float *distance_out,
                           ccl_private float3 *color_out,
                           ccl_private float4 *position_out)
{
  float4 cellPosition = floor(coord);
  float4 localPosition = coord - cellPosition;

  float distanceF1 = 8.0f;
  float distanceF2 = 8.0f;
  float4 offsetF1 = zero_float4();
  float4 positionF1 = zero_float4();
  float4 offsetF2 = zero_float4();
  float4 positionF2 = zero_float4();
  for (int u = -1; u <= 1; u++) {
    for (int k = -1; k <= 1; k++) {
      ccl_loop_no_unroll for (int j = -1; j <= 1; j++)
      {
        for (int i = -1; i <= 1; i++) {
          float4 cellOffset = make_float4(i, j, k, u);
          float4 pointPosition = cellOffset +
                                 hash_float4_to_float4(cellPosition + cellOffset) * randomness;
          float distanceToPoint = voronoi_distance_4d(
              pointPosition, localPosition, metric, exponent);
          if (distanceToPoint < distanceF1) {
            distanceF2 = distanceF1;
            distanceF1 = distanceToPoint;
            offsetF2 = offsetF1;
            offsetF1 = cellOffset;
            positionF2 = positionF1;
            positionF1 = pointPosition;
          }
          else if (distanceToPoint < distanceF2) {
            distanceF2 = distanceToPoint;
            offsetF2 = cellOffset;
            positionF2 = pointPosition;
          }
        }
      }
    }
  }
  *distance_out = distanceF2;
  *color_out = hash_float4_to_float3(cellPosition + offsetF2);
  *position_out = positionF2 + cellPosition;
}

ccl_device void voronoi_distance_to_edge(float4 coord,
                                         float randomness,
                                         ccl_private float *distance_out)
{
  float4 cellPosition = floor(coord);
  float4 localPosition = coord - cellPosition;

  float4 vectorToClosest = zero_float4();
  float minDistance = 8.0f;
  for (int u = -1; u <= 1; u++) {
    for (int k = -1; k <= 1; k++) {
      ccl_loop_no_unroll for (int j = -1; j <= 1; j++)
      {
        for (int i = -1; i <= 1; i++) {
          float4 cellOffset = make_float4(i, j, k, u);
          float4 vectorToPoint = cellOffset +
                                 hash_float4_to_float4(cellPosition + cellOffset) * randomness -
                                 localPosition;
          float distanceToPoint = dot(vectorToPoint, vectorToPoint);
          if (distanceToPoint < minDistance) {
            minDistance = distanceToPoint;
            vectorToClosest = vectorToPoint;
          }
        }
      }
    }
  }

  minDistance = 8.0f;
  for (int u = -1; u <= 1; u++) {
    for (int k = -1; k <= 1; k++) {
      ccl_loop_no_unroll for (int j = -1; j <= 1; j++)
      {
        for (int i = -1; i <= 1; i++) {
          float4 cellOffset = make_float4(i, j, k, u);
          float4 vectorToPoint = cellOffset +
                                 hash_float4_to_float4(cellPosition + cellOffset) * randomness -
                                 localPosition;
          float4 perpendicularToEdge = vectorToPoint - vectorToClosest;
          if (dot(perpendicularToEdge, perpendicularToEdge) > 0.0001f) {
            float distanceToEdge = dot((vectorToClosest + vectorToPoint) / 2.0f,
                                       normalize(perpendicularToEdge));
            minDistance = min(minDistance, distanceToEdge);
          }
        }
      }
    }
  }
  *distance_out = minDistance;
}

ccl_device void voronoi_n_sphere_radius_4d(float4 coord,
                                           float randomness,
                                           ccl_private float *radius_out)
{
  float4 cellPosition = floor(coord);
  float4 localPosition = coord - cellPosition;

  float4 closestPoint = zero_float4();
  float4 closestPointOffset = zero_float4();
  float minDistance = 8.0f;
  for (int u = -1; u <= 1; u++) {
    for (int k = -1; k <= 1; k++) {
      ccl_loop_no_unroll for (int j = -1; j <= 1; j++)
      {
        for (int i = -1; i <= 1; i++) {
          float4 cellOffset = make_float4(i, j, k, u);
          float4 pointPosition = cellOffset +
                                 hash_float4_to_float4(cellPosition + cellOffset) * randomness;
          float distanceToPoint = distance(pointPosition, localPosition);
          if (distanceToPoint < minDistance) {
            minDistance = distanceToPoint;
            closestPoint = pointPosition;
            closestPointOffset = cellOffset;
          }
        }
      }
    }
  }

  minDistance = 8.0f;
  float4 closestPointToClosestPoint = zero_float4();
  for (int u = -1; u <= 1; u++) {
    for (int k = -1; k <= 1; k++) {
      ccl_loop_no_unroll for (int j = -1; j <= 1; j++)
      {
        for (int i = -1; i <= 1; i++) {
          if (i == 0 && j == 0 && k == 0 && u == 0) {
            continue;
          }
          float4 cellOffset = make_float4(i, j, k, u) + closestPointOffset;
          float4 pointPosition = cellOffset +
                                 hash_float4_to_float4(cellPosition + cellOffset) * randomness;
          float distanceToPoint = distance(closestPoint, pointPosition);
          if (distanceToPoint < minDistance) {
            minDistance = distanceToPoint;
            closestPointToClosestPoint = pointPosition;
          }
        }
      }
    }
  }
  *radius_out = distance(closestPointToClosestPoint, closestPoint) / 2.0f;
}

/* **** Fractal Voronoi **** */

template<typename T>
ccl_device void fractal_voronoi_fx(VoronoiParams<T> &vp,
                                   VoronoiOutput<T> &vo,
                                   void (*voronoi_fx)(VoronoiParams<T> &))
{
  vp.octave_scale = 1.0f;
  vp.octave_amplitude = 1.0f;
  vp.octave_distance = 0.0f;
  vp.octave_color;
  vp.octave_postion;

  T voronoi_coord = vp.octave_coord;
  for (int i = 0; i <= ceilf(vp.detail); ++i) {
    vp.octave_coord = voronoi_coord * vp.octave_scale;
    voronoi_fx(vp);
    if (vp.detail == 0.0f || vp.roughness == 0.0f || vp.lacunarity == 0.0f) {
      vp.max_amplitude = 1.0f;
      vo.distance_out = vp.octave_distance;
      vo.color_out = vp.octave_color;
      vo.position_out = vp.octave_postion;
      return;
    }
    else if (i <= vp.detail) {
      vp.max_amplitude += vp.octave_amplitude;
      vo.distance_out += vp.octave_distance * vp.octave_amplitude;
      vo.color_out += vp.octave_color * vp.octave_amplitude;
      vo.position_out = lerp(
          vo.position_out, vp.octave_postion / vp.octave_scale, vp.octave_amplitude);
      vp.octave_scale *= vp.lacunarity;
      vp.octave_amplitude *= vp.roughness;
    }
    else {
      float remainder = vp.detail - floorf(vp.detail);
      if (remainder != 0.0f) {
        vp.max_amplitude = lerp(
            vp.max_amplitude, vp.max_amplitude + vp.octave_amplitude, remainder);
        vo.distance_out = lerp(
            vo.distance_out, vo.distance_out + vp.octave_distance * vp.octave_amplitude, remainder);
        vo.color_out = lerp(
            vo.color_out, vo.color_out + vp.octave_color * vp.octave_amplitude, remainder);
        vo.position_out = lerp(
            vo.position_out,
            lerp(vo.position_out, vp.octave_postion / vp.octave_scale, vp.octave_amplitude),
            remainder);
      }
    }
  }
}

template<typename T>
ccl_device void fractal_voronoi_distance_to_edge(T coord,
                                                 float detail,
                                                 float roughness,
                                                 float lacunarity,
                                                 float randomness,
                                                 ccl_private float *max_amplitude,
                                                 ccl_private float *distance_out)
{
  float octave_scale = 1.0f;
  float octave_amplitude = 1.0f;
  float octave_distance = 0.0f;

  *max_amplitude = 2.0f - randomness;
  *distance_out = 8.0f;
  for (int i = 0; i <= ceilf(detail); ++i) {
    voronoi_distance_to_edge(coord * octave_scale, randomness, &octave_distance);
    if (detail == 0.0f || roughness == 0.0f || lacunarity == 0.0f) {
      *distance_out = octave_distance;
      return;
    }
    else if (i <= detail) {
      *max_amplitude = lerp(*max_amplitude, (2.0f - randomness) * octave_scale, octave_amplitude);
      *distance_out = lerp(
          *distance_out, min(*distance_out, octave_distance / octave_scale), octave_amplitude);
      octave_scale *= lacunarity;
      octave_amplitude *= roughness;
    }
    else {
      float remainder = detail - floorf(detail);
      if (remainder != 0.0f) {
        float lerp_amplitude = lerp(
            *max_amplitude, (2.0f - randomness) * octave_scale, octave_amplitude);
        *max_amplitude = lerp(*max_amplitude, lerp_amplitude, remainder);
        float lerp_distance = lerp(
            *distance_out, min(*distance_out, octave_distance / octave_scale), octave_amplitude);
        *distance_out = lerp(*distance_out, min(*distance_out, lerp_distance), remainder);
      }
    }
  }
}

template<uint node_feature_mask>
ccl_device_noinline int svm_node_tex_voronoi(KernelGlobals kg,
                                             ccl_private ShaderData *sd,
                                             ccl_private float *stack,
                                             uint dimensions,
                                             uint feature,
                                             uint metric,
                                             int offset)
{
  uint4 stack_offsets = read_node(kg, &offset);
  uint4 defaults1 = read_node(kg, &offset);
  uint4 defaults2 = read_node(kg, &offset);

  uint coord_stack_offset, w_stack_offset, scale_stack_offset, detail_stack_offset;
  uint roughness_stack_offset, lacunarity_stack_offset, smoothness_stack_offset,
      exponent_stack_offset;
  uint randomness_stack_offset, distance_out_stack_offset, color_out_stack_offset,
      position_out_stack_offset;
  uint w_out_stack_offset, radius_out_stack_offset, normalize;

  svm_unpack_node_uchar4(stack_offsets.x,
                         &coord_stack_offset,
                         &w_stack_offset,
                         &scale_stack_offset,
                         &detail_stack_offset);
  svm_unpack_node_uchar4(stack_offsets.y,
                         &roughness_stack_offset,
                         &lacunarity_stack_offset,
                         &smoothness_stack_offset,
                         &exponent_stack_offset);
  svm_unpack_node_uchar4(stack_offsets.z,
                         &randomness_stack_offset,
                         &distance_out_stack_offset,
                         &color_out_stack_offset,
                         &position_out_stack_offset);
  svm_unpack_node_uchar3(
      stack_offsets.w, &w_out_stack_offset, &radius_out_stack_offset, &normalize);

  VoronoiParamsBase vpb;

  float3 voronoi_coord = stack_load_float3(stack, coord_stack_offset);
  float voronoi_w = stack_load_float_default(stack, w_stack_offset, defaults1.x);
  vpb.scale = stack_load_float_default(stack, scale_stack_offset, defaults1.y);
  vpb.detail = stack_load_float_default(stack, detail_stack_offset, defaults1.z);
  vpb.roughness = stack_load_float_default(stack, roughness_stack_offset, defaults1.w);
  vpb.lacunarity = stack_load_float_default(stack, smoothness_stack_offset, defaults2.x);
  vpb.smoothness = stack_load_float_default(stack, smoothness_stack_offset, defaults2.y);
  vpb.exponent = stack_load_float_default(stack, exponent_stack_offset, defaults2.z);
  vpb.randomness = stack_load_float_default(stack, randomness_stack_offset, defaults2.w);
  vpb.max_amplitude = 0.0f;

  NodeVoronoiFeature voronoi_feature = (NodeVoronoiFeature)feature;
  vpb.metric = (NodeVoronoiDistanceMetric)metric;

  VoronoiOutputBase vob;

  vob.distance_out = 0.0f;
  float voronoi_w_out = 0.0f;
  vob.radius_out = 0.0f;
  vob.color_out = make_float3(0.0f, 0.0f, 0.0f);
  float3 voronoi_position_out = make_float3(0.0f, 0.0f, 0.0f);

  vpb.detail = clamp(vpb.detail, 0.0f, 15.0f);
  vpb.roughness = clamp(vpb.roughness, 0.0f, 1.0f);
  vpb.randomness = clamp(vpb.randomness, 0.0f, 1.0f);
  vpb.smoothness = clamp(vpb.smoothness / 2.0f, 0.0f, 0.5f);

  voronoi_coord *= vpb.scale;
  voronoi_w *= vpb.scale;

  // switch (dimensions) {
  //   case 1: {
  //     switch (voronoi_feature) {
  //       case NODE_VORONOI_F1: {
  //         fractal_voronoi_f1<float>(w,
  //                                   detail,
  //                                   roughness,
  //                                   lacunarity,
  //                                   exponent,
  //                                   randomness,
  //                                   voronoi_metric,
  //                                   &max_amplitude,
  //                                   &distance_out,
  //                                   &color_out,
  //                                   &w_out);
  //         if (normalize) {
  //           distance_out /= (0.5f + 0.5f * randomness) * max_amplitude;
  //           color_out /= max_amplitude;
  //         }
  //         break;
  //       }
  //       case NODE_VORONOI_SMOOTH_F1: {
  //         fractal_voronoi_smooth_f1<float>(w,
  //                                          detail,
  //                                          roughness,
  //                                          lacunarity,
  //                                          smoothness,
  //                                          exponent,
  //                                          randomness,
  //                                          voronoi_metric,
  //                                          &max_amplitude,
  //                                          &distance_out,
  //                                          &color_out,
  //                                          &w_out);
  //         if (normalize) {
  //           distance_out /= (0.5f + 0.5f * randomness) * max_amplitude;
  //           color_out /= max_amplitude;
  //         }
  //         break;
  //       }
  //       case NODE_VORONOI_F2: {
  //         fractal_voronoi_f2<float>(w,
  //                                   detail,
  //                                   roughness,
  //                                   lacunarity,
  //                                   exponent,
  //                                   randomness,
  //                                   voronoi_metric,
  //                                   &max_amplitude,
  //                                   &distance_out,
  //                                   &color_out,
  //                                   &w_out);
  //         if (normalize) {
  //           if (detail == 0.0f || roughness == 0.0f || lacunarity == 0.0f) {
  //             distance_out /= (1.0f - randomness) + randomness * max_amplitude;
  //           }
  //           else {
  //             distance_out /= (1.0f - randomness) * ceilf(detail + 1.0f) +
  //                             randomness * max_amplitude;
  //           }
  //           color_out /= max_amplitude;
  //         }
  //         break;
  //       }
  //       case NODE_VORONOI_DISTANCE_TO_EDGE: {
  //         fractal_voronoi_distance_to_edge<float>(
  //             w, detail, roughness, lacunarity, randomness, &max_amplitude, &distance_out);
  //         if (normalize) {
  //           /* max_amplitude is used here to keep the code consistent, however it has a
  //           different
  //            * meaning than in F1, Smooth F1 and F2. Instead of the highest possible amplitude,
  //            it
  //            * represents an abstract factor needed to cancel out the amplitude attenuation
  //            caused
  //            * by the higher layers. */
  //           distance_out *= max_amplitude;
  //         }
  //         break;
  //       }
  //       case NODE_VORONOI_N_SPHERE_RADIUS:
  //         voronoi_n_sphere_radius_1d(w, randomness, &radius_out);
  //         break;
  //       default:
  //         kernel_assert(0);
  //     }
  //     w_out = safe_divide(w_out, scale);
  //     break;
  //   }
  //   case 2: {
  //     float2 coord_2d = make_float2(coord.x, coord.y);
  //     float2 position_out_2d = zero_float2();

  //    switch (voronoi_feature) {
  //      case NODE_VORONOI_F1: {
  //        fractal_voronoi_f1<float2>(coord_2d,
  //                                   detail,
  //                                   roughness,
  //                                   lacunarity,
  //                                   exponent,
  //                                   randomness,
  //                                   voronoi_metric,
  //                                   &max_amplitude,
  //                                   &distance_out,
  //                                   &color_out,
  //                                   &position_out_2d);
  //        if (normalize) {
  //          distance_out /= (0.5f + 0.5f * randomness) * max_amplitude *
  //                          voronoi_distance_2d(make_float2(1.0f, 1.0f),
  //                                              make_float2(0.0f, 0.0f),
  //                                              voronoi_metric,
  //                                              exponent);
  //          color_out /= max_amplitude;
  //        }
  //        break;
  //      }
  //      case NODE_VORONOI_SMOOTH_F1:
  //        IF_KERNEL_NODES_FEATURE(VORONOI_EXTRA)
  //        {
  //          fractal_voronoi_smooth_f1<float2>(coord_2d,
  //                                            detail,
  //                                            roughness,
  //                                            lacunarity,
  //                                            smoothness,
  //                                            exponent,
  //                                            randomness,
  //                                            voronoi_metric,
  //                                            &max_amplitude,
  //                                            &distance_out,
  //                                            &color_out,
  //                                            &position_out_2d);
  //          if (normalize) {
  //            distance_out /= (0.5f + 0.5f * randomness) * max_amplitude *
  //                            voronoi_distance_2d(make_float2(1.0f, 1.0f),
  //                                                make_float2(0.0f, 0.0f),
  //                                                voronoi_metric,
  //                                                exponent);
  //            color_out /= max_amplitude;
  //          }
  //          break;
  //        }
  //        break;
  //      case NODE_VORONOI_F2: {
  //        fractal_voronoi_f2<float2>(coord_2d,
  //                                   detail,
  //                                   roughness,
  //                                   lacunarity,
  //                                   exponent,
  //                                   randomness,
  //                                   voronoi_metric,
  //                                   &max_amplitude,
  //                                   &distance_out,
  //                                   &color_out,
  //                                   &position_out_2d);
  //        if (normalize) {
  //          if (detail == 0.0f || roughness == 0.0f || lacunarity == 0.0f) {
  //            distance_out /= (1.0f - randomness) +
  //                            randomness * max_amplitude *
  //                                voronoi_distance_2d(make_float2(1.0f, 1.0f),
  //                                                    make_float2(0.0f, 0.0f),
  //                                                    voronoi_metric,
  //                                                    exponent);
  //          }
  //          else {
  //            distance_out /= (1.0f - randomness) * ceilf(detail + 1.0f) +
  //                            randomness * max_amplitude *
  //                                voronoi_distance_2d(make_float2(1.0f, 1.0f),
  //                                                    make_float2(0.0f, 0.0f),
  //                                                    voronoi_metric,
  //                                                    exponent);
  //          }
  //          color_out /= max_amplitude;
  //        }
  //        break;
  //      }
  //      case NODE_VORONOI_DISTANCE_TO_EDGE: {
  //        fractal_voronoi_distance_to_edge<float2>(
  //            coord_2d, detail, roughness, lacunarity, randomness, &max_amplitude,
  //            &distance_out);
  //        if (normalize) {
  //          /* max_amplitude is used here to keep the code consistent, however it has a different
  //           * meaning than in F1, Smooth F1 and F2. Instead of the highest possible amplitude,
  //           it
  //           * represents an abstract factor needed to cancel out the amplitude attenuation
  //           caused
  //           * by the higher layers. */
  //          distance_out *= max_amplitude;
  //        }
  //        break;
  //      }
  //      case NODE_VORONOI_N_SPHERE_RADIUS:
  //        voronoi_n_sphere_radius_2d(coord_2d, randomness, &radius_out);
  //        break;
  //      default:
  //        kernel_assert(0);
  //    }
  //    position_out_2d = safe_divide_float2_float(position_out_2d, scale);
  //    position_out = make_float3(position_out_2d.x, position_out_2d.y, 0.0f);
  //    break;
  //  }
  //  case 3: {
  VoronoiParams<float3> vp{vpb};
  vp.octave_coord = voronoi_coord;
  vp.max_distance = voronoi_distance_3d(zero_float3(), one_float3(), vp.metric, vp.exponent);
  VoronoiOutput<float3> vo{vob};
  vo.position_out = zero_float3();
  //    switch (voronoi_feature) {
  //   case NODE_VORONOI_F1: {
  fractal_voronoi_fx<float3>(vp, vo, voronoi_f1_3d);
  vob = vo;
  voronoi_position_out = vo.position_out;
  voronoi_w_out = 0.0f;
  // if (normalize) {
  //   distance_out /= (0.5f + 0.5f * randomness) * max_amplitude *
  //                   voronoi_distance_3d(make_float3(1.0f, 1.0f, 1.0f),
  //                                       make_float3(0.0f, 0.0f, 0.0f),
  //                                       voronoi_metric,
  //                                       exponent);
  //   color_out /= max_amplitude;
  // }
  //         break;
  //       }
  //       case NODE_VORONOI_SMOOTH_F1:
  //         IF_KERNEL_NODES_FEATURE(VORONOI_EXTRA)
  //         {
  //           fractal_voronoi_smooth_f1<float3>(coord,
  //                                             detail,
  //                                             roughness,
  //                                             lacunarity,
  //                                             smoothness,
  //                                             exponent,
  //                                             randomness,
  //                                             voronoi_metric,
  //                                             &max_amplitude,
  //                                             &distance_out,
  //                                             &color_out,
  //                                             &position_out);
  //           if (normalize) {
  //             distance_out /= (0.5f + 0.5f * randomness) * max_amplitude *
  //                             voronoi_distance_3d(make_float3(1.0f, 1.0f, 1.0f),
  //                                                 make_float3(0.0f, 0.0f, 0.0f),
  //                                                 voronoi_metric,
  //                                                 exponent);
  //             color_out /= max_amplitude;
  //           }
  //           break;
  //         }
  //         break;
  //       case NODE_VORONOI_F2: {
  //         fractal_voronoi_f2<float3>(coord,
  //                                    detail,
  //                                    roughness,
  //                                    lacunarity,
  //                                    exponent,
  //                                    randomness,
  //                                    voronoi_metric,
  //                                    &max_amplitude,
  //                                    &distance_out,
  //                                    &color_out,
  //                                    &position_out);
  //         if (normalize) {
  //           if (detail == 0.0f || roughness == 0.0f || lacunarity == 0.0f) {
  //             distance_out /= (1.0f - randomness) +
  //                             randomness * max_amplitude *
  //                                 voronoi_distance_3d(make_float3(1.0f, 1.0f, 1.0f),
  //                                                     make_float3(0.0f, 0.0f, 0.0f),
  //                                                     voronoi_metric,
  //                                                     exponent);
  //           }
  //           else {
  //             distance_out /= (1.0f - randomness) * ceilf(detail + 1.0f) +
  //                             randomness * max_amplitude *
  //                                 voronoi_distance_3d(make_float3(1.0f, 1.0f, 1.0f),
  //                                                     make_float3(0.0f, 0.0f, 0.0f),
  //                                                     voronoi_metric,
  //                                                     exponent);
  //           }
  //           color_out /= max_amplitude;
  //         }
  //         break;
  //       }
  //       case NODE_VORONOI_DISTANCE_TO_EDGE: {
  //         fractal_voronoi_distance_to_edge<float3>(
  //             coord, detail, roughness, lacunarity, randomness, &max_amplitude, &distance_out);
  //         if (normalize) {
  //           /* max_amplitude is used here to keep the code consistent, however it has a
  //           different
  //            * meaning than in F1, Smooth F1 and F2. Instead of the highest possible amplitude,
  //            it
  //            * represents an abstract factor needed to cancel out the amplitude attenuation
  //            caused
  //            * by the higher layers. */
  //           distance_out *= max_amplitude;
  //         }
  //         break;
  //       }
  //       case NODE_VORONOI_N_SPHERE_RADIUS:
  //         voronoi_n_sphere_radius_3d(coord, randomness, &radius_out);
  //         break;
  //       default:
  //         kernel_assert(0);
  //     }
  //     position_out = safe_divide(position_out, scale);
  //     break;
  //   }

  //  case 4: {
  //    IF_KERNEL_NODES_FEATURE(VORONOI_EXTRA)
  //    {
  //      float4 coord_4d = make_float4(coord.x, coord.y, coord.z, w);
  //      float4 position_out_4d;

  //      switch (voronoi_feature) {
  //        case NODE_VORONOI_F1: {
  //          fractal_voronoi_f1<float4>(coord_4d,
  //                                     detail,
  //                                     roughness,
  //                                     lacunarity,
  //                                     exponent,
  //                                     randomness,
  //                                     voronoi_metric,
  //                                     &max_amplitude,
  //                                     &distance_out,
  //                                     &color_out,
  //                                     &position_out_4d);
  //          if (normalize) {
  //            distance_out /= (0.5f + 0.5f * randomness) * max_amplitude *
  //                            voronoi_distance_4d(make_float4(1.0f, 1.0f, 1.0f, 1.0f),
  //                                                make_float4(0.0f, 0.0f, 0.0f, 0.0f),
  //                                                voronoi_metric,
  //                                                exponent);
  //            color_out /= max_amplitude;
  //          }
  //          break;
  //        }
  //        case NODE_VORONOI_SMOOTH_F1: {
  //          fractal_voronoi_smooth_f1<float4>(coord_4d,
  //                                            detail,
  //                                            roughness,
  //                                            lacunarity,
  //                                            smoothness,
  //                                            exponent,
  //                                            randomness,
  //                                            voronoi_metric,
  //                                            &max_amplitude,
  //                                            &distance_out,
  //                                            &color_out,
  //                                            &position_out_4d);
  //          if (normalize) {
  //            distance_out /= (0.5f + 0.5f * randomness) * max_amplitude *
  //                            voronoi_distance_4d(make_float4(1.0f, 1.0f, 1.0f, 1.0f),
  //                                                make_float4(0.0f, 0.0f, 0.0f, 0.0f),
  //                                                voronoi_metric,
  //                                                exponent);
  //            color_out /= max_amplitude;
  //          }
  //          break;
  //        }
  //        case NODE_VORONOI_F2: {
  //          fractal_voronoi_f2<float4>(coord_4d,
  //                                     detail,
  //                                     roughness,
  //                                     lacunarity,
  //                                     exponent,
  //                                     randomness,
  //                                     voronoi_metric,
  //                                     &max_amplitude,
  //                                     &distance_out,
  //                                     &color_out,
  //                                     &position_out_4d);
  //          if (normalize) {
  //            if (detail == 0.0f || roughness == 0.0f || lacunarity == 0.0f) {
  //              distance_out /= (1.0f - randomness) +
  //                              randomness * max_amplitude *
  //                                  voronoi_distance_4d(make_float4(1.0f, 1.0f, 1.0f, 1.0f),
  //                                                      make_float4(0.0f, 0.0f, 0.0f, 0.0f),
  //                                                      voronoi_metric,
  //                                                      exponent);
  //            }
  //            else {
  //              distance_out /= (1.0f - randomness) * ceilf(detail + 1.0f) +
  //                              randomness * max_amplitude *
  //                                  voronoi_distance_4d(make_float4(1.0f, 1.0f, 1.0f, 1.0f),
  //                                                      make_float4(0.0f, 0.0f, 0.0f, 0.0f),
  //                                                      voronoi_metric,
  //                                                      exponent);
  //            }
  //            color_out /= max_amplitude;
  //          }
  //          break;
  //        }
  //        case NODE_VORONOI_DISTANCE_TO_EDGE: {
  //          fractal_voronoi_distance_to_edge<float4>(coord_4d,
  //                                                   detail,
  //                                                   roughness,
  //                                                   lacunarity,
  //                                                   randomness,
  //                                                   &max_amplitude,
  //                                                   &distance_out);
  //          if (normalize) {
  //            /* max_amplitude is used here to keep the code consistent, however it has a
  //            different
  //             * meaning than in F1, Smooth F1 and F2. Instead of the highest possible
  //             amplitude,
  //             * it represents an abstract factor needed to cancel out the amplitude
  //             attenuation
  //             * caused by the higher layers. */
  //            distance_out *= max_amplitude;
  //          }
  //          break;
  //        }
  //        case NODE_VORONOI_N_SPHERE_RADIUS:
  //          voronoi_n_sphere_radius_4d(coord_4d, randomness, &radius_out);
  //          break;
  //        default:
  //          kernel_assert(0);
  //      }
  //      position_out_4d = safe_divide(position_out_4d, scale);
  //      position_out = make_float3(position_out_4d.x, position_out_4d.y, position_out_4d.z);
  //      w_out = position_out_4d.w;
  //    }
  //    break;
  //  }
  //  default:
  //    kernel_assert(0);
  //}

   if (stack_valid(distance_out_stack_offset))
     stack_store_float(stack, distance_out_stack_offset, vob.distance_out);
   if (stack_valid(color_out_stack_offset))
     stack_store_float3(stack, color_out_stack_offset, vob.color_out);
   if (stack_valid(position_out_stack_offset))
     stack_store_float3(stack, position_out_stack_offset, voronoi_position_out);
   if (stack_valid(w_out_stack_offset))
     stack_store_float(stack, w_out_stack_offset, voronoi_w_out);
   if (stack_valid(radius_out_stack_offset))
     stack_store_float(stack, radius_out_stack_offset, vob.radius_out);
  return offset;
}

CCL_NAMESPACE_END
