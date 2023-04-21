/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once
#include "device/device.h"

#include "scene/background.h"
#include "scene/light.h"
#include "scene/mesh.h"
#include "scene/scene.h"
#include "scene/shader.h"
#include "scene/shader_graph.h"
#include "scene/shader_nodes.h"
#include "scene/stats.h"
#include "scene/svm.h"

#include "util/foreach.h"
#include "util/log.h"
#include "util/progress.h"
#include "util/task.h"
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
  uint normalize;
  NodeVoronoiFeature feature;
  NodeVoronoiDistanceMetric metric;
  float octave_scale;
  float octave_amplitude;
  float octave_distance;
  float3 octave_color;

  ccl_device_inline_method VoronoiParamsBase() = default;

  ccl_device_inline_method VoronoiParamsBase(const VoronoiParamsBase &) = default;
};

template<typename T> struct VoronoiParams : public VoronoiParamsBase {
  T octave_coord;
  T octave_postion;

  ccl_device_inline_method VoronoiParams(const VoronoiParamsBase &vpb) : VoronoiParamsBase(vpb) {}
};

struct VoronoiOutputBase {
  float distance_out;
  float radius_out;
  float3 color_out;

  ccl_device_inline_method VoronoiOutputBase() = default;

  ccl_device_inline_method VoronoiOutputBase(const VoronoiOutputBase &) = default;
};

template<typename T> struct VoronoiOutput : public VoronoiOutputBase {
  T position_out;

  ccl_device_inline_method VoronoiOutput(const VoronoiOutputBase &vob) : VoronoiOutputBase(vob) {}
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

ccl_device void voronoi_f1_1d(VoronoiParams<float> &vp)
{
  float cellPosition = floorf(vp.octave_coord);
  float localPosition = vp.octave_coord - cellPosition;

  float minDistance = 8.0f;
  float targetOffset = 0.0f;
  float targetPosition = 0.0f;
  for (int i = -1; i <= 1; i++) {
    float cellOffset = i;
    float pointPosition = cellOffset +
                          hash_float_to_float(cellPosition + cellOffset) * vp.randomness;
    float distanceToPoint = voronoi_distance_1d(
        pointPosition, localPosition, vp.metric, vp.exponent);
    if (distanceToPoint < minDistance) {
      targetOffset = cellOffset;
      minDistance = distanceToPoint;
      targetPosition = pointPosition;
    }
  }
  vp.octave_distance = minDistance;
  vp.octave_color = hash_float_to_float3(cellPosition + targetOffset);
  vp.octave_postion = targetPosition + cellPosition;
}

ccl_device void voronoi_smooth_f1_1d(VoronoiParams<float> &vp)
{
  float cellPosition = floorf(vp.octave_coord);
  float localPosition = vp.octave_coord - cellPosition;

  float smoothDistance = 8.0f;
  float smoothPosition = 0.0f;
  float3 smoothColor = make_float3(0.0f, 0.0f, 0.0f);
  for (int i = -2; i <= 2; i++) {
    float cellOffset = i;
    float pointPosition = cellOffset +
                          hash_float_to_float(cellPosition + cellOffset) * vp.randomness;
    float distanceToPoint = voronoi_distance_1d(
        pointPosition, localPosition, vp.metric, vp.exponent);
    float h = smoothstep(
        0.0f, 1.0f, 0.5f + 0.5f * (smoothDistance - distanceToPoint) / vp.smoothness);
    float correctionFactor = vp.smoothness * h * (1.0f - h);
    smoothDistance = mix(smoothDistance, distanceToPoint, h) - correctionFactor;
    correctionFactor /= 1.0f + 3.0f * vp.smoothness;
    float3 cellColor = hash_float_to_float3(cellPosition + cellOffset);
    smoothColor = mix(smoothColor, cellColor, h) - correctionFactor;
    smoothPosition = mix(smoothPosition, pointPosition, h) - correctionFactor;
  }
  vp.octave_distance = smoothDistance;
  vp.octave_color = smoothColor;
  vp.octave_postion = cellPosition + smoothPosition;
}

ccl_device void voronoi_f2_1d(VoronoiParams<float> &vp)
{
  float cellPosition = floorf(vp.octave_coord);
  float localPosition = vp.octave_coord - cellPosition;

  float distanceF1 = 8.0f;
  float distanceF2 = 8.0f;
  float offsetF1 = 0.0f;
  float positionF1 = 0.0f;
  float offsetF2 = 0.0f;
  float positionF2 = 0.0f;
  for (int i = -1; i <= 1; i++) {
    float cellOffset = i;
    float pointPosition = cellOffset +
                          hash_float_to_float(cellPosition + cellOffset) * vp.randomness;
    float distanceToPoint = voronoi_distance_1d(
        pointPosition, localPosition, vp.metric, vp.exponent);
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
  vp.octave_distance = distanceF2;
  vp.octave_color = hash_float_to_float3(cellPosition + offsetF2);
  vp.octave_postion = positionF2 + cellPosition;
}

ccl_device void voronoi_distance_to_edge_1d(VoronoiParams<float> &vp)
{
  float cellPosition = floorf(vp.octave_coord);
  float localPosition = vp.octave_coord - cellPosition;

  float midPointPosition = hash_float_to_float(cellPosition) * vp.randomness;
  float leftPointPosition = -1.0f + hash_float_to_float(cellPosition - 1.0f) * vp.randomness;
  float rightPointPosition = 1.0f + hash_float_to_float(cellPosition + 1.0f) * vp.randomness;
  float distanceToMidLeft = fabsf((midPointPosition + leftPointPosition) / 2.0f - localPosition);
  float distanceToMidRight = fabsf((midPointPosition + rightPointPosition) / 2.0f - localPosition);

  vp.octave_distance = min(distanceToMidLeft, distanceToMidRight);
}

ccl_device void voronoi_n_sphere_radius_1d(VoronoiParams<float> &vp, VoronoiOutput<float> &vo)
{
  float cellPosition = floorf(vp.octave_coord);
  float localPosition = vp.octave_coord - cellPosition;

  float closestPoint = 0.0f;
  float closestPointOffset = 0.0f;
  float minDistance = 8.0f;
  for (int i = -1; i <= 1; i++) {
    float cellOffset = i;
    float pointPosition = cellOffset +
                          hash_float_to_float(cellPosition + cellOffset) * vp.randomness;
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
    float pointPosition = cellOffset +
                          hash_float_to_float(cellPosition + cellOffset) * vp.randomness;
    float distanceToPoint = fabsf(closestPoint - pointPosition);
    if (distanceToPoint < minDistance) {
      minDistance = distanceToPoint;
      closestPointToClosestPoint = pointPosition;
    }
  }
  vo.radius_out = fabsf(closestPointToClosestPoint - closestPoint) / 2.0f;
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

ccl_device void voronoi_f1_2d(VoronoiParams<float2> &vp)
{
  float2 cellPosition = floor(vp.octave_coord);
  float2 localPosition = vp.octave_coord - cellPosition;

  float minDistance = 8.0f;
  float2 targetOffset = make_float2(0.0f, 0.0f);
  float2 targetPosition = make_float2(0.0f, 0.0f);
  for (int j = -1; j <= 1; j++) {
    for (int i = -1; i <= 1; i++) {
      float2 cellOffset = make_float2(i, j);
      float2 pointPosition = cellOffset +
                             hash_float2_to_float2(cellPosition + cellOffset) * vp.randomness;
      float distanceToPoint = voronoi_distance_2d(
          pointPosition, localPosition, vp.metric, vp.exponent);
      if (distanceToPoint < minDistance) {
        targetOffset = cellOffset;
        minDistance = distanceToPoint;
        targetPosition = pointPosition;
      }
    }
  }
  vp.octave_distance = minDistance;
  vp.octave_color = hash_float2_to_float3(cellPosition + targetOffset);
  vp.octave_postion = targetPosition + cellPosition;
}

ccl_device void voronoi_smooth_f1_2d(VoronoiParams<float2> &vp)
{
  float2 cellPosition = floor(vp.octave_coord);
  float2 localPosition = vp.octave_coord - cellPosition;

  float smoothDistance = 8.0f;
  float3 smoothColor = make_float3(0.0f, 0.0f, 0.0f);
  float2 smoothPosition = make_float2(0.0f, 0.0f);
  for (int j = -2; j <= 2; j++) {
    for (int i = -2; i <= 2; i++) {
      float2 cellOffset = make_float2(i, j);
      float2 pointPosition = cellOffset +
                             hash_float2_to_float2(cellPosition + cellOffset) * vp.randomness;
      float distanceToPoint = voronoi_distance_2d(
          pointPosition, localPosition, vp.metric, vp.exponent);
      float h = smoothstep(
          0.0f, 1.0f, 0.5f + 0.5f * (smoothDistance - distanceToPoint) / vp.smoothness);
      float correctionFactor = vp.smoothness * h * (1.0f - h);
      smoothDistance = mix(smoothDistance, distanceToPoint, h) - correctionFactor;
      correctionFactor /= 1.0f + 3.0f * vp.smoothness;
      float3 cellColor = hash_float2_to_float3(cellPosition + cellOffset);
      smoothColor = mix(smoothColor, cellColor, h) - correctionFactor;
      smoothPosition = mix(smoothPosition, pointPosition, h) - correctionFactor;
    }
  }
  vp.octave_distance = smoothDistance;
  vp.octave_color = smoothColor;
  vp.octave_postion = cellPosition + smoothPosition;
}

ccl_device void voronoi_f2_2d(VoronoiParams<float2> &vp)
{
  float2 cellPosition = floor(vp.octave_coord);
  float2 localPosition = vp.octave_coord - cellPosition;

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
                             hash_float2_to_float2(cellPosition + cellOffset) * vp.randomness;
      float distanceToPoint = voronoi_distance_2d(
          pointPosition, localPosition, vp.metric, vp.exponent);
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
  vp.octave_distance = distanceF2;
  vp.octave_color = hash_float2_to_float3(cellPosition + offsetF2);
  vp.octave_postion = positionF2 + cellPosition;
}

ccl_device void voronoi_distance_to_edge_2d(VoronoiParams<float2> &vp)
{
  float2 cellPosition = floor(vp.octave_coord);
  float2 localPosition = vp.octave_coord - cellPosition;

  float2 vectorToClosest = make_float2(0.0f, 0.0f);
  float minDistance = 8.0f;
  for (int j = -1; j <= 1; j++) {
    for (int i = -1; i <= 1; i++) {
      float2 cellOffset = make_float2(i, j);
      float2 vectorToPoint = cellOffset +
                             hash_float2_to_float2(cellPosition + cellOffset) * vp.randomness -
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
                             hash_float2_to_float2(cellPosition + cellOffset) * vp.randomness -
                             localPosition;
      float2 perpendicularToEdge = vectorToPoint - vectorToClosest;
      if (dot(perpendicularToEdge, perpendicularToEdge) > 0.0001f) {
        float distanceToEdge = dot((vectorToClosest + vectorToPoint) / 2.0f,
                                   normalize(perpendicularToEdge));
        minDistance = min(minDistance, distanceToEdge);
      }
    }
  }
  vp.octave_distance = minDistance;
}

ccl_device void voronoi_n_sphere_radius_2d(VoronoiParams<float2> &vp, VoronoiOutput<float2> &vo)
{
  float2 cellPosition = floor(vp.octave_coord);
  float2 localPosition = vp.octave_coord - cellPosition;

  float2 closestPoint = make_float2(0.0f, 0.0f);
  float2 closestPointOffset = make_float2(0.0f, 0.0f);
  float minDistance = 8.0f;
  for (int j = -1; j <= 1; j++) {
    for (int i = -1; i <= 1; i++) {
      float2 cellOffset = make_float2(i, j);
      float2 pointPosition = cellOffset +
                             hash_float2_to_float2(cellPosition + cellOffset) * vp.randomness;
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
                             hash_float2_to_float2(cellPosition + cellOffset) * vp.randomness;
      float distanceToPoint = distance(closestPoint, pointPosition);
      if (distanceToPoint < minDistance) {
        minDistance = distanceToPoint;
        closestPointToClosestPoint = pointPosition;
      }
    }
  }
  vo.radius_out = distance(closestPointToClosestPoint, closestPoint) / 2.0f;
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

ccl_device void voronoi_smooth_f1_3d(VoronoiParams<float3> &vp)
{
  float3 cellPosition = floor(vp.octave_coord);
  float3 localPosition = vp.octave_coord - cellPosition;

  float smoothDistance = 8.0f;
  float3 smoothColor = make_float3(0.0f, 0.0f, 0.0f);
  float3 smoothPosition = make_float3(0.0f, 0.0f, 0.0f);
  for (int k = -2; k <= 2; k++) {
    for (int j = -2; j <= 2; j++) {
      for (int i = -2; i <= 2; i++) {
        float3 cellOffset = make_float3(i, j, k);
        float3 pointPosition = cellOffset +
                               hash_float3_to_float3(cellPosition + cellOffset) * vp.randomness;
        float distanceToPoint = voronoi_distance_3d(
            pointPosition, localPosition, vp.metric, vp.exponent);
        float h = smoothstep(
            0.0f, 1.0f, 0.5f + 0.5f * (smoothDistance - distanceToPoint) / vp.smoothness);
        float correctionFactor = vp.smoothness * h * (1.0f - h);
        smoothDistance = mix(smoothDistance, distanceToPoint, h) - correctionFactor;
        correctionFactor /= 1.0f + 3.0f * vp.smoothness;
        float3 cellColor = hash_float3_to_float3(cellPosition + cellOffset);
        smoothColor = mix(smoothColor, cellColor, h) - correctionFactor;
        smoothPosition = mix(smoothPosition, pointPosition, h) - correctionFactor;
      }
    }
  }
  vp.octave_distance = smoothDistance;
  vp.octave_color = smoothColor;
  vp.octave_postion = cellPosition + smoothPosition;
}

ccl_device void voronoi_f2_3d(VoronoiParams<float3> &vp)
{
  float3 cellPosition = floor(vp.octave_coord);
  float3 localPosition = vp.octave_coord - cellPosition;

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
                               hash_float3_to_float3(cellPosition + cellOffset) * vp.randomness;
        float distanceToPoint = voronoi_distance_3d(
            pointPosition, localPosition, vp.metric, vp.exponent);
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
  vp.octave_distance = distanceF2;
  vp.octave_color = hash_float3_to_float3(cellPosition + offsetF2);
  vp.octave_postion = positionF2 + cellPosition;
}

ccl_device void voronoi_distance_to_edge_3d(VoronoiParams<float3> &vp)
{
  float3 cellPosition = floor(vp.octave_coord);
  float3 localPosition = vp.octave_coord - cellPosition;

  float3 vectorToClosest = make_float3(0.0f, 0.0f, 0.0f);
  float minDistance = 8.0f;
  for (int k = -1; k <= 1; k++) {
    for (int j = -1; j <= 1; j++) {
      for (int i = -1; i <= 1; i++) {
        float3 cellOffset = make_float3(i, j, k);
        float3 vectorToPoint = cellOffset +
                               hash_float3_to_float3(cellPosition + cellOffset) * vp.randomness -
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
                               hash_float3_to_float3(cellPosition + cellOffset) * vp.randomness -
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
  vp.octave_distance = minDistance;
}

ccl_device void voronoi_n_sphere_radius_3d(VoronoiParams<float3> &vp, VoronoiOutput<float3> &vo)
{
  float3 cellPosition = floor(vp.octave_coord);
  float3 localPosition = vp.octave_coord - cellPosition;

  float3 closestPoint = make_float3(0.0f, 0.0f, 0.0f);
  float3 closestPointOffset = make_float3(0.0f, 0.0f, 0.0f);
  float minDistance = 8.0f;
  for (int k = -1; k <= 1; k++) {
    for (int j = -1; j <= 1; j++) {
      for (int i = -1; i <= 1; i++) {
        float3 cellOffset = make_float3(i, j, k);
        float3 pointPosition = cellOffset +
                               hash_float3_to_float3(cellPosition + cellOffset) * vp.randomness;
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
                               hash_float3_to_float3(cellPosition + cellOffset) * vp.randomness;
        float distanceToPoint = distance(closestPoint, pointPosition);
        if (distanceToPoint < minDistance) {
          minDistance = distanceToPoint;
          closestPointToClosestPoint = pointPosition;
        }
      }
    }
  }
  vo.radius_out = distance(closestPointToClosestPoint, closestPoint) / 2.0f;
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

ccl_device void voronoi_f1_4d(VoronoiParams<float4> &vp)
{
  float4 cellPosition = floor(vp.octave_coord);
  float4 localPosition = vp.octave_coord - cellPosition;

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
                                 hash_float4_to_float4(cellPosition + cellOffset) * vp.randomness;
          float distanceToPoint = voronoi_distance_4d(
              pointPosition, localPosition, vp.metric, vp.exponent);
          if (distanceToPoint < minDistance) {
            targetOffset = cellOffset;
            minDistance = distanceToPoint;
            targetPosition = pointPosition;
          }
        }
      }
    }
  }
  vp.octave_distance = minDistance;
  vp.octave_color = hash_float4_to_float3(cellPosition + targetOffset);
  vp.octave_postion = targetPosition + cellPosition;
}

ccl_device void voronoi_smooth_f1_4d(VoronoiParams<float4> &vp)
{
  float4 cellPosition = floor(vp.octave_coord);
  float4 localPosition = vp.octave_coord - cellPosition;

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
                                 hash_float4_to_float4(cellPosition + cellOffset) * vp.randomness;
          float distanceToPoint = voronoi_distance_4d(
              pointPosition, localPosition, vp.metric, vp.exponent);
          float h = smoothstep(
              0.0f, 1.0f, 0.5f + 0.5f * (smoothDistance - distanceToPoint) / vp.smoothness);
          float correctionFactor = vp.smoothness * h * (1.0f - h);
          smoothDistance = mix(smoothDistance, distanceToPoint, h) - correctionFactor;
          correctionFactor /= 1.0f + 3.0f * vp.smoothness;
          float3 cellColor = hash_float4_to_float3(cellPosition + cellOffset);
          smoothColor = mix(smoothColor, cellColor, h) - correctionFactor;
          smoothPosition = mix(smoothPosition, pointPosition, h) - correctionFactor;
        }
      }
    }
  }
  vp.octave_distance = smoothDistance;
  vp.octave_color = smoothColor;
  vp.octave_postion = cellPosition + smoothPosition;
}

ccl_device void voronoi_f2_4d(VoronoiParams<float4> &vp)
{
  float4 cellPosition = floor(vp.octave_coord);
  float4 localPosition = vp.octave_coord - cellPosition;

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
                                 hash_float4_to_float4(cellPosition + cellOffset) * vp.randomness;
          float distanceToPoint = voronoi_distance_4d(
              pointPosition, localPosition, vp.metric, vp.exponent);
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
  vp.octave_distance = distanceF2;
  vp.octave_color = hash_float4_to_float3(cellPosition + offsetF2);
  vp.octave_postion = positionF2 + cellPosition;
}

ccl_device void voronoi_distance_to_edge_4d(VoronoiParams<float4> &vp)
{
  float4 cellPosition = floor(vp.octave_coord);
  float4 localPosition = vp.octave_coord - cellPosition;

  float4 vectorToClosest = zero_float4();
  float minDistance = 8.0f;
  for (int u = -1; u <= 1; u++) {
    for (int k = -1; k <= 1; k++) {
      ccl_loop_no_unroll for (int j = -1; j <= 1; j++)
      {
        for (int i = -1; i <= 1; i++) {
          float4 cellOffset = make_float4(i, j, k, u);
          float4 vectorToPoint = cellOffset +
                                 hash_float4_to_float4(cellPosition + cellOffset) * vp.randomness -
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
                                 hash_float4_to_float4(cellPosition + cellOffset) * vp.randomness -
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
  vp.octave_distance = minDistance;
}

ccl_device void voronoi_n_sphere_radius_4d(VoronoiParams<float4> &vp, VoronoiOutput<float4> &vo)
{
  float4 cellPosition = floor(vp.octave_coord);
  float4 localPosition = vp.octave_coord - cellPosition;

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
                                 hash_float4_to_float4(cellPosition + cellOffset) * vp.randomness;
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
                                 hash_float4_to_float4(cellPosition + cellOffset) * vp.randomness;
          float distanceToPoint = distance(closestPoint, pointPosition);
          if (distanceToPoint < minDistance) {
            minDistance = distanceToPoint;
            closestPointToClosestPoint = pointPosition;
          }
        }
      }
    }
  }
  vo.radius_out = distance(closestPointToClosestPoint, closestPoint) / 2.0f;
}

/* **** Fractal Voronoi **** */

template<typename T>
ccl_device void fractal_voronoi_x_fx(VoronoiParams<T> &vp,
                                     VoronoiOutput<T> &vo,
                                     void (*voronoi_x_fx)(VoronoiParams<T> &))
{
  vp.octave_scale = 1.0f;
  vp.octave_amplitude = 1.0f;
  vp.octave_distance = 0.0f;
  bool zeroinput = vp.detail == 0.0f || vp.roughness == 0.0f || vp.lacunarity == 0.0f;

  T voronoi_coord = vp.octave_coord;
  for (int i = 0; i <= ceilf(vp.detail); ++i) {
    vp.octave_coord = voronoi_coord * vp.octave_scale;
    voronoi_x_fx(vp);
    if (zeroinput) {
      vp.max_amplitude = 1.0f;
      vo.distance_out = vp.octave_distance;
      vo.color_out = vp.octave_color;
      vo.position_out = vp.octave_postion;
      break;
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
        vo.distance_out = lerp(vo.distance_out,
                               vo.distance_out + vp.octave_distance * vp.octave_amplitude,
                               remainder);
        vo.color_out = lerp(
            vo.color_out, vo.color_out + vp.octave_color * vp.octave_amplitude, remainder);
        vo.position_out = lerp(
            vo.position_out,
            lerp(vo.position_out, vp.octave_postion / vp.octave_scale, vp.octave_amplitude),
            remainder);
      }
    }
  }

  if (vp.normalize) {
    if (vp.feature == NODE_VORONOI_F2) {
      if (zeroinput) {
        vo.distance_out /= (1.0f - vp.randomness) +
                           vp.randomness * vp.max_amplitude * vp.max_distance;
      }
      else {
        vo.distance_out /= (1.0f - vp.randomness) * ceilf(vp.detail + 1.0f) +
                           vp.randomness * vp.max_amplitude * vp.max_distance;
      }
    }
    else {
      vo.distance_out /= (0.5f + 0.5f * vp.randomness) * vp.max_amplitude * vp.max_distance;
    }
    vo.color_out /= vp.max_amplitude;
  }
}

template<typename T>
ccl_device void fractal_voronoi_distance_to_edge(VoronoiParams<T> &vp, VoronoiOutput<T> &vo)
{
  vp.octave_scale = 1.0f;
  vp.octave_amplitude = 1.0f;
  vp.octave_distance = 8.0f;
  bool zeroinput = vp.detail == 0.0f || vp.roughness == 0.0f || vp.lacunarity == 0.0f;

  T voronoi_coord = vp.octave_coord;
  vp.max_amplitude = 2.0f - vp.randomness;
  for (int i = 0; i <= ceilf(vp.detail); ++i) {
    vp.octave_coord = voronoi_coord * vp.octave_scale;
    voronoi_distance_to_edge(vp);
    if (zeroinput) {
      vo.distance_out = vp.octave_distance;
      break;
    }
    else if (i <= detail) {
      vp.max_amplitude = lerp(
          vp.max_amplitude, (2.0f - vp.randomness) * vp.octave_scale, vp.octave_amplitude);
      vp.octave_distance = lerp(vp.octave_distance,
                                min(vp.octave_distance, octave_distance / octave_scale),
                                octave_amplitude);
      octave_scale *= lacunarity;
      octave_amplitude *= roughness;
    }
    else {
      float remainder = detail - floorf(detail);
      if (remainder != 0.0f) {
        float lerp_amplitude = lerp(
            vp.max_amplitude, (2.0f - vp.randomness) * octave_scale, octave_amplitude);
        vp.max_amplitude = lerp(vp.max_amplitude, lerp_amplitude, remainder);
        float lerp_distance = lerp(vp.octave_distance,
                                   min(vp.octave_distance, octave_distance / octave_scale),
                                   octave_amplitude);
        vp.octave_distance = lerp(
            vp.octave_distance, min(vp.octave_distance, lerp_distance), remainder);
      }
    }
  }
  if (vp.normalize) {
    /* vp.max_amplitude is used here to keep the code consistent, however it has a different
     * meaning than in F1, Smooth F1 and F2. Instead of the highest possible amplitude, it
     * represents an abstract factor needed to cancel out the amplitude attenuation caused by the
     * higher layers. */
    vo.distance_out *= vp.max_amplitude;
  }
  break;
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

  vpb.feature = (NodeVoronoiFeature)feature;
  vpb.metric = (NodeVoronoiDistanceMetric)metric;

  VoronoiOutputBase vob;

  vob.distance_out = 0.0f;
  vob.radius_out = 0.0f;
  vob.color_out = make_float3(0.0f, 0.0f, 0.0f);
  float3 voronoi_position_out = make_float3(0.0f, 0.0f, 0.0f);
  float voronoi_w_out = 0.0f;

  vpb.detail = clamp(vpb.detail, 0.0f, 15.0f);
  vpb.roughness = clamp(vpb.roughness, 0.0f, 1.0f);
  vpb.randomness = clamp(vpb.randomness, 0.0f, 1.0f);
  vpb.smoothness = clamp(vpb.smoothness / 2.0f, 0.0f, 0.5f);

  voronoi_coord *= vpb.scale;
  voronoi_w *= vpb.scale;

  switch (dimensions) {
    case 1: {
      VoronoiParams<float> vp{vpb};
      vp.octave_coord = voronoi_w;
      vp.max_distance = 1.0f;
      VoronoiOutput<float> vo{vob};
      vo.position_out = 0.0f;

      switch (vpb.feature) {
        case NODE_VORONOI_F1: {
          fractal_voronoi_x_fx<float>(vp, vo, voronoi_f1_1d);
          break;
        }
        case NODE_VORONOI_SMOOTH_F1: {
          fractal_voronoi_x_fx<float>(vp, vo, voronoi_smooth_f1_1d);
          break;
        }
        case NODE_VORONOI_F2: {
          fractal_voronoi_x_fx<float>(vp, vo, voronoi_f2_1d);
          break;
        }
        case NODE_VORONOI_DISTANCE_TO_EDGE: {
          fractal_voronoi_distance_to_edge<float>(vp, vo);
          break;
        }
        case NODE_VORONOI_N_SPHERE_RADIUS:
          voronoi_n_sphere_radius_1d(vp, vo);
          break;
        default:
          kernel_assert(0);
      }
      vo.position_out = safe_divide(vo.position_out, vpb.scale);

      vob = vo;
      voronoi_w = vo.position_out;
      break;
    }
    case 2: {
      VoronoiParams<float2> vp{vpb};
      vp.octave_coord = make_float2(voronoi_coord.x, voronoi_coord.y);
      vp.max_distance = voronoi_distance_2d(zero_float2(), one_float2(), vp.metric, vp.exponent);
      VoronoiOutput<float2> vo{vob};
      vo.position_out = zero_float2();

      switch (vpb.feature) {
        case NODE_VORONOI_F1: {
          fractal_voronoi_x_fx<float2>(vp, vo, voronoi_f1_2d);
          break;
        }
        case NODE_VORONOI_SMOOTH_F1:
          IF_KERNEL_NODES_FEATURE(VORONOI_EXTRA)
          {
            fractal_voronoi_x_fx<float2>(vp, vo, voronoi_smooth_f1_2d);
            break;
          }
          break;
        case NODE_VORONOI_F2: {
          fractal_voronoi_x_fx<float2>(vp, vo, voronoi_f2_2d);
          break;
        }
        case NODE_VORONOI_DISTANCE_TO_EDGE: {
          fractal_voronoi_distance_to_edge<float2>(vp, vo);
          break;
        }
        case NODE_VORONOI_N_SPHERE_RADIUS:
          voronoi_n_sphere_radius_2d(vp, vo);
          break;
        default:
          kernel_assert(0);
      }
      vo.position_out = safe_divide_float2_float(vo.position_out, vpb.scale);

      vob = vo;
      voronoi_position_out = make_float3(vo.position_out.x, vo.position_out.y, 0.0f);
      break;
    }
    case 3: {
      VoronoiParams<float3> vp{vpb};
      vp.octave_coord = voronoi_coord;
      vp.max_distance = voronoi_distance_3d(zero_float3(), one_float3(), vp.metric, vp.exponent);
      VoronoiOutput<float3> vo{vob};
      vo.position_out = zero_float3();

      switch (vpb.feature) {
        case NODE_VORONOI_F1: {
          fractal_voronoi_x_fx<float3>(vp, vo, voronoi_f1_3d);
          break;
        }
        case NODE_VORONOI_SMOOTH_F1:
          IF_KERNEL_NODES_FEATURE(VORONOI_EXTRA)
          {
            fractal_voronoi_x_fx<float3>(vp, vo, voronoi_smooth_f1_3d);
            break;
          }
          break;
        case NODE_VORONOI_F2: {
          fractal_voronoi_x_fx<float3>(vp, vo, voronoi_f2_3d);
          break;
        }
        case NODE_VORONOI_DISTANCE_TO_EDGE: {
          fractal_voronoi_distance_to_edge<float3>(vp, vo);
          break;
        }
        case NODE_VORONOI_N_SPHERE_RADIUS:
          voronoi_n_sphere_radius_3d(vp, vo);
          break;
        default:
          kernel_assert(0);
      }
      vo.position_out = safe_divide(vo.position_out, vpb.scale);

      vob = vo;
      voronoi_position_out = vo.position_out;
      break;
    }

    case 4: {
      IF_KERNEL_NODES_FEATURE(VORONOI_EXTRA)
      {
        VoronoiParams<float4> vp{vpb};
        vp.octave_coord = make_float4(
            voronoi_coord.x, voronoi_coord.y, voronoi_coord.z, voronoi_w);
        vp.max_distance = voronoi_distance_4d(zero_float4(), one_float4(), vp.metric, vp.exponent);
        VoronoiOutput<float4> vo{vob};
        vo.position_out = zero_float4();

        switch (vpb.feature) {
          case NODE_VORONOI_F1: {
            fractal_voronoi_x_fx<float4>(vp, vo, voronoi_f1_4d);
            break;
          }
          case NODE_VORONOI_SMOOTH_F1: {
            fractal_voronoi_x_fx<float4>(vp, vo, voronoi_smooth_f1_4d);
            break;
          }
          case NODE_VORONOI_F2: {
            fractal_voronoi_x_fx<float4>(vp, vo, voronoi_f2_4d);
            break;
          }
          case NODE_VORONOI_DISTANCE_TO_EDGE: {
            fractal_voronoi_distance_to_edge<float4>(vp, vo);
            break;
          }
          case NODE_VORONOI_N_SPHERE_RADIUS:
            voronoi_n_sphere_radius_4d(vp, vo);
            break;
          default:
            kernel_assert(0);
        }
        vo.position_out = safe_divide(vo.position_out, vpb.scale);

        vob = vo;
        voronoi_position_out = make_float3(
            vo.position_out.x, vo.position_out.y, vo.position_out.z);
        voronoi_w_out = vo.position_out.w;
      }
      break;
    }
    default:
      kernel_assert(0);
  }

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
