/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once
#define NODE_FRACTAL_VORONOI_H

#include "node_voronoi.h"
#include "stdcycles.h"
#include "vector2.h"
#include "vector4.h"

#define vector3 point

/* **** 1D Fractal Voronoi **** */

void fractal_voronoi_f1(float w,
                        float detail,
                        float roughness,
                        float lacunarity,
                        float exponent,
                        float randomness,
                        string metric,
                        output float max_amplitude,
                        output float outDistance,
                        output color outColor,
                        output float outW)
{
  voronoi_f1(w, exponent, randomness, metric, outDistance, outColor, outW);
  max_amplitude = 1.0;
}

void fractal_voronoi_smooth_f1(float w,
                               float detail,
                               float roughness,
                               float lacunarity,
                               float smoothness,
                               float exponent,
                               float randomness,
                               string metric,
                               output float max_amplitude,
                               output float outDistance,
                               output color outColor,
                               output float outW)
{
  voronoi_smooth_f1(w, smoothness, exponent, randomness, metric, outDistance, outColor, outW);
  max_amplitude = 1.0;
}

void fractal_voronoi_f2(float w,
                        float detail,
                        float roughness,
                        float lacunarity,
                        float exponent,
                        float randomness,
                        string metric,
                        output float max_amplitude,
                        output float outDistance,
                        output color outColor,
                        output float outW)
{
  voronoi_f2(w, exponent, randomness, metric, outDistance, outColor, outW);
  max_amplitude = 1.0;
}

void fractal_voronoi_distance_to_edge(float w,
                                      float detail,
                                      float lacunarity,
                                      float randomness,
                                      float normalize,
                                      output float outDistance)
{
  voronoi_distance_to_edge(w, randomness, outDistance);
}

/* **** 2D Fractal Voronoi **** */

void fractal_voronoi_f1(vector2 coord,
                        float detail,
                        float roughness,
                        float lacunarity,
                        float exponent,
                        float randomness,
                        string metric,
                        float max_distance,
                        output float max_amplitude,
                        output float outDistance,
                        output color outColor,
                        output vector2 outPosition)
{
  voronoi_f1(coord, exponent, randomness, metric, outDistance, outColor, outPosition);
  max_amplitude = max_distance;
}

void fractal_voronoi_smooth_f1(vector2 coord,
                               float detail,
                               float roughness,
                               float lacunarity,
                               float smoothness,
                               float exponent,
                               float randomness,
                               string metric,
                               float max_distance,
                               output float max_amplitude,
                               output float outDistance,
                               output color outColor,
                               output vector2 outPosition)
{
  voronoi_smooth_f1(
      coord, smoothness, exponent, randomness, metric, outDistance, outColor, outPosition);
  max_amplitude = max_distance;
}

void fractal_voronoi_f2(vector2 coord,
                        float detail,
                        float roughness,
                        float lacunarity,
                        float exponent,
                        float randomness,
                        string metric,
                        float max_distance,
                        output float max_amplitude,
                        output float outDistance,
                        output color outColor,
                        output vector2 outPosition)
{
  voronoi_f2(coord, exponent, randomness, metric, outDistance, outColor, outPosition);
  max_amplitude = max_distance;
}

void fractal_voronoi_distance_to_edge(vector2 coord,
                                      float detail,
                                      float lacunarity,
                                      float randomness,
                                      float normalize,
                                      output float outDistance)
{
  voronoi_distance_to_edge(coord, randomness, outDistance);
}

/* **** 3D Fractal Voronoi **** */

void fractal_voronoi_f1(vector3 coord,
                        float detail,
                        float roughness,
                        float lacunarity,
                        float exponent,
                        float randomness,
                        string metric,
                        float max_distance,
                        output float max_amplitude,
                        output float outDistance,
                        output color outColor,
                        output vector3 outPosition)
{
  float octave_scale = 1.0;
  float octave_amplitude = 1.0;
  float octave_distance = 0.0;

  max_amplitude = 0.0;
  outDistance = 0.0;
  for (float i = 0; i <= ceil(detail); ++i) {
    voronoi_f1(coord, exponent, randomness, metric, outDistance, outColor, outPosition);
    if (detail == 0.0 || roughness == 0.0 || lacunarity == 0.0) {
      max_amplitude = max_distance;
      outDistance = octave_distance;
      return;
    }
    else if (i <= detail) {
      max_amplitude += max_distance * octave_amplitude;
      outDistance += octave_distance * octave_amplitude;
      outPosition /= octave_scale;
      octave_scale *= lacunarity;
      octave_amplitude *= roughness;
    }
    else {
      float remainder = detail - floor(detail);
      if (remainder != 0.0) {
        max_amplitude += max_distance * octave_amplitude;
        float lerp_distance = outDistance + octave_distance * octave_amplitude;
        outDistance = (1.0 - remainder) * outDistance + remainder * lerp_distance;
        outPosition /= octave_scale;
      }
    }
  }
}

void fractal_voronoi_smooth_f1(vector3 coord,
                               float detail,
                               float roughness,
                               float lacunarity,
                               float smoothness,
                               float exponent,
                               float randomness,
                               string metric,
                               float max_distance,
                               output float max_amplitude,
                               output float outDistance,
                               output color outColor,
                               output vector3 outPosition)
{
  voronoi_smooth_f1(
      coord, smoothness, exponent, randomness, metric, outDistance, outColor, outPosition);
  max_amplitude = max_distance;
}

void fractal_voronoi_f2(vector3 coord,
                        float detail,
                        float roughness,
                        float lacunarity,
                        float exponent,
                        float randomness,
                        string metric,
                        float max_distance,
                        output float max_amplitude,
                        output float outDistance,
                        output color outColor,
                        output vector3 outPosition)
{
  voronoi_f2(coord, exponent, randomness, metric, outDistance, outColor, outPosition);
  max_amplitude = max_distance;
}

void fractal_voronoi_distance_to_edge(vector3 coord,
                                      float detail,
                                      float lacunarity,
                                      float randomness,
                                      float normalize,
                                      output float outDistance)
{
  voronoi_distance_to_edge(coord, randomness, outDistance);
}

/* **** 4D Fractal Voronoi **** */

void fractal_voronoi_f1(vector4 coord,
                        float detail,
                        float roughness,
                        float lacunarity,
                        float exponent,
                        float randomness,
                        string metric,
                        float max_distance,
                        output float max_amplitude,
                        output float outDistance,
                        output color outColor,
                        output vector4 outPosition)
{
  voronoi_f1(coord, exponent, randomness, metric, outDistance, outColor, outPosition);
  max_amplitude = max_distance;
}

void fractal_voronoi_smooth_f1(vector4 coord,
                               float detail,
                               float roughness,
                               float lacunarity,
                               float smoothness,
                               float exponent,
                               float randomness,
                               string metric,
                               float max_distance,
                               output float max_amplitude,
                               output float outDistance,
                               output color outColor,
                               output vector4 outPosition)
{
  voronoi_smooth_f1(
      coord, smoothness, exponent, randomness, metric, outDistance, outColor, outPosition);
  max_amplitude = max_distance;
}

void fractal_voronoi_f2(vector4 coord,
                        float detail,
                        float roughness,
                        float lacunarity,
                        float exponent,
                        float randomness,
                        string metric,
                        float max_distance,
                        output float max_amplitude,
                        output float outDistance,
                        output color outColor,
                        output vector4 outPosition)
{
  voronoi_f2(coord, exponent, randomness, metric, outDistance, outColor, outPosition);
  max_amplitude = max_distance;
}

void fractal_voronoi_distance_to_edge(vector4 coord,
                                      float detail,
                                      float lacunarity,
                                      float randomness,
                                      float normalize,
                                      output float outDistance)
{
  voronoi_distance_to_edge(coord, randomness, outDistance);
}
