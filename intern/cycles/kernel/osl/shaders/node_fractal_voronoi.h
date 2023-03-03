/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include "node_voronoi.h"
#include "stdcycles.h"
#include "vector2.h"
#include "vector4.h"

#define vector3 point

/* The Fractal Voronoi functions are all exactly the same in different dimensions except for the
 * input type. */

/* **** 1D Fractal Voronoi **** */

void fractal_voronoi_f1(float coord,
                        float detail,
                        float roughness,
                        float lacunarity,
                        float exponent,
                        float randomness,
                        string metric,
                        output float max_amplitude,
                        output float outDistance,
                        output color outColor,
                        output float outPosition)
{
  float octave_scale = 1.0;
  float octave_amplitude = 1.0;
  float octave_distance = 0.0;
  color octave_color;
  float octave_postion;

  max_amplitude = 0.0;
  outDistance = 0.0;
  for (int i = 0; i <= ceil(detail); ++i) {
    voronoi_f1(coord * octave_scale,
               exponent,
               randomness,
               metric,
               octave_distance,
               octave_color,
               octave_postion);
    if (detail == 0.0 || roughness == 0.0 || lacunarity == 0.0) {
      max_amplitude = 1.0;
      outDistance = octave_distance;
      outColor = octave_color;
      outPosition = octave_postion;
      return;
    }
    else if (i <= detail) {
      max_amplitude += octave_amplitude;
      outDistance += octave_distance * octave_amplitude;
      outColor += octave_color * octave_amplitude;
      outPosition = mix(outPosition, octave_postion / octave_scale, octave_amplitude);
      octave_scale *= lacunarity;
      octave_amplitude *= roughness;
    }
    else {
      float remainder = detail - floor(detail);
      if (remainder != 0.0) {
        max_amplitude = mix(max_amplitude, max_amplitude + octave_amplitude, remainder);
        outDistance = mix(
            outDistance, outDistance + octave_distance * octave_amplitude, remainder);
        outColor = mix(outColor, outColor + octave_color * octave_amplitude, remainder);
        outPosition = mix(outPosition,
                          mix(outPosition, octave_postion / octave_scale, octave_amplitude),
                          remainder);
      }
    }
  }
}

void fractal_voronoi_smooth_f1(float coord,
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
                               output float outPosition)
{
  float octave_scale = 1.0;
  float octave_amplitude = 1.0;
  float octave_distance = 0.0;
  color octave_color;
  float octave_postion;

  max_amplitude = 0.0;
  outDistance = 0.0;
  for (int i = 0; i <= ceil(detail); ++i) {
    voronoi_smooth_f1(coord * octave_scale,
                      smoothness,
                      exponent,
                      randomness,
                      metric,
                      octave_distance,
                      octave_color,
                      octave_postion);
    if (detail == 0.0 || roughness == 0.0 || lacunarity == 0.0) {
      max_amplitude = 1.0;
      outDistance = octave_distance;
      outColor = octave_color;
      outPosition = octave_postion;
      return;
    }
    else if (i <= detail) {
      max_amplitude += octave_amplitude;
      outDistance += octave_distance * octave_amplitude;
      outColor += octave_color * octave_amplitude;
      outPosition = mix(outPosition, octave_postion / octave_scale, octave_amplitude);
      octave_scale *= lacunarity;
      octave_amplitude *= roughness;
    }
    else {
      float remainder = detail - floor(detail);
      if (remainder != 0.0) {
        max_amplitude = mix(max_amplitude, max_amplitude + octave_amplitude, remainder);
        outDistance = mix(
            outDistance, outDistance + octave_distance * octave_amplitude, remainder);
        outColor = mix(outColor, outColor + octave_color * octave_amplitude, remainder);
        outPosition = mix(outPosition,
                          mix(outPosition, octave_postion / octave_scale, octave_amplitude),
                          remainder);
      }
    }
  }
}

void fractal_voronoi_f2(float coord,
                        float detail,
                        float roughness,
                        float lacunarity,
                        float exponent,
                        float randomness,
                        string metric,
                        output float max_amplitude,
                        output float outDistance,
                        output color outColor,
                        output float outPosition)
{
  float octave_scale = 1.0;
  float octave_amplitude = 1.0;
  float octave_distance = 0.0;
  color octave_color;
  float octave_postion;

  max_amplitude = 0.0;
  outDistance = 0.0;
  for (int i = 0; i <= ceil(detail); ++i) {
    voronoi_f2(coord * octave_scale,
               exponent,
               randomness,
               metric,
               octave_distance,
               octave_color,
               octave_postion);
    if (detail == 0.0 || roughness == 0.0 || lacunarity == 0.0) {
      max_amplitude = 1.0;
      outDistance = octave_distance;
      outColor = octave_color;
      outPosition = octave_postion;
      return;
    }
    else if (i <= detail) {
      max_amplitude += octave_amplitude;
      outDistance += octave_distance * octave_amplitude;
      outColor += octave_color * octave_amplitude;
      outPosition = mix(outPosition, octave_postion / octave_scale, octave_amplitude);
      octave_scale *= lacunarity;
      octave_amplitude *= roughness;
    }
    else {
      float remainder = detail - floor(detail);
      if (remainder != 0.0) {
        max_amplitude = mix(max_amplitude, max_amplitude + octave_amplitude, remainder);
        outDistance = mix(
            outDistance, outDistance + octave_distance * octave_amplitude, remainder);
        outColor = mix(outColor, outColor + octave_color * octave_amplitude, remainder);
        outPosition = mix(outPosition,
                          mix(outPosition, octave_postion / octave_scale, octave_amplitude),
                          remainder);
      }
    }
  }
}

void fractal_voronoi_distance_to_edge(float coord,
                                      float detail,
                                      float roughness,
                                      float lacunarity,
                                      float randomness,
                                      output float max_amplitude,
                                      output float outDistance)
{
  float octave_scale = 1.0;
  float octave_amplitude = 1.0;
  float octave_distance = 0.0;

  max_amplitude = 2.0 - randomness;
  outDistance = 8.0;
  for (int i = 0; i <= ceil(detail); ++i) {
    voronoi_distance_to_edge(coord * octave_scale, randomness, octave_distance);
    if (detail == 0.0 || roughness == 0.0 || lacunarity == 0.0) {
      outDistance = octave_distance;
      return;
    }
    else if (i <= detail) {
      max_amplitude = mix(max_amplitude, (2.0 - randomness) * octave_scale, octave_amplitude);
      outDistance = mix(
          outDistance, min(outDistance, octave_distance / octave_scale), octave_amplitude);
      octave_scale *= lacunarity;
      octave_amplitude *= roughness;
    }
    else {
      float remainder = detail - floor(detail);
      if (remainder != 0.0) {
        float lerp_amplitude = mix(
            max_amplitude, (2.0 - randomness) * octave_scale, octave_amplitude);
        max_amplitude = mix(max_amplitude, lerp_amplitude, remainder);
        float lerp_distance = mix(
            outDistance, min(outDistance, octave_distance / octave_scale), octave_amplitude);
        outDistance = mix(outDistance, min(outDistance, lerp_distance), remainder);
      }
    }
  }
}

/* **** 2D Fractal Voronoi **** */

void fractal_voronoi_f1(vector2 coord,
                        float detail,
                        float roughness,
                        float lacunarity,
                        float exponent,
                        float randomness,
                        string metric,
                        output float max_amplitude,
                        output float outDistance,
                        output color outColor,
                        output vector2 outPosition)
{
  float octave_scale = 1.0;
  float octave_amplitude = 1.0;
  float octave_distance = 0.0;
  color octave_color;
  vector2 octave_postion;

  max_amplitude = 0.0;
  outDistance = 0.0;
  for (int i = 0; i <= ceil(detail); ++i) {
    voronoi_f1(coord * octave_scale,
               exponent,
               randomness,
               metric,
               octave_distance,
               octave_color,
               octave_postion);
    if (detail == 0.0 || roughness == 0.0 || lacunarity == 0.0) {
      max_amplitude = 1.0;
      outDistance = octave_distance;
      outColor = octave_color;
      outPosition = octave_postion;
      return;
    }
    else if (i <= detail) {
      max_amplitude += octave_amplitude;
      outDistance += octave_distance * octave_amplitude;
      outColor += octave_color * octave_amplitude;
      outPosition = mix(outPosition, octave_postion / octave_scale, octave_amplitude);
      octave_scale *= lacunarity;
      octave_amplitude *= roughness;
    }
    else {
      float remainder = detail - floor(detail);
      if (remainder != 0.0) {
        max_amplitude = mix(max_amplitude, max_amplitude + octave_amplitude, remainder);
        outDistance = mix(
            outDistance, outDistance + octave_distance * octave_amplitude, remainder);
        outColor = mix(outColor, outColor + octave_color * octave_amplitude, remainder);
        outPosition = mix(outPosition,
                          mix(outPosition, octave_postion / octave_scale, octave_amplitude),
                          remainder);
      }
    }
  }
}

void fractal_voronoi_smooth_f1(vector2 coord,
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
                               output vector2 outPosition)
{
  float octave_scale = 1.0;
  float octave_amplitude = 1.0;
  float octave_distance = 0.0;
  color octave_color;
  vector2 octave_postion;

  max_amplitude = 0.0;
  outDistance = 0.0;
  for (int i = 0; i <= ceil(detail); ++i) {
    voronoi_smooth_f1(coord * octave_scale,
                      smoothness,
                      exponent,
                      randomness,
                      metric,
                      octave_distance,
                      octave_color,
                      octave_postion);
    if (detail == 0.0 || roughness == 0.0 || lacunarity == 0.0) {
      max_amplitude = 1.0;
      outDistance = octave_distance;
      outColor = octave_color;
      outPosition = octave_postion;
      return;
    }
    else if (i <= detail) {
      max_amplitude += octave_amplitude;
      outDistance += octave_distance * octave_amplitude;
      outColor += octave_color * octave_amplitude;
      outPosition = mix(outPosition, octave_postion / octave_scale, octave_amplitude);
      octave_scale *= lacunarity;
      octave_amplitude *= roughness;
    }
    else {
      float remainder = detail - floor(detail);
      if (remainder != 0.0) {
        max_amplitude = mix(max_amplitude, max_amplitude + octave_amplitude, remainder);
        outDistance = mix(
            outDistance, outDistance + octave_distance * octave_amplitude, remainder);
        outColor = mix(outColor, outColor + octave_color * octave_amplitude, remainder);
        outPosition = mix(outPosition,
                          mix(outPosition, octave_postion / octave_scale, octave_amplitude),
                          remainder);
      }
    }
  }
}

void fractal_voronoi_f2(vector2 coord,
                        float detail,
                        float roughness,
                        float lacunarity,
                        float exponent,
                        float randomness,
                        string metric,
                        output float max_amplitude,
                        output float outDistance,
                        output color outColor,
                        output vector2 outPosition)
{
  float octave_scale = 1.0;
  float octave_amplitude = 1.0;
  float octave_distance = 0.0;
  color octave_color;
  vector2 octave_postion;

  max_amplitude = 0.0;
  outDistance = 0.0;
  for (int i = 0; i <= ceil(detail); ++i) {
    voronoi_f2(coord * octave_scale,
               exponent,
               randomness,
               metric,
               octave_distance,
               octave_color,
               octave_postion);
    if (detail == 0.0 || roughness == 0.0 || lacunarity == 0.0) {
      max_amplitude = 1.0;
      outDistance = octave_distance;
      outColor = octave_color;
      outPosition = octave_postion;
      return;
    }
    else if (i <= detail) {
      max_amplitude += octave_amplitude;
      outDistance += octave_distance * octave_amplitude;
      outColor += octave_color * octave_amplitude;
      outPosition = mix(outPosition, octave_postion / octave_scale, octave_amplitude);
      octave_scale *= lacunarity;
      octave_amplitude *= roughness;
    }
    else {
      float remainder = detail - floor(detail);
      if (remainder != 0.0) {
        max_amplitude = mix(max_amplitude, max_amplitude + octave_amplitude, remainder);
        outDistance = mix(
            outDistance, outDistance + octave_distance * octave_amplitude, remainder);
        outColor = mix(outColor, outColor + octave_color * octave_amplitude, remainder);
        outPosition = mix(outPosition,
                          mix(outPosition, octave_postion / octave_scale, octave_amplitude),
                          remainder);
      }
    }
  }
}

void fractal_voronoi_distance_to_edge(vector2 coord,
                                      float detail,
                                      float roughness,
                                      float lacunarity,
                                      float randomness,
                                      output float max_amplitude,
                                      output float outDistance)
{
  float octave_scale = 1.0;
  float octave_amplitude = 1.0;
  float octave_distance = 0.0;

  max_amplitude = 2.0 - randomness;
  outDistance = 8.0;
  for (int i = 0; i <= ceil(detail); ++i) {
    voronoi_distance_to_edge(coord * octave_scale, randomness, octave_distance);
    if (detail == 0.0 || roughness == 0.0 || lacunarity == 0.0) {
      outDistance = octave_distance;
      return;
    }
    else if (i <= detail) {
      max_amplitude = mix(max_amplitude, (2.0 - randomness) * octave_scale, octave_amplitude);
      outDistance = mix(
          outDistance, min(outDistance, octave_distance / octave_scale), octave_amplitude);
      octave_scale *= lacunarity;
      octave_amplitude *= roughness;
    }
    else {
      float remainder = detail - floor(detail);
      if (remainder != 0.0) {
        float lerp_amplitude = mix(
            max_amplitude, (2.0 - randomness) * octave_scale, octave_amplitude);
        max_amplitude = mix(max_amplitude, lerp_amplitude, remainder);
        float lerp_distance = mix(
            outDistance, min(outDistance, octave_distance / octave_scale), octave_amplitude);
        outDistance = mix(outDistance, min(outDistance, lerp_distance), remainder);
      }
    }
  }
}

/* **** 3D Fractal Voronoi **** */

void fractal_voronoi_f1(vector3 coord,
                        float detail,
                        float roughness,
                        float lacunarity,
                        float exponent,
                        float randomness,
                        string metric,
                        output float max_amplitude,
                        output float outDistance,
                        output color outColor,
                        output vector3 outPosition)
{
  float octave_scale = 1.0;
  float octave_amplitude = 1.0;
  float octave_distance = 0.0;
  color octave_color;
  vector3 octave_postion;

  max_amplitude = 0.0;
  outDistance = 0.0;
  for (int i = 0; i <= ceil(detail); ++i) {
    voronoi_f1(coord * octave_scale,
               exponent,
               randomness,
               metric,
               octave_distance,
               octave_color,
               octave_postion);
    if (detail == 0.0 || roughness == 0.0 || lacunarity == 0.0) {
      max_amplitude = 1.0;
      outDistance = octave_distance;
      outColor = octave_color;
      outPosition = octave_postion;
      return;
    }
    else if (i <= detail) {
      max_amplitude += octave_amplitude;
      outDistance += octave_distance * octave_amplitude;
      outColor += octave_color * octave_amplitude;
      outPosition = mix(outPosition, octave_postion / octave_scale, octave_amplitude);
      octave_scale *= lacunarity;
      octave_amplitude *= roughness;
    }
    else {
      float remainder = detail - floor(detail);
      if (remainder != 0.0) {
        max_amplitude = mix(max_amplitude, max_amplitude + octave_amplitude, remainder);
        outDistance = mix(
            outDistance, outDistance + octave_distance * octave_amplitude, remainder);
        outColor = mix(outColor, outColor + octave_color * octave_amplitude, remainder);
        outPosition = mix(outPosition,
                          mix(outPosition, octave_postion / octave_scale, octave_amplitude),
                          remainder);
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
                               output float max_amplitude,
                               output float outDistance,
                               output color outColor,
                               output vector3 outPosition)
{
  float octave_scale = 1.0;
  float octave_amplitude = 1.0;
  float octave_distance = 0.0;
  color octave_color;
  vector3 octave_postion;

  max_amplitude = 0.0;
  outDistance = 0.0;
  for (int i = 0; i <= ceil(detail); ++i) {
    voronoi_smooth_f1(coord * octave_scale,
                      smoothness,
                      exponent,
                      randomness,
                      metric,
                      octave_distance,
                      octave_color,
                      octave_postion);
    if (detail == 0.0 || roughness == 0.0 || lacunarity == 0.0) {
      max_amplitude = 1.0;
      outDistance = octave_distance;
      outColor = octave_color;
      outPosition = octave_postion;
      return;
    }
    else if (i <= detail) {
      max_amplitude += octave_amplitude;
      outDistance += octave_distance * octave_amplitude;
      outColor += octave_color * octave_amplitude;
      outPosition = mix(outPosition, octave_postion / octave_scale, octave_amplitude);
      octave_scale *= lacunarity;
      octave_amplitude *= roughness;
    }
    else {
      float remainder = detail - floor(detail);
      if (remainder != 0.0) {
        max_amplitude = mix(max_amplitude, max_amplitude + octave_amplitude, remainder);
        outDistance = mix(
            outDistance, outDistance + octave_distance * octave_amplitude, remainder);
        outColor = mix(outColor, outColor + octave_color * octave_amplitude, remainder);
        outPosition = mix(outPosition,
                          mix(outPosition, octave_postion / octave_scale, octave_amplitude),
                          remainder);
      }
    }
  }
}

void fractal_voronoi_f2(vector3 coord,
                        float detail,
                        float roughness,
                        float lacunarity,
                        float exponent,
                        float randomness,
                        string metric,
                        output float max_amplitude,
                        output float outDistance,
                        output color outColor,
                        output vector3 outPosition)
{
  float octave_scale = 1.0;
  float octave_amplitude = 1.0;
  float octave_distance = 0.0;
  color octave_color;
  vector3 octave_postion;

  max_amplitude = 0.0;
  outDistance = 0.0;
  for (int i = 0; i <= ceil(detail); ++i) {
    voronoi_f2(coord * octave_scale,
               exponent,
               randomness,
               metric,
               octave_distance,
               octave_color,
               octave_postion);
    if (detail == 0.0 || roughness == 0.0 || lacunarity == 0.0) {
      max_amplitude = 1.0;
      outDistance = octave_distance;
      outColor = octave_color;
      outPosition = octave_postion;
      return;
    }
    else if (i <= detail) {
      max_amplitude += octave_amplitude;
      outDistance += octave_distance * octave_amplitude;
      outColor += octave_color * octave_amplitude;
      outPosition = mix(outPosition, octave_postion / octave_scale, octave_amplitude);
      octave_scale *= lacunarity;
      octave_amplitude *= roughness;
    }
    else {
      float remainder = detail - floor(detail);
      if (remainder != 0.0) {
        max_amplitude = mix(max_amplitude, max_amplitude + octave_amplitude, remainder);
        outDistance = mix(
            outDistance, outDistance + octave_distance * octave_amplitude, remainder);
        outColor = mix(outColor, outColor + octave_color * octave_amplitude, remainder);
        outPosition = mix(outPosition,
                          mix(outPosition, octave_postion / octave_scale, octave_amplitude),
                          remainder);
      }
    }
  }
}

void fractal_voronoi_distance_to_edge(vector3 coord,
                                      float detail,
                                      float roughness,
                                      float lacunarity,
                                      float randomness,
                                      output float max_amplitude,
                                      output float outDistance)
{
  float octave_scale = 1.0;
  float octave_amplitude = 1.0;
  float octave_distance = 0.0;

  max_amplitude = 2.0 - randomness;
  outDistance = 8.0;
  for (int i = 0; i <= ceil(detail); ++i) {
    voronoi_distance_to_edge(coord * octave_scale, randomness, octave_distance);
    if (detail == 0.0 || roughness == 0.0 || lacunarity == 0.0) {
      outDistance = octave_distance;
      return;
    }
    else if (i <= detail) {
      max_amplitude = mix(max_amplitude, (2.0 - randomness) * octave_scale, octave_amplitude);
      outDistance = mix(
          outDistance, min(outDistance, octave_distance / octave_scale), octave_amplitude);
      octave_scale *= lacunarity;
      octave_amplitude *= roughness;
    }
    else {
      float remainder = detail - floor(detail);
      if (remainder != 0.0) {
        float lerp_amplitude = mix(
            max_amplitude, (2.0 - randomness) * octave_scale, octave_amplitude);
        max_amplitude = mix(max_amplitude, lerp_amplitude, remainder);
        float lerp_distance = mix(
            outDistance, min(outDistance, octave_distance / octave_scale), octave_amplitude);
        outDistance = mix(outDistance, min(outDistance, lerp_distance), remainder);
      }
    }
  }
}

/* **** 4D Fractal Voronoi **** */

void fractal_voronoi_f1(vector4 coord,
                        float detail,
                        float roughness,
                        float lacunarity,
                        float exponent,
                        float randomness,
                        string metric,
                        output float max_amplitude,
                        output float outDistance,
                        output color outColor,
                        output vector4 outPosition)
{
  float octave_scale = 1.0;
  float octave_amplitude = 1.0;
  float octave_distance = 0.0;
  color octave_color;
  vector4 octave_postion;

  max_amplitude = 0.0;
  outDistance = 0.0;
  for (int i = 0; i <= ceil(detail); ++i) {
    voronoi_f1(coord * octave_scale,
               exponent,
               randomness,
               metric,
               octave_distance,
               octave_color,
               octave_postion);
    if (detail == 0.0 || roughness == 0.0 || lacunarity == 0.0) {
      max_amplitude = 1.0;
      outDistance = octave_distance;
      outColor = octave_color;
      outPosition = octave_postion;
      return;
    }
    else if (i <= detail) {
      max_amplitude += octave_amplitude;
      outDistance += octave_distance * octave_amplitude;
      outColor += octave_color * octave_amplitude;
      outPosition = mix(outPosition, octave_postion / octave_scale, octave_amplitude);
      octave_scale *= lacunarity;
      octave_amplitude *= roughness;
    }
    else {
      float remainder = detail - floor(detail);
      if (remainder != 0.0) {
        max_amplitude = mix(max_amplitude, max_amplitude + octave_amplitude, remainder);
        outDistance = mix(
            outDistance, outDistance + octave_distance * octave_amplitude, remainder);
        outColor = mix(outColor, outColor + octave_color * octave_amplitude, remainder);
        outPosition = mix(outPosition,
                          mix(outPosition, octave_postion / octave_scale, octave_amplitude),
                          remainder);
      }
    }
  }
}

void fractal_voronoi_smooth_f1(vector4 coord,
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
                               output vector4 outPosition)
{
  float octave_scale = 1.0;
  float octave_amplitude = 1.0;
  float octave_distance = 0.0;
  color octave_color;
  vector4 octave_postion;

  max_amplitude = 0.0;
  outDistance = 0.0;
  for (int i = 0; i <= ceil(detail); ++i) {
    voronoi_smooth_f1(coord * octave_scale,
                      smoothness,
                      exponent,
                      randomness,
                      metric,
                      octave_distance,
                      octave_color,
                      octave_postion);
    if (detail == 0.0 || roughness == 0.0 || lacunarity == 0.0) {
      max_amplitude = 1.0;
      outDistance = octave_distance;
      outColor = octave_color;
      outPosition = octave_postion;
      return;
    }
    else if (i <= detail) {
      max_amplitude += octave_amplitude;
      outDistance += octave_distance * octave_amplitude;
      outColor += octave_color * octave_amplitude;
      outPosition = mix(outPosition, octave_postion / octave_scale, octave_amplitude);
      octave_scale *= lacunarity;
      octave_amplitude *= roughness;
    }
    else {
      float remainder = detail - floor(detail);
      if (remainder != 0.0) {
        max_amplitude = mix(max_amplitude, max_amplitude + octave_amplitude, remainder);
        outDistance = mix(
            outDistance, outDistance + octave_distance * octave_amplitude, remainder);
        outColor = mix(outColor, outColor + octave_color * octave_amplitude, remainder);
        outPosition = mix(outPosition,
                          mix(outPosition, octave_postion / octave_scale, octave_amplitude),
                          remainder);
      }
    }
  }
}

void fractal_voronoi_f2(vector4 coord,
                        float detail,
                        float roughness,
                        float lacunarity,
                        float exponent,
                        float randomness,
                        string metric,
                        output float max_amplitude,
                        output float outDistance,
                        output color outColor,
                        output vector4 outPosition)
{
  float octave_scale = 1.0;
  float octave_amplitude = 1.0;
  float octave_distance = 0.0;
  color octave_color;
  vector4 octave_postion;

  max_amplitude = 0.0;
  outDistance = 0.0;
  for (int i = 0; i <= ceil(detail); ++i) {
    voronoi_f2(coord * octave_scale,
               exponent,
               randomness,
               metric,
               octave_distance,
               octave_color,
               octave_postion);
    if (detail == 0.0 || roughness == 0.0 || lacunarity == 0.0) {
      max_amplitude = 1.0;
      outDistance = octave_distance;
      outColor = octave_color;
      outPosition = octave_postion;
      return;
    }
    else if (i <= detail) {
      max_amplitude += octave_amplitude;
      outDistance += octave_distance * octave_amplitude;
      outColor += octave_color * octave_amplitude;
      outPosition = mix(outPosition, octave_postion / octave_scale, octave_amplitude);
      octave_scale *= lacunarity;
      octave_amplitude *= roughness;
    }
    else {
      float remainder = detail - floor(detail);
      if (remainder != 0.0) {
        max_amplitude = mix(max_amplitude, max_amplitude + octave_amplitude, remainder);
        outDistance = mix(
            outDistance, outDistance + octave_distance * octave_amplitude, remainder);
        outColor = mix(outColor, outColor + octave_color * octave_amplitude, remainder);
        outPosition = mix(outPosition,
                          mix(outPosition, octave_postion / octave_scale, octave_amplitude),
                          remainder);
      }
    }
  }
}

void fractal_voronoi_distance_to_edge(vector4 coord,
                                      float detail,
                                      float roughness,
                                      float lacunarity,
                                      float randomness,
                                      output float max_amplitude,
                                      output float outDistance)
{
  float octave_scale = 1.0;
  float octave_amplitude = 1.0;
  float octave_distance = 0.0;

  max_amplitude = 2.0 - randomness;
  outDistance = 8.0;
  for (int i = 0; i <= ceil(detail); ++i) {
    voronoi_distance_to_edge(coord * octave_scale, randomness, octave_distance);
    if (detail == 0.0 || roughness == 0.0 || lacunarity == 0.0) {
      outDistance = octave_distance;
      return;
    }
    else if (i <= detail) {
      max_amplitude = mix(max_amplitude, (2.0 - randomness) * octave_scale, octave_amplitude);
      outDistance = mix(
          outDistance, min(outDistance, octave_distance / octave_scale), octave_amplitude);
      octave_scale *= lacunarity;
      octave_amplitude *= roughness;
    }
    else {
      float remainder = detail - floor(detail);
      if (remainder != 0.0) {
        float lerp_amplitude = mix(
            max_amplitude, (2.0 - randomness) * octave_scale, octave_amplitude);
        max_amplitude = mix(max_amplitude, lerp_amplitude, remainder);
        float lerp_distance = mix(
            outDistance, min(outDistance, octave_distance / octave_scale), octave_amplitude);
        outDistance = mix(outDistance, min(outDistance, lerp_distance), remainder);
      }
    }
  }
}
