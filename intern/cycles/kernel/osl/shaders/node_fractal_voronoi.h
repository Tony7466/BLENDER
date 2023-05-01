/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include "node_voronoi.h"
#include "stdcycles.h"
#include "vector2.h"
#include "vector4.h"

#define vector3 point

/* **** Normalization **** */

void normalize_voronoi_x_fx(VoronoiParams params,
                            output VoronoiOutput Output,
                            float max_amplitude,
                            int zero_input)
{
  if (params.feature == "f2") {
    if (zero_input) {
      Output.Distance /= (1.0 - params.randomness) +
                         params.randomness * max_amplitude * params.max_distance * 1.5;
    }
    else {
      Output.Distance /= (1.0 - params.randomness) * ceil(params.detail + 1.0) +
                         params.randomness * max_amplitude * params.max_distance * 1.5;
    }
  }
  else {
    Output.Distance /= (0.5 + 0.5 * params.randomness) * max_amplitude * params.max_distance;
  }
  Output.Color /= max_amplitude;
}

/* **** 1D Fractal Voronoi **** */

VoronoiOutput fractal_voronoi_x_fx(VoronoiParams params, float coord)
{
  float amplitude = 1.0;
  float max_amplitude = 0.0;
  float scale = 1.0;

  VoronoiOutput Output;
  Output.Distance = 0.0;
  Output.Color = color(0.0, 0.0, 0.0);
  Output.Position = vector4(0.0, 0.0, 0.0, 0.0);
  int zero_input = params.detail == 0.0 || params.roughness == 0.0 || params.lacunarity == 0.0;

  for (int i = 0; i <= ceil(params.detail); ++i) {
    VoronoiOutput octave;
    if (params.feature == "f1") {
      octave = voronoi_f1(params, coord * scale);
    }
    else if (params.feature == "smooth_f1") {
      octave = voronoi_smooth_f1(params, coord * scale);
    }
    else {
      octave = voronoi_f2(params, coord * scale);
    }

    if (zero_input) {
      max_amplitude = 1.0;
      Output.Distance = octave.Distance;
      Output.Color = octave.Color;
      Output.Position = octave.Position;
      break;
    }
    else if (i <= params.detail) {
      max_amplitude += amplitude;
      Output.Distance += octave.Distance * amplitude;
      Output.Color += octave.Color * amplitude;
      Output.Position = mix(Output.Position, octave.Position / scale, amplitude);
      scale *= params.lacunarity;
      amplitude *= params.roughness;
    }
    else {
      float remainder = params.detail - floor(params.detail);
      if (remainder != 0.0) {
        max_amplitude = mix(max_amplitude, max_amplitude + amplitude, remainder);
        Output.Distance = mix(
            Output.Distance, Output.Distance + octave.Distance * amplitude, remainder);
        Output.Color = mix(Output.Color, Output.Color + octave.Color * amplitude, remainder);
        Output.Position = mix(
            Output.Position, mix(Output.Position, octave.Position / scale, amplitude), remainder);
      }
    }
  }

  if (params.normalize) {
    normalize_voronoi_x_fx(params, Output, max_amplitude, zero_input);
  }

  Output.Position = safe_divide(Output.Position, params.scale);

  return Output;
}

float fractal_voronoi_distance_to_edge(VoronoiParams params, float coord)
{
  float amplitude = 1.0;
  float max_amplitude = 1.5 - 0.5 * params.randomness;
  float scale = 1.0;
  float distance = 8.0;

  int zero_input = params.detail == 0.0 || params.roughness == 0.0 || params.lacunarity == 0.0;

  for (int i = 0; i <= ceil(params.detail); ++i) {
    float octave_distance = voronoi_distance_to_edge(params, coord * scale);

    if (zero_input) {
      distance = octave_distance;
      break;
    }
    else if (i <= params.detail) {
      max_amplitude = mix(max_amplitude, (1.5 - 0.5 * params.randomness) * scale, amplitude);
      distance = mix(distance, min(distance, octave_distance / scale), amplitude);
      scale *= params.lacunarity;
      amplitude *= params.roughness;
    }
    else {
      float remainder = params.detail - floor(params.detail);
      if (remainder != 0.0) {
        float lerp_amplitude = mix(max_amplitude, (1.5 - 0.5 * params.randomness) * scale, amplitude);
        max_amplitude = mix(max_amplitude, lerp_amplitude, remainder);
        float lerp_distance = mix(distance, min(distance, octave_distance / scale), amplitude);
        distance = mix(distance, min(distance, lerp_distance), remainder);
      }
    }
  }

  if (params.normalize) {
    /* max_amplitude is used here to keep the code consistent, however it has a different
     * meaning than in F1, Smooth F1 and F2. Instead of the highest possible amplitude, it
     * represents an abstract factor needed to cancel out the amplitude attenuation caused by the
     * higher layers. */
    distance *= max_amplitude;
  }

  return distance;
}

/* **** 2D Fractal Voronoi **** */

VoronoiOutput fractal_voronoi_x_fx(VoronoiParams params, vector2 coord)
{
  float amplitude = 1.0;
  float max_amplitude = 0.0;
  float scale = 1.0;

  VoronoiOutput Output;
  Output.Distance = 0.0;
  Output.Color = color(0.0, 0.0, 0.0);
  Output.Position = vector4(0.0, 0.0, 0.0, 0.0);
  int zero_input = params.detail == 0.0 || params.roughness == 0.0 || params.lacunarity == 0.0;

  for (int i = 0; i <= ceil(params.detail); ++i) {
    VoronoiOutput octave;
    if (params.feature == "f1") {
      octave = voronoi_f1(params, coord * scale);
    }
    else if (params.feature == "smooth_f1") {
      octave = voronoi_smooth_f1(params, coord * scale);
    }
    else {
      octave = voronoi_f2(params, coord * scale);
    }

    if (zero_input) {
      max_amplitude = 1.0;
      Output.Distance = octave.Distance;
      Output.Color = octave.Color;
      Output.Position = octave.Position;
      break;
    }
    else if (i <= params.detail) {
      max_amplitude += amplitude;
      Output.Distance += octave.Distance * amplitude;
      Output.Color += octave.Color * amplitude;
      Output.Position = mix(Output.Position, octave.Position / scale, amplitude);
      scale *= params.lacunarity;
      amplitude *= params.roughness;
    }
    else {
      float remainder = params.detail - floor(params.detail);
      if (remainder != 0.0) {
        max_amplitude = mix(max_amplitude, max_amplitude + amplitude, remainder);
        Output.Distance = mix(
            Output.Distance, Output.Distance + octave.Distance * amplitude, remainder);
        Output.Color = mix(Output.Color, Output.Color + octave.Color * amplitude, remainder);
        Output.Position = mix(
            Output.Position, mix(Output.Position, octave.Position / scale, amplitude), remainder);
      }
    }
  }

  if (params.normalize) {
    normalize_voronoi_x_fx(params, Output, max_amplitude, zero_input);
  }

  Output.Position = safe_divide(Output.Position, params.scale);

  return Output;
}

float fractal_voronoi_distance_to_edge(VoronoiParams params, vector2 coord)
{
  float amplitude = 1.0;
  float max_amplitude = 1.5 - 0.5 * params.randomness;
  float scale = 1.0;
  float distance = 8.0;

  int zero_input = params.detail == 0.0 || params.roughness == 0.0 || params.lacunarity == 0.0;

  for (int i = 0; i <= ceil(params.detail); ++i) {
    float octave_distance = voronoi_distance_to_edge(params, coord * scale);

    if (zero_input) {
      distance = octave_distance;
      break;
    }
    else if (i <= params.detail) {
      max_amplitude = mix(max_amplitude, (1.5 - 0.5 * params.randomness) * scale, amplitude);
      distance = mix(distance, min(distance, octave_distance / scale), amplitude);
      scale *= params.lacunarity;
      amplitude *= params.roughness;
    }
    else {
      float remainder = params.detail - floor(params.detail);
      if (remainder != 0.0) {
        float lerp_amplitude = mix(max_amplitude, (1.5 - 0.5 * params.randomness) * scale, amplitude);
        max_amplitude = mix(max_amplitude, lerp_amplitude, remainder);
        float lerp_distance = mix(distance, min(distance, octave_distance / scale), amplitude);
        distance = mix(distance, min(distance, lerp_distance), remainder);
      }
    }
  }

  if (params.normalize) {
    /* max_amplitude is used here to keep the code consistent, however it has a different
     * meaning than in F1, Smooth F1 and F2. Instead of the highest possible amplitude, it
     * represents an abstract factor needed to cancel out the amplitude attenuation caused by the
     * higher layers. */
    distance *= max_amplitude;
  }

  return distance;
}

/* **** 3D Fractal Voronoi **** */

VoronoiOutput fractal_voronoi_x_fx(VoronoiParams params, vector3 coord)
{
  float amplitude = 1.0;
  float max_amplitude = 0.0;
  float scale = 1.0;

  VoronoiOutput Output;
  Output.Distance = 0.0;
  Output.Color = color(0.0, 0.0, 0.0);
  Output.Position = vector4(0.0, 0.0, 0.0, 0.0);
  int zero_input = params.detail == 0.0 || params.roughness == 0.0 || params.lacunarity == 0.0;

  for (int i = 0; i <= ceil(params.detail); ++i) {
    VoronoiOutput octave;
    if (params.feature == "f1") {
      octave = voronoi_f1(params, coord * scale);
    }
    else if (params.feature == "smooth_f1") {
      octave = voronoi_smooth_f1(params, coord * scale);
    }
    else {
      octave = voronoi_f2(params, coord * scale);
    }

    if (zero_input) {
      max_amplitude = 1.0;
      Output.Distance = octave.Distance;
      Output.Color = octave.Color;
      Output.Position = octave.Position;
      break;
    }
    else if (i <= params.detail) {
      max_amplitude += amplitude;
      Output.Distance += octave.Distance * amplitude;
      Output.Color += octave.Color * amplitude;
      Output.Position = mix(Output.Position, octave.Position / scale, amplitude);
      scale *= params.lacunarity;
      amplitude *= params.roughness;
    }
    else {
      float remainder = params.detail - floor(params.detail);
      if (remainder != 0.0) {
        max_amplitude = mix(max_amplitude, max_amplitude + amplitude, remainder);
        Output.Distance = mix(
            Output.Distance, Output.Distance + octave.Distance * amplitude, remainder);
        Output.Color = mix(Output.Color, Output.Color + octave.Color * amplitude, remainder);
        Output.Position = mix(
            Output.Position, mix(Output.Position, octave.Position / scale, amplitude), remainder);
      }
    }
  }

  if (params.normalize) {
    normalize_voronoi_x_fx(params, Output, max_amplitude, zero_input);
  }

  Output.Position = safe_divide(Output.Position, params.scale);

  return Output;
}

float fractal_voronoi_distance_to_edge(VoronoiParams params, vector3 coord)
{
  float amplitude = 1.0;
  float max_amplitude = 1.5 - 0.5 * params.randomness;
  float scale = 1.0;
  float distance = 8.0;

  int zero_input = params.detail == 0.0 || params.roughness == 0.0 || params.lacunarity == 0.0;

  for (int i = 0; i <= ceil(params.detail); ++i) {
    float octave_distance = voronoi_distance_to_edge(params, coord * scale);

    if (zero_input) {
      distance = octave_distance;
      break;
    }
    else if (i <= params.detail) {
      max_amplitude = mix(max_amplitude, (1.5 - 0.5 * params.randomness) * scale, amplitude);
      distance = mix(distance, min(distance, octave_distance / scale), amplitude);
      scale *= params.lacunarity;
      amplitude *= params.roughness;
    }
    else {
      float remainder = params.detail - floor(params.detail);
      if (remainder != 0.0) {
        float lerp_amplitude = mix(max_amplitude, (1.5 - 0.5 * params.randomness) * scale, amplitude);
        max_amplitude = mix(max_amplitude, lerp_amplitude, remainder);
        float lerp_distance = mix(distance, min(distance, octave_distance / scale), amplitude);
        distance = mix(distance, min(distance, lerp_distance), remainder);
      }
    }
  }

  if (params.normalize) {
    /* max_amplitude is used here to keep the code consistent, however it has a different
     * meaning than in F1, Smooth F1 and F2. Instead of the highest possible amplitude, it
     * represents an abstract factor needed to cancel out the amplitude attenuation caused by the
     * higher layers. */
    distance *= max_amplitude;
  }

  return distance;
}

/* **** 4D Fractal Voronoi **** */

VoronoiOutput fractal_voronoi_x_fx(VoronoiParams params, vector4 coord)
{
  float amplitude = 1.0;
  float max_amplitude = 0.0;
  float scale = 1.0;

  VoronoiOutput Output;
  Output.Distance = 0.0;
  Output.Color = color(0.0, 0.0, 0.0);
  Output.Position = vector4(0.0, 0.0, 0.0, 0.0);
  int zero_input = params.detail == 0.0 || params.roughness == 0.0 || params.lacunarity == 0.0;

  for (int i = 0; i <= ceil(params.detail); ++i) {
    VoronoiOutput octave;
    if (params.feature == "f1") {
      octave = voronoi_f1(params, coord * scale);
    }
    else if (params.feature == "smooth_f1") {
      octave = voronoi_smooth_f1(params, coord * scale);
    }
    else {
      octave = voronoi_f2(params, coord * scale);
    }

    if (zero_input) {
      max_amplitude = 1.0;
      Output.Distance = octave.Distance;
      Output.Color = octave.Color;
      Output.Position = octave.Position;
      break;
    }
    else if (i <= params.detail) {
      max_amplitude += amplitude;
      Output.Distance += octave.Distance * amplitude;
      Output.Color += octave.Color * amplitude;
      Output.Position = mix(Output.Position, octave.Position / scale, amplitude);
      scale *= params.lacunarity;
      amplitude *= params.roughness;
    }
    else {
      float remainder = params.detail - floor(params.detail);
      if (remainder != 0.0) {
        max_amplitude = mix(max_amplitude, max_amplitude + amplitude, remainder);
        Output.Distance = mix(
            Output.Distance, Output.Distance + octave.Distance * amplitude, remainder);
        Output.Color = mix(Output.Color, Output.Color + octave.Color * amplitude, remainder);
        Output.Position = mix(
            Output.Position, mix(Output.Position, octave.Position / scale, amplitude), remainder);
      }
    }
  }

  if (params.normalize) {
    normalize_voronoi_x_fx(params, Output, max_amplitude, zero_input);
  }

  Output.Position = safe_divide(Output.Position, params.scale);

  return Output;
}

float fractal_voronoi_distance_to_edge(VoronoiParams params, vector4 coord)
{
  float amplitude = 1.0;
  float max_amplitude = 1.5 - 0.5 * params.randomness;
  float scale = 1.0;
  float distance = 8.0;

  int zero_input = params.detail == 0.0 || params.roughness == 0.0 || params.lacunarity == 0.0;

  for (int i = 0; i <= ceil(params.detail); ++i) {
    float octave_distance = voronoi_distance_to_edge(params, coord * scale);

    if (zero_input) {
      distance = octave_distance;
      break;
    }
    else if (i <= params.detail) {
      max_amplitude = mix(max_amplitude, (1.5 - 0.5 * params.randomness) * scale, amplitude);
      distance = mix(distance, min(distance, octave_distance / scale), amplitude);
      scale *= params.lacunarity;
      amplitude *= params.roughness;
    }
    else {
      float remainder = params.detail - floor(params.detail);
      if (remainder != 0.0) {
        float lerp_amplitude = mix(max_amplitude, (1.5 - 0.5 * params.randomness) * scale, amplitude);
        max_amplitude = mix(max_amplitude, lerp_amplitude, remainder);
        float lerp_distance = mix(distance, min(distance, octave_distance / scale), amplitude);
        distance = mix(distance, min(distance, lerp_distance), remainder);
      }
    }
  }

  if (params.normalize) {
    /* max_amplitude is used here to keep the code consistent, however it has a different
     * meaning than in F1, Smooth F1 and F2. Instead of the highest possible amplitude, it
     * represents an abstract factor needed to cancel out the amplitude attenuation caused by the
     * higher layers. */
    distance *= max_amplitude;
  }

  return distance;
}
