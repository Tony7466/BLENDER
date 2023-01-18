/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include "node_voronoi.h"
#include "stdcycles.h"
#include "vector2.h"
#include "vector4.h"

#define vector3 point

 /* **** 1D Fractal Voronoi **** */

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
  voronoi_f1(coord, exponent, randomness, metric, outDistance, outColor, outPosition);
  max_amplitude = max_distance;
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
  voronoi_smooth_f1(coord, smoothness, exponent, randomness, metric, outDistance, outColor, outPosition);
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
