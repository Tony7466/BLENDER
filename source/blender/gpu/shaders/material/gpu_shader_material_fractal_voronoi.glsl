#pragma BLENDER_REQUIRE(gpu_shader_common_hash.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_common_math_utils.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_material_voronoi.glsl)

/* **** 1D Fractal Voronoi **** */

void fractal_voronoi_f1(float w,
                        float detail,
                        float roughness,
                        float lacunarity,
                        float randomness,
                        out float max_amplitude,
                        out float outDistance,
                        out vec4 outColor,
                        out float outw)
{
  voronoi_f1(w, randomness, outDistance, outColor, outw);
  max_amplitude = 1.0;
}

void fractal_voronoi_smooth_f1(float w,
                               float detail,
                               float roughness,
                               float lacunarity,
                               float smoothness,
                               float randomness,
                               out float max_amplitude,
                               out float outDistance,
                               out vec4 outColor,
                               out float outw)
{
  voronoi_smooth_f1(w, smoothness, randomness, outDistance, outColor, outw);
  max_amplitude = 1.0;
}

void fractal_voronoi_f2(float w,
                        float detail,
                        float roughness,
                        float lacunarity,
                        float randomness,
                        out float max_amplitude,
                        out float outDistance,
                        out vec4 outColor,
                        out float outw)
{
  voronoi_f2(w, randomness, outDistance, outColor, outw);
  max_amplitude = 1.0;
}

void fractal_voronoi_distance_to_edge(float w,
                                      float detail,
                                      float lacunarity,
                                      float randomness,
                                      float bool_normalize,
                                      out float outDistance)
{
  voronoi_distance_to_edge(w, randomness, outDistance);
}

/* **** 2D Fractal Voronoi **** */

void fractal_voronoi_f1(vec2 coord,
                        float detail,
                        float roughness,
                        float lacunarity,
                        float exponent,
                        float randomness,
                        float metric,
                        float max_distance,
                        out float max_amplitude,
                        out float outDistance,
                        out vec4 outColor,
                        out vec2 outPosition)
{
  voronoi_f1(coord, exponent, randomness, metric, outDistance, outColor, outPosition);
  max_amplitude = max_distance;
}

void fractal_voronoi_smooth_f1(vec2 coord,
                               float detail,
                               float roughness,
                               float lacunarity,
                               float smoothness,
                               float exponent,
                               float randomness,
                               float metric,
                               float max_distance,
                               out float max_amplitude,
                               out float outDistance,
                               out vec4 outColor,
                               out vec2 outPosition)
{
  voronoi_smooth_f1(
      coord, smoothness, exponent, randomness, metric, outDistance, outColor, outPosition);
  max_amplitude = max_distance;
}

void fractal_voronoi_f2(vec2 coord,
                        float detail,
                        float roughness,
                        float lacunarity,
                        float exponent,
                        float randomness,
                        float metric,
                        float max_distance,
                        out float max_amplitude,
                        out float outDistance,
                        out vec4 outColor,
                        out vec2 outPosition)
{
  voronoi_f2(coord, exponent, randomness, metric, outDistance, outColor, outPosition);
  max_amplitude = max_distance;
}

void fractal_voronoi_distance_to_edge(vec2 coord,
                                      float detail,
                                      float lacunarity,
                                      float randomness,
                                      float bool_normalize,
                                      out float outDistance)
{
  voronoi_distance_to_edge(coord, randomness, outDistance);
}

/* **** 3D Fractal Voronoi **** */

void fractal_voronoi_f1(vec3 coord,
                        float detail,
                        float roughness,
                        float lacunarity,
                        float exponent,
                        float randomness,
                        float metric,
                        float max_distance,
                        out float max_amplitude,
                        out float outDistance,
                        out vec4 outColor,
                        out vec3 outPosition)
{
  voronoi_f1(coord, exponent, randomness, metric, outDistance, outColor, outPosition);
  max_amplitude = max_distance;
}

void fractal_voronoi_smooth_f1(vec3 coord,
                               float detail,
                               float roughness,
                               float lacunarity,
                               float smoothness,
                               float exponent,
                               float randomness,
                               float metric,
                               float max_distance,
                               out float max_amplitude,
                               out float outDistance,
                               out vec4 outColor,
                               out vec3 outPosition)
{
  voronoi_smooth_f1(
      coord, smoothness, exponent, randomness, metric, outDistance, outColor, outPosition);
  max_amplitude = max_distance;
}

void fractal_voronoi_f2(vec3 coord,
                        float detail,
                        float roughness,
                        float lacunarity,
                        float exponent,
                        float randomness,
                        float metric,
                        float max_distance,
                        out float max_amplitude,
                        out float outDistance,
                        out vec4 outColor,
                        out vec3 outPosition)
{
  voronoi_f2(coord, exponent, randomness, metric, outDistance, outColor, outPosition);
  max_amplitude = max_distance;
}

void fractal_voronoi_distance_to_edge(vec3 coord,
                                      float detail,
                                      float lacunarity,
                                      float randomness,
                                      float bool_normalize,
                                      out float outDistance)
{
  voronoi_distance_to_edge(coord, randomness, outDistance);
}

/* **** 4D Fractal Voronoi **** */

void fractal_voronoi_f1(vec4 coord,
                        float detail,
                        float roughness,
                        float lacunarity,
                        float exponent,
                        float randomness,
                        float metric,
                        float max_distance,
                        out float max_amplitude,
                        out float outDistance,
                        out vec4 outColor,
                        out vec4 outPosition)
{
  voronoi_f1(coord, exponent, randomness, metric, outDistance, outColor, outPosition);
  max_amplitude = max_distance;
}

void fractal_voronoi_smooth_f1(vec4 coord,
                               float detail,
                               float roughness,
                               float lacunarity,
                               float smoothness,
                               float exponent,
                               float randomness,
                               float metric,
                               float max_distance,
                               out float max_amplitude,
                               out float outDistance,
                               out vec4 outColor,
                               out vec4 outPosition)
{
  voronoi_smooth_f1(
      coord, smoothness, exponent, randomness, metric, outDistance, outColor, outPosition);
  max_amplitude = max_distance;
}

void fractal_voronoi_f2(vec4 coord,
                        float detail,
                        float roughness,
                        float lacunarity,
                        float exponent,
                        float randomness,
                        float metric,
                        float max_distance,
                        out float max_amplitude,
                        out float outDistance,
                        out vec4 outColor,
                        out vec4 outPosition)
{
  voronoi_f2(coord, exponent, randomness, metric, outDistance, outColor, outPosition);
  max_amplitude = max_distance;
}

void fractal_voronoi_distance_to_edge(vec4 coord,
                                      float detail,
                                      float lacunarity,
                                      float randomness,
                                      float bool_normalize,
                                      out float outDistance)
{
  voronoi_distance_to_edge(coord, randomness, outDistance);
}
