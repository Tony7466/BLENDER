#pragma BLENDER_REQUIRE(gpu_shader_common_hash.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_common_math_utils.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_material_voronoi.glsl)

/* **** 1D Fractal Voronoi **** */

void fractal_voronoi_f1(float w,
                        float detail,
                        float roughness,
                        float lacunarity,
                        float randomness,
                        float max_distance,
                        out float max_amplitude,
                        out float outDistance,
                        out vec4 outColor,
                        out float outw)
{
  voronoi_f1(w, randomness, outDistance, outColor, outw);
}

void fractal_voronoi_smooth_f1(float w,
                               float detail,
                               float roughness,
                               float lacunarity,
                               float smoothness,
                               float randomness,
                               float max_distance,
                               out float max_amplitude,
                               out float outDistance,
                               out vec4 outColor,
                               out float outw)
{
}

void fractal_voronoi_f2(float w,
                        float detail,
                        float roughness,
                        float lacunarity,
                        float randomness,
                        float max_distance,
                        out float max_amplitude,
                        out float outDistance,
                        out vec4 outColor,
                        out float outw)
{
}

void fractal_voronoi_distance_to_edge(float w,
                                      float detail,
                                      float lacunarity,
                                      float randomness,
                                      float bool_normalize,
                                      out float outDistance)
{
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
}

void fractal_voronoi_distance_to_edge(vec2 coord,
                                      float detail,
                                      float lacunarity,
                                      float randomness,
                                      float bool_normalize,
                                      out float outDistance)
{
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
}

void fractal_voronoi_distance_to_edge(vec3 coord,
                                      float detail,
                                      float lacunarity,
                                      float randomness,
                                      float bool_normalize,
                                      out float outDistance)
{
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
}

void fractal_voronoi_distance_to_edge(vec4 coord,
                                      float detail,
                                      float lacunarity,
                                      float randomness,
                                      float bool_normalize,
                                      out float outDistance)
{
}
