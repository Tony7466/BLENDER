#pragma BLENDER_REQUIRE(gpu_shader_common_hash.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_common_math_utils.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_material_voronoi.glsl)

/* The 2D, 3D and 4D Fractal Voronoi functions are exactly the same in different dimensions except
 * for the input type. The 1D Fractal Voronoi functions don't have the "exponent" and "metric"
 * function parameters as they aren't needed in the 1D Voronoi calculations. */

/* **** 1D Fractal Voronoi **** */

void fractal_voronoi_f1(float coord,
                        float detail,
                        float roughness,
                        float lacunarity,
                        float randomness,
                        out float max_amplitude,
                        out float outDistance,
                        out vec4 outColor,
                        out float outPosition)
{
  float octave_scale = 1.0;
  float octave_amplitude = 1.0;
  float octave_distance = 0.0;
  vec4 octave_color;
  float octave_postion;

  max_amplitude = 0.0;
  outDistance = 0.0;
  for (int i = 0; i <= ceilf(detail); ++i) {
    voronoi_f1(coord * octave_scale, randomness, octave_distance, octave_color, octave_postion);
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
      float remainder = detail - floorf(detail);
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
                               float randomness,
                               out float max_amplitude,
                               out float outDistance,
                               out vec4 outColor,
                               out float outPosition)
{
  float octave_scale = 1.0;
  float octave_amplitude = 1.0;
  float octave_distance = 0.0;
  vec4 octave_color;
  float octave_postion;

  max_amplitude = 0.0;
  outDistance = 0.0;
  for (int i = 0; i <= ceilf(detail); ++i) {
    voronoi_smooth_f1(coord * octave_scale,
                      smoothness,
                      randomness,
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
      float remainder = detail - floorf(detail);
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
                        float randomness,
                        out float max_amplitude,
                        out float outDistance,
                        out vec4 outColor,
                        out float outPosition)
{
  float octave_scale = 1.0;
  float octave_amplitude = 1.0;
  float octave_distance = 0.0;
  vec4 octave_color;
  float octave_postion;

  max_amplitude = 0.0;
  outDistance = 0.0;
  for (int i = 0; i <= ceilf(detail); ++i) {
    voronoi_f2(coord * octave_scale, randomness, octave_distance, octave_color, octave_postion);
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
      float remainder = detail - floorf(detail);
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
                                      out float max_amplitude,
                                      out float outDistance)
{
  float octave_scale = 1.0f;
  float octave_amplitude = 1.0f;
  float octave_distance = 0.0f;

  max_amplitude = 2.0f - randomness;
  outDistance = 8.0f;
  for (int i = 0; i <= ceilf(detail); ++i) {
    voronoi_distance_to_edge(coord * octave_scale, randomness, octave_distance);
    if (detail == 0.0f || roughness == 0.0f || lacunarity == 0.0f) {
      outDistance = octave_distance;
      return;
    }
    else if (i <= detail) {
      max_amplitude = mix(max_amplitude, (2.0f - randomness) * octave_scale, octave_amplitude);
      outDistance = mix(
          outDistance, min(outDistance, octave_distance / octave_scale), octave_amplitude);
      octave_scale *= lacunarity;
      octave_amplitude *= roughness;
    }
    else {
      float remainder = detail - floorf(detail);
      if (remainder != 0.0f) {
        float lerp_amplitude = mix(
            max_amplitude, (2.0f - randomness) * octave_scale, octave_amplitude);
        max_amplitude = mix(max_amplitude, lerp_amplitude, remainder);
        float lerp_distance = mix(
            outDistance, min(outDistance, octave_distance / octave_scale), octave_amplitude);
        outDistance = mix(outDistance, min(outDistance, lerp_distance), remainder);
      }
    }
  }
}

/* **** 2D Fractal Voronoi **** */

void fractal_voronoi_f1(vec2 coord,
                        float detail,
                        float roughness,
                        float lacunarity,
                        float exponent,
                        float randomness,
                        float metric,
                        out float max_amplitude,
                        out float outDistance,
                        out vec4 outColor,
                        out vec2 outPosition)
{
  float octave_scale = 1.0;
  float octave_amplitude = 1.0;
  float octave_distance = 0.0;
  vec4 octave_color;
  vec2 octave_postion;

  max_amplitude = 0.0;
  outDistance = 0.0;
  for (int i = 0; i <= ceilf(detail); ++i) {
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
      float remainder = detail - floorf(detail);
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

void fractal_voronoi_smooth_f1(vec2 coord,
                               float detail,
                               float roughness,
                               float lacunarity,
                               float smoothness,
                               float exponent,
                               float randomness,
                               float metric,
                               out float max_amplitude,
                               out float outDistance,
                               out vec4 outColor,
                               out vec2 outPosition)
{
  float octave_scale = 1.0;
  float octave_amplitude = 1.0;
  float octave_distance = 0.0;
  vec4 octave_color;
  vec2 octave_postion;

  max_amplitude = 0.0;
  outDistance = 0.0;
  for (int i = 0; i <= ceilf(detail); ++i) {
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
      float remainder = detail - floorf(detail);
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

void fractal_voronoi_f2(vec2 coord,
                        float detail,
                        float roughness,
                        float lacunarity,
                        float exponent,
                        float randomness,
                        float metric,
                        out float max_amplitude,
                        out float outDistance,
                        out vec4 outColor,
                        out vec2 outPosition)
{
  float octave_scale = 1.0;
  float octave_amplitude = 1.0;
  float octave_distance = 0.0;
  vec4 octave_color;
  vec2 octave_postion;

  max_amplitude = 0.0;
  outDistance = 0.0;
  for (int i = 0; i <= ceilf(detail); ++i) {
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
      float remainder = detail - floorf(detail);
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

void fractal_voronoi_distance_to_edge(vec2 coord,
                                      float detail,
                                      float roughness,
                                      float lacunarity,
                                      float randomness,
                                      out float max_amplitude,
                                      out float outDistance)
{
  float octave_scale = 1.0f;
  float octave_amplitude = 1.0f;
  float octave_distance = 0.0f;

  max_amplitude = 2.0f - randomness;
  outDistance = 8.0f;
  for (int i = 0; i <= ceilf(detail); ++i) {
    voronoi_distance_to_edge(coord * octave_scale, randomness, octave_distance);
    if (detail == 0.0f || roughness == 0.0f || lacunarity == 0.0f) {
      outDistance = octave_distance;
      return;
    }
    else if (i <= detail) {
      max_amplitude = mix(max_amplitude, (2.0f - randomness) * octave_scale, octave_amplitude);
      outDistance = mix(
          outDistance, min(outDistance, octave_distance / octave_scale), octave_amplitude);
      octave_scale *= lacunarity;
      octave_amplitude *= roughness;
    }
    else {
      float remainder = detail - floorf(detail);
      if (remainder != 0.0f) {
        float lerp_amplitude = mix(
            max_amplitude, (2.0f - randomness) * octave_scale, octave_amplitude);
        max_amplitude = mix(max_amplitude, lerp_amplitude, remainder);
        float lerp_distance = mix(
            outDistance, min(outDistance, octave_distance / octave_scale), octave_amplitude);
        outDistance = mix(outDistance, min(outDistance, lerp_distance), remainder);
      }
    }
  }
}

/* **** 3D Fractal Voronoi **** */

void fractal_voronoi_f1(vec3 coord,
                        float detail,
                        float roughness,
                        float lacunarity,
                        float exponent,
                        float randomness,
                        float metric,
                        out float max_amplitude,
                        out float outDistance,
                        out vec4 outColor,
                        out vec3 outPosition)
{
  float octave_scale = 1.0;
  float octave_amplitude = 1.0;
  float octave_distance = 0.0;
  vec4 octave_color;
  vec3 octave_postion;

  max_amplitude = 0.0;
  outDistance = 0.0;
  for (int i = 0; i <= ceilf(detail); ++i) {
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
      float remainder = detail - floorf(detail);
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

void fractal_voronoi_smooth_f1(vec3 coord,
                               float detail,
                               float roughness,
                               float lacunarity,
                               float smoothness,
                               float exponent,
                               float randomness,
                               float metric,
                               out float max_amplitude,
                               out float outDistance,
                               out vec4 outColor,
                               out vec3 outPosition)
{
  float octave_scale = 1.0;
  float octave_amplitude = 1.0;
  float octave_distance = 0.0;
  vec4 octave_color;
  vec3 octave_postion;

  max_amplitude = 0.0;
  outDistance = 0.0;
  for (int i = 0; i <= ceilf(detail); ++i) {
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
      float remainder = detail - floorf(detail);
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

void fractal_voronoi_f2(vec3 coord,
                        float detail,
                        float roughness,
                        float lacunarity,
                        float exponent,
                        float randomness,
                        float metric,
                        out float max_amplitude,
                        out float outDistance,
                        out vec4 outColor,
                        out vec3 outPosition)
{
  float octave_scale = 1.0;
  float octave_amplitude = 1.0;
  float octave_distance = 0.0;
  vec4 octave_color;
  vec3 octave_postion;

  max_amplitude = 0.0;
  outDistance = 0.0;
  for (int i = 0; i <= ceilf(detail); ++i) {
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
      float remainder = detail - floorf(detail);
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

void fractal_voronoi_distance_to_edge(vec3 coord,
                                      float detail,
                                      float roughness,
                                      float lacunarity,
                                      float randomness,
                                      out float max_amplitude,
                                      out float outDistance)
{
  float octave_scale = 1.0f;
  float octave_amplitude = 1.0f;
  float octave_distance = 0.0f;

  max_amplitude = 2.0f - randomness;
  outDistance = 8.0f;
  for (int i = 0; i <= ceilf(detail); ++i) {
    voronoi_distance_to_edge(coord * octave_scale, randomness, octave_distance);
    if (detail == 0.0f || roughness == 0.0f || lacunarity == 0.0f) {
      outDistance = octave_distance;
      return;
    }
    else if (i <= detail) {
      max_amplitude = mix(max_amplitude, (2.0f - randomness) * octave_scale, octave_amplitude);
      outDistance = mix(
          outDistance, min(outDistance, octave_distance / octave_scale), octave_amplitude);
      octave_scale *= lacunarity;
      octave_amplitude *= roughness;
    }
    else {
      float remainder = detail - floorf(detail);
      if (remainder != 0.0f) {
        float lerp_amplitude = mix(
            max_amplitude, (2.0f - randomness) * octave_scale, octave_amplitude);
        max_amplitude = mix(max_amplitude, lerp_amplitude, remainder);
        float lerp_distance = mix(
            outDistance, min(outDistance, octave_distance / octave_scale), octave_amplitude);
        outDistance = mix(outDistance, min(outDistance, lerp_distance), remainder);
      }
    }
  }
}

/* **** 4D Fractal Voronoi **** */

void fractal_voronoi_f1(vec4 coord,
                        float detail,
                        float roughness,
                        float lacunarity,
                        float exponent,
                        float randomness,
                        float metric,
                        out float max_amplitude,
                        out float outDistance,
                        out vec4 outColor,
                        out vec4 outPosition)
{
  float octave_scale = 1.0;
  float octave_amplitude = 1.0;
  float octave_distance = 0.0;
  vec4 octave_color;
  vec4 octave_postion;

  max_amplitude = 0.0;
  outDistance = 0.0;
  for (int i = 0; i <= ceilf(detail); ++i) {
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
      float remainder = detail - floorf(detail);
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

void fractal_voronoi_smooth_f1(vec4 coord,
                               float detail,
                               float roughness,
                               float lacunarity,
                               float smoothness,
                               float exponent,
                               float randomness,
                               float metric,
                               out float max_amplitude,
                               out float outDistance,
                               out vec4 outColor,
                               out vec4 outPosition)
{
  float octave_scale = 1.0;
  float octave_amplitude = 1.0;
  float octave_distance = 0.0;
  vec4 octave_color;
  vec4 octave_postion;

  max_amplitude = 0.0;
  outDistance = 0.0;
  for (int i = 0; i <= ceilf(detail); ++i) {
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
      float remainder = detail - floorf(detail);
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

void fractal_voronoi_f2(vec4 coord,
                        float detail,
                        float roughness,
                        float lacunarity,
                        float exponent,
                        float randomness,
                        float metric,
                        out float max_amplitude,
                        out float outDistance,
                        out vec4 outColor,
                        out vec4 outPosition)
{
  float octave_scale = 1.0;
  float octave_amplitude = 1.0;
  float octave_distance = 0.0;
  vec4 octave_color;
  vec4 octave_postion;

  max_amplitude = 0.0;
  outDistance = 0.0;
  for (int i = 0; i <= ceilf(detail); ++i) {
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
      float remainder = detail - floorf(detail);
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

void fractal_voronoi_distance_to_edge(vec4 coord,
                                      float detail,
                                      float roughness,
                                      float lacunarity,
                                      float randomness,
                                      out float max_amplitude,
                                      out float outDistance)
{
  float octave_scale = 1.0f;
  float octave_amplitude = 1.0f;
  float octave_distance = 0.0f;

  max_amplitude = 2.0f - randomness;
  outDistance = 8.0f;
  for (int i = 0; i <= ceilf(detail); ++i) {
    voronoi_distance_to_edge(coord * octave_scale, randomness, octave_distance);
    if (detail == 0.0f || roughness == 0.0f || lacunarity == 0.0f) {
      outDistance = octave_distance;
      return;
    }
    else if (i <= detail) {
      max_amplitude = mix(max_amplitude, (2.0f - randomness) * octave_scale, octave_amplitude);
      outDistance = mix(
          outDistance, min(outDistance, octave_distance / octave_scale), octave_amplitude);
      octave_scale *= lacunarity;
      octave_amplitude *= roughness;
    }
    else {
      float remainder = detail - floorf(detail);
      if (remainder != 0.0f) {
        float lerp_amplitude = mix(
            max_amplitude, (2.0f - randomness) * octave_scale, octave_amplitude);
        max_amplitude = mix(max_amplitude, lerp_amplitude, remainder);
        float lerp_distance = mix(
            outDistance, min(outDistance, octave_distance / octave_scale), octave_amplitude);
        outDistance = mix(outDistance, min(outDistance, lerp_distance), remainder);
      }
    }
  }
}
