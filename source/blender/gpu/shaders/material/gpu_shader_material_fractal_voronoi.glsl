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

  max_amplitude = 0.0;
  outDistance = 0.0;
  for (float i = 0; i <= ceil(detail); ++i) {
    voronoi_f1(coord * octave_scale, randomness, octave_distance, outColor, outPosition);
    if (detail == 0.0 || roughness == 0.0 || lacunarity == 0.0) {
      max_amplitude = 1.0;
      outDistance = octave_distance;
      return;
    }
    else if (i <= detail) {
      max_amplitude += octave_amplitude;
      outDistance += octave_distance * octave_amplitude;
      outPosition /= octave_scale;
      octave_scale *= lacunarity;
      octave_amplitude *= roughness;
    }
    else {
      float remainder = detail - floor(detail);
      if (remainder != 0.0) {
        max_amplitude += octave_amplitude;
        float lerp_distance = outDistance + octave_distance * octave_amplitude;
        outDistance = (1.0 - remainder) * outDistance + remainder * lerp_distance;
        outPosition /= octave_scale;
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

  max_amplitude = 0.0;
  outDistance = 0.0;
  for (float i = 0; i <= ceil(detail); ++i) {
    voronoi_smooth_f1(
        coord * octave_scale, smoothness, randomness, octave_distance, outColor, outPosition);
    if (detail == 0.0 || roughness == 0.0 || lacunarity == 0.0) {
      max_amplitude = 1.0;
      outDistance = octave_distance;
      return;
    }
    else if (i <= detail) {
      max_amplitude += octave_amplitude;
      outDistance += octave_distance * octave_amplitude;
      outPosition /= octave_scale;
      octave_scale *= lacunarity;
      octave_amplitude *= roughness;
    }
    else {
      float remainder = detail - floor(detail);
      if (remainder != 0.0) {
        max_amplitude += octave_amplitude;
        float lerp_distance = outDistance + octave_distance * octave_amplitude;
        outDistance = (1.0 - remainder) * outDistance + remainder * lerp_distance;
        outPosition /= octave_scale;
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

  max_amplitude = 0.0;
  outDistance = 0.0;
  for (float i = 0; i <= ceil(detail); ++i) {
    voronoi_f2(coord * octave_scale, randomness, octave_distance, outColor, outPosition);
    if (detail == 0.0 || roughness == 0.0 || lacunarity == 0.0) {
      max_amplitude = 1.0;
      outDistance = octave_distance;
      return;
    }
    else if (i <= detail) {
      max_amplitude += octave_amplitude;
      outDistance += octave_distance * octave_amplitude;
      outPosition /= octave_scale;
      octave_scale *= lacunarity;
      octave_amplitude *= roughness;
    }
    else {
      float remainder = detail - floor(detail);
      if (remainder != 0.0) {
        max_amplitude += octave_amplitude;
        float lerp_distance = outDistance + octave_distance * octave_amplitude;
        outDistance = (1.0 - remainder) * outDistance + remainder * lerp_distance;
        outPosition /= octave_scale;
      }
    }
  }
}

void fractal_voronoi_distance_to_edge(float coord,
                                      float detail,
                                      float lacunarity,
                                      float randomness,
                                      float bool_normalize,
                                      out float outDistance)
{
  float octave_scale = 1.0;
  float octave_distance = 0.0;

  outDistance = 8.0;
  for (float i = 0; i <= ceil(detail); ++i) {
    voronoi_distance_to_edge(coord * octave_scale, randomness, octave_distance);
    outDistance = min(outDistance, octave_distance / octave_scale);
    octave_scale *= lacunarity;
  }
  if (bool_normalize != 0.0) {
    outDistance *= (2.0 - randomness) * octave_scale / lacunarity;
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
                        float max_distance,
                        out float max_amplitude,
                        out float outDistance,
                        out vec4 outColor,
                        out vec2 outPosition)
{
  float octave_scale = 1.0;
  float octave_amplitude = 1.0;
  float octave_distance = 0.0;

  max_amplitude = 0.0;
  outDistance = 0.0;
  for (float i = 0; i <= ceil(detail); ++i) {
    voronoi_f1(coord * octave_scale,
               exponent,
               randomness,
               metric,
               octave_distance,
               outColor,
               outPosition);
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
  float octave_scale = 1.0;
  float octave_amplitude = 1.0;
  float octave_distance = 0.0;

  max_amplitude = 0.0;
  outDistance = 0.0;
  for (float i = 0; i <= ceil(detail); ++i) {
    voronoi_smooth_f1(coord * octave_scale,
                      smoothness,
                      exponent,
                      randomness,
                      metric,
                      octave_distance,
                      outColor,
                      outPosition);
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
  float octave_scale = 1.0;
  float octave_amplitude = 1.0;
  float octave_distance = 0.0;

  max_amplitude = 0.0;
  outDistance = 0.0;
  for (float i = 0; i <= ceil(detail); ++i) {
    voronoi_f2(coord * octave_scale,
               exponent,
               randomness,
               metric,
               octave_distance,
               outColor,
               outPosition);
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

void fractal_voronoi_distance_to_edge(vec2 coord,
                                      float detail,
                                      float lacunarity,
                                      float randomness,
                                      float bool_normalize,
                                      out float outDistance)
{
  float octave_scale = 1.0;
  float octave_distance = 0.0;

  outDistance = 8.0;
  for (float i = 0; i <= ceil(detail); ++i) {
    voronoi_distance_to_edge(coord * octave_scale, randomness, octave_distance);
    outDistance = min(outDistance, octave_distance / octave_scale);
    octave_scale *= lacunarity;
  }
  if (bool_normalize != 0.0) {
    outDistance *= (2.0 - randomness) * octave_scale / lacunarity;
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
                        float max_distance,
                        out float max_amplitude,
                        out float outDistance,
                        out vec4 outColor,
                        out vec3 outPosition)
{
  float octave_scale = 1.0;
  float octave_amplitude = 1.0;
  float octave_distance = 0.0;

  max_amplitude = 0.0;
  outDistance = 0.0;
  for (float i = 0; i <= ceil(detail); ++i) {
    voronoi_f1(coord * octave_scale,
               exponent,
               randomness,
               metric,
               octave_distance,
               outColor,
               outPosition);
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
  float octave_scale = 1.0;
  float octave_amplitude = 1.0;
  float octave_distance = 0.0;

  max_amplitude = 0.0;
  outDistance = 0.0;
  for (float i = 0; i <= ceil(detail); ++i) {
    voronoi_smooth_f1(coord * octave_scale,
                      smoothness,
                      exponent,
                      randomness,
                      metric,
                      octave_distance,
                      outColor,
                      outPosition);
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
  float octave_scale = 1.0;
  float octave_amplitude = 1.0;
  float octave_distance = 0.0;

  max_amplitude = 0.0;
  outDistance = 0.0;
  for (float i = 0; i <= ceil(detail); ++i) {
    voronoi_f2(coord * octave_scale,
               exponent,
               randomness,
               metric,
               octave_distance,
               outColor,
               outPosition);
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

void fractal_voronoi_distance_to_edge(vec3 coord,
                                      float detail,
                                      float lacunarity,
                                      float randomness,
                                      float bool_normalize,
                                      out float outDistance)
{
  float octave_scale = 1.0;
  float octave_distance = 0.0;

  outDistance = 8.0;
  for (float i = 0; i <= ceil(detail); ++i) {
    voronoi_distance_to_edge(coord * octave_scale, randomness, octave_distance);
    outDistance = min(outDistance, octave_distance / octave_scale);
    octave_scale *= lacunarity;
  }
  if (bool_normalize != 0.0) {
    outDistance *= (2.0 - randomness) * octave_scale / lacunarity;
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
                        float max_distance,
                        out float max_amplitude,
                        out float outDistance,
                        out vec4 outColor,
                        out vec4 outPosition)
{
  float octave_scale = 1.0;
  float octave_amplitude = 1.0;
  float octave_distance = 0.0;

  max_amplitude = 0.0;
  outDistance = 0.0;
  for (float i = 0; i <= ceil(detail); ++i) {
    voronoi_f1(coord * octave_scale,
               exponent,
               randomness,
               metric,
               octave_distance,
               outColor,
               outPosition);
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
  float octave_scale = 1.0;
  float octave_amplitude = 1.0;
  float octave_distance = 0.0;

  max_amplitude = 0.0;
  outDistance = 0.0;
  for (float i = 0; i <= ceil(detail); ++i) {
    voronoi_smooth_f1(coord * octave_scale,
                      smoothness,
                      exponent,
                      randomness,
                      metric,
                      octave_distance,
                      outColor,
                      outPosition);
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
  float octave_scale = 1.0;
  float octave_amplitude = 1.0;
  float octave_distance = 0.0;

  max_amplitude = 0.0;
  outDistance = 0.0;
  for (float i = 0; i <= ceil(detail); ++i) {
    voronoi_f2(coord * octave_scale,
               exponent,
               randomness,
               metric,
               octave_distance,
               outColor,
               outPosition);
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

void fractal_voronoi_distance_to_edge(vec4 coord,
                                      float detail,
                                      float lacunarity,
                                      float randomness,
                                      float bool_normalize,
                                      out float outDistance)
{
  float octave_scale = 1.0;
  float octave_distance = 0.0;

  outDistance = 8.0;
  for (float i = 0; i <= ceil(detail); ++i) {
    voronoi_distance_to_edge(coord * octave_scale, randomness, octave_distance);
    outDistance = min(outDistance, octave_distance / octave_scale);
    octave_scale *= lacunarity;
  }
  if (bool_normalize != 0.0) {
    outDistance *= (2.0 - randomness) * octave_scale / lacunarity;
  }
}
