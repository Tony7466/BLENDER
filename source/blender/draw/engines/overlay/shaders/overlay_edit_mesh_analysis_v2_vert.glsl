/* SPDX-FileCopyrightText: 2016-2022 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma BLENDER_REQUIRE(common_view_clipping_lib.glsl)
#pragma BLENDER_REQUIRE(common_view_lib.glsl)

vec3 hsv_to_rgb(vec3 hsv)
{
  vec3 nrgb = abs(hsv.x * 6.0 - vec3(3.0, 2.0, 4.0)) * vec3(1, -1, -1) + vec3(-1, 2, 2);
  nrgb = clamp(nrgb, 0.0, 1.0);
  return ((nrgb - 1.0) * hsv.y + 1.0) * hsv.z;
}

vec3 weight_to_rgb(float x)
{
  if (x >= 1.0f)  // angle overlay
  {
    x -= 1.5f;
    float t;
    if (x > -0.001f && x < 0.001f)
      return hsv_to_rgb(vec3(0.333, 0, 0.25));
    else {
      t = x;
    }

    float h = clamp(0.333 + t + (t > 0 ? 0.20f : -0.20f), 0, 0.666);
    float s = clamp((t > 0 ? t : -t) * 6, 0.10, 1);
    float v = 0.25;
    return hsv_to_rgb(vec3(h, s, v));
  }

  // sharp overlay

  if (x < 0.0) {
    /* Minimum color, gray */
    return vec3(0.25, 0.25, 0.25);
  }
  return texture(weightTex, x).rgb;
}

void main()
{
  GPU_INTEL_VERTEX_SHADER_WORKAROUND

  vec3 world_pos = point_object_to_world(pos);
  gl_Position = point_world_to_ndc(world_pos);
  geometry_in.weightColor = vec4(weight_to_rgb(weight.x), 1);
  geometry_in.vid.x = floatBitsToInt(weight.y);
  if (weight.x < 0) {
    /* sharp mode, unmarked edge */
    geometry_in.vid.y = -1;
  }
  else {
    geometry_in.vid.y = floatBitsToInt(weight.z);
  }
  view_clipping_distances(world_pos);
}
