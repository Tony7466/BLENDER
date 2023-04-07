/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

/* TODO(lukas): Fix colors in OSL. */

float color_srgb_to_scene_linear(float c)
{
  if (c < 0.04045)
    return (c < 0.0) ? 0.0 : c * (1.0 / 12.92);
  else
    return pow((c + 0.055) * (1.0 / 1.055), 2.4);
}

float color_scene_linear_to_srgb(float c)
{
  if (c < 0.0031308)
    return (c < 0.0) ? 0.0 : c * 12.92;
  else
    return 1.055 * pow(c, 1.0 / 2.4) - 0.055;
}

color color_srgb_to_scene_linear(color c)
{
  return color(color_srgb_to_scene_linear(c[0]),
               color_srgb_to_scene_linear(c[1]),
               color_srgb_to_scene_linear(c[2]));
}

color color_scene_linear_to_srgb(color c)
{
  return color(color_scene_linear_to_srgb(c[0]),
               color_scene_linear_to_srgb(c[1]),
               color_scene_linear_to_srgb(c[2]));
}

color color_unpremultiply(color c, float alpha)
{
  if (alpha != 1.0 && alpha != 0.0)
    return c / alpha;

  return c;
}
