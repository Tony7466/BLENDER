/* SPDX-FileCopyrightText: 2018-2022 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#define COS45 (0.70710678118)
#define SIN45 (0.70710678118)

float square_sdf(vec2 absCo, float half_width)
{
  vec2 extruded = vec2(max(0.0, absCo.x - half_width), max(0.0, absCo.y - half_width));
  return dot(extruded, extruded);
}

vec2 rotate_45(vec2 co)
{
  return vec2(COS45 * co.x - SIN45 * co.y, SIN45 * co.x + COS45 * co.y);
}

void main()
{
  vec2 absUV = abs(uv);
  vec2 co = vec2(max(absUV.x - extrusion.x, 0.0), max(absUV.y - extrusion.y, 0.0));

  co = (is_diamond == 1) ? abs(rotate_45(co)) : co;
  float distSquared = square_sdf(co, sdf_shape_radius);

  /* Needed to draw two dots for the wide reroute nodes. */
  vec2 biCenteredUV = abs(absUV - extrusion);

  /* Black mask with a white dot */
  float mask_dot = smoothstep(dotThresholds[1], dotThresholds[0], dot(biCenteredUV, biCenteredUV));

  /* Alpha for the socket: White where the socket is, black outside of it. */
  float mask_all = smoothstep(thresholds[3], thresholds[2], distSquared);

  /* Mask for the outline. The inner part of the socket is masked with black. */
  bool noOutline = thresholds[2] - thresholds[0] < 0.0001;
  float mask_outline = noOutline ? 0.0 : smoothstep(thresholds[0], thresholds[1], distSquared);
  mask_outline += mask_dot;

  fragColor = mix(finalColor, finalOutlineColor, mask_outline);
  fragColor.a *= mask_all;
}
