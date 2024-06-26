/* SPDX-FileCopyrightText: 2018-2022 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#define ANTIALIAS 1.5
#define MINIMUM_ALPHA 0.5

void main()
{
  fragColor = finalColor;

  if ((isMainLine != 0) && (dashFactor < 1.0)) {
    float distance_along_line = lineLength * lineUV.x;
    float normalized_distance = fract(distance_along_line / dashLength);

    /* Checking if `normalized_distance <= dashFactor` is already enough for a basic
     * dash, however we want to handle a nice anti-alias. */

    float dash_center = dashLength * dashFactor * 0.5;
    float normalized_distance_triangle =
        1.0 - abs((fract((distance_along_line - dash_center) / dashLength)) * 2.0 - 1.0);
    float t = aspect * ANTIALIAS / dashLength;
    float slope = 1.0 / (2.0 * t);

    float unclamped_alpha = 1.0 - slope * (normalized_distance_triangle - dashFactor + t);
    float alpha = max(dashAlpha, min(unclamped_alpha, 1.0));

    fragColor.a *= alpha;
  }

  fragColor.a *= smoothstep(lineThickness, lineThickness - ANTIALIAS, abs(colorGradient));
  if (isMainLine != 0 && isSplitLine != 0) {
    float v = lineUV.y;
    fragColor.rgb = vec3(0.0);
    if (v > 0.5 && v < 1.0) {
      float t = 1.0 - pow(abs((v - 0.75) * 4.0), 2);
      fragColor.rgb = finalColor.rgb * t;
    }
    else if (v > 0.0 && v < 0.3) {
      float t = 1.0 - pow(abs(v - 0.15) * 6.6666, 2);
      fragColor.rgb = vec3(0.8) * t;
    }
  }
}
