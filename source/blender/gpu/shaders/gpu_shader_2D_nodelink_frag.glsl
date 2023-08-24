/* SPDX-FileCopyrightText: 2018-2022 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#define ANTIALIAS 1.5
#define MINIMUM_ALPHA 0.5

float dash_mask(float distance_along_line, float dash_length, float dash_factor)
{
  float normalized_distance = fract(distance_along_line / dash_length);

  /* Checking if `normalized_distance <= dashFactor` is already enough for a basic
   * dash, however we want to handle a nice antialias. */

  float normalized_distance_triangle = abs(normalized_distance * 2.0 - 1.0);
  float t = aspect * ANTIALIAS / dash_length;
  float slope = 1.0 / (2.0 * t);

  float unclamped_alpha = 1.0 - slope * (normalized_distance_triangle - dash_factor + t);

  return max(0.0, min(unclamped_alpha, 1.0));
}

void main()
{
  fragColor = finalColor;

  if ((isMainLine != 0) && (dashFactor < 1.0)) {
    float distance_along_line = lineLength * lineU;

    float current_scale_level = floor(dashLevel);
    float next_scale_level = ceil(dashLevel);
    float mix_factor = smoothstep(0.4, 0.6, fract(dashLevel));

    float current_dash_length = dashLength * pow(2.0, current_scale_level);

    /* Widen the gap to approach the width of the next level.
     * XXX (Leon): This assumes the dashFactor is at least 0.5. */
    float current_dash_factor = 1.0 - ((1.0 - dashFactor) * (1.0 + mix_factor));
    float current_level_dash_mask = dash_mask(
        distance_along_line, current_dash_length, current_dash_factor);

    float next_dash_length = current_dash_length * 2.0;
    float next_level_dash_mask = dash_mask(distance_along_line, next_dash_length, dashFactor);

    float dash_alpha = max(dashAlpha, current_level_dash_mask + mix_factor * next_level_dash_mask);

    fragColor.a *= dash_alpha;
  }

  fragColor.a = min(fragColor.a,
                    smoothstep(lineThickness, lineThickness - ANTIALIAS, abs(colorGradient)));
}
