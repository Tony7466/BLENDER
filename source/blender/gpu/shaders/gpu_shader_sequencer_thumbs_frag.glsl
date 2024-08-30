/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

float sdf_rounded_box(vec2 pos, vec2 size, float radius) //@TODO: share with other seq shader?
{
  vec2 q = abs(pos) - size + radius;
  return min(max(q.x, q.y), 0.0) + length(max(q, 0.0)) - radius;
}

void main()
{
  vec2 co = pos_interp;
  
  /* Snap to pixel grid coordinates. */ //@TODO: share with other seq shader?
  vec2 pos1 = round(vec2(strip_rect.x, strip_rect.z));
  vec2 pos2 = round(vec2(strip_rect.y, strip_rect.w));
  /* Make sure strip is at least 1px wide. */
  pos2.x = max(pos2.x, pos1.x + 1.0);
  vec2 size = (pos2 - pos1) * 0.5;
  vec2 center = (pos1 + pos2) * 0.5;
  vec2 pos = round(co);

  float radius = round_radius;
  if (radius > size.x) {
    radius = 0.0;
  }

  /* Sample thumbnail texture, modulate with uniform color. */
  vec4 col = texture(image, texCoord_interp) * color;

  /* Outside of strip rounded rectangle? */
  float sdf = sdf_rounded_box(pos - center, size, radius);
  if (sdf > 0.0) {
    col = vec4(0.0);
  }

  fragColor = col;
}
