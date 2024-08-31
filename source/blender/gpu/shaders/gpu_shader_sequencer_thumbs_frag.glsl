/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

float sdf_rounded_box(vec2 pos, vec2 size, float radius)  //@TODO: share with other seq shader?
{
  vec2 q = abs(pos) - size + radius;
  return min(max(q.x, q.y), 0.0) + length(max(q, 0.0)) - radius;
}

void main()
{
  vec2 co = pos_interp;

  SeqStripThumbData thumb = thumb_data[thumb_id];

  /* Snap to pixel grid coordinates. */  //@TODO: share with other seq shader?
  vec2 pos1 = round(vec2(thumb.left, thumb.bottom));
  vec2 pos2 = round(vec2(thumb.right, thumb.top));
  /* Make sure strip is at least 1px wide. */
  pos2.x = max(pos2.x, pos1.x + 1.0);
  vec2 size = (pos2 - pos1) * 0.5;
  vec2 center = (pos1 + pos2) * 0.5;
  vec2 pos = round(co);

  float radius = context_data.round_radius;
  if (radius > size.x) {
    radius = 0.0;
  }

  /* Sample thumbnail texture, modulate with color. */
  vec4 col = texture(image, texCoord_interp) * thumb.tint_color;

  /* Outside of strip rounded rectangle? */
  float sdf = sdf_rounded_box(pos - center, size, radius);
  if (sdf > 0.0) {
    col = vec4(0.0);
  }

  fragColor = col;
}
