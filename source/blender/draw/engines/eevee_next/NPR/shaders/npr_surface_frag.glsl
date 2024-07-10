/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

void main()
{
  ivec2 texel = ivec2(gl_FragCoord.xy);
  if (texelFetch(npr_index_tx, texel, 0).r != npr_index) {
    /* TODO(NPR): Convert NPR index to depth and use depth testing instead. */
    discard;
    return;
  }

  out_color = vec4(1.0);
}
