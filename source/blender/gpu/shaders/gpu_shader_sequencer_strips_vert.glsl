/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

void main()
{
  int id = gl_InstanceID;
  strip_id = id;
  int vid = gl_VertexID;
  SeqStripDrawData strip = strip_data[id];
  vec4 rect = vec4(strip.left_handle, strip.bottom, strip.right_handle, strip.top);
  /* Expand rasterized rectangle by 1px so that we can do outlines. */
  rect.x -= context_data.pixelx;
  rect.z += context_data.pixelx;
  rect.y -= context_data.pixely;
  rect.w += context_data.pixely;

  vec2 co;
  vec2 uv;
  if (vid == 0) {
    co = rect.xw;
    uv = vec2(0, 1);
  }
  else if (vid == 1) {
    co = rect.xy;
    uv = vec2(0, 0);
  }
  else if (vid == 2) {
    co = rect.zw;
    uv = vec2(1, 1);
  }
  else {
    co = rect.zy;
    uv = vec2(1, 0);
  }

  coInterp = co;
  gl_Position = ModelViewProjectionMatrix * vec4(co, 0.0f, 1.0f);
  uvInterp = uv;
}
