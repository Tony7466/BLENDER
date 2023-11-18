/* SPDX-FileCopyrightText: 2018-2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma BLENDER_REQUIRE(common_view_clipping_lib.glsl)
#pragma BLENDER_REQUIRE(common_view_lib.glsl)

void draw_edge(vec4 a, vec4 b, vec4 c, vec4 color)
{
  vec4 trans = c - (a + b) * 0.5f;
  vec4 eps = vec4(0.0f, 0.0f, -1e-5f, 0.0f);

  gl_Position = a;
  geometry_out.weightColorGeo = color;
  gpu_EmitVertex();

  gl_Position = a * 0.80f + b * 0.10f + c * 0.10f + eps;
  geometry_out.weightColorGeo = color;
  gpu_EmitVertex();

  gl_Position = b;
  geometry_out.weightColorGeo = color;
  gpu_EmitVertex();

  gl_Position = b * 0.80f + a * 0.10f + c * 0.10f + eps;
  geometry_out.weightColorGeo = color;
  gpu_EmitVertex();

  EndPrimitive();
}

void main(void)
{
  vec4 gray = vec4(0.25f, 0.25f, 0.25f, 1.0f);
  vec4 red = vec4(1.0f, 0.0f, 0.0f, 1.0f);
  vec4 green = vec4(0.0f, 1.0f, 0.0f, 1.0f);
  vec4 w[3] = {gray, gray, gray};

  ivec2 vid0 = geometry_in[0].vid;
  ivec2 vid1 = geometry_in[1].vid;
  ivec2 vid2 = geometry_in[2].vid;

  gl_Position = gl_in[0].gl_Position;
  geometry_out.weightColorGeo = gray;
  gpu_EmitVertex();
  gl_Position = gl_in[1].gl_Position;
  geometry_out.weightColorGeo = gray;
  gpu_EmitVertex();
  gl_Position = gl_in[2].gl_Position;
  geometry_out.weightColorGeo = gray;
  gpu_EmitVertex();
  EndPrimitive();

  if (vid0[1] == vid1[0])
    draw_edge(gl_in[0].gl_Position,
              gl_in[1].gl_Position,
              gl_in[2].gl_Position,
              geometry_in[0].weightColor);
  if (vid1[1] == vid2[0])
    draw_edge(gl_in[1].gl_Position,
              gl_in[2].gl_Position,
              gl_in[0].gl_Position,
              geometry_in[1].weightColor);
  if (vid2[1] == vid0[0])
    draw_edge(gl_in[2].gl_Position,
              gl_in[0].gl_Position,
              gl_in[1].gl_Position,
              geometry_in[2].weightColor);
}
