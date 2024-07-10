/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

void npr_passthrough(in Closure shader, float weight, out Closure npr)
{
  npr = shader;
}

void npr_input(out vec4 color)
{
#if defined(NPR_SHADER) && defined(GPU_FRAGMENT_SHADER)
  color = g_combined_color;
#else
  color = vec4(0.0);
#endif
}

void npr_output(vec4 color, out vec4 out_color)
{
  out_color = color;
}
