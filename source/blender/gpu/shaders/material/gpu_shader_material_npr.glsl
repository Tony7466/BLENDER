/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

void npr_passthrough(in Closure Shader, float weight, out Closure NPR)
{
  NPR = Shader;
}

void npr_input(out vec4 Color)
{
  /* TODO(NPR) */
  Color = vec4(1);
}

void npr_output(vec4 Color)
{
  /* TODO(NPR) */
}
