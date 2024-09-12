/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

void npr_input(vec3 offset,
               out vec4 combined_color,
               out vec4 diffuse_color,
               out vec4 diffuse_direct,
               out vec4 diffuse_indirect,
               out vec4 specular_color,
               out vec4 specular_direct,
               out vec4 specular_indirect,
               out vec3 position,
               out vec3 normal)
{
#if defined(NPR_SHADER) && defined(GPU_FRAGMENT_SHADER)
  npr_input_impl(offset.xy,
                 combined_color,
                 diffuse_color,
                 diffuse_direct,
                 diffuse_indirect,
                 specular_color,
                 specular_direct,
                 specular_indirect,
                 position,
                 normal);
#else
  combined_color = vec4(0.0);
  diffuse_color = vec4(0.0);
  diffuse_direct = vec4(0.0);
  diffuse_indirect = vec4(0.0);
  specular_color = vec4(0.0);
  specular_direct = vec4(0.0);
  specular_indirect = vec4(0.0);
  position = g_data.P;
  normal = g_data.N;
#endif
}

void npr_output(vec4 color, out vec4 out_color)
{
  out_color = color;
}
