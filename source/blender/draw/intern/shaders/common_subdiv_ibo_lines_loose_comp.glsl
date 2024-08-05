/* SPDX-FileCopyrightText: 2021-2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* To be compiled with common_subdiv_lib.glsl */

layout(std430, binding = 1) writeonly buffer outputLinesIndices
{
  uint output_lines[];
};

layout(std430, binding = 2) readonly buffer LinesLooseFlags
{
  uint lines_loose_flags[];
};


void main()
{
  uint index = get_global_invocation_index();
  if (index >= total_dispatch_size) {
    return;
  }

  /* In the loose lines case, we execute for each line, with two vertices per line. */
  uint line_offset = edge_loose_offset + index * 2;
  uint loop_index = num_subdiv_loops + index * 2;

  if (lines_loose_flags[index] != 0) {
    /* Line is hidden. */
    output_lines[line_offset] = 0xffffffff;
    output_lines[line_offset + 1] = 0xffffffff;
  }
  else {
    output_lines[line_offset] = loop_index;
    output_lines[line_offset + 1] = loop_index + 1;
  }
}
