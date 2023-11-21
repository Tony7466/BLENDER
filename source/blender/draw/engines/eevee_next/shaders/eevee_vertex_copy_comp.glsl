/* SPDX-FileCopyrightText: 2022-2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

void main()
{
  uint vertex = gl_GlobalInvocationID.x;
  if (vertex >= length) {
    return;
  }
  out_buf[offset + vertex] = vec4(
      in_buf[vertex * stride + 0], in_buf[vertex * stride + 1], in_buf[vertex * stride + 2], 1.0);
}
