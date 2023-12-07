/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#ifdef GPU_VERTEX_SHADER
void main()
{
  gl_PointSize =1.f;
  out_value = in_value;
}
#endif

#ifdef GPU_FRAGMENT_SHADER
void main()
{
  discard;
}
#endif
