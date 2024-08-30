/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

void main()
{
  gl_Position = ModelViewProjectionMatrix * vec4(pos.xy, 0.0f, 1.0f);
  texCoord_interp = texCoord;
  pos_interp = pos.xy;
}
