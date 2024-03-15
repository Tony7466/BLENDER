/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

void main()
{
  fragColor = gl_PrimitiveID < curvesInfoBlock[0] ? leftColor : finalColor;
}
