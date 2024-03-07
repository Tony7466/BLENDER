/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

void main()
{
  fragColor = interp.final_color;
  fragColor.a *= clamp((2.0f /* lineWidth */ + 1.0f /* SMOOTH_WIDTH */) * 0.5 - abs(interp_noperspective.smoothline), 0.0, 1.0);
}
