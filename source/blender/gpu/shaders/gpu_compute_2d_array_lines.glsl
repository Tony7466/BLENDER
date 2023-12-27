/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 *  Constructs a 2D array index buffer with 'ncurves' rows and 'elements_per_curve*2'
 *  columns. Each row contains 'elements_per_curve' pairs of indexes.
 *  e.g., for elements_per_curve=32, first two rows are
 *  0 1 1 2 2 3 ... 31 32
 *  33 34 34 35 35 36 .. 64 65
 *  The index buffer can then be used to draw 'ncurves' curves with 'elements_per_curve+1'
 *  vertexes each, using GL_LINES primitives. Intended to be used if GL_LINE_STRIP
 *  primitives can't be used for some reason.
 */
void main()
{
  uvec3 gid = gl_GlobalInvocationID;
  uvec3 nthreads = gl_NumWorkGroups * gl_WorkGroupSize;
  for (uint y = gid.y + gid.z * nthreads.y; y < ncurves; y += nthreads.y * nthreads.z) {
    for (uint x = gid.x; x < elements_per_curve; x += nthreads.x) {
      uint store_index = (x + y * elements_per_curve) * 2;
      out_indices[store_index] = x + y * (elements_per_curve + 1);
      out_indices[store_index + 1] = x + y * (elements_per_curve + 1) + 1;
    }
  }
}
