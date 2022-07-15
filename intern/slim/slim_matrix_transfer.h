/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <stdio.h>

/*
  Struct that holds all the information and data matrices to be transfered from the native
  Blender part to SLIM, named as follows:

  Matrix/Vector   | contains pointers to arrays of:
  ________________|_____________________________________________
  v_matrices      | vertex positions
  uv_matrices     | UV positions of vertices
  PPmatrice       | positions of pinned vertices
  el_vectors      | Edge lengths
  w_vectors       | weights pre vertex
  f_matrices      | vertexindex-triplets making up the faces
  p_matrices      | indices of pinned vertices
  Ematrix         | vertexindex-tuples making up edges
  Bvector         | vertexindices of boundary vertices
  ---------------------------------------------------------------
*/

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  int n_charts;
  int *n_verts, *n_faces, *n_pinned_vertices, *n_boundary_vertices, *n_edges;

  bool *succeeded;

  double **v_matrices, **uv_matrices, **pp_matrices;
  double **el_vectors;
  float **w_vectors;

  int **f_matrices, **p_matrices, **e_matrices;
  int **b_vectors;

  bool fixed_boundary;
  bool pinned_vertices;
  bool with_weighted_parameterization;
  double weight_influence;
  bool transform_islands;
  int reflection_mode;
  double relative_scale;

  /* external */
  int n_iterations;
  bool skip_initialization;
  bool is_minimize_stretch;
} SLIMMatrixTransfer;

#ifdef __cplusplus
}
#endif
