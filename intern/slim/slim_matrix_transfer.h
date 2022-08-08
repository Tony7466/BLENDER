/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once


/* Struct that holds all the information and data matrices to be transfered from the native
 * Blender part to SLIM, named as follows:
 *
 * Matrix/Vector   | contains pointers to arrays of:
 * ________________|_____________________________________________
 * v_matrices      | vertex positions
 * uv_matrices     | UV positions of vertices
 * PPmatrice       | positions of pinned vertices
 * el_vectors      | Edge lengths
 * w_vectors       | weights pre vertex
 * f_matrices      | vertexindex-triplets making up the faces
 * p_matrices      | indices of pinned vertices
 * Ematrix         | vertexindex-tuples making up edges
 * Bvector         | vertexindices of boundary vertices
 * ________________|_____________________________________________ */


struct SLIMMatrixTransfer {
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

  /* External. */
  int n_iterations;
  bool skip_initialization;
  bool is_minimize_stretch;

  void parametrize(
    int n_iterations,
    bool are_border_vertices_pinned,
    bool skip_initialization);

  void transfer_uvs_blended_live(
    void* slim_data_ptr,
    int uv_chart_index);

  void transfer_uvs_blended(
    void* slim,
    int uv_chart_index,
    float blend);

  void parametrize_single_iteration(int uv_chart_index, void* slim);

  void parametrize_live(
    int uv_chart_index,
    void* slim_data_ptr,
    int n_pins,
    int* selected_pinned_vertex_indices,
    double* selected_pinned_vertex_positions_2D,
    int n_selected_pins,
    int* selected_pins);

  void* setup(
    int uv_chart_index,
    bool are_border_vertices_pinned,
    bool skip_initialization) const;

  void free_data(void* slim_data_ptr);
};
