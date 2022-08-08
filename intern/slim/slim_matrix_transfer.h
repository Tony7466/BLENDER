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



struct SLIMMatrixTransferChart {
  int n_verts = 0;
  int n_faces = 0;
  int n_pinned_vertices = 0;
  int n_boundary_vertices = 0;
  int n_edges = 0;

  bool succeeded = false;

  double* v_matrices = nullptr;
  double* uv_matrices = nullptr;
  double* pp_matrices = nullptr;
  double* el_vectors = nullptr;
  float* w_vectors = nullptr;

  int* f_matrices = nullptr;
  int* p_matrices = nullptr;
  int* e_matrices = nullptr;
  int* b_vectors = nullptr;

  void* data = nullptr;
};

struct SLIMMatrixTransfer {
  int n_charts = 0;

  bool fixed_boundary = false;
  bool pinned_vertices = false;
  bool with_weighted_parameterization = false;
  double weight_influence = 0.0;
  bool transform_islands = false;
  int reflection_mode = 0;
  double relative_scale = 0.0;

  /* External. */
  int n_iterations = 0;
  bool skip_initialization = false;
  bool is_minimize_stretch = false;

  std::vector<SLIMMatrixTransferChart> mt_charts;

  void parametrize(
    int n_iterations,
    bool are_border_vertices_pinned,
    bool skip_initialization);

  void transfer_uvs_blended_live(
    SLIMData* slim_data,
    int uv_chart_index);

  void transfer_uvs_blended(
    SLIMData* slim_data,
    int uv_chart_index,
    float blend);

  void parametrize_single_iteration(int uv_chart_index, SLIMData* slim_data);

  void parametrize_live(
    int uv_chart_index,

    SLIMData* slim_data,

    int n_pins, std::vector<int>& pinned_vertex_indices, std::vector<double>& pinned_vertex_positions_2D,

    int n_selected_pins, std::vector<int>& selected_pins);

  void* setup(
    int uv_chart_index,
    bool are_border_vertices_pinned,
    bool skip_initialization) const;

  void free_data(SLIMData* slim_data);
};
