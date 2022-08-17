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


namespace igl {
  struct SLIMData;
}


struct SLIMMatrixTransferChart {
  int n_verts = 0;
  int n_faces = 0;
  int n_pinned_vertices = 0;
  int n_boundary_vertices = 0;
  int n_edges = 0;

  bool succeeded = false;

  std::vector<double> v_matrices;
  std::vector<double> uv_matrices;
  std::vector<double> pp_matrices;
  std::vector<double> el_vectors;
  std::vector<float> w_vectors;

  std::vector<int> f_matrices;
  std::vector<int> p_matrices;
  std::vector<int> e_matrices;
  std::vector<int> b_vectors;

  igl::SLIMData* data = nullptr;

  SLIMMatrixTransferChart() = default;
  SLIMMatrixTransferChart(SLIMMatrixTransferChart&&) = default;

  SLIMMatrixTransferChart(const SLIMMatrixTransferChart&) = delete;
  SLIMMatrixTransferChart& operator=(const SLIMMatrixTransferChart&) = delete;
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

  SLIMMatrixTransfer() = default;
  SLIMMatrixTransfer(const SLIMMatrixTransfer&) = delete;
  SLIMMatrixTransfer& operator=(const SLIMMatrixTransfer&) = delete;

  void parametrize(
    int n_iterations,
    bool are_border_vertices_pinned,
    bool skip_initialization);

  void transfer_uvs_blended_live(SLIMMatrixTransferChart& mt_chart);

  void transfer_uvs_blended(
    SLIMMatrixTransferChart& mt_chart,
    float blend);

  void parametrize_single_iteration(SLIMMatrixTransferChart& mt_chart);

  void parametrize_live(
    SLIMMatrixTransferChart& mt_chart,
    int n_pins,
    std::vector<int>& pinned_vertex_indices,
    std::vector<double>& pinned_vertex_positions_2D,
    int n_selected_pins,
    std::vector<int>& selected_pins);

  igl::SLIMData* setup(
    SLIMMatrixTransferChart& mt_chart,
    bool are_border_vertices_pinned,
    bool skip_initialization) const;

  void free_data(igl::SLIMData* slim_data);
};
