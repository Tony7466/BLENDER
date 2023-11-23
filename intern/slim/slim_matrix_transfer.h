/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <memory>
#include <vector>

namespace slim {

struct SLIMData;

typedef std::unique_ptr<SLIMData> SLIMDataPtr;

/* MatrixTransferChart holds all information and data matrices to be
 * transferred from Blender to SLIM. */
struct MatrixTransferChart {
  int n_verts = 0;
  int n_faces = 0;
  int n_pinned_vertices = 0;
  int n_boundary_vertices = 0;
  int n_edges = 0;

  bool succeeded = false;

  /* Vertex positions. */
  std::vector<double> v_matrices;
  /* UV positions of vertices. */
  std::vector<double> uv_matrices;
  /* Positions of pinned vertices. */
  std::vector<double> pp_matrices;
  /* Edge lengths. */
  std::vector<double> el_vectors;
  /* Weights per vertex. */
  std::vector<float> w_vectors;

  /* Vertex index triplets making up faces. */
  std::vector<int> f_matrices;
  /* Indices of pinned vertices. */
  std::vector<int> p_matrices;
  /* Vertex index tuples making up edges. */
  std::vector<int> e_matrices;
  /* Vertex indices of boundary vertices. */
  std::vector<int> b_vectors;

  SLIMDataPtr data;

  MatrixTransferChart();
  MatrixTransferChart(MatrixTransferChart &&);

  MatrixTransferChart(const MatrixTransferChart &) = delete;
  MatrixTransferChart &operator=(const MatrixTransferChart &) = delete;

  ~MatrixTransferChart();

  void try_slim_solve(int iter_num);
  void parametrize_single_iteration();

  void transfer_uvs_blended_live();
  void transfer_uvs_blended(float blend);

  void free_slim_data();
};

struct MatrixTransfer {
  bool fixed_boundary = false;
  bool use_weights = false;
  double weight_influence = 0.0;
  int reflection_mode = 0;
  int n_iterations = 0;
  bool skip_initialization = false;
  bool is_minimize_stretch = false;

  std::vector<MatrixTransferChart> charts;

  MatrixTransfer();
  MatrixTransfer(const MatrixTransfer &) = delete;
  MatrixTransfer &operator=(const MatrixTransfer &) = delete;
  ~MatrixTransfer();

  void parametrize();

  void parametrize_live(MatrixTransferChart &chart,
                        int n_pins,
                        const std::vector<int> &pinned_vertex_indices,
                        const std::vector<double> &pinned_vertex_positions_2D,
                        int n_selected_pins,
                        const std::vector<int> &selected_pins);

  void setup_slim_data(MatrixTransferChart &chart, int n_iterations = 0) const;
};

}  // namespace slim
