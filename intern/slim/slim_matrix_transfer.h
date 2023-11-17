/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <memory>
#include <vector>

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
}  // namespace igl

namespace slim {

typedef std::unique_ptr<igl::SLIMData> SLIMDataPtr;

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

  SLIMDataPtr data;

  SLIMMatrixTransferChart();
  SLIMMatrixTransferChart(SLIMMatrixTransferChart &&);

  SLIMMatrixTransferChart(const SLIMMatrixTransferChart &) = delete;
  SLIMMatrixTransferChart &operator=(const SLIMMatrixTransferChart &) = delete;

  ~SLIMMatrixTransferChart();

  void try_slim_solve(int iter_num);
  void parametrize_single_iteration();

  void transfer_uvs_blended_live();
  void transfer_uvs_blended(float blend);

  void free_slim_data();
};

struct SLIMMatrixTransfer {
  int n_charts = 0;

  bool fixed_boundary = false;
  bool pinned_vertices = false;
  bool with_weighted_parameterization = false;
  double weight_influence = 0.0;
  int reflection_mode = 0;
  double relative_scale = 0.0;

  /* External. */
  int n_iterations = 0;
  bool skip_initialization = false;
  bool is_minimize_stretch = false;

  std::vector<SLIMMatrixTransferChart> mt_charts;

  SLIMMatrixTransfer();
  SLIMMatrixTransfer(const SLIMMatrixTransfer &) = delete;
  SLIMMatrixTransfer &operator=(const SLIMMatrixTransfer &) = delete;
  ~SLIMMatrixTransfer();

  void parametrize(int n_iterations, bool are_border_vertices_pinned, bool skip_initialization);

  void parametrize_live(SLIMMatrixTransferChart &mt_chart,
                        int n_pins,
                        const std::vector<int> &pinned_vertex_indices,
                        const std::vector<double> &pinned_vertex_positions_2D,
                        int n_selected_pins,
                        const std::vector<int> &selected_pins);

  void setup_slim_data(SLIMMatrixTransferChart &mt_chart,
                       bool are_border_vertices_pinned,
                       bool skip_initialization) const;

  void setup_slim_data(SLIMMatrixTransferChart &mt_chart,
                       int n_iterations,
                       bool border_vertices_are_pinned,
                       bool skip_initialization) const;
};

}  // namespace slim
