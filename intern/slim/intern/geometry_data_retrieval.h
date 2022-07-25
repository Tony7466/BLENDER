/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <Eigen/Dense>
#include <stdio.h>

#include "slim.h"
#include "slim_matrix_transfer.h"
#include "uv_initializer.h"

using namespace Eigen;
using namespace igl;

namespace slim {

struct GeometryData {
  Map<MatrixXd> vertex_positions3d = Map<MatrixXd>(NULL, 0, 0);
  Map<MatrixXd> uv_positions2d = Map<MatrixXd>(NULL, 0, 0);
  MatrixXd positions_of_pinned_vertices2d;
  Map<Matrix<double, Dynamic, Dynamic, RowMajor>> positions_of_explicitly_pinned_vertices2d =
      Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(NULL, 0, 0);

  Map<MatrixXi> faces_by_vertexindices = Map<MatrixXi>(NULL, 0, 0);
  Map<MatrixXi> edges_by_vertexindices = Map<MatrixXi>(NULL, 0, 0);
  VectorXi pinned_vertex_indices;
  Map<VectorXi> explicitly_pinned_vertex_indices = Map<VectorXi>(NULL, 0);

  Map<VectorXd> edge_lengths = Map<VectorXd>(NULL, 0);
  Map<VectorXi> boundary_vertex_indices = Map<VectorXi>(NULL, 0);
  Map<VectorXf> weights_per_vertex = Map<VectorXf>(NULL, 0);

  int columns_2 = 2;
  int columns_3 = 3;
  int number_of_vertices;
  int number_of_faces;
  int number_of_edges_twice;
  int number_of_boundary_vertices;
  int number_of_pinned_vertices;

  bool with_weighted_parameteriztion;
  double weight_influence;
};

void construct_slim_data(const GeometryData &gd,
                         SLIMData *slim_data,
                         bool skip_initialization,
                         int reflection_mode,
                         double relative_scale);

void retrieve_pinned_vertices(GeometryData &gd, bool border_vertices_are_pinned);

void retrieve_geometry_data_matrices(const SLIMMatrixTransfer *transferred_data,
                                     const int uv_chart_index,
                                     GeometryData &gd);
}  // namespace slim
