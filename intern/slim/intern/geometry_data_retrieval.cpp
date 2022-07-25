/* SPDX-License-Identifier: GPL-2.0-or-later */


#include "geometry_data_retrieval.h"

#include <iostream>

#include "slim.h"
#include "slim_matrix_transfer.h"
#include <Eigen/Dense>

#include "BLI_assert.h"

using namespace igl;
using namespace Eigen;

namespace slim {

void create_weights_per_face(SLIMData *slim_data)
{
  if (!slim_data->valid) {
    return;
  }

  if (!slim_data->withWeightedParameterization) {
    slim_data->weightPerFaceMap = Eigen::VectorXf::Ones(slim_data->F.rows());
    return;
  }

  std::cout << "weightmap: " << slim_data->weightmap << std::endl;

  slim_data->weightPerFaceMap = Eigen::VectorXf(slim_data->F.rows());

  /* the actual weight is max_factor ^ (2 * (mean - 0.5)) */
  int weight_influence_sign = (slim_data->weightInfluence >= 0) ? 1 : -1;
  double max_factor = std::abs(slim_data->weightInfluence) + 1;

  for (int fid = 0; fid < slim_data->F.rows(); fid++) {
    Eigen::RowVector3i row = slim_data->F.row(fid);
    float w1, w2, w3, mean, weight_factor, flipped_mean;
    w1 = slim_data->weightmap(row(0));
    w2 = slim_data->weightmap(row(1));
    w3 = slim_data->weightmap(row(2));
    mean = (w1 + w2 + w3) / 3;
    flipped_mean = 1 - mean;

    weight_factor = std::pow(max_factor, weight_influence_sign * 2 * (flipped_mean - 0.5));
    slim_data->weightPerFaceMap(fid) = weight_factor;
  }
}

void set_geometry_data_matrices(const GeometryData &gd, SLIMData *slim_data)
{
  if (!slim_data->valid) {
    return;
  }

  slim_data->V = gd.vertex_positions3d;
  slim_data->F = gd.faces_by_vertexindices;
  slim_data->b = gd.pinned_vertex_indices;
  slim_data->bc = gd.positions_of_pinned_vertices2d;
  slim_data->V_o = gd.uv_positions2d;
  slim_data->oldUVs = gd.uv_positions2d;
  slim_data->weightmap = gd.weights_per_vertex;
  create_weights_per_face(slim_data);
}

bool has_valid_preinitialized_map(const GeometryData &gd)
{
  if (gd.uv_positions2d.rows() == gd.vertex_positions3d.rows() &&
      gd.uv_positions2d.cols() == gd.columns_2) {

    int number_of_flips = count_flips(
        gd.vertex_positions3d, gd.faces_by_vertexindices, gd.uv_positions2d);
    bool no_flips_present = (number_of_flips == 0);
    return (no_flips_present);
  }
  return false;
}

/*
 If we use interactive parametrisation, we usually start form an existing, flip-free unwrapping.
 Also, pinning of vertices has some issues with initialisation with convex border.
 We therefore may want to skip initialization. however, to skip initialization we need a
 preexisting valid starting map.
 */
bool can_initialization_be_skipped(const GeometryData &gd, bool skip_initialization)
{
  return (skip_initialization && has_valid_preinitialized_map(gd));
}

void construct_slim_data(const GeometryData &gd,
                       SLIMData *slim_data,
                       bool skip_initialization,
                       int reflection_mode,
                       double relative_scale)
{
  BLI_assert(slim_data->valid);

  slim_data->skipInitialization = can_initialization_be_skipped(gd, skip_initialization);
  slim_data->weightInfluence = gd.weight_influence;
  slim_data->relativeScale = relative_scale;
  slim_data->reflection_mode = reflection_mode;
  slim_data->withWeightedParameterization = gd.with_weighted_parameteriztion;
  set_geometry_data_matrices(gd, slim_data);

  double penalty_for_violating_pinned_positions = pow(10, 9);
  slim_data->soft_const_p = penalty_for_violating_pinned_positions;
  slim_data->slim_energy = SLIMData::SYMMETRIC_DIRICHLET;
}

void combine_matrices_of_pinned_and_boundary_vertices(GeometryData &gd)
{
  // over-allocate pessimistically to avoid multiple reallocation
  int upper_bound_on_number_of_pinned_vertices = gd.number_of_boundary_vertices + gd.number_of_pinned_vertices;
  gd.pinned_vertex_indices = VectorXi(upper_bound_on_number_of_pinned_vertices);
  gd.positions_of_pinned_vertices2d = MatrixXd(upper_bound_on_number_of_pinned_vertices, gd.columns_2);

  // since border vertices use vertex indices 0 ... #bordervertices we can do:
  gd.pinned_vertex_indices.segment(0, gd.number_of_boundary_vertices) = gd.boundary_vertex_indices;
  gd.positions_of_pinned_vertices2d.block(0, 0, gd.number_of_boundary_vertices, gd.columns_2) =
      gd.uv_positions2d.block(0, 0, gd.number_of_boundary_vertices, gd.columns_2);

  int index = gd.number_of_boundary_vertices;
  int highest_vertex_index = (gd.boundary_vertex_indices)(index - 1);

  for (Map<VectorXi>::InnerIterator it(gd.explicitly_pinned_vertex_indices, 0); it; ++it) {
    int vertex_index = it.value();
    if (vertex_index > highest_vertex_index) {
      gd.pinned_vertex_indices(index) = vertex_index;
      gd.positions_of_pinned_vertices2d.row(index) = gd.uv_positions2d.row(vertex_index);
      index++;
    }
  }

  int actual_number_of_pinned_vertices = index;
  gd.pinned_vertex_indices.conservativeResize(actual_number_of_pinned_vertices);
  gd.positions_of_pinned_vertices2d.conservativeResize(actual_number_of_pinned_vertices, gd.columns_2);

  gd.number_of_pinned_vertices = actual_number_of_pinned_vertices;
}

/*
 if the border is fixed, we simply pin the border vertices additionally to other pinned vertices.
 */
void retrieve_pinned_vertices(GeometryData &gd, bool border_vertices_are_pinned)
{
  if (border_vertices_are_pinned) {
    combine_matrices_of_pinned_and_boundary_vertices(gd);
  }
  else {
    gd.pinned_vertex_indices = VectorXi(gd.explicitly_pinned_vertex_indices);
    gd.positions_of_pinned_vertices2d = MatrixXd(gd.positions_of_explicitly_pinned_vertices2d);
  }
}

void retrieve_geometry_data_matrices(const SLIMMatrixTransfer *transferred_data,
                                  const int uv_chart_index,
                                  GeometryData &gd)
{
  gd.number_of_vertices = transferred_data->n_verts[uv_chart_index];
  gd.number_of_faces = transferred_data->n_faces[uv_chart_index];
  // n_edges in transferred_data accounts for boundary edges only once
  gd.number_of_edges_twice = transferred_data->n_edges[uv_chart_index] +
                          transferred_data->n_boundary_vertices[uv_chart_index];
  gd.number_of_boundary_vertices = transferred_data->n_boundary_vertices[uv_chart_index];
  gd.number_of_pinned_vertices = transferred_data->n_pinned_vertices[uv_chart_index];

  new (&gd.vertex_positions3d)
      Map<MatrixXd>(transferred_data->v_matrices[uv_chart_index], gd.number_of_vertices, gd.columns_3);
  new (&gd.uv_positions2d)
      Map<MatrixXd>(transferred_data->uv_matrices[uv_chart_index], gd.number_of_vertices, gd.columns_2);
  gd.positions_of_pinned_vertices2d = MatrixXd();

  new (&gd.faces_by_vertexindices)
      Map<MatrixXi>(transferred_data->f_matrices[uv_chart_index], gd.number_of_faces, gd.columns_3);
  new (&gd.edges_by_vertexindices) Map<MatrixXi>(
      transferred_data->e_matrices[uv_chart_index], gd.number_of_edges_twice, gd.columns_2);
  gd.pinned_vertex_indices = VectorXi();

  new (&gd.edge_lengths)
      Map<VectorXd>(transferred_data->el_vectors[uv_chart_index], gd.number_of_edges_twice);
  new (&gd.boundary_vertex_indices)
      Map<VectorXi>(transferred_data->b_vectors[uv_chart_index], gd.number_of_boundary_vertices);

  gd.with_weighted_parameteriztion = transferred_data->with_weighted_parameterization;
  new (&gd.weights_per_vertex)
      Map<VectorXf>(transferred_data->w_vectors[uv_chart_index], gd.number_of_vertices);
  gd.weight_influence = transferred_data->weight_influence;

  if (gd.number_of_pinned_vertices != 0) {
    new (&gd.explicitly_pinned_vertex_indices)
        Map<VectorXi>(transferred_data->p_matrices[uv_chart_index], gd.number_of_pinned_vertices);
    new (&gd.positions_of_explicitly_pinned_vertices2d)
        Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(
            transferred_data->pp_matrices[uv_chart_index], gd.number_of_pinned_vertices, gd.columns_2);
  }
}
}
