/* SPDX-License-Identifier: GPL-2.0-or-later */

#include <Eigen/Dense>

#include "slim.h"
#include "slim_matrix_transfer.h"

#include "BLI_assert.h"
#include "area_compensation.h"
#include "geometry_data_retrieval.h"
#include "least_squares_relocator.h"

namespace slim {

using namespace igl;

SLIMMatrixTransferChart::SLIMMatrixTransferChart() = default;
SLIMMatrixTransferChart::SLIMMatrixTransferChart(SLIMMatrixTransferChart &&) = default;
SLIMMatrixTransferChart::~SLIMMatrixTransferChart() = default;

void SLIMMatrixTransferChart::free_slim_data()
{
  data.reset(nullptr);
};

/* Setup call from the native C part. Necessary for interactive parametrisation. */
void SLIMMatrixTransfer::setup_slim_data(SLIMMatrixTransferChart &mt_chart,
                                         bool are_border_vertices_pinned,
                                         bool skip_initialization) const
{
  setup_slim_data(mt_chart, 0, are_border_vertices_pinned, skip_initialization);
}

SLIMMatrixTransfer::SLIMMatrixTransfer() = default;
SLIMMatrixTransfer::~SLIMMatrixTransfer() = default;

void initialize_uvs(GeometryData &gd, SLIMData &slim_data)
{
  MatrixXd vertex_positions2d = slim_data.V;
  MatrixXi faces_by_vertex_indices = slim_data.F;
  VectorXi boundary_vertex_indices = gd.boundary_vertex_indices;
  MatrixXd uv_positions2d = slim_data.V_o;

  Eigen::MatrixXd uv_positions_of_boundary(boundary_vertex_indices.rows(), 2);
  map_vertices_to_convex_border(uv_positions_of_boundary);

  bool all_vertices_on_boundary = (slim_data.V_o.rows() == uv_positions_of_boundary.rows());
  if (all_vertices_on_boundary) {
    slim_data.V_o = uv_positions_of_boundary;
    return;
  }

  mvc(gd.faces_by_vertexindices,
      gd.vertex_positions3d,
      gd.edges_by_vertexindices,
      gd.edge_lengths,
      boundary_vertex_indices,
      uv_positions_of_boundary,
      slim_data.V_o);
}

void initialize_if_needed(GeometryData &gd, SLIMData &slim_data)
{
  BLI_assert(slim_data.valid);

  if (!slim_data.skipInitialization) {
    initialize_uvs(gd, slim_data);
  }
}

/* Transfers all the matrices from the native part and initialises SLIM. */
void SLIMMatrixTransfer::setup_slim_data(SLIMMatrixTransferChart &mt_chart,
                                         int n_iterations,
                                         bool border_vertices_are_pinned,
                                         bool skip_initialization) const
{
  SLIMDataPtr slim_data = std::make_unique<SLIMDataPtr::element_type>();

  try {
    if (!mt_chart.succeeded) {
      throw SlimFailedException();
    }

    GeometryData geometry_data(*this, mt_chart);

    geometry_data.retrieve_pinned_vertices(border_vertices_are_pinned);
    mt_chart.n_pinned_vertices = geometry_data.number_of_pinned_vertices;

    geometry_data.construct_slim_data(*slim_data, skip_initialization, reflection_mode);
    slim_data->nIterations = n_iterations;

    initialize_if_needed(geometry_data, *slim_data);
    transform_initialization_if_necessary(*slim_data);

    correct_mesh_surface_area_if_necessary(*slim_data);

    slim_precompute(slim_data->V,
                    slim_data->F,
                    slim_data->V_o,
                    *slim_data,
                    slim_data->slim_energy,
                    slim_data->b,
                    slim_data->bc,
                    slim_data->soft_const_p);
  }
  catch (SlimFailedException &) {
    slim_data->valid = false;
    mt_chart.succeeded = false;
  }

  mt_chart.data = std::move(slim_data);
}

}  // namespace slim