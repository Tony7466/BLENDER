/* SPDX-License-Identifier: GPL-2.0-or-later */

#include <iostream>
#include <stdlib.h>

#include "area_compensation.h"
#include "geometry_data_retrieval.h"
#include "least_squares_relocator.h"
#include "slim.h"
#include "uv_initializer.h"

#include "BLI_assert.h"

#include "doublearea.h"
#include "map_vertices_to_circle.h"

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

namespace slim {

using namespace igl;
using namespace Eigen;

void transfer_uvs_back_to_native_part_live(SLIMMatrixTransferChart &mt_chart, Eigen::MatrixXd &uv)
{
  if (!mt_chart.succeeded) {
    return;
  }

  auto &uv_coordinate_array = mt_chart.uv_matrices;
  size_t uv_idx = 0;
  int number_of_vertices = mt_chart.n_verts;

  for (int i = 0; i < number_of_vertices; i++) {
    uv_coordinate_array[uv_idx++] = uv(i, 0);
    uv_coordinate_array[uv_idx++] = uv(i, 1);
  }
}

void transfer_uvs_back_to_native_part(SLIMMatrixTransferChart &mt_chart, Eigen::MatrixXd &uv)
{
  if (!mt_chart.succeeded) {
    return;
  }

  auto &uv_coordinate_array = mt_chart.uv_matrices;
  int number_of_vertices = mt_chart.n_verts;

  for (int i = 0; i < number_of_vertices; i++) {
    for (int j = 0; j < 2; j++) {
      uv_coordinate_array[i * 2 + j] = uv(i, j);
    }
  }
}

Eigen::MatrixXd get_interactive_result_blended_with_original(float blend,
                                                             const SLIMData &slim_data)
{
  Eigen::MatrixXd original_map_weighted = blend * slim_data.oldUVs;
  Eigen::MatrixXd interactive_result_map = (1.0 - blend) * slim_data.V_o;
  return original_map_weighted + interactive_result_map;
}

static void adjust_pins(SLIMData &slim_data,
                        int n_pins,
                        const std::vector<int> &pinned_vertex_indices,
                        const std::vector<double> &pinned_vertex_positions2d,
                        int n_selected_pins,
                        const std::vector<int> &selected_pins)
{
  if (!slim_data.valid) {
    return;
  }

  Eigen::VectorXi old_pin_indices = slim_data.b;
  Eigen::MatrixXd old_pin_positions = slim_data.bc;

  slim_data.b.resize(n_pins);
  slim_data.bc.resize(n_pins, 2);

  int old_pin_pointer = 0;
  int selected_pin_pointer = 0;

  for (int new_pin_pointer = 0; new_pin_pointer < n_pins; new_pin_pointer++) {

    int pinned_vertex_index = pinned_vertex_indices[new_pin_pointer];
    slim_data.b(new_pin_pointer) = pinned_vertex_index;

    while ((old_pin_pointer < old_pin_indices.size()) &&
           (old_pin_indices(old_pin_pointer) < pinned_vertex_index))
    {
      ++old_pin_pointer;
    }
    bool old_pointer_valid = (old_pin_pointer < old_pin_indices.size()) &&
                             (old_pin_indices(old_pin_pointer) == pinned_vertex_index);

    while ((selected_pin_pointer < n_selected_pins) &&
           (selected_pins[selected_pin_pointer] < pinned_vertex_index))
    {
      ++selected_pin_pointer;
    }
    bool pin_selected = (selected_pin_pointer < n_selected_pins) &&
                        (selected_pins[selected_pin_pointer] == pinned_vertex_index);

    if (!pin_selected && old_pointer_valid) {
      slim_data.bc.row(new_pin_pointer) = old_pin_positions.row(old_pin_pointer);
    }
    else {
      slim_data.bc(new_pin_pointer, 0) = pinned_vertex_positions2d[2 * new_pin_pointer];
      slim_data.bc(new_pin_pointer, 1) = pinned_vertex_positions2d[2 * new_pin_pointer + 1];
    }
  }
}

void SLIMMatrixTransferChart::transfer_uvs_blended_live()
{
  if (!succeeded) {
    return;
  }
  correct_map_surface_area_if_necessary(*data);
  transfer_uvs_back_to_native_part_live(*this, data->V_o);
}

/* Called from the native part during each iteration of interactive parametrisation.
 * The blend parameter decides the linear blending between the original UV map and the one
 * optained from the accumulated SLIM iterations so far. */
void SLIMMatrixTransferChart::transfer_uvs_blended(float blend)
{
  if (!succeeded) {
    return;
  }

  Eigen::MatrixXd blended_uvs = get_interactive_result_blended_with_original(blend, *data);
  correct_map_surface_area_if_necessary(*data);
  transfer_uvs_back_to_native_part(*this, blended_uvs);
}

void SLIMMatrixTransferChart::try_slim_solve(int iter_num)
{
  if (!succeeded) {
    return;
  }

  try {
    slim_solve(*data, iter_num);
  }
  catch (SlimFailedException &) {
    succeeded = false;
  }
}

/* Executes a single iteration of SLIM, must follow a proper setup & initialisation. */
void SLIMMatrixTransferChart::parametrize_single_iteration()
{
  int number_of_iterations = 1;
  try_slim_solve(number_of_iterations);
}

/* Executes slim iterations during live unwrap. needs to provide new selected-pin positions. */
void SLIMMatrixTransfer::parametrize_live(SLIMMatrixTransferChart &mt_chart,
                                          int n_pins,
                                          const std::vector<int> &pinned_vertex_indices,
                                          const std::vector<double> &pinned_vertex_positions_2D,
                                          int n_selected_pins,
                                          const std::vector<int> &selected_pins)
{
  int number_of_iterations = 3;
  adjust_pins(*mt_chart.data,
              n_pins,
              pinned_vertex_indices,
              pinned_vertex_positions_2D,
              n_selected_pins,
              selected_pins);

  mt_chart.try_slim_solve(number_of_iterations);
}

void SLIMMatrixTransfer::parametrize(int n_iterations,
                                     bool are_border_vertices_pinned,
                                     bool skip_initialization)
{
  for (int uv_chart_index = 0; uv_chart_index < n_charts; uv_chart_index++) {
    SLIMMatrixTransferChart &mt_chart = mt_charts[uv_chart_index];
    setup_slim_data(mt_chart, n_iterations, are_border_vertices_pinned, skip_initialization);

    mt_chart.try_slim_solve(n_iterations);

    correct_map_surface_area_if_necessary(*mt_chart.data);
    transfer_uvs_back_to_native_part(mt_chart, mt_chart.data->V_o);

    mt_chart.free_slim_data();
  }
};

}  // namespace slim
