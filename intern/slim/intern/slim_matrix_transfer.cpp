/* SPDX-License-Identifier: GPL-2.0-or-later */

#include <Eigen/Dense>

#include "slim.h"
#include "slim_matrix_transfer.h"
#include "slim_parametrizer.h"

#include "area_compensation.h"

#include <igl/Timer.h>

using namespace igl;
using namespace slim;

void SLIMMatrixTransfer::transfer_uvs_blended_live(SLIMMatrixTransferChart& mt_chart)
{
  if (!mt_chart.succeeded) {
    return;
  }
  correct_map_surface_area_if_necessary(mt_chart.data);
  transfer_uvs_back_to_native_part_live(mt_chart, mt_chart.data->V_o);
}

/* Called from the native part during each iteration of interactive parametrisation.
 * The blend parameter decides the linear blending between the original UV map and the one
 * optained from the accumulated SLIM iterations so far. */
void SLIMMatrixTransfer::transfer_uvs_blended(
                               SLIMMatrixTransferChart& mt_chart,
                               float blend)
{
  if (!mt_chart.succeeded) {
    return;
  }

  Eigen::MatrixXd blended_uvs = get_interactive_result_blended_with_original(blend, mt_chart.data);
  correct_map_surface_area_if_necessary(mt_chart.data);
  transfer_uvs_back_to_native_part(mt_chart, blended_uvs);
}

/* Setup call from the native C part. Necessary for interactive parametrisation. */
SLIMData* SLIMMatrixTransfer::setup(
                 SLIMMatrixTransferChart& mt_chart,
                 bool are_border_vertices_pinned,
                 bool skip_initialization) const
{
  igl::Timer timer;
  timer.start();
  SLIMData *slim_data = setup_slim(
      *this, mt_chart, 0, timer, are_border_vertices_pinned, skip_initialization);
  return slim_data;
}

/* Executes a single iteration of SLIM, to be called from the native part. It recasts the pointer
 * to a SLIM object. */
void SLIMMatrixTransfer::parametrize_single_iteration(SLIMMatrixTransferChart& mt_chart)
{
  param_slim_single_iteration(this, mt_chart);
}

/* Executes slim iterations during live unwrap. needs to provide new selected-pin positions. */
void SLIMMatrixTransfer::parametrize_live(
                           SLIMMatrixTransferChart& mt_chart,
                           int n_pins,
                           std::vector<int>& pinned_vertex_indices,
                           std::vector<double>& pinned_vertex_positions_2D,
                           int n_selected_pins,
                           std::vector<int>& selected_pins)
{
  param_slim_live_unwrap(this,
                         mt_chart,
                         n_pins,
                         pinned_vertex_indices,
                         pinned_vertex_positions_2D,
                         n_selected_pins,
                         selected_pins);
}

void SLIMMatrixTransfer::parametrize(
                      int n_iterations,
                      bool are_border_vertices_pinned,
                      bool skip_initialization)
{
  param_slim(this, n_iterations, are_border_vertices_pinned, skip_initialization);
}

void SLIMMatrixTransfer::free_data(SLIMData* slim_data)
{
  free_slim_data(slim_data);
}
