/* SPDX-License-Identifier: GPL-2.0-or-later */

#include <Eigen/Dense>

#include "slim.h"
#include "slim_matrix_transfer.h"
#include "slim_parametrizer.h"

#include "area_compensation.h"

#include <igl/Timer.h>

using namespace igl;
using namespace slim;

void SLIMMatrixTransfer::transfer_uvs_blended_live(
                                    void *slim_data_ptr,
                                    int uv_chart_index)
{
  if (!succeeded[uv_chart_index]) {
    return;
  }
  SLIMData *slim_data = (SLIMData *)slim_data_ptr;
  correct_map_surface_area_if_necessary(slim_data);
  transfer_uvs_back_to_native_part_live(this, slim_data->V_o, uv_chart_index);
}

/* Called from the native part during each iteration of interactive parametrisation.
 * The blend parameter decides the linear blending between the original UV map and the one
 * optained from the accumulated SLIM iterations so far. */
void SLIMMatrixTransfer::transfer_uvs_blended(
                               void *slim_data_ptr,
                               int uv_chart_index,
                               float blend)
{
  if (!succeeded[uv_chart_index]) {
    return;
  }

  SLIMData *slim_data = (SLIMData *)slim_data_ptr;
  Eigen::MatrixXd blended_uvs = get_interactive_result_blended_with_original(blend, slim_data);
  correct_map_surface_area_if_necessary(slim_data);
  transfer_uvs_back_to_native_part(this, blended_uvs, uv_chart_index);
}

/* Setup call from the native C part. Necessary for interactive parametrisation. */
void* SLIMMatrixTransfer::setup(
                 int uv_chart_index,
                 bool are_border_vertices_pinned,
                 bool skip_initialization) const
{
  igl::Timer timer;
  timer.start();
  SLIMData *slim_data = setup_slim(
      this, 0, uv_chart_index, timer, are_border_vertices_pinned, skip_initialization);
  return slim_data;
}

/* Executes a single iteration of SLIM, to be called from the native part. It recasts the pointer
 * to a SLIM object. */
void SLIMMatrixTransfer::parametrize_single_iteration(
                                       int uv_chart_index,
                                       void *slim_data_ptr)
{
  SLIMData *slim_data = (SLIMData *)slim_data_ptr;
  param_slim_single_iteration(this, uv_chart_index, slim_data);
}

/* Executes slim iterations during live unwrap. needs to provide new selected-pin positions. */
void SLIMMatrixTransfer::parametrize_live(
                           int uv_chart_index,
                           void *slim_data_ptr,
                           int n_pins,
                           int *pinned_vertex_indices,
                           double *pinned_vertex_positions_2D,
                           int n_selected_pins,
                           int *selected_pins)
{
  SLIMData *slim_data = (SLIMData *)slim_data_ptr;
  param_slim_live_unwrap(this,
                         uv_chart_index,
                         slim_data,
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

void SLIMMatrixTransfer::free_data(void *slim_data_ptr)
{
  free_slim_data((SLIMData *)slim_data_ptr);
}
