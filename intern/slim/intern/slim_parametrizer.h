/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "slim.h"
#include "slim_matrix_transfer.h"
#include <igl/Timer.h>

using namespace igl;

namespace slim {
Eigen::MatrixXd get_interactive_result_blended_with_original(float blend,
                                                             const SLIMData *slim_data);
SLIMData *setup_slim(const SLIMMatrixTransfer *transferred_data,
                     int n_iterations,
                     int uv_chart_index,
                     igl::Timer &timer,
                     bool border_vertices_are_pinned,
                     bool skip_initialization);
void try_slim_solve(SLIMMatrixTransfer *mt, int uv_chart_index, SLIMData &data, int iter_num);
void transfer_uvs_back_to_native_part_live(SLIMMatrixTransfer *mt,
                                           Eigen::MatrixXd &uv,
                                           int uv_chart_index);
void transfer_uvs_back_to_native_part(SLIMMatrixTransfer *mt,
                                      Eigen::MatrixXd &uv,
                                      int uv_chart_index);
void param_slim_single_iteration(SLIMMatrixTransfer *mt, int uv_chart_index, SLIMData *slim_data);
void param_slim_live_unwrap(SLIMMatrixTransfer *mt,
                            int uv_chart_index,
                            SLIMData *slim_data,
                            int n_pins,
                            int *pinned_vertex_indices,
                            double *pinned_vertex_positions2d,
                            int n_selected_pins,
                            int *selected_pins);
void param_slim(SLIMMatrixTransfer *mt,
                int n_iterations,
                bool fix_border,
                bool skip_initialization);
void free_slim_data(SLIMData *slim_data);
}  // namespace slim
