/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "slim.h"
#include "slim_matrix_transfer.h"
#include <igl/Timer.h>

using namespace igl;

namespace slim {
Eigen::MatrixXd get_interactive_result_blended_with_original(float blend,
                                                             const SLIMData& slim_data);
void try_slim_solve(SLIMMatrixTransfer *mt, SLIMMatrixTransferChart& mt_chart, int iter_num);

void transfer_uvs_back_to_native_part_live(SLIMMatrixTransferChart& mt_chart,
                                           Eigen::MatrixXd &uv);

void transfer_uvs_back_to_native_part(SLIMMatrixTransferChart& mt_chart,
                                      Eigen::MatrixXd &uv);

void param_slim_single_iteration(SLIMMatrixTransfer *mt, SLIMMatrixTransferChart& mt_chart);

void param_slim_live_unwrap(SLIMMatrixTransfer *mt,
                            SLIMMatrixTransferChart& mt_chart,
                            int n_pins,
                            std::vector<int>& pinned_vertex_indices,
                            std::vector<double>& pinned_vertex_positions2d,
                            int n_selected_pins,
                            std::vector<int>& selected_pins);

void param_slim(SLIMMatrixTransfer *mt,
                int n_iterations,
                bool fix_border,
                bool skip_initialization);
}  // namespace slim
