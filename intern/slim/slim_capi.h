/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "slim_matrix_transfer.h"

#ifdef __cplusplus
extern "C" {
#endif

void SLIM_parametrize(SLIMMatrixTransfer *mt,
                      int n_iterations,
                      bool are_border_vertices_pinned,
                      bool skip_initialization);
void SLIM_transfer_uvs_blended_live(SLIMMatrixTransfer *mt,
                                    void *slim_data_ptr,
                                    int uv_chart_index);
void SLIM_transfer_uvs_blended(SLIMMatrixTransfer *mt,
                               void *slim,
                               int uv_chart_index,
                               float blend);
void SLIM_parametrize_single_iteration(SLIMMatrixTransfer *mt, int uv_chart_index, void *slim);
void SLIM_parametrize_live(SLIMMatrixTransfer *mt,
                           int uv_chart_index,
                           void *slim_data_ptr,
                           int n_pins,
                           int *selected_pinned_vertex_indices,
                           double *selected_pinned_vertex_positions_2D,
                           int n_selected_pins,
                           int *selected_pins);
void *SLIM_setup(SLIMMatrixTransfer *mt,
                 int uv_chart_index,
                 bool are_border_vertices_pinned,
                 bool skip_initialization);
void SLIM_free_data(void *slim_data_ptr);

#ifdef __cplusplus
}
#endif
