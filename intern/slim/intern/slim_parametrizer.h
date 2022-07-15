/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "slim.h"
#include "slim_matrix_transfer.h"
#include <igl/Timer.h>

using namespace igl;

/*	The header file that exposes the C++ functions to the native C part of Blender, see thesis.
 */

Eigen::MatrixXd getInteractiveResultBlendedWithOriginal(float blend, SLIMData *slimData);
SLIMData *setup_slim(SLIMMatrixTransfer *transferredData,
                     int nIterations,
                     int uvChartIndex,
                     igl::Timer &timer,
                     bool borderVerticesArePinned,
                     bool skipInitialization);
void try_slim_solve(SLIMMatrixTransfer *mt, int uvChartIndex, SLIMData &data, int iter_num);
void transferUvsBackToNativePartLive(SLIMMatrixTransfer *mt,
                                     Eigen::MatrixXd &UV,
                                     int uvChartIndex);
void transferUvsBackToNativePart(SLIMMatrixTransfer *mt, Eigen::MatrixXd &UV, int uvChartIndex);
void param_slim_single_iteration(SLIMMatrixTransfer *mt, int uv_chart_index, SLIMData *slimData);
void param_slim_live_unwrap(SLIMMatrixTransfer *mt,
                            int uv_chart_index,
                            SLIMData *slimData,
                            int n_pins,
                            int *pinnedVertexIndices,
                            double *pinnedVertexPositions2D,
                            int n_selected_pins,
                            int *selected_pins);
void param_slim(SLIMMatrixTransfer *mt, int n_iterations, bool fixBorder, bool skipInitialization);
void free_slim_data(SLIMData *slimData);
