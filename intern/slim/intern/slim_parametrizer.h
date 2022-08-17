/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "slim.h"
#include "slim_matrix_transfer.h"
#include <igl/Timer.h>

using namespace igl;

namespace slim {
Eigen::MatrixXd get_interactive_result_blended_with_original(float blend,
                                                             const SLIMData& slim_data);

void transfer_uvs_back_to_native_part_live(SLIMMatrixTransferChart& mt_chart,
                                           Eigen::MatrixXd &uv);

void transfer_uvs_back_to_native_part(SLIMMatrixTransferChart& mt_chart,
                                      Eigen::MatrixXd &uv);

}  // namespace slim
