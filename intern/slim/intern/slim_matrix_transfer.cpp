/* SPDX-License-Identifier: GPL-2.0-or-later */

#include <Eigen/Dense>

#include "slim.h"
#include "slim_matrix_transfer.h"
#include "slim_parametrizer.h"

#include "area_compensation.h"

#include <igl/Timer.h>

namespace slim {

using namespace igl;

SLIMMatrixTransferChart::SLIMMatrixTransferChart() = default;
SLIMMatrixTransferChart::SLIMMatrixTransferChart(SLIMMatrixTransferChart&&) = default;
SLIMMatrixTransferChart::~SLIMMatrixTransferChart() = default;

SLIMMatrixTransfer::SLIMMatrixTransfer() = default;
SLIMMatrixTransfer::~SLIMMatrixTransfer() = default;


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

/* Setup call from the native C part. Necessary for interactive parametrisation. */
void SLIMMatrixTransfer::setup_slim_data(
                 SLIMMatrixTransferChart& mt_chart,
                 bool are_border_vertices_pinned,
                 bool skip_initialization) const
{
  igl::Timer timer;
  timer.start();
  setup_slim_data(mt_chart, 0, timer, are_border_vertices_pinned, skip_initialization);
}

}