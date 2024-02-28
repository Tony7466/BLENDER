/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "COM_GaussianAlphaYBlurOperation.h"

namespace blender::compositor {

GaussianAlphaYBlurOperation::GaussianAlphaYBlurOperation()
    : GaussianAlphaBlurBaseOperation(eDimension::Y)
{
}

/* TODO(manzanilla): to be removed with tiled implementation. */
void GaussianAlphaYBlurOperation::init_execution()
{
  GaussianAlphaBlurBaseOperation::init_execution();

  init_mutex();
}

/* TODO(manzanilla): to be removed with tiled implementation. */
void GaussianAlphaYBlurOperation::update_gauss()
{
  if (gausstab_ == nullptr) {
    update_size();
    float rad = max_ff(size_ * data_.sizey, 0.0f);
    rad = min_ff(rad, MAX_GAUSSTAB_RADIUS);
    filtersize_ = min_ii(ceil(rad), MAX_GAUSSTAB_RADIUS);

    gausstab_ = BlurBaseOperation::make_gausstab(rad, filtersize_);
  }

  if (distbuf_inv_ == nullptr) {
    update_size();
    float rad = max_ff(size_ * data_.sizey, 0.0f);
    filtersize_ = min_ii(ceil(rad), MAX_GAUSSTAB_RADIUS);

    distbuf_inv_ = BlurBaseOperation::make_dist_fac_inverse(rad, filtersize_, falloff_);
  }
}

void GaussianAlphaYBlurOperation::deinit_execution()
{
  GaussianAlphaBlurBaseOperation::deinit_execution();

  if (gausstab_) {
    MEM_freeN(gausstab_);
    gausstab_ = nullptr;
  }

  if (distbuf_inv_) {
    MEM_freeN(distbuf_inv_);
    distbuf_inv_ = nullptr;
  }

  deinit_mutex();
}

}  // namespace blender::compositor
