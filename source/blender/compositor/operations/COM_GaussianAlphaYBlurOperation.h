/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "COM_GaussianAlphaBlurBaseOperation.h"

namespace blender::compositor {

/* TODO(manzanilla): everything to be removed with tiled implementation except the constructor. */
class GaussianAlphaYBlurOperation : public GaussianAlphaBlurBaseOperation {
 private:
  void update_gauss();

 public:
  GaussianAlphaYBlurOperation();

  /**
   * \brief initialize the execution
   */
  void init_execution() override;

  /**
   * Deinitialize the execution
   */
  void deinit_execution() override;
};

}  // namespace blender::compositor
