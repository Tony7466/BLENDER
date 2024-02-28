/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "COM_GaussianBlurBaseOperation.h"

namespace blender::compositor {

/* TODO(manzanilla): everything to be removed with tiled implementation except the constructor. */
class GaussianYBlurOperation : public GaussianBlurBaseOperation {
 private:
  void update_gauss();

 public:
  GaussianYBlurOperation();

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
