/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "COM_MultiThreadedOperation.h"

namespace blender::compositor {

class KuwaharaAnisotropicStructureTensorOperation : public MultiThreadedOperation {
  SocketReader *image_reader_;

 public:
  KuwaharaAnisotropicStructureTensorOperation();

  void init_execution() override;
  void deinit_execution() override;
  void update_memory_buffer_partial(MemoryBuffer *output,
                                    const rcti &area,
                                    Span<MemoryBuffer *> inputs) override;
};

}  // namespace blender::compositor
