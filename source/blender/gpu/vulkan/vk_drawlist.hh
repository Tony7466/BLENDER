/* SPDX-FileCopyrightText: 2022 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "BLI_vector.hh"

#include "gpu_drawlist_private.hh"

#include "vk_storage_buffer.hh"

namespace blender::gpu {
class VKBatch;

struct GLDrawCommandIndexed {};

class VKDrawList : public DrawList {
 private:
  VKBatch *batch_ = nullptr;
  VKStorageBuffer command_buffer_;
  int length_;
  int command_index_ = 0;

 public:
  VKDrawList(int list_length);
  void append(GPUBatch *batch, int instance_first, int instance_count) override;
  void submit() override;

 private:
  bool is_indexed() const;

  template<typename CommandType> CommandType &get_command() const
  {
    return MutableSpan<CommandType>(
        static_cast<CommandType *>(command_buffer_.buffer_get().mapped_memory_get()),
        length_)[command_index_];
  }
};

}  // namespace blender::gpu
