/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#if !defined(GPU_SHADER) && !defined(GLSL_CPP_STUBS)
#  pragma once

#  include "GPU_shader_shared_utils.hh"

namespace blender::draw::select {

#endif

/* Matches eV3DSelectMode */
enum SelectType : uint32_t {
  SELECT_ALL = 0u,
  SELECT_PICK_ALL = 1u,
  SELECT_PICK_NEAREST = 2u,
};

struct SelectInfoData {
  int2 cursor;
  SelectType mode;
  uint _pad0;
};
BLI_STATIC_ASSERT_ALIGN(SelectInfoData, 16)

#if !defined(GPU_SHADER) && !defined(GLSL_CPP_STUBS)
}  // namespace blender::draw::select
#endif
