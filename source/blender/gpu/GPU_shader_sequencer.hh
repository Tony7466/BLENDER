/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#ifndef USE_GPU_SHADER_CREATE_INFO
#  pragma once
#  include "GPU_shader_shared_utils.hh"
#endif

#define GPU_SEQ_STRIP_DRAW_DATA_LEN 128

enum eGPUSeqFlags : uint32_t {
  GPU_SEQ_FLAG_BOTTOM_PART = (1u << 0u),
  GPU_SEQ_FLAG_SINGLE_IMAGE = (1u << 1u),
  GPU_SEQ_FLAG_COLOR_BAND = (1u << 2u),
  GPU_SEQ_FLAG_TRANSITION = (1u << 3u),
  GPU_SEQ_FLAG_LOCKED = (1u << 4u),
  GPU_SEQ_FLAG_MISSING_TITLE = (1u << 5u),
  GPU_SEQ_FLAG_MISSING_CONTENT = (1u << 6u),
  GPU_SEQ_FLAG_SELECTED = (1u << 7u),
  GPU_SEQ_FLAG_ACTIVE = (1u << 8u),
};

struct SeqStripDrawData {
  float content_start, content_end, bottom, top;
  float left_handle, right_handle, strip_content_top;
  uint flags;
  uint col_background;
  uint col_outline;
  uint col_color_band;
  uint col_transition_in, col_transition_out;
  float _pad0, _pad1, _pad2;
};
BLI_STATIC_ASSERT_ALIGN(SeqStripDrawData, 16)

struct SeqContextDrawData {
  float pixelx, pixely;
  float round_radius;
  uint col_back;
};
BLI_STATIC_ASSERT_ALIGN(SeqContextDrawData, 16)
