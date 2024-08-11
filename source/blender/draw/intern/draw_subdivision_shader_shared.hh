/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef GPU_SHADER
#  include "GPU_shader_shared_utils.hh"
#endif

struct DRWSubdivUboStorage {
  /* Offsets in the buffers data where the source and destination data start. */
  int src_offset;
  int dst_offset;

  /* Parameters for the DRWPatchMap. */
  int min_patch_face;
  int max_patch_face;
  int max_depth;
  int patches_are_triangular;

  /* Coarse topology information. */
  int coarse_face_count;
  uint edge_loose_offset;

  /* Subdiv topology information. */
  uint num_subdiv_loops;

  /* Sculpt data. */
  bool32_t has_sculpt_mask;

  /* Masks for the extra coarse face data. */
  uint coarse_face_select_mask;
  uint coarse_face_smooth_mask;
  uint coarse_face_active_mask;
  uint coarse_face_hidden_mask;
  uint coarse_face_loopstart_mask;

  /* Total number of elements to process. */
  uint total_dispatch_size;

  bool32_t is_edit_mode;

  bool32_t use_hide;

  int _pad3;
  int _pad4;
};

static_assert((sizeof(DRWSubdivUboStorage) % 16) == 0,
              "DRWSubdivUboStorage is not padded to a multiple of the size of vec4");
