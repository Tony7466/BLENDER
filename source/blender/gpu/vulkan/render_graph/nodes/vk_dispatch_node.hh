/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_types_pipeline.hh"
#include "vk_common.hh"

namespace blender::gpu::render_graph {
struct VKDispatchNode {
  struct Data {
    VKPipelineData pipeline_data;
    uint32_t group_count_x;
    uint32_t group_count_y;
    uint32_t group_count_z;
  };
};
}  // namespace blender::gpu::render_graph
