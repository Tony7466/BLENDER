/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "testing/testing.h"

#include "vk_render_graph.hh"

namespace blender::gpu {

class CommandBufferStub : public VKRenderGraphCommandBuffer {
 public:
  Vector<std::string> commands;

  void pipeline_barrier(VkPipelineStageFlags src_stage_mask,
                        VkPipelineStageFlags dst_stage_mask,
                        VkDependencyFlags dependency_flags,
                        uint32_t memory_barrier_count,
                        const VkMemoryBarrier *p_memory_barriers,
                        uint32_t buffer_memory_barrier_count,
                        const VkBufferMemoryBarrier *p_buffer_memory_barriers,
                        uint32_t image_memory_barrier_count,
                        const VkImageMemoryBarrier *p_image_memory_barriers)
  {
    std::stringstream ss;
    ss << "pipeline_barrier(\n";
    for (VkImageMemoryBarrier image_barrier :
         Span<VkImageMemoryBarrier>(p_image_memory_barriers, image_memory_barrier_count))
    {
      ss << " - image_barrier\n";
    }
    ss << ")\n";

    commands.append(ss.str());
  }
};

union VkImageHandle {
  VkImage vk_image;
  uint64_t handle;
};

TEST(vk_render_graph, clear_and_present)
{
  VkImageHandle back_buffer;
  back_buffer.handle = 1u;

  VKRenderGraph render_graph(std::make_unique<CommandBufferStub>(),
                             std::make_unique<Sequential>());
  render_graph.add_image(
      back_buffer.vk_image, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR, ResourceOwner::SWAPCHAIN);

  VkClearColorValue color = {};
  VkImageSubresourceRange range = {};
  render_graph.add_clear_image_node(back_buffer.vk_image, color, range);

  render_graph.submit_for_present(back_buffer.vk_image);

  // Test for transition to TRANSFER_DST
  // Test clear command
  // Test for transition to present
}

}  // namespace blender::gpu
