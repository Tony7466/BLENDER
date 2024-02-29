/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "testing/testing.h"

#include "vk_render_graph.hh"

namespace blender::gpu {

class CommandBufferLog : public VKRenderGraphCommandBuffer {
 public:
  CommandBufferLog(Vector<std::string> &log) : log(log) {}
  Vector<std::string> &log;

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
      ss << " - image_barrier(image=";
      ss << image_barrier.image;
      ss << ", old_layout=" << image_barrier.oldLayout;
      ss << ", new_layout=" << image_barrier.newLayout;
      ss << ")\n";
    }
    ss << ")\n";

    log.append(ss.str());
  }
};

/**
 * Union to create a dummy vulkan handler.
 *
 * Due to platform differences the actual VKObjectType type can be different (`uint64_t` or
 * `VkObjectType_T*`).
 */
template<typename VKObjectType> union VkHandle {
  VKObjectType vk_image;
  uint64_t handle;

  VkHandle(uint64_t handle) : handle(handle) {}
};

TEST(vk_render_graph, transfer_and_present)
{
  VkHandle<VkImage> back_buffer(1u);

  Vector<std::string> log;
  VKRenderGraph render_graph(std::make_unique<CommandBufferLog>(log),
                             std::make_unique<Sequential>());
  render_graph.add_image(
      back_buffer.vk_image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, ResourceOwner::SWAP_CHAIN);

  render_graph.submit_for_present(back_buffer.vk_image);

  EXPECT_EQ(1, log.size());
  EXPECT_EQ(
      "pipeline_barrier(\n"
      " - image_barrier(image=0x1, old_layout=7, new_layout=1000001002)\n"
      ")\n",
      log[0]);

  // Test for transition to present
}

TEST(vk_render_graph, clear_and_present)
{
  VkHandle<VkImage> back_buffer(1u);

  Vector<std::string> log;
  VKRenderGraph render_graph(std::make_unique<CommandBufferLog>(log),
                             std::make_unique<Sequential>());
  render_graph.add_image(
      back_buffer.vk_image, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR, ResourceOwner::SWAP_CHAIN);

  VkClearColorValue color = {};
  VkImageSubresourceRange range = {};
  render_graph.add_clear_image_node(back_buffer.vk_image, color, range);

  render_graph.submit_for_present(back_buffer.vk_image);

  EXPECT_EQ(3, log.size());

  // Test for transition to TRANSFER_DST
  // Test clear command
  // Test for transition to present
}

}  // namespace blender::gpu
