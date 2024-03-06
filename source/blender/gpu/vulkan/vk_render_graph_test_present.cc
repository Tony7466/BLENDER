/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "testing/testing.h"

#include "vk_render_graph_test_types.hh"

namespace blender::gpu {

TEST(vk_render_graph, transfer_and_present)
{
  VkHandle<VkImage> back_buffer(1u);

  Vector<std::string> log;
  VKCommandBufferWrapper wrapper;
  VKRenderGraph render_graph(std::make_unique<CommandBufferLog>(log),
                             std::make_unique<Sequential>());
  render_graph.add_image(
      back_buffer, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, ResourceOwner::SWAP_CHAIN);

  render_graph.submit_for_present(back_buffer);

  EXPECT_EQ(1, log.size());
  EXPECT_EQ(
      "pipeline_barrier(src_stage_mask=, dst_stage_mask=\n"
      " - image_barrier(src_access_mask=, dst_access_mask=, "
      "old_layout=VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, "
      "new_layout=VK_IMAGE_LAYOUT_PRESENT_SRC_KHR, image=0x1, subresource_range=\n"
      "    aspect_mask=, base_mip_level=0, level_count=0, base_array_layer=0, layer_count=0  )\n"
      ")",
      log[0]);
}

TEST(vk_render_graph, clear_and_present)
{
  VkHandle<VkImage> back_buffer(1u);

  Vector<std::string> log;
  VKRenderGraph render_graph(std::make_unique<CommandBufferLog>(log),
                             std::make_unique<Sequential>());
  render_graph.add_image(back_buffer, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR, ResourceOwner::SWAP_CHAIN);

  VkClearColorValue color = {};
  VkImageSubresourceRange range = {};
  render_graph.add_clear_image_node(back_buffer, color, range);

  render_graph.submit_for_present(back_buffer);

  EXPECT_EQ(3, log.size());

  EXPECT_EQ(
      "pipeline_barrier(src_stage_mask=, dst_stage_mask=\n"
      " - image_barrier(src_access_mask=, dst_access_mask=, "
      "old_layout=VK_IMAGE_LAYOUT_PRESENT_SRC_KHR, "
      "new_layout=VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, image=0x1, subresource_range=\n"
      "    aspect_mask=, base_mip_level=0, level_count=0, base_array_layer=0, layer_count=0  )\n"
      ")",
      log[0]);
  EXPECT_EQ("clear_color_image(image=0x1, image_layout=VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL)",
            log[1]);
  EXPECT_EQ(
      "pipeline_barrier(src_stage_mask=, dst_stage_mask=\n"
      " - image_barrier(src_access_mask=, dst_access_mask=, "
      "old_layout=VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, "
      "new_layout=VK_IMAGE_LAYOUT_PRESENT_SRC_KHR, image=0x1, subresource_range=\n"
      "    aspect_mask=, base_mip_level=0, level_count=0, base_array_layer=0, layer_count=0  )\n"
      ")",
      log[2]);
}

}  // namespace blender::gpu
