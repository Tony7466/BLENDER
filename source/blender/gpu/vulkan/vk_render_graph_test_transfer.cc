/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "testing/testing.h"

#include "vk_render_graph_test_types.hh"

namespace blender::gpu {

/**
 * Fill a single buffer and read it back.
 */
TEST(vk_render_graph, fill_and_read_back)
{
  VkHandle<VkBuffer> buffer(1u);

  Vector<std::string> log;
  VKCommandBufferWrapper wrapper;
  VKRenderGraph render_graph(std::make_unique<CommandBufferLog>(log),
                             std::make_unique<Sequential>());
  render_graph.add_buffer(buffer);
  render_graph.add_fill_buffer_node(buffer, 1024, 42);
  render_graph.submit_buffer_for_read_back(buffer);

  EXPECT_EQ(1, log.size());
  EXPECT_EQ("fill_buffer(dst_buffer=0x1, dst_offset=0, size=1024, data=42)", log[0]);
}

/**
 * Fill a single buffer, copy it to a staging buffer and read the staging buffer back.
 */
TEST(vk_render_graph, fill_transfer_and_read_back)
{
  VkHandle<VkBuffer> buffer(1u);
  VkHandle<VkBuffer> staging_buffer(2u);

  Vector<std::string> log;
  VKCommandBufferWrapper wrapper;
  VKRenderGraph render_graph(std::make_unique<CommandBufferLog>(log),
                             std::make_unique<Sequential>());
  render_graph.add_buffer(buffer);
  render_graph.add_fill_buffer_node(buffer, 1024, 42);
  render_graph.add_buffer(staging_buffer);
  VkBufferCopy region = {};
  region.srcOffset = 0;
  region.dstOffset = 0;
  region.size = 1024;
  render_graph.add_copy_buffer_node(buffer, staging_buffer, region);
  render_graph.submit_buffer_for_read_back(staging_buffer);

  EXPECT_EQ(3, log.size());
  EXPECT_EQ("fill_buffer(dst_buffer=0x1, dst_offset=0, size=1024, data=42)", log[0]);
  EXPECT_EQ(
      "pipeline_barrier(src_stage_mask=VK_PIPELINE_STAGE_TRANSFER_BIT, "
      "dst_stage_mask=VK_PIPELINE_STAGE_TRANSFER_BIT\n"
      " - buffer_barrier(src_access_mask=VK_ACCESS_TRANSFER_WRITE_BIT, "
      "dst_access_mask=VK_ACCESS_TRANSFER_READ_BIT, buffer=0x1, offset=0, "
      "size=18446744073709551615)\n"
      ")",
      log[1]);
  EXPECT_EQ(
      "copy_buffer(src_buffer=0x1, dst_buffer=0x2\n"
      " - region(src_offset=0, dst_offset=0, size=1024)\n"
      ")",
      log[2]);
}

/**
 * Fill a buffer twice, before reading back.
 *
 * Between the two fills a write->write barrier should be created.
 */
TEST(vk_render_graph, fill_fill_read_back)
{
  VkHandle<VkBuffer> buffer(1u);

  Vector<std::string> log;
  VKCommandBufferWrapper wrapper;
  VKRenderGraph render_graph(std::make_unique<CommandBufferLog>(log),
                             std::make_unique<Sequential>());
  render_graph.add_buffer(buffer);
  render_graph.add_fill_buffer_node(buffer, 1024, 0);
  render_graph.add_fill_buffer_node(buffer, 1024, 42);
  render_graph.submit_buffer_for_read_back(buffer);

  EXPECT_EQ(3, log.size());
  EXPECT_EQ("fill_buffer(dst_buffer=0x1, dst_offset=0, size=1024, data=0)", log[0]);
  EXPECT_EQ(
      "pipeline_barrier(src_stage_mask=VK_PIPELINE_STAGE_TRANSFER_BIT, "
      "dst_stage_mask=VK_PIPELINE_STAGE_TRANSFER_BIT\n"
      " - buffer_barrier(src_access_mask=VK_ACCESS_TRANSFER_WRITE_BIT, "
      "dst_access_mask=VK_ACCESS_TRANSFER_WRITE_BIT, buffer=0x1, offset=0, "
      "size=18446744073709551615)\n"
      ")",
      log[1]);
  EXPECT_EQ("fill_buffer(dst_buffer=0x1, dst_offset=0, size=1024, data=42)", log[2]);
}

}  // namespace blender::gpu
