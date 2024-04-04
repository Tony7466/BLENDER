/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "testing/testing.h"

#include "vk_render_graph_test_types.hh"

namespace blender::gpu::render_graph {

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

/**
 * Fill a single buffer, copy it to a staging buffer and read the staging buffer back.
 */
TEST(vk_render_graph, clear_clear_copy_and_read_back)
{
  VkHandle<VkImage> src_image(1u);
  VkHandle<VkImage> dst_image(2u);
  VkHandle<VkBuffer> staging_buffer(3u);

  Vector<std::string> log;
  VKCommandBufferWrapper wrapper;
  VKRenderGraph render_graph(std::make_unique<CommandBufferLog>(log),
                             std::make_unique<Sequential>());
  render_graph.add_image(src_image, VK_IMAGE_LAYOUT_UNDEFINED, ResourceOwner::APPLICATION);
  render_graph.add_image(dst_image, VK_IMAGE_LAYOUT_UNDEFINED, ResourceOwner::APPLICATION);
  render_graph.add_buffer(staging_buffer);
  VkClearColorValue color_white = {};
  color_white.float32[0] = 1.0f;
  color_white.float32[1] = 1.0f;
  color_white.float32[2] = 1.0f;
  color_white.float32[3] = 1.0f;
  VkClearColorValue color_black = {};
  color_black.float32[0] = 0.0f;
  color_black.float32[1] = 0.0f;
  color_black.float32[2] = 0.0f;
  color_black.float32[3] = 1.0f;
  VkImageSubresourceRange range = {};
  VkImageCopy vk_image_copy = {};
  VkBufferImageCopy vk_buffer_image_copy = {};

  render_graph.add_clear_image_node(src_image, color_white, range);
  render_graph.add_clear_image_node(dst_image, color_black, range);
  render_graph.add_copy_image_node(src_image, dst_image, vk_image_copy);
  render_graph.add_copy_image_to_buffer_node(dst_image, staging_buffer, vk_buffer_image_copy);
  render_graph.submit_buffer_for_read_back(staging_buffer);

  EXPECT_EQ(8, log.size());
  EXPECT_EQ(
      "pipeline_barrier(src_stage_mask=VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, "
      "dst_stage_mask=VK_PIPELINE_STAGE_TRANSFER_BIT\n"
      " - image_barrier(src_access_mask=, dst_access_mask=VK_ACCESS_TRANSFER_WRITE_BIT, "
      "old_layout=VK_IMAGE_LAYOUT_UNDEFINED, new_layout=VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, "
      "image=0x1, subresource_range=\n"
      "    aspect_mask=VK_IMAGE_ASPECT_COLOR_BIT, base_mip_level=0, level_count=4294967295, "
      "base_array_layer=0, layer_count=4294967295  )\n"
      ")",
      log[0]);
  EXPECT_EQ("clear_color_image(image=0x1, image_layout=VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL)",
            log[1]);

  EXPECT_EQ(
      "pipeline_barrier(src_stage_mask=VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, "
      "dst_stage_mask=VK_PIPELINE_STAGE_TRANSFER_BIT\n"
      " - image_barrier(src_access_mask=, dst_access_mask=VK_ACCESS_TRANSFER_WRITE_BIT, "
      "old_layout=VK_IMAGE_LAYOUT_UNDEFINED, new_layout=VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, "
      "image=0x2, subresource_range=\n"
      "    aspect_mask=VK_IMAGE_ASPECT_COLOR_BIT, base_mip_level=0, level_count=4294967295, "
      "base_array_layer=0, layer_count=4294967295  )\n"
      ")",
      log[2]);
  EXPECT_EQ("clear_color_image(image=0x2, image_layout=VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL)",
            log[3]);

  EXPECT_EQ(
      "pipeline_barrier(src_stage_mask=VK_PIPELINE_STAGE_TRANSFER_BIT, "
      "dst_stage_mask=VK_PIPELINE_STAGE_TRANSFER_BIT\n"
      " - image_barrier(src_access_mask=VK_ACCESS_TRANSFER_WRITE_BIT, "
      "dst_access_mask=VK_ACCESS_TRANSFER_READ_BIT, "
      "old_layout=VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, "
      "new_layout=VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, image=0x1, subresource_range=\n"
      "    aspect_mask=VK_IMAGE_ASPECT_COLOR_BIT, base_mip_level=0, level_count=4294967295, "
      "base_array_layer=0, layer_count=4294967295  )\n"
      " - image_barrier(src_access_mask=VK_ACCESS_TRANSFER_WRITE_BIT, "
      "dst_access_mask=VK_ACCESS_TRANSFER_WRITE_BIT, "
      "old_layout=VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, "
      "new_layout=VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, image=0x2, subresource_range=\n"
      "    aspect_mask=VK_IMAGE_ASPECT_COLOR_BIT, base_mip_level=0, level_count=4294967295, "
      "base_array_layer=0, layer_count=4294967295  )\n"
      ")",
      log[4]);
  EXPECT_EQ(
      "copy_image(src_image=0x1, src_image_layout=VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, "
      "dst_image=0x2, dst_image_layout=VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL\n"
      " - region(src_subresource=\n"
      "    aspect_mask=, mip_level=0, base_array_layer=0, layer_count=0  , src_offset=\n"
      "    x=0, y=0, z=0  , dst_subresource=\n"
      "    aspect_mask=, mip_level=0, base_array_layer=0, layer_count=0  , dst_offset=\n"
      "    x=0, y=0, z=0  , extent=\n"
      "    width=0, height=0, depth=0  )\n"
      ")",
      log[5]);

  EXPECT_EQ(
      "pipeline_barrier(src_stage_mask=VK_PIPELINE_STAGE_TRANSFER_BIT, "
      "dst_stage_mask=VK_PIPELINE_STAGE_TRANSFER_BIT\n"
      " - image_barrier(src_access_mask=VK_ACCESS_TRANSFER_WRITE_BIT, "
      "dst_access_mask=VK_ACCESS_TRANSFER_READ_BIT, "
      "old_layout=VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, "
      "new_layout=VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, image=0x2, subresource_range=\n"
      "    aspect_mask=VK_IMAGE_ASPECT_COLOR_BIT, base_mip_level=0, level_count=4294967295, "
      "base_array_layer=0, layer_count=4294967295  )\n"
      ")",
      log[6]);
  EXPECT_EQ(
      "copy_image_to_buffer(src_image=0x2, src_image_layout=VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, "
      "dst_buffer=0x3\n"
      " - region(buffer_offset=0, buffer_row_length=0, buffer_image_height=0, image_subresource=\n"
      "    aspect_mask=, mip_level=0, base_array_layer=0, layer_count=0  , image_offset=\n"
      "    x=0, y=0, z=0  , image_extent=\n    width=0, height=0, depth=0  )\n"
      ")",
      log[7]);
}

/**
 * Clear an image, blit it to another image, copy to a staging buffer and read back.
 */
TEST(vk_render_graph, clear_blit_copy_and_read_back)
{
  VkHandle<VkImage> src_image(1u);
  VkHandle<VkImage> dst_image(2u);
  VkHandle<VkBuffer> staging_buffer(3u);

  Vector<std::string> log;
  VKCommandBufferWrapper wrapper;
  VKRenderGraph render_graph(std::make_unique<CommandBufferLog>(log),
                             std::make_unique<Sequential>());
  render_graph.add_image(src_image, VK_IMAGE_LAYOUT_UNDEFINED, ResourceOwner::APPLICATION);
  render_graph.add_image(dst_image, VK_IMAGE_LAYOUT_UNDEFINED, ResourceOwner::APPLICATION);
  render_graph.add_buffer(staging_buffer);
  VkClearColorValue color_black = {};
  color_black.float32[0] = 0.0f;
  color_black.float32[1] = 0.0f;
  color_black.float32[2] = 0.0f;
  color_black.float32[3] = 1.0f;
  VkImageSubresourceRange range = {};
  VkImageBlit vk_image_blit = {};
  VkBufferImageCopy vk_buffer_image_copy = {};

  render_graph.add_clear_image_node(src_image, color_black, range);
  render_graph.add_blit_image_node(src_image, dst_image, vk_image_blit, VK_FILTER_LINEAR);
  render_graph.add_copy_image_to_buffer_node(dst_image, staging_buffer, vk_buffer_image_copy);
  render_graph.submit_buffer_for_read_back(staging_buffer);

  EXPECT_EQ(6, log.size());
  EXPECT_EQ(
      "pipeline_barrier(src_stage_mask=VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, "
      "dst_stage_mask=VK_PIPELINE_STAGE_TRANSFER_BIT\n"
      " - image_barrier(src_access_mask=, dst_access_mask=VK_ACCESS_TRANSFER_WRITE_BIT, "
      "old_layout=VK_IMAGE_LAYOUT_UNDEFINED, "
      "new_layout=VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, image=0x1, subresource_range=\n"
      "    aspect_mask=VK_IMAGE_ASPECT_COLOR_BIT, base_mip_level=0, level_count=4294967295, "
      "base_array_layer=0, layer_count=4294967295  )\n"
      ")",
      log[0]);
  EXPECT_EQ("clear_color_image(image=0x1, image_layout=VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL)",
            log[1]);
  EXPECT_EQ(
      "pipeline_barrier(src_stage_mask=VK_PIPELINE_STAGE_TRANSFER_BIT, "
      "dst_stage_mask=VK_PIPELINE_STAGE_TRANSFER_BIT\n"
      " - image_barrier(src_access_mask=VK_ACCESS_TRANSFER_WRITE_BIT, "
      "dst_access_mask=VK_ACCESS_TRANSFER_READ_BIT, "
      "old_layout=VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, "
      "new_layout=VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, image=0x1, subresource_range=\n"
      "    aspect_mask=VK_IMAGE_ASPECT_COLOR_BIT, base_mip_level=0, level_count=4294967295, "
      "base_array_layer=0, layer_count=4294967295  )\n"
      " - image_barrier(src_access_mask=, dst_access_mask=VK_ACCESS_TRANSFER_WRITE_BIT, "
      "old_layout=VK_IMAGE_LAYOUT_UNDEFINED, new_layout=VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, "
      "image=0x2, subresource_range=\n"
      "    aspect_mask=VK_IMAGE_ASPECT_COLOR_BIT, base_mip_level=0, level_count=4294967295, "
      "base_array_layer=0, layer_count=4294967295  )\n"
      ")",
      log[2]);
  EXPECT_EQ(
      "blit_image(src_image=0x1, src_image_layout=VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, "
      "dst_image=0x2, dst_image_layout=VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, "
      "filter=VK_FILTER_LINEAR\n"
      " - region(src_subresource=\n"
      "    aspect_mask=, mip_level=0, base_array_layer=0, layer_count=0  , dst_subresource=\n"
      "    aspect_mask=, mip_level=0, base_array_layer=0, layer_count=0  )\n"
      ")",
      log[3]);
  EXPECT_EQ(
      "pipeline_barrier(src_stage_mask=VK_PIPELINE_STAGE_TRANSFER_BIT, "
      "dst_stage_mask=VK_PIPELINE_STAGE_TRANSFER_BIT\n"
      " - image_barrier(src_access_mask=VK_ACCESS_TRANSFER_WRITE_BIT, "
      "dst_access_mask=VK_ACCESS_TRANSFER_READ_BIT, "
      "old_layout=VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, "
      "new_layout=VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, image=0x2, subresource_range=\n"
      "    aspect_mask=VK_IMAGE_ASPECT_COLOR_BIT, base_mip_level=0, level_count=4294967295, "
      "base_array_layer=0, layer_count=4294967295  )\n"
      ")",
      log[4]);
  EXPECT_EQ(
      "copy_image_to_buffer(src_image=0x2, src_image_layout=VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, "
      "dst_buffer=0x3\n"
      " - region(buffer_offset=0, buffer_row_length=0, buffer_image_height=0, image_subresource=\n"
      "    aspect_mask=, mip_level=0, base_array_layer=0, layer_count=0  , image_offset=\n"
      "    x=0, y=0, z=0  , image_extent=\n"
      "    width=0, height=0, depth=0  )\n"
      ")",
      log[5]);
}
}  // namespace blender::gpu::render_graph
