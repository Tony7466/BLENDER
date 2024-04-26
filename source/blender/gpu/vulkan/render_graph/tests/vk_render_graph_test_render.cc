/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "testing/testing.h"

#include "vk_render_graph_test_types.hh"

namespace blender::gpu::render_graph {

TEST(vk_render_graph, begin_clear_attachments_end_read_back)
{
  VkHandle<VkImage> image(1u);
  VkHandle<VkImageView> image_view(2u);
  VkHandle<VkBuffer> buffer(3u);

  Vector<std::string> log;
  VKCommandBufferWrapper wrapper;
  VKResourceStateTracker resources;
  VKRenderGraph render_graph(std::make_unique<CommandBufferLog>(log), resources);
  resources.add_image(image, VK_IMAGE_LAYOUT_UNDEFINED, ResourceOwner::APPLICATION);
  resources.add_buffer(buffer);

  {
    VKResourceAccessInfo access_info = {};
    access_info.images.append(
        {image, VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT, VK_IMAGE_ASPECT_COLOR_BIT});
    VKBeginRenderingNode::CreateInfo begin_rendering(access_info);
    begin_rendering.node_data.color_attachment_count = 1;
    begin_rendering.node_data.color_attachments[0].sType =
        VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO_KHR;
    begin_rendering.node_data.color_attachments[0].imageLayout =
        VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    begin_rendering.node_data.color_attachments[0].imageView = image_view;
    begin_rendering.node_data.color_attachments[0].loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    begin_rendering.node_data.color_attachments[0].storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    begin_rendering.node_data.vk_rendering_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
    begin_rendering.node_data.vk_rendering_info.colorAttachmentCount = 1;
    begin_rendering.node_data.vk_rendering_info.layerCount = 1;
    begin_rendering.node_data.vk_rendering_info.pColorAttachments =
        begin_rendering.node_data.color_attachments;

    render_graph.add_node(begin_rendering);
  }

  {
    VKResourceAccessInfo access_info = {};
    access_info.images.append(
        {image, VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT, VK_IMAGE_ASPECT_COLOR_BIT});
    VKClearAttachmentsNode::CreateInfo clear_attachments(access_info);
    clear_attachments.node_data.attachment_count = 1;
    clear_attachments.node_data.attachments[0].aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    clear_attachments.node_data.attachments[0].clearValue.color.float32[0] = 0.2;
    clear_attachments.node_data.attachments[0].clearValue.color.float32[1] = 0.4;
    clear_attachments.node_data.attachments[0].clearValue.color.float32[2] = 0.6;
    clear_attachments.node_data.attachments[0].clearValue.color.float32[3] = 1.0;
    clear_attachments.node_data.attachments[0].colorAttachment = 0;
    clear_attachments.node_data.vk_clear_rect.baseArrayLayer = 0;
    clear_attachments.node_data.vk_clear_rect.layerCount = 1;
    clear_attachments.node_data.vk_clear_rect.rect.extent.width = 1920;
    clear_attachments.node_data.vk_clear_rect.rect.extent.height = 1080;
    render_graph.add_node(clear_attachments);
  }

  {
    VKEndRenderingNode::CreateInfo end_rendering = {};
    render_graph.add_node(end_rendering);
  }

  {
    VKCopyImageToBufferNode::CreateInfo copy_image_to_buffer = {};
    copy_image_to_buffer.src_image = image;
    copy_image_to_buffer.dst_buffer = buffer;
    render_graph.add_node(copy_image_to_buffer);
  }

  render_graph.submit_buffer_for_read(buffer);
  EXPECT_EQ(3, log.size());
}

}  // namespace blender::gpu::render_graph
