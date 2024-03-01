/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_render_graph_commands.hh"

namespace blender::gpu {

void VKCommandBufferWrapper::bind_pipeline(VkPipelineBindPoint pipeline_bind_point,
                                           VkPipeline pipeline)
{
  vkCmdBindPipeline(command_buffer_, pipeline_bind_point, pipeline);
}

void VKCommandBufferWrapper::bind_descriptor_sets(VkPipelineBindPoint pipeline_bind_point,
                                                  VkPipelineLayout layout,
                                                  uint32_t first_set,
                                                  uint32_t descriptor_set_count,
                                                  const VkDescriptorSet *p_descriptor_sets,
                                                  uint32_t dynamic_offset_count,
                                                  const uint32_t *p_dynamic_offsets)
{
  vkCmdBindDescriptorSets(command_buffer_,
                          pipeline_bind_point,
                          layout,
                          first_set,
                          descriptor_set_count,
                          p_descriptor_sets,
                          dynamic_offset_count,
                          p_dynamic_offsets);
}

void VKCommandBufferWrapper::bind_index_buffer(VkBuffer buffer,
                                               VkDeviceSize offset,
                                               VkIndexType index_type)
{
  vkCmdBindIndexBuffer(command_buffer_, buffer, offset, index_type);
}

void VKCommandBufferWrapper::bind_vertex_buffers(uint32_t first_binding,
                                                 uint32_t binding_count,
                                                 const VkBuffer *p_buffers,
                                                 const VkDeviceSize *p_offsets)
{
  vkCmdBindVertexBuffers(command_buffer_, first_binding, binding_count, p_buffers, p_offsets);
}

void VKCommandBufferWrapper::draw(uint32_t vertex_count,
                                  uint32_t instance_count,
                                  uint32_t first_vertex,
                                  uint32_t first_instance)
{
  vkCmdDraw(command_buffer_, vertex_count, instance_count, first_vertex, first_instance);
}

void VKCommandBufferWrapper::draw_indexed(uint32_t index_count,
                                          uint32_t instance_count,
                                          uint32_t first_index,
                                          int32_t vertex_offset,
                                          uint32_t first_instance)
{
  vkCmdDrawIndexed(
      command_buffer_, index_count, instance_count, first_index, vertex_offset, first_instance);
}

void VKCommandBufferWrapper::draw_indirect(VkBuffer buffer,
                                           VkDeviceSize offset,
                                           uint32_t draw_count,
                                           uint32_t stride)
{
  vkCmdDrawIndirect(command_buffer_, buffer, offset, draw_count, stride);
}

void VKCommandBufferWrapper::draw_indexed_indirect(VkBuffer buffer,
                                                   VkDeviceSize offset,
                                                   uint32_t draw_count,
                                                   uint32_t stride)
{
  vkCmdDrawIndexedIndirect(command_buffer_, buffer, offset, draw_count, stride);
}

void VKCommandBufferWrapper::dispatch(uint32_t group_count_x,
                                      uint32_t group_count_y,
                                      uint32_t group_count_z)
{
  vkCmdDispatch(command_buffer_, group_count_x, group_count_y, group_count_z);
}

void VKCommandBufferWrapper::dispatch_indirect(VkBuffer buffer, VkDeviceSize offset)
{
  vkCmdDispatchIndirect(command_buffer_, buffer, offset);
}

void VKCommandBufferWrapper::copy_buffer(VkBuffer src_buffer,
                                         VkBuffer dst_buffer,
                                         uint32_t region_count,
                                         const VkBufferCopy *p_regions)
{
  vkCmdCopyBuffer(command_buffer_, src_buffer, dst_buffer, region_count, p_regions);
}

void VKCommandBufferWrapper::copy_image(VkImage src_image,
                                        VkImageLayout src_image_layout,
                                        VkImage dst_image,
                                        VkImageLayout dst_image_layout,
                                        uint32_t region_count,
                                        const VkImageCopy *p_regions)
{
  vkCmdCopyImage(command_buffer_,
                 src_image,
                 src_image_layout,
                 dst_image,
                 dst_image_layout,
                 region_count,
                 p_regions);
}

void VKCommandBufferWrapper::blit_image(VkImage src_image,
                                        VkImageLayout src_image_layout,
                                        VkImage dst_image,
                                        VkImageLayout dst_image_layout,
                                        uint32_t region_count,
                                        const VkImageBlit *p_regions,
                                        VkFilter filter)
{
  vkCmdBlitImage(command_buffer_,
                 src_image,
                 src_image_layout,
                 dst_image,
                 dst_image_layout,
                 region_count,
                 p_regions,
                 filter);
}

void VKCommandBufferWrapper::copy_buffer_to_image(VkBuffer src_buffer,
                                                  VkImage dst_image,
                                                  VkImageLayout dst_image_layout,
                                                  uint32_t region_count,
                                                  const VkBufferImageCopy *p_regions)
{
  vkCmdCopyBufferToImage(
      command_buffer_, src_buffer, dst_image, dst_image_layout, region_count, p_regions);
}

void VKCommandBufferWrapper::copy_image_to_buffer(VkImage src_image,
                                                  VkImageLayout src_image_layout,
                                                  VkBuffer dst_buffer,
                                                  uint32_t region_count,
                                                  const VkBufferImageCopy *p_regions)
{
  vkCmdCopyImageToBuffer(
      command_buffer_, src_image, src_image_layout, dst_buffer, region_count, p_regions);
}

void VKCommandBufferWrapper::fill_buffer(VkBuffer dst_buffer,
                                         VkDeviceSize dst_offset,
                                         VkDeviceSize size,
                                         uint32_t data)
{
  vkCmdFillBuffer(command_buffer_, dst_buffer, dst_offset, size, data);
}

void VKCommandBufferWrapper::clear_color_image(VkImage image,
                                               VkImageLayout image_layout,
                                               const VkClearColorValue *p_color,
                                               uint32_t range_count,
                                               const VkImageSubresourceRange *p_ranges)
{
  vkCmdClearColorImage(command_buffer_, image, image_layout, p_color, range_count, p_ranges);
}

void VKCommandBufferWrapper::clear_depth_stencil_image(
    VkImage image,
    VkImageLayout image_layout,
    const VkClearDepthStencilValue *p_depth_stencil,
    uint32_t range_count,
    const VkImageSubresourceRange *p_ranges)
{
  vkCmdClearDepthStencilImage(
      command_buffer_, image, image_layout, p_depth_stencil, range_count, p_ranges);
}

void VKCommandBufferWrapper::clear_attachments(uint32_t attachment_count,
                                               const VkClearAttachment *p_attachments,
                                               uint32_t rect_count,
                                               const VkClearRect *p_rects)
{
  vkCmdClearAttachments(command_buffer_, attachment_count, p_attachments, rect_count, p_rects);
}

void VKCommandBufferWrapper::pipeline_barrier(
    VkPipelineStageFlags src_stage_mask,
    VkPipelineStageFlags dst_stage_mask,
    VkDependencyFlags dependency_flags,
    uint32_t memory_barrier_count,
    const VkMemoryBarrier *p_memory_barriers,
    uint32_t buffer_memory_barrier_count,
    const VkBufferMemoryBarrier *p_buffer_memory_barriers,
    uint32_t image_memory_barrier_count,
    const VkImageMemoryBarrier *p_image_memory_barriers)
{
  vkCmdPipelineBarrier(command_buffer_,
                       src_stage_mask,
                       dst_stage_mask,
                       dependency_flags,
                       memory_barrier_count,
                       p_memory_barriers,
                       buffer_memory_barrier_count,
                       p_buffer_memory_barriers,
                       image_memory_barrier_count,
                       p_image_memory_barriers);
}

void VKCommandBufferWrapper::push_constants(VkPipelineLayout layout,
                                            VkShaderStageFlags stage_flags,
                                            uint32_t offset,
                                            uint32_t size,
                                            const void *p_values)
{
  vkCmdPushConstants(command_buffer_, layout, stage_flags, offset, size, p_values);
}

void VKCommandBufferWrapper::begin_render_pass(const VkRenderPassBeginInfo *p_render_pass_begin,
                                               VkSubpassContents contents)
{
  vkCmdBeginRenderPass(command_buffer_, p_render_pass_begin, contents);
}

void VKCommandBufferWrapper::end_render_pass()
{
  vkCmdEndRenderPass(command_buffer_);
}
}  // namespace blender::gpu
