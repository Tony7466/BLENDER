/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "testing/testing.h"

#include "vk_common.hh"
#include "vk_render_graph.hh"
#include "vk_to_string.hh"

namespace blender::gpu {

class CommandBufferLog : public VKRenderGraphCommandBuffer {
  Vector<std::string> &log_;
  bool is_recording_ = false;
  bool is_cpu_synchronizing_ = false;

 public:
  CommandBufferLog(Vector<std::string> &log) : log_(log) {}
  virtual ~CommandBufferLog() {}

  void begin_recording() override
  {
    BLI_assert_msg(!is_recording_,
                   "`CommandBufferLog::begin_recording` is called, when the command buffer is "
                   "already recording.");
    is_recording_ = true;
  }

  void end_recording() override
  {
    BLI_assert_msg(is_recording_,
                   "`CommandBufferLog::end_recording` is called, when the command buffer is "
                   "not recording.");
    is_recording_ = false;
  }

  void submit_with_cpu_synchronization() override
  {
    BLI_assert_msg(!is_recording_, "`CommandBufferLog` is submitted when still recording.");
    BLI_assert_msg(!is_cpu_synchronizing_,
                   "`CommandBufferLog::submit_with_cpu_synchronization` is called, when the "
                   "command buffer is "
                   "still synchronizing.");
    is_cpu_synchronizing_ = true;
  };
  void wait_for_cpu_synchronization() override
  {
    BLI_assert_msg(!is_recording_, "`CommandBufferLog` is synchronizing when still recording.");
    BLI_assert_msg(
        is_cpu_synchronizing_,
        "`CommandBufferLog::wait_for_cpu_synchronization` is called, when the command buffer is "
        "not synchronizing.");
    is_cpu_synchronizing_ = false;
  };

  void bind_pipeline(VkPipelineBindPoint pipeline_bind_point, VkPipeline pipeline) override
  {
    UNUSED_VARS(pipeline_bind_point, pipeline);
    BLI_assert_msg(is_recording_,
                   "Command is added to command buffer, which isn't in recording state.");
    BLI_assert_unreachable();
  }

  void bind_descriptor_sets(VkPipelineBindPoint pipeline_bind_point,
                            VkPipelineLayout layout,
                            uint32_t first_set,
                            uint32_t descriptor_set_count,
                            const VkDescriptorSet *p_descriptor_sets,
                            uint32_t dynamic_offset_count,
                            const uint32_t *p_dynamic_offsets) override
  {
    UNUSED_VARS(pipeline_bind_point,
                layout,
                first_set,
                descriptor_set_count,
                p_descriptor_sets,
                dynamic_offset_count,
                p_dynamic_offsets);
    BLI_assert_msg(is_recording_,
                   "Command is added to command buffer, which isn't in recording state.");
    BLI_assert_unreachable();
  }

  void bind_index_buffer(VkBuffer buffer, VkDeviceSize offset, VkIndexType index_type) override
  {
    UNUSED_VARS(buffer, offset, index_type);
    BLI_assert_msg(is_recording_,
                   "Command is added to command buffer, which isn't in recording state.");
    BLI_assert_unreachable();
  }

  void bind_vertex_buffers(uint32_t first_binding,
                           uint32_t binding_count,
                           const VkBuffer *p_buffers,
                           const VkDeviceSize *p_offsets) override
  {
    UNUSED_VARS(first_binding, binding_count, p_buffers, p_offsets);
    BLI_assert_msg(is_recording_,
                   "Command is added to command buffer, which isn't in recording state.");
    BLI_assert_unreachable();
  }

  void draw(uint32_t vertex_count,
            uint32_t instance_count,
            uint32_t first_vertex,
            uint32_t first_instance) override
  {
    UNUSED_VARS(vertex_count, instance_count, first_vertex, first_instance);
    BLI_assert_msg(is_recording_,
                   "Command is added to command buffer, which isn't in recording state.");
    BLI_assert_unreachable();
  }

  void draw_indexed(uint32_t index_count,
                    uint32_t instance_count,
                    uint32_t first_index,
                    int32_t vertex_offset,
                    uint32_t first_instance) override
  {
    UNUSED_VARS(index_count, instance_count, first_index, vertex_offset, first_instance);
    BLI_assert_msg(is_recording_,
                   "Command is added to command buffer, which isn't in recording state.");
    BLI_assert_unreachable();
  }

  void draw_indirect(VkBuffer buffer,
                     VkDeviceSize offset,
                     uint32_t draw_count,
                     uint32_t stride) override
  {
    UNUSED_VARS(buffer, offset, draw_count, stride);
    BLI_assert_msg(is_recording_,
                   "Command is added to command buffer, which isn't in recording state.");
    BLI_assert_unreachable();
  }

  void draw_indexed_indirect(VkBuffer buffer,
                             VkDeviceSize offset,
                             uint32_t draw_count,
                             uint32_t stride) override
  {
    UNUSED_VARS(buffer, offset, draw_count, stride);
    BLI_assert_msg(is_recording_,
                   "Command is added to command buffer, which isn't in recording state.");
    BLI_assert_unreachable();
  }

  void dispatch(uint32_t group_count_x, uint32_t group_count_y, uint32_t group_count_z) override
  {
    UNUSED_VARS(group_count_x, group_count_y, group_count_z);
    BLI_assert_msg(is_recording_,
                   "Command is added to command buffer, which isn't in recording state.");
    BLI_assert_unreachable();
  }

  void dispatch_indirect(VkBuffer buffer, VkDeviceSize offset) override
  {
    UNUSED_VARS(buffer, offset);
    BLI_assert_msg(is_recording_,
                   "Command is added to command buffer, which isn't in recording state.");
    BLI_assert_unreachable();
  }

  void copy_buffer(VkBuffer src_buffer,
                   VkBuffer dst_buffer,
                   uint32_t region_count,
                   const VkBufferCopy *p_regions) override
  {
    UNUSED_VARS(src_buffer, dst_buffer, region_count, p_regions);
    BLI_assert_msg(is_recording_,
                   "Command is added to command buffer, which isn't in recording state.");
    std::stringstream ss;
    ss << "copy_buffer(";
    ss << "src_buffer=" << src_buffer;
    ss << ", dst_buffer=" << dst_buffer;
    ss << "\n";
    for (const VkBufferCopy &region : Span<const VkBufferCopy>(p_regions, region_count)) {
      ss << " - region(" << to_string(region, 1) << ")\n";
    }
    ss << ")";
    log_.append(ss.str());
  }

  void copy_image(VkImage src_image,
                  VkImageLayout src_image_layout,
                  VkImage dst_image,
                  VkImageLayout dst_image_layout,
                  uint32_t region_count,
                  const VkImageCopy *p_regions) override
  {
    UNUSED_VARS(src_image, src_image_layout, dst_image, dst_image_layout, region_count, p_regions);
    BLI_assert_msg(is_recording_,
                   "Command is added to command buffer, which isn't in recording state.");
    BLI_assert_unreachable();
  }

  void blit_image(VkImage src_image,
                  VkImageLayout src_image_layout,
                  VkImage dst_image,
                  VkImageLayout dst_image_layout,
                  uint32_t region_count,
                  const VkImageBlit *p_regions,
                  VkFilter filter) override
  {
    UNUSED_VARS(
        src_image, src_image_layout, dst_image, dst_image_layout, region_count, p_regions, filter);
    BLI_assert_msg(is_recording_,
                   "Command is added to command buffer, which isn't in recording state.");
    BLI_assert_unreachable();
  }

  void copy_buffer_to_image(VkBuffer src_buffer,
                            VkImage dst_image,
                            VkImageLayout dst_image_layout,
                            uint32_t region_count,
                            const VkBufferImageCopy *p_regions) override
  {
    UNUSED_VARS(src_buffer, dst_image, dst_image_layout, region_count, p_regions);
    BLI_assert_msg(is_recording_,
                   "Command is added to command buffer, which isn't in recording state.");
    BLI_assert_unreachable();
  }

  void copy_image_to_buffer(VkImage src_image,
                            VkImageLayout src_image_layout,
                            VkBuffer dst_buffer,
                            uint32_t region_count,
                            const VkBufferImageCopy *p_regions) override
  {
    UNUSED_VARS(src_image, src_image_layout, dst_buffer, region_count, p_regions);
    BLI_assert_msg(is_recording_,
                   "Command is added to command buffer, which isn't in recording state.");
    BLI_assert_unreachable();
  }

  void fill_buffer(VkBuffer dst_buffer,
                   VkDeviceSize dst_offset,
                   VkDeviceSize size,
                   uint32_t data) override
  {
    BLI_assert_msg(is_recording_,
                   "Command is added to command buffer, which isn't in recording state.");
    std::stringstream ss;
    ss << "fill_buffer(";
    ss << "dst_buffer=" << dst_buffer;
    ss << ", dst_offset=" << dst_offset;
    ss << ", size=" << size;
    ss << ", data=" << data;
    ss << ")";
    log_.append(ss.str());
  }

  void clear_color_image(VkImage image,
                         VkImageLayout image_layout,
                         const VkClearColorValue *p_color,
                         uint32_t range_count,
                         const VkImageSubresourceRange *p_ranges) override
  {
    UNUSED_VARS(p_color, range_count, p_ranges);
    BLI_assert_msg(is_recording_,
                   "Command is added to command buffer, which isn't in recording state.");
    std::stringstream ss;
    ss << "clear_color_image(";
    ss << "image=" << image;
    ss << ", image_layout=" << to_string(image_layout);
    ss << ")";
    log_.append(ss.str());
  }

  void clear_depth_stencil_image(VkImage image,
                                 VkImageLayout image_layout,
                                 const VkClearDepthStencilValue *p_depth_stencil,
                                 uint32_t range_count,
                                 const VkImageSubresourceRange *p_ranges) override
  {
    UNUSED_VARS(image, image_layout, p_depth_stencil, range_count, p_ranges);
    BLI_assert_msg(is_recording_,
                   "Command is added to command buffer, which isn't in recording state.");
    BLI_assert_unreachable();
  }

  void clear_attachments(uint32_t attachment_count,
                         const VkClearAttachment *p_attachments,
                         uint32_t rect_count,
                         const VkClearRect *p_rects) override
  {
    UNUSED_VARS(attachment_count, p_attachments, rect_count, p_rects);
    BLI_assert_msg(is_recording_,
                   "Command is added to command buffer, which isn't in recording state.");
    BLI_assert_unreachable();
  }

  void pipeline_barrier(VkPipelineStageFlags src_stage_mask,
                        VkPipelineStageFlags dst_stage_mask,
                        VkDependencyFlags dependency_flags,
                        uint32_t memory_barrier_count,
                        const VkMemoryBarrier *p_memory_barriers,
                        uint32_t buffer_memory_barrier_count,
                        const VkBufferMemoryBarrier *p_buffer_memory_barriers,
                        uint32_t image_memory_barrier_count,
                        const VkImageMemoryBarrier *p_image_memory_barriers) override
  {
    UNUSED_VARS(dependency_flags, memory_barrier_count, p_memory_barriers);
    BLI_assert_msg(is_recording_,
                   "Command is added to command buffer, which isn't in recording state.");
    std::stringstream ss;
    ss << "pipeline_barrier(";
    ss << "src_stage_mask=" << to_string_vk_shader_stage_flags(src_stage_mask);
    ss << ", dst_stage_mask=" << to_string_vk_shader_stage_flags(dst_stage_mask);
    ss << "\n";
    for (VkImageMemoryBarrier image_barrier :
         Span<VkImageMemoryBarrier>(p_image_memory_barriers, image_memory_barrier_count))
    {
      ss << " - image_barrier(" << to_string(image_barrier, 1) << ")\n";
    }
    for (VkBufferMemoryBarrier buffer_barrier :
         Span<VkBufferMemoryBarrier>(p_buffer_memory_barriers, buffer_memory_barrier_count))
    {
      ss << " - buffer_barrier(" << to_string(buffer_barrier, 1) << ")\n";
    }
    ss << ")";

    log_.append(ss.str());
  }

  void push_constants(VkPipelineLayout layout,
                      VkShaderStageFlags stage_flags,
                      uint32_t offset,
                      uint32_t size,
                      const void *p_values) override
  {
    UNUSED_VARS(layout, stage_flags, offset, size, p_values);
    BLI_assert_msg(is_recording_,
                   "Command is added to command buffer, which isn't in recording state.");
    BLI_assert_unreachable();
  }

  void begin_render_pass(const VkRenderPassBeginInfo *p_render_pass_begin,
                         VkSubpassContents contents) override
  {
    UNUSED_VARS(p_render_pass_begin, contents);
    BLI_assert_msg(is_recording_,
                   "Command is added to command buffer, which isn't in recording state.");
    BLI_assert_unreachable();
  }

  void end_render_pass() override
  {
    BLI_assert_msg(is_recording_,
                   "Command is added to command buffer, which isn't in recording state.");
    BLI_assert_unreachable();
  }
};

/**
 * Union to create a dummy vulkan handler.
 *
 * Due to platform differences the actual VKObjectType type can be different (`uint64_t` or
 * `VkObjectType_T*`).
 */
template<typename VKObjectType> union VkHandle {
  VKObjectType vk_handle;
  uint64_t handle;

  VkHandle(uint64_t handle) : handle(handle) {}

  operator VKObjectType() const
  {
    return vk_handle;
  }
};

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
      "pipeline_barrier(src_stage_mask=, dst_stage_mask=\n"
      " - buffer_barrier(src_access_mask=VK_ACCESS_TRANSFER_WRITE_BIT, "
      "dst_access_mask=VK_ACCESS_TRANSFER_WRITE_BIT, buffer=0x1, offset=0, "
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
      "pipeline_barrier(src_stage_mask=, dst_stage_mask=\n"
      " - buffer_barrier(src_access_mask=VK_ACCESS_TRANSFER_WRITE_BIT, "
      "dst_access_mask=VK_ACCESS_TRANSFER_WRITE_BIT, buffer=0x1, offset=0, "
      "size=18446744073709551615)\n"
      ")",
      log[1]);
  EXPECT_EQ("fill_buffer(dst_buffer=0x1, dst_offset=0, size=1024, data=42)", log[2]);
}

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
