/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "testing/testing.h"

#include "vk_common.hh"
#include "vk_render_graph.hh"

namespace blender::gpu {

class CommandBufferLog : public VKRenderGraphCommandBuffer {
  Vector<std::string> &log_;

 public:
  CommandBufferLog(Vector<std::string> &log) : log_(log) {}
  virtual ~CommandBufferLog() {}

  void begin_recording() override {}
  void end_recording() override {}
  void bind_pipeline(VkPipelineBindPoint pipeline_bind_point, VkPipeline pipeline) override
  {
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
    BLI_assert_unreachable();
  }

  void bind_index_buffer(VkBuffer buffer, VkDeviceSize offset, VkIndexType index_type) override
  {
    BLI_assert_unreachable();
  }

  void bind_vertex_buffers(uint32_t first_binding,
                           uint32_t binding_count,
                           const VkBuffer *p_buffers,
                           const VkDeviceSize *p_offsets) override
  {
    BLI_assert_unreachable();
  }

  void draw(uint32_t vertex_count,
            uint32_t instance_count,
            uint32_t first_vertex,
            uint32_t first_instance) override
  {
    BLI_assert_unreachable();
  }

  void draw_indexed(uint32_t index_count,
                    uint32_t instance_count,
                    uint32_t first_index,
                    int32_t vertex_offset,
                    uint32_t first_instance) override
  {
    BLI_assert_unreachable();
  }

  void draw_indirect(VkBuffer buffer,
                     VkDeviceSize offset,
                     uint32_t draw_count,
                     uint32_t stride) override
  {
    BLI_assert_unreachable();
  }

  void draw_indexed_indirect(VkBuffer buffer,
                             VkDeviceSize offset,
                             uint32_t draw_count,
                             uint32_t stride) override
  {
    BLI_assert_unreachable();
  }

  void dispatch(uint32_t group_count_x, uint32_t group_count_y, uint32_t group_count_z) override
  {
    BLI_assert_unreachable();
  }

  void dispatch_indirect(VkBuffer buffer, VkDeviceSize offset) override
  {
    BLI_assert_unreachable();
  }

  void copy_buffer(VkBuffer src_buffer,
                   VkBuffer dst_buffer,
                   uint32_t region_count,
                   const VkBufferCopy *p_regions) override
  {
    BLI_assert_unreachable();
  }

  void copy_image(VkImage src_image,
                  VkImageLayout src_image_layout,
                  VkImage dst_image,
                  VkImageLayout dst_image_layout,
                  uint32_t region_count,
                  const VkImageCopy *p_regions) override
  {
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
    BLI_assert_unreachable();
  }

  void copy_buffer_to_image(VkBuffer src_buffer,
                            VkImage dst_image,
                            VkImageLayout dst_image_layout,
                            uint32_t region_count,
                            const VkBufferImageCopy *p_regions) override
  {
    BLI_assert_unreachable();
  }

  void copy_image_to_buffer(VkImage src_image,
                            VkImageLayout src_image_layout,
                            VkBuffer dst_buffer,
                            uint32_t region_count,
                            const VkBufferImageCopy *p_regions) override
  {
    BLI_assert_unreachable();
  }

  void fill_buffer(VkBuffer dst_buffer,
                   VkDeviceSize dst_offset,
                   VkDeviceSize size,
                   uint32_t data) override
  {
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
    BLI_assert_unreachable();
  }

  void clear_attachments(uint32_t attachment_count,
                         const VkClearAttachment *p_attachments,
                         uint32_t rect_count,
                         const VkClearRect *p_rects) override
  {
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
    std::stringstream ss;
    ss << "pipeline_barrier(\n";
    for (VkImageMemoryBarrier image_barrier :
         Span<VkImageMemoryBarrier>(p_image_memory_barriers, image_memory_barrier_count))
    {
      ss << " - image_barrier(image=";
      ss << image_barrier.image;
      ss << ", old_layout=" << to_string(image_barrier.oldLayout);
      ss << ", new_layout=" << to_string(image_barrier.newLayout);
      ss << ")\n";
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
    BLI_assert_unreachable();
  }

  void begin_render_pass(const VkRenderPassBeginInfo *p_render_pass_begin,
                         VkSubpassContents contents) override
  {
    BLI_assert_unreachable();
  }

  void end_render_pass() override
  {
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

TEST(vk_render_graph, fill_and_read_back)
{
  VkHandle<VkBuffer> buffer(1u);

  Vector<std::string> log;
  CommandBufferLog l(log);
  VKCommandBufferWrapper wrapper;
  VKRenderGraph render_graph(std::make_unique<CommandBufferLog>(log),
                             std::make_unique<Sequential>());
  render_graph.add_buffer(buffer);
  render_graph.add_fill_buffer_node(buffer, 1024, 0);
  render_graph.submit_buffer_for_read_back(buffer);

  EXPECT_EQ(1, log.size());
  EXPECT_EQ("fill_buffer(dst_buffer=0x1, dst_offset=0, size=1024, data=0)", log[0]);
}

TEST(vk_render_graph, transfer_and_present)
{
  VkHandle<VkImage> back_buffer(1u);

  Vector<std::string> log;
  CommandBufferLog l(log);
  VKCommandBufferWrapper wrapper;
  VKRenderGraph render_graph(std::make_unique<CommandBufferLog>(log),
                             std::make_unique<Sequential>());
  render_graph.add_image(
      back_buffer, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, ResourceOwner::SWAP_CHAIN);

  render_graph.submit_for_present(back_buffer);

  EXPECT_EQ(1, log.size());
  EXPECT_EQ(
      "pipeline_barrier(\n"
      " - image_barrier(image=0x1, old_layout=VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, "
      "new_layout=VK_IMAGE_LAYOUT_PRESENT_SRC_KHR)\n"
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

  // Test for transition to TRANSFER_DST
  // Test clear command
  // Test for transition to present
}

}  // namespace blender::gpu
