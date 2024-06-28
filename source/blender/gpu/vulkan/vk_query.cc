/* SPDX-FileCopyrightText: 2022 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_query.hh"
#include "vk_backend.hh"
#include "vk_context.hh"
#include "vk_memory.hh"

namespace blender::gpu {

VKQueryPool::~VKQueryPool()
{
  VKBackend &backend = VKBackend::get();
  const VKDevice &device = backend.device;
  VK_ALLOCATION_CALLBACKS;

  while (!vk_query_pools_.is_empty()) {
    VkQueryPool vk_query_pool = vk_query_pools_.pop_last();
    vkDestroyQueryPool(device.vk_handle(), vk_query_pool, vk_allocation_callbacks);
  }
}

uint32_t VKQueryPool::query_index_in_pool() const
{
  return queries_issued_ - (vk_query_pools_.size() - 1) * query_chunk_len_;
}

void VKQueryPool::init(GPUQueryType type)
{
  BLI_assert(vk_query_pools_.is_empty());
  queries_allocated_ = 0;
  queries_issued_ = 0;
  vk_query_type_ = to_vk_query_type(type);
}

void VKQueryPool::begin_query()
{
  if (queries_issued_ == queries_allocated_) {
    VKBackend &backend = VKBackend::get();
    const VKDevice &device = backend.device;
    VK_ALLOCATION_CALLBACKS;

    VkQueryPoolCreateInfo create_info = {};
    create_info.sType = VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO;
    create_info.queryType = vk_query_type_;
    create_info.queryCount = query_chunk_len_;

    VkQueryPool vk_query_pool = VK_NULL_HANDLE;
    vkCreateQueryPool(device.vk_handle(), &create_info, vk_allocation_callbacks, &vk_query_pool);
    vk_query_pools_.append(vk_query_pool);
    queries_allocated_ += query_chunk_len_;
  }

  VKContext &context = *VKContext::get();

  render_graph::VKBeginQueryNode::Data begin_query = {};
  begin_query.vk_query_pool = vk_query_pools_.last();
  begin_query.query_index = query_index_in_pool();
  context.render_graph.add_node(begin_query);
}

void VKQueryPool::end_query()
{
  VKContext &context = *VKContext::get();
  render_graph::VKEndQueryNode::Data end_query = {};
  end_query.vk_query_pool = vk_query_pools_.last();
  end_query.query_index = query_index_in_pool();
  context.render_graph.add_node(end_query);
  queries_issued_ += 1;
}

void VKQueryPool::get_occlusion_result(MutableSpan<uint32_t> r_values)
{
  if (vk_query_pools_.is_empty()) {
    return;
  }
  // TODO: base this on queries_issued and not the vector len.

  VKBackend &backend = VKBackend::get();
  const VKDevice &device = backend.device;
  for (int index : IndexRange(vk_query_pools_.size() - 1)) {
    VkQueryPool vk_query_pool = vk_query_pools_[index];
    uint32_t *r_values_chunk = &r_values[index * query_chunk_len_];
    vkGetQueryPoolResults(device.vk_handle(),
                          vk_query_pool,
                          0,
                          query_chunk_len_,
                          query_chunk_len_ * sizeof(uint32_t),
                          r_values_chunk,
                          sizeof(uint32_t),
                          VK_QUERY_RESULT_WAIT_BIT);
  }

  {
    VkQueryPool vk_query_pool = vk_query_pools_.last();
    uint32_t *r_values_chunk = &r_values[(vk_query_pools_.size() - 1) * query_chunk_len_];
    uint32_t values_left = queries_issued_ - (vk_query_pools_.size() - 1) * query_chunk_len_;
    vkGetQueryPoolResults(device.vk_handle(),
                          vk_query_pool,
                          0,
                          values_left,
                          values_left * sizeof(uint32_t),
                          r_values_chunk,
                          sizeof(uint32_t),
                          VK_QUERY_RESULT_WAIT_BIT);
  }
}

}  // namespace blender::gpu
